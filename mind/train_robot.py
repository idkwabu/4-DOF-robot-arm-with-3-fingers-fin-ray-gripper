"""Fine-tune Qwen3-0.6B into Dum-E's intent classifier.

The model learns one thing: turn a request into {"action", "direction"}.
Every angle is resolved by mind.py, so the target is tiny and the system
prompt is the only long part of each row.

Two details matter more than the hyperparameters:

- apply_chat_template must be called with enable_thinking=False here, exactly
  as mind.py does at inference. Without it Qwen3 emits a <think> preamble and
  never reaches the JSON.
- max_length must exceed the longest formatted row. The previous run used 256
  against ~380-token rows, which silently truncated the answer off the end of
  every single example.
"""

import json
import random
import sys
from pathlib import Path

import torch
from datasets import load_dataset
from peft import LoraConfig, PeftModel, get_peft_model
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from trl import SFTConfig, SFTTrainer

sys.path.insert(0, str(Path(__file__).parent))
import mind

BASE_DIR = Path(__file__).resolve().parent
BASE_MODEL = "Qwen/Qwen3-0.6B"
DATASET_FILE = BASE_DIR / "robot_dataset.jsonl"
EVAL_DATASET_FILE = BASE_DIR / "robot_dataset_eval.jsonl"
OUTPUT_DIR = BASE_DIR / "dum-e-qwen3-robot"
SYSTEM = (BASE_DIR / "system_prompt.txt").read_text(encoding="utf-8").strip()
MAX_STEPS = 1000
EVAL_SAMPLES = 60

tokenizer = AutoTokenizer.from_pretrained(BASE_MODEL, use_fast=True)
if tokenizer.pad_token is None:
    tokenizer.pad_token = tokenizer.eos_token

quantization = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.bfloat16,
    bnb_4bit_use_double_quant=True,
)

model = AutoModelForCausalLM.from_pretrained(
    BASE_MODEL,
    quantization_config=quantization,
    device_map="auto",
)

train_dataset = load_dataset("json", data_files=str(DATASET_FILE), split="train")
eval_dataset = load_dataset("json", data_files=str(EVAL_DATASET_FILE), split="train")


def format_example(example):
    return {
        "text": tokenizer.apply_chat_template(
            example["messages"],
            tokenize=False,
            add_generation_prompt=False,
            enable_thinking=False,
        )
    }


train_dataset = train_dataset.map(format_example, remove_columns=train_dataset.column_names)
eval_dataset = eval_dataset.map(format_example, remove_columns=eval_dataset.column_names)

tokenizer.padding_side = "right"
random.seed(42)

lengths = [len(tokenizer(text)["input_ids"]) for text in train_dataset["text"]]
max_length = max(lengths) + 8
print(f"rows: {len(train_dataset)} train, {len(eval_dataset)} holdout")
print(f"token length min/mean/max: {min(lengths)}/{sum(lengths) // len(lengths)}/{max(lengths)}")
print(f"max_length: {max_length}")

# Attention and MLP projections, so the adapter can steer both the attention
# paths and the feed-forward paths that pick an action and a direction.
lora = LoraConfig(
    r=8,
    lora_alpha=16,
    lora_dropout=0.05,
    bias="none",
    task_type="CAUSAL_LM",
    target_modules=[
        "q_proj",
        "k_proj",
        "v_proj",
        "o_proj",
        "gate_proj",
        "up_proj",
        "down_proj",
    ],
)

trainer = SFTTrainer(
    model=model,
    processing_class=tokenizer,
    train_dataset=train_dataset,
    eval_dataset=eval_dataset,
    peft_config=lora,
    args=SFTConfig(
        output_dir=str(OUTPUT_DIR),
        dataset_text_field="text",
        max_length=max_length,
        per_device_train_batch_size=1,
        gradient_accumulation_steps=4,
        max_steps=MAX_STEPS,
        learning_rate=1e-4,
        lr_scheduler_type="cosine",
        warmup_steps=100,
        max_grad_norm=1.0,
        optim="paged_adamw_8bit",
        logging_steps=1,
        save_strategy="steps",
        save_steps=250,
        eval_strategy="no",
        gradient_checkpointing=False,
        fp16=False,
        bf16=True,
        report_to="none",
        seed=42,
        dataloader_pin_memory=False,
    ),
)

if MAX_STEPS > 0:
    trainer.train()
    trainer.save_model(str(OUTPUT_DIR))
    tokenizer.save_pretrained(str(OUTPUT_DIR))
    print(f"Saved the LoRA adapter to {OUTPUT_DIR}")


def predict(prompts: list[str]) -> list[str]:
    trained = PeftModel.from_pretrained(
        AutoModelForCausalLM.from_pretrained(
            BASE_MODEL, quantization_config=quantization, device_map="auto"
        ),
        str(OUTPUT_DIR),
    )
    trained.eval()
    outputs = []
    for prompt in prompts:
        text = tokenizer.apply_chat_template(
            [
                {"role": "system", "content": SYSTEM},
                {"role": "user", "content": prompt},
            ],
            tokenize=False,
            add_generation_prompt=True,
            enable_thinking=False,
        )
        inputs = tokenizer(text, return_tensors="pt").to(trained.device)
        with torch.no_grad():
            generated = trained.generate(
                **inputs,
                max_new_tokens=64,
                do_sample=False,
                pad_token_id=tokenizer.eos_token_id,
            )
        raw = tokenizer.decode(generated[0][len(inputs["input_ids"][0]):], skip_special_tokens=True)
        outputs.append(raw.strip())
    del trained
    torch.cuda.empty_cache()
    return outputs


held_out_records = [
    json.loads(line)
    for line in EVAL_DATASET_FILE.read_text(encoding="utf-8").splitlines()
    if line.strip()
]
held_out = random.Random(7).sample(
    sorted(held_out_records, key=lambda record: record["messages"][1]["content"]),
    EVAL_SAMPLES,
)
prompts = [record["messages"][1]["content"] for record in held_out]
expected = [
    tuple(json.loads(record["messages"][2]["content"])[key] for key in ("action", "direction"))
    for record in held_out
]

correct = 0
failures = []
for prompt, want, raw in zip(prompts, expected, predict(prompts)):
    got = mind.parse_intent(raw)
    if got == want:
        correct += 1
    else:
        failures.append((prompt, want, raw, got))

print(f"\nheld-out intent accuracy: {correct}/{len(prompts)} = {correct / len(prompts):.1%}")
for prompt, want, raw, got in failures:
    print(f"  user:      {prompt!r}\n  expected:  {want}\n  raw:       {raw!r}\n  parsed:    {got}")

if failures:
    raise SystemExit(f"only {correct}/{len(prompts)} held-out prompts matched")
