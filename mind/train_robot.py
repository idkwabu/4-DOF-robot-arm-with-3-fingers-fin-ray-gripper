"""Fine-tune Qwen3-0.6B into Dum-E's intent classifier.

Training progress includes periodic validation loss and balanced exact-intent
accuracy. The final holdout is reserved for the post-training check only.
"""

import json
import random
import sys
from collections import defaultdict
from pathlib import Path

import torch
from datasets import load_dataset
from peft import LoraConfig
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from trl import SFTConfig, SFTTrainer

sys.path.insert(0, str(Path(__file__).parent))
import mind

BASE_DIR = Path(__file__).resolve().parent
BASE_MODEL = "Qwen/Qwen3-0.6B"
DATASET_FILE = BASE_DIR / "robot_dataset.jsonl"
VALIDATION_DATASET_FILE = BASE_DIR / "robot_dataset_validation.jsonl"
FINAL_HOLDOUT_FILE = BASE_DIR / "robot_dataset_eval.jsonl"
OUTPUT_DIR = BASE_DIR / "dum-e-qwen3-robot"
SYSTEM = (BASE_DIR / "system_prompt.txt").read_text(encoding="utf-8").strip()
MAX_STEPS = 1000
EVAL_STEPS = 100
LOGGING_STEPS = 10
INTENT_SAMPLES_PER_CLASS = 8
GENERATION_BATCH_SIZE = 8
FINAL_HOLDOUT_SAMPLES = 60


def load_records(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def balanced_intent_sample(records: list[dict]) -> list[dict]:
    """Choose a fixed, balanced sample so progress accuracy is comparable."""
    by_intent: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for record in records:
        target = json.loads(record["messages"][2]["content"])
        by_intent[(target["action"], target["direction"])].append(record)

    if not by_intent:
        raise ValueError("Validation dataset is empty.")

    rng = random.Random(42)
    sample = []
    for intent, examples in sorted(by_intent.items()):
        examples = sorted(examples, key=lambda row: row["messages"][1]["content"])
        rng.shuffle(examples)
        sample.extend(examples[:INTENT_SAMPLES_PER_CLASS])

    print(
        f"validation accuracy sample: {len(sample)} prompts across "
        f"{len(by_intent)} intents (up to {INTENT_SAMPLES_PER_CLASS} each)"
    )
    return sample


def score_intents(model, records: list[dict], tokenizer) -> tuple[int, list[tuple]]:
    was_training = model.training
    original_padding_side = tokenizer.padding_side
    model.eval()
    tokenizer.padding_side = "left"
    correct = 0
    failures = []
    try:
        for start in range(0, len(records), GENERATION_BATCH_SIZE):
            batch = records[start:start + GENERATION_BATCH_SIZE]
            prompts = [
                tokenizer.apply_chat_template(
                    [
                        {"role": "system", "content": SYSTEM},
                        {"role": "user", "content": row["messages"][1]["content"]},
                    ],
                    tokenize=False,
                    add_generation_prompt=True,
                    enable_thinking=False,
                )
                for row in batch
            ]
            inputs = tokenizer(prompts, return_tensors="pt", padding=True).to(model.device)
            with torch.no_grad():
                generated = model.generate(
                    **inputs,
                    max_new_tokens=64,
                    do_sample=False,
                    pad_token_id=tokenizer.eos_token_id,
                )
            continuations = generated[:, inputs["input_ids"].shape[1]:]
            for row, continuation in zip(batch, continuations):
                raw = tokenizer.decode(continuation, skip_special_tokens=True).strip()
                got = mind.parse_intent(raw)
                target = json.loads(row["messages"][2]["content"])
                expected = (target["action"], target["direction"])
                if got == expected:
                    correct += 1
                else:
                    failures.append(
                        (
                            row["messages"][1]["content"],
                            expected,
                            raw,
                            got,
                        )
                    )
    finally:
        tokenizer.padding_side = original_padding_side
        if was_training:
            model.train()

    return correct, failures


class IntentAccuracyTrainer(SFTTrainer):
    """Add generation-based exact intent accuracy to evaluation metrics."""

    def __init__(self, *args, intent_validation_records: list[dict], intent_tokenizer, **kwargs):
        super().__init__(*args, **kwargs)
        self.intent_validation_records = intent_validation_records
        self.intent_tokenizer = intent_tokenizer

    def evaluate(self, eval_dataset=None, ignore_keys=None, metric_key_prefix="eval"):
        was_training = self.model.training
        try:
            metrics = super().evaluate(
                eval_dataset=eval_dataset,
                ignore_keys=ignore_keys,
                metric_key_prefix=metric_key_prefix,
            )
            correct, _ = score_intents(
                self.model, self.intent_validation_records, self.intent_tokenizer
            )
            accuracy = correct / len(self.intent_validation_records)
            accuracy_key = f"{metric_key_prefix}_intent_accuracy"
            metrics[accuracy_key] = accuracy
            self.log({accuracy_key: accuracy})
            print(
                f"Step {self.state.global_step}: validation intent accuracy "
                f"{accuracy:.1%} ({len(self.intent_validation_records)} balanced prompts)"
            )
            return metrics
        finally:
            if was_training:
                self.model.train()


def main() -> int:
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
    validation_dataset = load_dataset(
        "json", data_files=str(VALIDATION_DATASET_FILE), split="train"
    )
    validation_records = balanced_intent_sample(load_records(VALIDATION_DATASET_FILE))
    final_holdout_records = load_records(FINAL_HOLDOUT_FILE)

    def format_example(example):
        return {
            "text": tokenizer.apply_chat_template(
                example["messages"],
                tokenize=False,
                add_generation_prompt=False,
                enable_thinking=False,
            )
        }

    train_dataset = train_dataset.map(
        format_example, remove_columns=train_dataset.column_names
    )
    validation_dataset = validation_dataset.map(
        format_example, remove_columns=validation_dataset.column_names
    )

    tokenizer.padding_side = "right"
    lengths = [len(tokenizer(text)["input_ids"]) for text in train_dataset["text"]]
    max_length = max(lengths) + 8
    print(
        f"rows: {len(train_dataset)} train, {len(validation_dataset)} validation, "
        f"{len(final_holdout_records)} final holdout"
    )
    print(
        f"token length min/mean/max: "
        f"{min(lengths)}/{sum(lengths) // len(lengths)}/{max(lengths)}"
    )
    print(f"max_length: {max_length}")

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

    trainer = IntentAccuracyTrainer(
        model=model,
        processing_class=tokenizer,
        train_dataset=train_dataset,
        eval_dataset=validation_dataset,
        intent_validation_records=validation_records,
        intent_tokenizer=tokenizer,
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
            logging_steps=LOGGING_STEPS,
            save_strategy="steps",
            save_steps=EVAL_STEPS,
            save_total_limit=2,
            eval_strategy="steps",
            eval_steps=EVAL_STEPS,
            load_best_model_at_end=True,
            metric_for_best_model="eval_intent_accuracy",
            greater_is_better=True,
            gradient_checkpointing=False,
            fp16=False,
            bf16=True,
            report_to="none",
            seed=42,
            dataloader_pin_memory=False,
            disable_tqdm=False,
        ),
    )

    trainer.train()
    trainer.save_model(str(OUTPUT_DIR))
    tokenizer.save_pretrained(str(OUTPUT_DIR))
    print(
        f"Saved best-validation adapter to {OUTPUT_DIR} "
        f"(best checkpoint {trainer.state.best_model_checkpoint}, "
        f"intent accuracy {trainer.state.best_metric:.1%})"
    )

    held_out = random.Random(7).sample(
        sorted(
            final_holdout_records,
            key=lambda record: record["messages"][1]["content"],
        ),
        min(FINAL_HOLDOUT_SAMPLES, len(final_holdout_records)),
    )
    correct, failures = score_intents(trainer.model, held_out, tokenizer)
    accuracy = correct / len(held_out)
    print(
        f"\nfinal held-out intent accuracy: "
        f"{correct}/{len(held_out)} = {accuracy:.1%}"
    )
    for prompt, expected, raw, parsed in failures:
        print(
            f"  user:      {prompt!r}\n"
            f"  expected:  {expected}\n"
            f"  raw:       {raw!r}\n"
            f"  parsed:    {parsed}"
        )
    if correct != len(held_out):
        raise SystemExit(
            f"only {correct}/{len(held_out)} final held-out prompts matched"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
