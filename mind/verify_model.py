"""End-to-end check of the deployed pipeline against the held-out split.

For every held-out prompt this runs the same order mind.ask_bot uses:

1. build_local_command, which owns direction words, numbers and the gripper
2. the model, for whatever is left

Scoring the model alone would be misleading, because most held-out prompts
never reach it.
"""

import json
import random
import sys
from collections import Counter
from pathlib import Path

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

sys.path.insert(0, str(Path(__file__).parent))
import mind

BASE_DIR = Path(__file__).resolve().parent
EVAL_FILE = BASE_DIR / "robot_dataset_eval.jsonl"
SAMPLE = 10_000
THRESHOLD = 0.95
MODEL_SAMPLE = 120


def angles(command: dict) -> dict:
    return {joint: command.get(joint) for joint in mind.ALL_JOINTS}


def main() -> int:
    records = [json.loads(line) for line in EVAL_FILE.read_text(encoding="utf-8").splitlines() if line.strip()]
    stale = [
        record["messages"][0]["content"]
        for record in records
        if record["messages"][0]["content"] != mind.SYSTEM_PROMPT
    ]
    if stale:
        print(f"FAIL: {len(stale)} holdout rows were built against an older system_prompt.txt; regenerate the dataset")
        return 1
    records = random.Random(11).sample(records, min(SAMPLE, len(records)))

    local_rows, model_rows = [], []
    for record in records:
        prompt = record["messages"][1]["content"]
        target = json.loads(record["messages"][2]["content"])
        expected = mind.resolve_command(target["action"], target["direction"])
        got = mind.build_local_command(prompt)
        (local_rows if got is not None else model_rows).append((prompt, expected, got))

    local_ok = sum(1 for _, expected, got in local_rows if got == expected)
    print(f"held-out prompts sampled: {len(records)}")
    print(f"answered locally:         {len(local_rows)} ({len(local_rows) / len(records):.1%})")
    print(f"deferred to the model:    {len(model_rows)}")
    if local_rows:
        print(f"local accuracy:           {local_ok}/{len(local_rows)} = {local_ok / len(local_rows):.1%}")

    for prompt, expected, got in local_rows:
        if got != expected:
            print(f"  LOCAL MISS {prompt!r}\n    expected {angles(expected)}\n    got      {angles(got)}")

    if not model_rows:
        print("\nno prompts reached the model")
        return 0

    model_rows = random.Random(5).sample(model_rows, min(MODEL_SAMPLE, len(model_rows)))
    model, tokenizer = mind._load_model()
    correct = 0
    for prompt, expected, _ in model_rows:
        text = tokenizer.apply_chat_template(
            [{"role": "system", "content": mind.SYSTEM_PROMPT}, {"role": "user", "content": prompt}],
            tokenize=False,
            add_generation_prompt=True,
            enable_thinking=False,
        )
        inputs = tokenizer(text, return_tensors="pt").to(model.device)
        with torch.no_grad():
            generated = model.generate(
                **inputs,
                max_new_tokens=64,
                do_sample=False,
                pad_token_id=tokenizer.eos_token_id,
            )
        raw = tokenizer.decode(generated[0][len(inputs["input_ids"][0]):], skip_special_tokens=True).strip()
        intent = mind.parse_intent(raw)
        got = mind.resolve_command(*intent) if intent else None
        if got == expected:
            correct += 1
        else:
            print(f"  MODEL MISS {prompt!r}\n    expected {expected['action']}/{expected['direction']}"
                  f" {angles(expected)}\n    raw      {raw!r}\n    parsed   {intent}")

    accuracy = correct / len(model_rows)
    print(f"\nmodel accuracy on its own share: {correct}/{len(model_rows)} = {accuracy:.1%}")
    print("action mix the model actually saw:", dict(Counter(expected["action"] for _, expected, _ in model_rows)))
    if len(model_rows) < len(records) - len(local_rows):
        print(f"note: {len(records) - len(local_rows) - len(model_rows)} model-bound prompts were not sampled")

    scored = len(local_rows) + len(model_rows)
    end_to_end = (local_ok + correct) / scored
    print(f"end-to-end accuracy on scored rows: {local_ok + correct}/{scored} = {end_to_end:.1%}")
    if end_to_end < THRESHOLD:
        print(f"\nFAIL: end-to-end {end_to_end:.1%} is below the {THRESHOLD:.0%} gate")
        return 1
    print(f"\nPASS: end-to-end {end_to_end:.1%} >= {THRESHOLD:.0%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
