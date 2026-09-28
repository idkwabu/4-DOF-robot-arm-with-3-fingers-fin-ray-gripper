"""Merge Dum-E's LoRA adapter into a standalone model directory."""

from pathlib import Path

import torch
from peft import PeftModel
from transformers import AutoModelForCausalLM, AutoTokenizer

BASE_DIR = Path(__file__).resolve().parent
BASE_MODEL = "Qwen/Qwen3-0.6B"
ADAPTER_DIR = BASE_DIR / "dum-e-qwen3-robot"
OUTPUT_DIR = BASE_DIR / "dum-e-qwen3-merged"

if not (ADAPTER_DIR / "adapter_config.json").exists():
    raise SystemExit(f"no adapter at {ADAPTER_DIR}; run train_robot.py first")

base_model = AutoModelForCausalLM.from_pretrained(
    BASE_MODEL,
    torch_dtype=torch.float16,
    device_map="cpu",
    low_cpu_mem_usage=True,
)
adapter_model = PeftModel.from_pretrained(base_model, str(ADAPTER_DIR))
merged_model = adapter_model.merge_and_unload()
merged_model.save_pretrained(
    str(OUTPUT_DIR),
    safe_serialization=True,
    max_shard_size="2GB",
)
AutoTokenizer.from_pretrained(BASE_MODEL, use_fast=True).save_pretrained(str(OUTPUT_DIR))
print(f"Saved merged model to {OUTPUT_DIR}")
