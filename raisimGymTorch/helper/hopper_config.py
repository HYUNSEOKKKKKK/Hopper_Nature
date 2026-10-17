"""Paths and configuration shared by the public Hopper entry points."""

import argparse
from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[2]
TASK_DIR = REPO_ROOT / "raisimGymTorch/env/envs/hopper"


def positive_int(value):
    result = int(value)
    if result < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return result


def checkpoint_path(value):
    path = Path(value).expanduser().resolve()
    match = re.fullmatch(r"full_(\d+)\.pt", path.name)
    if not match or not path.is_file():
        raise argparse.ArgumentTypeError("provide an existing full_<iteration>.pt checkpoint")
    for name in (f"mean{match[1]}.csv", f"var{match[1]}.csv", "cfg.yaml"):
        if not (path.parent / name).is_file():
            raise argparse.ArgumentTypeError(f"missing checkpoint companion: {path.parent / name}")
    return path


def load_config(path=None, *, num_envs=None, num_threads=None, headless=False, seed=None):
    from ruamel.yaml import YAML

    source = Path(path).expanduser().resolve() if path else TASK_DIR / "cfg.yaml"
    with source.open() as stream:
        cfg = YAML().load(stream)
    if num_envs is not None:
        cfg["environment"]["num_envs"] = num_envs
    if num_threads is not None:
        cfg["environment"]["num_threads"] = num_threads
    if headless:
        cfg["environment"]["render"] = False
        cfg["record_video"] = False
    if seed is not None:
        cfg["seed"] = seed
    return cfg
