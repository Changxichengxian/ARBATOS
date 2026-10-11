"""生成轮腿长期研究的固定开发、验证、最终和压力场景清单。"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import random


DEADLINE = "2026-10-14T02:10:00+08:00"
TRAIN_SEEDS = [101, 211, 307]
SCREEN_SEEDS = [41, 73]
SPLITS = {
    "development": {"stop": 2, "push": 2, "combined": 2},
    "validation": {"stop": 18, "push": 18, "combined": 9},
    "final": {"stop": 24, "push": 24, "combined": 12},
}
GENERATOR_SEEDS = {"development": 6103, "validation": 45181, "final": 60247}


def canonical_bytes(value) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":")).encode("utf-8")


def cases_sha256(cases) -> str:
    return hashlib.sha256(canonical_bytes(cases)).hexdigest()


def _signed_values(count: int, low: float, high: float, rng: random.Random) -> list[float]:
    """覆盖上下界并尽量均衡正负号，随后确定性打散。"""
    magnitudes = []
    for index in range(count):
        fraction = 0.5 if count == 1 else index / (count - 1)
        magnitudes.append(low + (high - low) * fraction)
    values = [magnitude * (1.0 if index % 2 == 0 else -1.0)
              for index, magnitude in enumerate(magnitudes)]
    rng.shuffle(values)
    return values


def _balance_combined_pushes(speeds: list[float], pushes: list[float]) -> list[float]:
    """保持推力正负均衡，并让同向/反向组合在可行范围内最均衡。"""
    count = len(speeds)
    speed_positive = sum(value > 0 for value in speeds)
    speed_negative = count-speed_positive
    push_positive = (count+1)//2
    candidates = sorted(range(count+1), key=lambda value: (abs(2*value-count), value))
    for same_direction in candidates:
        numerator = same_direction-speed_negative+push_positive
        if numerator % 2:
            continue
        positive_same = numerator//2
        negative_same = same_direction-positive_same
        if (0 <= positive_same <= speed_positive and
                0 <= negative_same <= speed_negative):
            positive_seen = negative_seen = 0
            result = []
            for speed, push in zip(speeds, pushes):
                if speed > 0:
                    same = positive_seen < positive_same
                    positive_seen += 1
                else:
                    same = negative_seen < negative_same
                    negative_seen += 1
                sign = math.copysign(1.0, speed) * (1.0 if same else -1.0)
                result.append(abs(push)*sign)
            return result
    raise RuntimeError("无法构造正负均衡的组合场景")


def build_split(split: str, counts: dict[str, int], generator_seed: int,
                *, pressure: bool = False) -> list[dict]:
    rng = random.Random(generator_seed)
    cases = []
    seed_base = {"development": 31000, "validation": 41000,
                 "final": 51000, "pressure": 61000}[split]
    cursor = 0
    for case_name in ("stop", "push", "combined"):
        count = counts[case_name]
        speeds = _signed_values(count, 0.8 if pressure else 0.35,
                                0.8 if pressure else 0.75, rng)
        pushes = _signed_values(count, 40.0 if pressure else 12.0,
                               40.0 if pressure else 35.0, rng)
        # 组合场景同时覆盖同向和反向外力，单任务只平衡真正生效的量。
        if case_name == "combined":
            pushes = _balance_combined_pushes(speeds, pushes)
        for index in range(count):
            randomize = bool(index % 2)
            delay = (index // 2) % 2 if randomize else 0
            cases.append({
                "id": f"{split}-{case_name}-{index:03d}",
                "seed": seed_base + cursor,
                "randomize": randomize,
                "options": {
                    "case": case_name,
                    "speed": round(speeds[index], 6),
                    "push": round(pushes[index], 6),
                    "delay": delay,
                    "duration": 8.0,
                },
            })
            cursor += 1
    return cases


def document(split: str, cases: list[dict], *, primary_gate: bool) -> dict:
    return {
        "schema": 1,
        "split": split,
        "primary_gate": primary_gate,
        "deadline": DEADLINE,
        "generated_at": datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds"),
        "training_seeds": TRAIN_SEEDS,
        "screening_seeds": SCREEN_SEEDS,
        "cases_sha256": cases_sha256(cases),
        "case_count": len(cases),
        "cases": cases,
    }


def generate(output: Path, include_pressure: bool = True) -> dict[str, dict]:
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"场景目录已存在，拒绝覆盖：{output}")
    output.mkdir(parents=True)
    result = {}
    for split, counts in SPLITS.items():
        cases = build_split(split, counts, GENERATOR_SEEDS[split])
        result[split] = document(split, cases, primary_gate=split != "development")
    if include_pressure:
        counts = {"stop": 2, "push": 2, "combined": 2}
        cases = build_split("pressure", counts, 61997, pressure=True)
        result["pressure"] = document("pressure", cases, primary_gate=False)
    for split, payload in result.items():
        (output/f"{split}.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    metadata = {
        "schema": 1,
        "deadline": DEADLINE,
        "training_seeds": TRAIN_SEEDS,
        "screening_seeds": SCREEN_SEEDS,
        "splits": {name: {"case_count": value["case_count"],
                           "cases_sha256": value["cases_sha256"],
                           "primary_gate": value["primary_gate"]}
                   for name, value in result.items()},
    }
    (output/"metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--no-pressure", action="store_true")
    args = parser.parse_args()
    result = generate(args.output, include_pressure=not args.no_pressure)
    print(json.dumps({name: {"count": value["case_count"],
                             "cases_sha256": value["cases_sha256"]}
                      for name, value in result.items()}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
