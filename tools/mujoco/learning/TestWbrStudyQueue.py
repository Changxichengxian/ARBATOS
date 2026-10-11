"""不运行 MuJoCo，检查研究队列的恢复、绑定和停止边界。"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from RunWbrStudyQueue import (is_complete, make_job_binding, request_child_stop,
                              training_checkpoint)


def write(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def make_checkpoint(directory: Path, steps: int, source: dict,
                    training_hash: str, *, valid: bool = True) -> Path:
    directory.mkdir(parents=True)
    checkpoint = directory/"checkpoint.zip"
    checkpoint.write_bytes(f"checkpoint-{steps}".encode())
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    write(directory/"policy.json", {
        "seed": 41,
        "env_config": {"reward_mode": "dense", "action_mode": "full",
                       "gas_force": 450.0},
        **source,
        "training_source_sha256": training_hash,
        "training_steps": steps,
        "checkpoint_sha256": digest if valid else "bad",
    })
    return directory


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        source = {"library_sha256": "library", "experiment_identity": "identity"}
        train_hash = "train-source"
        valid_old = make_checkpoint(root/"attempt-1", 32768, source, train_hash)
        make_checkpoint(root/"attempt-2", 49152, source, train_hash, valid=False)
        valid_new = make_checkpoint(root/"attempt-3", 40960, source, train_hash)
        empty_latest = root/"attempt-4"
        empty_latest.mkdir()
        arguments = ["--output", str(root/"train"), "--steps", "131072",
                     "--seed", "41", "--reward-mode", "dense",
                     "--action-mode", "full"]
        selected = training_checkpoint(
            [valid_old, root/"attempt-2", valid_new, empty_latest],
            arguments, source, train_hash)
        assert selected == valid_new

        snapshot = root/"snapshot"
        learning = snapshot/"tools"/"mujoco"/"learning"
        learning.mkdir(parents=True)
        (learning/"TrainWbrStudy.py").write_text("# frozen train\n", encoding="utf-8")
        library = snapshot/"controller.dll"
        library.write_bytes(b"controller")
        write(snapshot/"manifest.json", {
            "library": "controller.dll",
            "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
            "files": {"tools/mujoco/learning/WbrLearningEnv.py": "env-hash"},
        })
        binding = make_job_binding("train", arguments,
                                   {"learning_directory": str(learning),
                                    "python": sys.executable})
        assert binding["payload"]["arguments"] == arguments
        assert binding["payload"]["script_sha256"] == binding["payload"][
            "source_hashes"]["TrainWbrStudy.py"]

        cases = root/"cases.json"
        write(cases, {"cases_sha256": "case-hash"})
        evaluation = root/"evaluation"
        eval_binding = {"scenarios_sha256": "case-hash", **source,
                        "evaluator_sha256": "evaluate-source", "policy_directory": None,
                        "policy_artifacts": None, "policy_lock_sha256": None,
                        "env_config": {"reward_mode": "dense", "action_mode": "full",
                                       "gas_force": 450.0},
                        "baseline_cache_sha256": None, "record": False}
        encoded = json.dumps(eval_binding, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":")).encode("utf-8")
        eval_hash = hashlib.sha256(encoded).hexdigest()
        write(evaluation/"run.json", {"binding": eval_binding,
                                      "binding_sha256": eval_hash})
        write(evaluation/"summary.json", {"acceptance": {"checks": {
            "physical_complete": {"pass": False}}},
            "run_binding_sha256": eval_hash})
        eval_args = ["--cases", str(cases), "--output", str(evaluation),
                     "--baseline-only"]
        assert not is_complete("evaluate", root/"empty-evaluation", eval_args,
                               source, "evaluate-source")
        assert not is_complete("evaluate", evaluation, eval_args, source,
                               "evaluate-source")
        write(evaluation/"summary.json", {"acceptance": {"checks": {
            "physical_complete": {"pass": True}}},
            "run_binding_sha256": eval_hash})
        assert is_complete("evaluate", evaluation, eval_args, source,
                           "evaluate-source")

        child_output = root/"child"
        code = ("import pathlib,sys,time; p=pathlib.Path(sys.argv[1])/'stop.request'; "
                "\nwhile not p.exists(): time.sleep(.02)")
        child = subprocess.Popen([sys.executable, "-c", code, str(child_output)])
        attempt = {}
        request_child_stop(child, child_output, attempt, grace_seconds=5)
        assert child.poll() is not None
        assert attempt["stop_mode"] == "graceful"

    print(json.dumps({
        "checkpoint_fallback": {"pass": True, "selected_steps": 40960},
        "baseline_physics_gate": {"pass": True},
        "graceful_child_stop": {"pass": True},
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
