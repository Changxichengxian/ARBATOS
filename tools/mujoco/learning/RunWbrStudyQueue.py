"""串行运行冻结的研究计划；保留日志，拒绝重复队列，支持中断续跑。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def process_alive(pid):
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel.OpenProcess.restype = wintypes.HANDLE
        kernel.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = kernel.OpenProcess(0x1000, False, int(pid))
        if not handle:
            return ctypes.get_last_error() != 87  # 查不到是退出；无权限则保守拒绝重复启动。
        try:
            code = wintypes.DWORD()
            return not kernel.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value == 259
        finally:
            kernel.CloseHandle(handle)
    try:
        os.kill(int(pid), 0)
        return True
    except ProcessLookupError:
        return False


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_bytes((json.dumps(value, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    temporary.replace(path)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def option(arguments, name, default=None):
    return arguments[arguments.index(name)+1] if name in arguments else default


def source_binding(learning_directory):
    root = Path(learning_directory).resolve().parents[2]
    manifest = json.loads((root/"manifest.json").read_text(encoding="utf-8"))
    library = root/manifest["library"]
    if sha256_file(library) != manifest["library_sha256"]:
        raise RuntimeError("冻结控制动态库哈希不匹配")
    selected = {key:value for key,value in manifest["files"].items()
                if not key.startswith("tools/mujoco/learning/") or
                key in ("tools/mujoco/learning/WbrLearningEnv.py",
                        "tools/mujoco/learning/WbrLearningBridge.c",
                        "tools/mujoco/learning/StudyWbrPolicy.py")}
    identity = hashlib.sha256(json.dumps(selected, sort_keys=True).encode()).hexdigest()
    return {"library_sha256": manifest["library_sha256"],
            "experiment_identity": identity}


def input_fingerprint(path):
    path = Path(path).resolve()
    if path.is_file():
        return {"path": str(path), "sha256": sha256_file(path)}
    if path.is_dir():
        files = []
        for name in ("policy.json", "policy.npz", "summary.json"):
            candidate = path/name
            if candidate.is_file():
                files.append({"name": name, "sha256": sha256_file(candidate)})
        if not files:
            raise RuntimeError(f"输入目录缺少可绑定产物：{path}")
        return {"path": str(path), "files": files}
    raise FileNotFoundError(f"研究输入不存在：{path}")


def make_job_binding(kind, arguments, plan):
    learning = Path(plan["learning_directory"]).resolve()
    script = {"train":"TrainWbrStudy.py", "search":"SearchWbrLinear.py",
              "evaluate":"EvaluateWbrStudy.py"}[kind]
    inputs = {}
    for flag in ("--cases", "--policy", "--lock", "--baseline-cache"):
        value = option(arguments, flag)
        if value is not None:
            inputs[flag] = input_fingerprint(value)
    companion_names = ({"search": ("SearchWbrLinear.py", "EvaluateWbrStudy.py",
                                    "BuildWbrStudyCases.py")}.get(kind, (script,)))
    payload = {"kind": kind, "arguments": list(map(str, arguments)),
               "python": str(plan.get("python", sys.executable)),
               "script": str((learning/script).resolve()),
               "script_sha256": sha256_file(learning/script),
               "source_hashes": {name: sha256_file(learning/name)
                                   for name in companion_names},
               "source": source_binding(learning), "inputs": inputs}
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True,
                         separators=(",", ":")).encode("utf-8")
    return {"payload": payload, "sha256": hashlib.sha256(encoded).hexdigest()}


def output_argument(arguments):
    return Path(arguments[arguments.index("--output")+1])


def expected_env(arguments):
    return {"reward_mode": option(arguments, "--reward-mode", "dense"),
            "action_mode": option(arguments, "--action-mode", "full"),
            "gas_force": 450.0}


def policy_artifacts(directory):
    directory = Path(directory).resolve()
    contract_path = directory/"policy.json"
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    files = [contract_path]
    files += sorted(path for path in directory.iterdir() if path.is_file() and
                    path != contract_path and (path.suffix == ".npz" or
                    "linear" in path.name.lower() or "coeff" in path.name.lower()))
    result = {path.name: sha256_file(path) for path in files}
    if len(files) < 2:
        coefficients = contract.get("coefficients")
        if contract.get("kind") != "linear" or not isinstance(coefficients, list):
            raise RuntimeError(f"策略产物不完整：{directory}")
        encoded = json.dumps(coefficients, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":")).encode("utf-8")
        result["embedded_linear_coefficients"] = hashlib.sha256(encoded).hexdigest()
    return contract, result


def validate_output_binding(kind, directory, arguments, source, script_sha256,
                            source_hashes=None):
    directory = Path(directory)
    if kind == "train":
        run = json.loads((directory/"run.json").read_text(encoding="utf-8"))
        expected = {"seed": int(option(arguments, "--seed", 41)),
                    "env_config": expected_env(arguments),
                    "requested_total_steps": int(option(arguments, "--steps", 131072)),
                    **source}
        for key, value in expected.items():
            if run.get(key) != value:
                raise RuntimeError(f"训练输出绑定不匹配：{directory} {key}")
        if run.get("training_source_sha256") != script_sha256:
            raise RuntimeError(f"训练源码绑定不匹配：{directory}")
    elif kind == "search":
        state = json.loads((directory/"search-state.json").read_text(encoding="utf-8"))
        cases = json.loads(Path(option(arguments, "--cases")).read_text(encoding="utf-8"))
        expected = {"seed": int(option(arguments, "--seed", 41)),
                    "scenarios_sha256": cases["cases_sha256"],
                    "env_config": {"reward_mode": "dense", "action_mode": "symmetric",
                                   "gas_force": 450.0},
                    "generations": int(option(arguments, "--generations", 5)),
                    "population": int(option(arguments, "--population", 8)), **source}
        for key, value in expected.items():
            if state.get("binding", {}).get(key) != value:
                raise RuntimeError(f"搜索输出绑定不匹配：{directory} {key}")
        hashes = state.get("binding", {}).get("source_hashes", {})
        if source_hashes and hashes != source_hashes:
            raise RuntimeError(f"搜索源码绑定不匹配：{directory}")
    else:
        run = json.loads((directory/"run.json").read_text(encoding="utf-8"))
        binding = run.get("binding", {})
        encoded = json.dumps(binding, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":")).encode("utf-8")
        run_hash = hashlib.sha256(encoded).hexdigest()
        if run.get("binding_sha256") != run_hash:
            raise RuntimeError(f"评价运行绑定自身哈希不匹配：{directory}")
        cases = json.loads(Path(option(arguments, "--cases")).read_text(encoding="utf-8"))
        for key, value in {"scenarios_sha256": cases["cases_sha256"], **source}.items():
            if binding.get(key) != value:
                raise RuntimeError(f"评价输出绑定不匹配：{directory} {key}")
        if binding.get("evaluator_sha256") != script_sha256:
            raise RuntimeError(f"评价源码绑定不匹配：{directory}")
        policy = option(arguments, "--policy")
        if policy:
            contract, artifacts = policy_artifacts(policy)
            if binding.get("policy_directory") != str(Path(policy).resolve()):
                raise RuntimeError(f"评价策略目录绑定不匹配：{directory}")
            if binding.get("policy_artifacts") != artifacts:
                raise RuntimeError(f"评价策略文件绑定不匹配：{directory}")
            expected_config = contract.get("env_config")
        else:
            expected_config = expected_env(arguments)
            if binding.get("policy_directory") is not None or binding.get("policy_artifacts") is not None:
                raise RuntimeError(f"基线评价意外绑定了策略：{directory}")
        if binding.get("env_config") != expected_config:
            raise RuntimeError(f"评价环境配置绑定不匹配：{directory}")
        lock = option(arguments, "--lock")
        if lock:
            lock_data = json.loads(Path(lock).read_text(encoding="utf-8"))
            if binding.get("policy_lock_sha256") != lock_data.get("lock_sha256"):
                raise RuntimeError(f"评价锁绑定不匹配：{directory}")
        elif binding.get("policy_lock_sha256") is not None:
            raise RuntimeError(f"评价意外绑定了未声明的锁：{directory}")
        cache = option(arguments, "--baseline-cache")
        cache_file = None
        if cache:
            cache_file = Path(cache)
            cache_file = cache_file/"summary.json" if cache_file.is_dir() else cache_file
        expected_cache = sha256_file(cache_file) if cache_file else None
        if binding.get("baseline_cache_sha256") != expected_cache:
            raise RuntimeError(f"评价基线缓存绑定不匹配：{directory}")
        if binding.get("record") != ("--record" in arguments):
            raise RuntimeError(f"评价轨迹保存配置绑定不匹配：{directory}")


def is_complete(kind, directory, arguments, source, script_sha256,
                source_hashes=None):
    metadata = Path(directory)/("search-state.json" if kind == "search" else "run.json")
    if not metadata.is_file():
        return False
    validate_output_binding(kind, directory, arguments, source, script_sha256,
                            source_hashes)
    if kind == "evaluate":
        summary_path = directory/"summary.json"
        if not summary_path.is_file():
            return False
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        run = json.loads((directory/"run.json").read_text(encoding="utf-8"))
        if summary.get("run_binding_sha256") != run.get("binding_sha256"):
            raise RuntimeError(f"评价汇总与运行绑定不匹配：{directory}")
        if "--baseline-only" in arguments:
            return bool(summary.get("acceptance", {}).get("checks", {}).get(
                "physical_complete", {}).get("pass"))
        return True
    if not (directory/"status.json").is_file():
        return False
    status = json.loads((directory/"status.json").read_text(encoding="utf-8"))
    if status.get("status") != "completed" or not (directory/"policy.json").is_file():
        return False
    if kind == "train":
        expected = int(arguments[arguments.index("--steps")+1])
        return int(status.get("steps", 0)) >= expected
    return True


def training_checkpoint(directories, arguments, source, training_source_sha256):
    candidates = []
    for directory in directories:
        directory = Path(directory)
        for contract in [directory/"policy.json", *directory.glob("checkpoints/*/policy.json")]:
            checkpoint = contract.parent/"checkpoint.zip"
            if not contract.is_file() or not checkpoint.is_file():
                continue
            try:
                payload = json.loads(contract.read_text(encoding="utf-8"))
                expected = {"seed": int(option(arguments, "--seed", 41)),
                            "env_config": expected_env(arguments), **source}
                if any(payload.get(key) != value for key, value in expected.items()):
                    continue
                if payload.get("training_source_sha256") != training_source_sha256:
                    continue
                if payload.get("checkpoint_sha256") != sha256_file(checkpoint):
                    continue
                candidates.append((int(payload["training_steps"]), contract.parent))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
    return max(candidates, default=(0,None), key=lambda item:item[0])[1]


def resolve_arguments(arguments, state):
    result = []
    for value in arguments:
        if value.startswith("{job:") and value.endswith("}"):
            dependency = state["jobs"][value[5:-1]]
            if dependency["status"] != "completed":
                raise RuntimeError("所依赖的研究步骤尚未完成")
            value = dependency["actual_output"]
        result.append(value)
    return result


def request_child_stop(process, output, attempt, grace_seconds=45.0):
    """先请求子任务自行保存退出；超时后只终止当前 Popen 对应的进程树。"""
    if process is None or process.poll() is not None:
        return
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output/"stop.request").touch()
    attempt["stop_requested_at"] = datetime.now(timezone.utc).isoformat()
    try:
        process.wait(timeout=grace_seconds)
        attempt["stop_mode"] = "graceful"
        return
    except subprocess.TimeoutExpired:
        pass
    # poll 使用 Popen 持有的进程句柄；只有同一个子进程仍存活时才按其 PID 操作。
    if process.poll() is not None:
        return
    attempt["forced_stop_at"] = datetime.now(timezone.utc).isoformat()
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       check=False, timeout=15)
    else:
        process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=10)
    attempt["stop_mode"] = "forced_tree" if os.name == "nt" else "forced"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    plan_path = args.plan.resolve()
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan.get("schema") != 1 or not plan.get("jobs"):
        raise ValueError("研究计划格式错误")
    ids = [job["id"] for job in plan["jobs"]]
    if len(set(ids)) != len(ids):
        raise ValueError("研究步骤名称重复")
    if any(job["kind"] not in ("train", "search", "evaluate") for job in plan["jobs"]):
        raise ValueError("研究步骤类型错误")
    root = plan_path.parent
    planned_outputs = []
    for job in plan["jobs"]:
        raw_output = str(output_argument(job["arguments"]))
        if raw_output.startswith("{job:"):
            raise ValueError("任务输出目录不能使用依赖占位符")
        planned_outputs.append(str(Path(raw_output).resolve()))
    if len(set(planned_outputs)) != len(planned_outputs):
        raise ValueError("研究步骤不能重复使用同一输出目录")
    state_path = root/"queue-state.json"
    plan_hash = hashlib.sha256(plan_path.read_bytes()).hexdigest()
    deadline = datetime.fromisoformat(plan["deadline"]).timestamp()
    lock_stream = (root/"queue.lock").open("a+b")
    lock_stream.seek(0)
    if lock_stream.read(1) == b"":
        lock_stream.write(b"0")
        lock_stream.flush()
    lock_stream.seek(0)
    if os.name == "nt":
        import msvcrt
        msvcrt.locking(lock_stream.fileno(), msvcrt.LK_NBLCK, 1)
    else:
        import fcntl
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    # 操作系统锁随进程退出自动释放；不能只靠残留的 PID 判断。
    if state_path.exists():
        if not args.resume:
            raise RuntimeError("队列已有记录，请使用 --resume 继续")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state["plan_sha256"] != plan_hash:
            raise RuntimeError("研究计划已改变，拒绝混合续跑")
        for entry in state["jobs"].values():
            if entry["status"] == "running" and entry["attempts"]:
                previous_pid = entry["attempts"][-1].get("pid")
                if previous_pid and process_alive(previous_pid):
                    raise RuntimeError(f"上一轮子进程 {previous_pid} 仍在运行，拒绝重复启动")
        if state.get("orphan_child_pid") and process_alive(state["orphan_child_pid"]):
            raise RuntimeError("上一轮留下的子进程仍在运行，需先核查")
        state.pop("orphan_child_pid", None)
    else:
        existing = [path for path in planned_outputs if Path(path).exists()]
        if existing:
            raise FileExistsError(f"新研究计划拒绝预存输出：{existing}")
        state = dict(schema=1, plan_sha256=plan_hash, jobs={job_id:dict(status="queued", attempts=[])
                     for job_id in ids})
    state.update(status="running", pid=os.getpid(), current_job=None)

    def save():
        state["updated_at"] = datetime.now(timezone.utc).isoformat()
        write_json(state_path, state)

    process = None
    try:
        save()
        for job in plan["jobs"]:
            entry = state["jobs"][job["id"]]
            if time.time() >= deadline or (root/"stop.request").exists():
                state["status"] = "interrupted"
                save()
                return
            arguments = resolve_arguments(job["arguments"], state)
            original_output = output_argument(arguments).resolve()
            binding = make_job_binding(job["kind"], arguments, plan)
            if entry.get("job_binding") is None:
                if entry["attempts"]:
                    raise RuntimeError(f"旧队列步骤缺少可核验绑定：{job['id']}")
                entry["job_binding"] = binding
            elif entry["job_binding"] != binding:
                raise RuntimeError(f"研究步骤命令、源码或输入绑定已改变：{job['id']}")
            source = binding["payload"]["source"]
            script_sha = binding["payload"]["script_sha256"]
            source_hashes = binding["payload"]["source_hashes"]
            known_outputs = []
            for attempt_row in entry["attempts"]:
                known = attempt_row.get("actual_output")
                if known is None and attempt_row.get("command"):
                    known = str(output_argument(attempt_row["command"]).resolve())
                if known:
                    known_outputs.append(Path(known).resolve())
            if entry["attempts"]:
                previous_output = known_outputs[-1]
                if Path(entry.get("actual_output", previous_output)).resolve() != previous_output:
                    raise RuntimeError(f"队列记录的输出目录不属于最后一次已知尝试：{job['id']}")
            else:
                previous_output = original_output
                if previous_output.exists():
                    raise RuntimeError(f"未启动步骤出现外来输出目录：{previous_output}")
            if entry["status"] == "completed":
                if not is_complete(job["kind"], previous_output, arguments, source,
                                   script_sha, source_hashes):
                    raise RuntimeError(f"已完成步骤的产物不再完整：{job['id']}")
                continue
            if (entry["attempts"] and previous_output.exists() and
                    is_complete(job["kind"], previous_output, arguments, source,
                                script_sha, source_hashes)):
                entry.update(status="completed", actual_output=str(previous_output))
                save()
                continue
            if (entry["attempts"] and job["kind"] == "evaluate" and
                    "--baseline-only" in arguments and
                    (previous_output/"summary.json").is_file()):
                entry["status"] = "needs_attention"
                state["status"] = "needs_attention"
                save()
                return
            actual_output = original_output
            restart_without_checkpoint = False
            if entry["attempts"]:
                if not previous_output.exists() and job["kind"] != "train":
                    raise RuntimeError(f"已知尝试的输出目录丢失：{previous_output}")
                if job["kind"] == "train":
                    checkpoint = training_checkpoint(
                        known_outputs, arguments, source, script_sha)
                    actual_output = original_output.with_name(original_output.name+f"-resume-{len(entry['attempts'])+1:02d}")
                    if actual_output.exists():
                        raise RuntimeError(f"续训输出目录已存在且不属于已知尝试：{actual_output}")
                    if checkpoint:
                        arguments += ["--resume", str(checkpoint)]
                    else:
                        restart_without_checkpoint = True
                else:
                    actual_output = previous_output
                    if not args.resume or (root/"stop.request").exists() or time.time() >= deadline:
                        raise RuntimeError("只有显式续跑且停止文件已清除、期限未到，才能恢复原输出")
                    stop_file = actual_output/"stop.request"
                    if stop_file.exists():
                        stop_file.unlink()
                    arguments += ["--resume"]
            arguments[arguments.index("--output")+1] = str(actual_output)
            script = {"train":"TrainWbrStudy.py", "search":"SearchWbrLinear.py",
                      "evaluate":"EvaluateWbrStudy.py"}[job["kind"]]
            command = [plan.get("python", sys.executable), "-u", str(Path(plan["learning_directory"])/script), *arguments]
            logs = root/"logs"
            logs.mkdir(exist_ok=True)
            log = logs/f"{job['id']}-{len(entry['attempts'])+1:02d}.log"
            attempt = dict(command=command, log=str(log), actual_output=str(actual_output),
                           job_binding_sha256=binding["sha256"],
                           restart_without_checkpoint=restart_without_checkpoint,
                           started_at=datetime.now(timezone.utc).isoformat())
            entry.update(status="running", actual_output=str(actual_output))
            entry["attempts"].append(attempt)
            state["current_job"] = job["id"]
            save()
            with log.open("ab") as stream:
                process = subprocess.Popen(command, cwd=plan["learning_directory"], stdout=stream,
                    stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                    start_new_session=os.name != "nt")
                attempt["pid"] = process.pid
                save()
                while process.poll() is None:
                    if time.time() >= deadline or (root/"stop.request").exists():
                        request_child_stop(process, actual_output, attempt)
                        save()
                    time.sleep(5.)
                attempt.update(exit_code=process.returncode, finished_at=datetime.now(timezone.utc).isoformat())
            process = None
            completed = (attempt["exit_code"] == 0 and
                         is_complete(job["kind"], actual_output, arguments, source,
                                     script_sha, source_hashes))
            entry["status"] = "completed" if completed else "needs_attention"
            print(json.dumps(dict(job=job["id"], status=entry["status"], output=str(actual_output))), flush=True)
            if not completed:
                state["status"] = "needs_attention"
                save()
                return
            save()
        state.update(status="completed", current_job=None)
        save()
    except BaseException as error:
        state.update(status="needs_attention", error=repr(error))
        save()
        if process is not None and process.poll() is None:
            request_child_stop(process, Path(state["jobs"][state["current_job"]]["actual_output"]),
                               state["jobs"][state["current_job"]]["attempts"][-1])
            if process.poll() is None:
                state["orphan_child_pid"] = process.pid
            else:
                state.pop("orphan_child_pid", None)
            save()
        raise
    finally:
        lock_stream.close()


if __name__ == "__main__":
    main()
