"""保存经检查后的F73从零重跑决定及实际启动记录。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
base = workspace / "work/study-20261012/long-trace"
target = repo / "tools/mujoco/learning/results/study-20261012/trace-F-73-resume"
progress_path = repo / "tools/mujoco/learning/研究进度.json"
read = lambda p: json.loads(p.read_text(encoding="utf-8"))
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def require(value, message):
    if not value:
        raise RuntimeError(message)

def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+"\n", encoding="utf-8", newline="\n")

require(sys.argv[1:] in ([], ["--started"]), "未知操作")
progress = read(progress_path)
now = datetime.now().astimezone().isoformat()
proof = read(base / "暂停重跑预检.json")
require(proof["action"] == "restart_seed_73_from_zero", "实际预检不符")
if sys.argv[1:] == []:
    rejected = read(target / "恢复预检.json")
    rejected.update(executed=False, rejected=True, rejection_reason=proof["reason"])
    write(target / "被否决的按步数续训方案.json", rejected)
    shutil.copyfile(base / "暂停重跑预检.json", target / "恢复预检.json")
    shutil.copyfile(workspace / "work/RestartTraceAfterPause.py", target / "RestartTraceAfterPause.py")
    shutil.copyfile(__file__, target / Path(__file__).name)
    authorization = read(target / "用户恢复与续训记录.json")
    authorization.pop("selected_checkpoint", None)
    authorization.update(selected_steps=0, action=proof["action"], decision_updated_at=now,
        recovery_entry_sha256=proof["recovery_entry_sha256"], reason=proof["reason"], boundary=proof["boundary"],
        rejected_proposal="原32768步续训方案经更新次数核对否决，未启动过；旧模型全部保留。")
    write(target / "用户恢复与续训记录.json", authorization)
    write(base / "用户明确恢复记录.json", authorization)
    progress["user_resume"] = authorization
    progress["long_trace_F"].pop("selected_resume_checkpoint", None)
    progress["long_trace_F"].update(selected_resume_steps=0, stage="restart_authorized_second_seed", restart_reason=proof["reason"])
    progress["next"] = "从原seed73重新完成F73的262144步，旧42496步暂停尝试不计入最终候选。完成后核对模型更新计数768和实际来源，再锁定E/F四份模型并只执行登记的新24例比较。保存工具须支持train-resume-02路径。"
    progress["resume_instructions"] = "本次用户已明确继续，旧goal paused记录更新时间1791769664早于本授权，不应据其再次停机；新的暂停/取消或较新的目标暂停记录仍优先。此次使用work/RestartTraceAfterPause.py，仅允许F73重跑一次；如再次中断需重新核对训练计数和更新次数，不能盲用旧恢复入口。"
    progress["completed"].append("实际优化器更新计数否决了32768步续训方案，该方案未执行。保留原42496步模型，从原seed73重新完整训练，以保持最终候选262144步和768更新计数；未选择性能较好的随机种子或模型。")
else:
    state = read(base / "training/queue-state.json")
    job = state["jobs"]["F-trace-73-train"]
    attempt = job["attempts"][-1]
    actual = Path(job["actual_output"])
    require(state["status"] == job["status"] == "running" and len(job["attempts"]) == 2, "新队列未正常启动")
    require(attempt["restart_without_checkpoint"] and "--resume" not in attempt["command"], "没有按登记从零开始")
    require(actual == base / "training/73/train-resume-02", "新输出来源不符")
    run, status = read(actual / "run.json"), read(actual / "status.json")
    require(run["initial_steps"] == 0 and run["resume"] is None and run["seed"] == 73 and run["requested_total_steps"] == 262144, "新训练来源不符")
    require(status["status"] == "running" and status["steps"] >= 4096, "尚无实际运行进展")
    require(len(state["jobs"]["F-trace-41-train"]["attempts"]) == 1, "第一组被重跑")
    shutil.copyfile(base / "training/queue-state.json", target / "恢复后队列.json")
    shutil.copyfile(actual / "run.json", target / "重跑运行来源.json")
    shutil.copyfile(actual / "status.json", target / "重跑实际进度.json")
    progress.update(status="screening_running", active_process=dict(kind="F73_restart", queue_pid=state["pid"], trainer_pid=status["pid"], output=str(actual)))
    progress["long_trace_F"].update(stage="second_seed_training_restarted", queue_pid=state["pid"], trainer_pid=status["pid"], training_steps_at_save=status["steps"],
        queue_status_at_save=state["status"], actual_training_output=str(actual), os_processes_checked_at=now)
    progress["completed"].append(f"原seed73重跑已实际运行到{status['steps']}步，run.json证明从零开始且原262144预算未变；F41仍为原完成尝试。")
progress["updated_at"] = now
write(progress_path, progress)
print(json.dumps(dict(status=progress["status"], stage=progress["long_trace_F"]["stage"], updated_at=now), ensure_ascii=False))
