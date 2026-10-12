"""只保存已暂停研究的退出状态和检查点，不运行或恢复实验。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
base = workspace / "work/study-20261012/long-trace"
source = base / "training/73/train"
target = repo / "tools/mujoco/learning/results/study-20261012/trace-F-73-paused"
read = lambda p: json.loads(p.read_text(encoding="utf-8"))
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")

status, policy = read(source / "status.json"), read(source / "policy.json")
queue = read(base / "training/queue-state.json")
job = queue["jobs"]["F-trace-73-train"]
require(status["status"] == "interrupted" and status["stop_reason"] == "stop_file", "未正常响应停止请求")
require(status["steps"] == policy["training_steps"] < 262144, "暂停步数和保存模型不一致")
require(job["attempts"][-1]["stop_mode"] == "graceful" and job["attempts"][-1]["exit_code"] == 0, "队列没有确认正常退出")
require(queue["status"] == job["status"] == "needs_attention", "队列仍在运行")
require((base / "training/stop.request").exists(), "停止标记丢失")
processes = subprocess.check_output([
    "powershell", "-NoProfile", "-Command",
    "@(Get-CimInstance Win32_Process | Where-Object { $_.ProcessId -in @(1504,17560,30472,26556) } | Select-Object ProcessId,CommandLine) | ConvertTo-Json -Compress"
], text=True, encoding="utf-8").strip()
require(processes in ("", "[]"), "研究父进程或训练进程仍在运行")
require(digest(source / "checkpoint.zip") == policy["checkpoint_sha256"], "检查点摘要不一致")
require(digest(source / "policy.npz") == policy["policy_sha256"], "导出策略摘要不一致")
with zipfile.ZipFile(source / "checkpoint.zip") as archive:
    require(archive.testzip() is None, "检查点压缩文件损坏")
    require({"policy.pth", "policy.optimizer.pth", "data"} <= set(archive.namelist()), "缺少网络或优化器")
target.mkdir(exist_ok=False)
names = ("checkpoint.zip", "policy.npz", "policy.json", "run.json", "status.json", "aligned-episodes.jsonl", "progress.jsonl", "latest-checkpoint.json", "stop.request")
for name in names:
    shutil.copyfile(source / name, target / name)
    require(digest(source / name) == digest(target / name), "保存文件不一致: " + name)
for name in ("queue-state.json", "plan.json"):
    shutil.copyfile(base / "training" / name, target / name)
shutil.copyfile(__file__, target / Path(__file__).name)
now = datetime.now().astimezone().isoformat()
record = dict(status="paused", recorded_at=now, reason="目标状态已为 paused；定时跟进遵守暂停，不恢复研究。后台训练收到停止文件后自行保存退出。",
              seed=73, saved_steps=status["steps"], requested_steps=262144, queue_stop_mode="graceful",
              process_check="队列及训练父子进程1504/17560/30472/26556均已退出", source=str(source),
              checkpoint_sha256=policy["checkpoint_sha256"], policy_sha256=policy["policy_sha256"],
              files={name: digest(target / name) for name in names},
              checks="检查点与元数据摘要、压缩包完整性、网络和优化器文件存在及复制一致性；本次未恢复训练、未评价策略。",
              resume_requires_user=True, independent_performance_verified=False)
write(target / "暂停保存记录.json", record)
progress_path = repo / "tools/mujoco/learning/研究进度.json"
progress = read(progress_path)
progress["status"] = "paused"
progress["paused_at"] = now
progress["pause_record"] = str((target / "暂停保存记录.json").relative_to(repo)).replace("\\", "/")
progress["resume_requires_user"] = True
progress["long_trace_F"].update(stage="paused_second_seed", queue_pid=None, trainer_pid=None,
    training_steps_at_save=status["steps"], os_processes_checked_at=now, os_processes_at_save=[],
    queue_status_at_save=queue["status"], pause_record=progress["pause_record"])
progress["completed"].append(f"目标已暂停；F73在{status['steps']}步响应停止文件后保存网络、优化器和日志并正常退出。队列和训练进程均已退出，不自动恢复；未进行F独立性能评价。")
write(progress_path, progress)
(workspace / "outputs/轮腿研究暂停记录.txt").write_text(
    f"轮腿研究已暂停\n保存时间：{now}\n最后一组训练保存到 {status['steps']} / 262144 步。\n后台队列和训练进程已正常退出。模型、优化器及日志已保存。\n这次只保存并停止，没有恢复训练或评价策略。等待用户明确恢复。\n", encoding="utf-8")
print(json.dumps(record, ensure_ascii=False, indent=2))
