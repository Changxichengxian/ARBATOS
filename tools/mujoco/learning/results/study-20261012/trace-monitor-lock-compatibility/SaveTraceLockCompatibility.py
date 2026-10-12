"""保存真实监控锁字段缺失及严格外层兼容证据，保留第一份完整结果。"""
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
from TraceMonitorLockCompatibility import base, workspace, read, sha, require, verify_context

repo = Path(r"D:\ARBATOS")
target = repo / "tools/mujoco/learning/results/study-20261012/trace-monitor-lock-compatibility"
def git(*args, input=None):
    return subprocess.run(["git", "-C", str(repo), *args], input=input, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout

require(git("branch", "--show-current").strip() == b"main" and not git("diff", "--cached", "--name-only").strip(), "分支或索引不符")
require(not target.exists(), "兼容阶段已经保存")
proof = read(base / "lock-compatibility-v2/外层锁兼容核验.json")
require(proof["status"] == "passed" and all(sha(workspace / "work" / name) == value for name, value in proof["source_files"].items()), "兼容实测来源改变")
require(all(sha(Path(path)) == value for path, value in proof["original_result_files"].items()), "原首组结果被修改")
queue = read(base / "evaluation/queue-state.json")
require(queue["status"] == "running" and queue["jobs"]["E-41"]["status"] == "completed"
    and len(queue["jobs"]["E-41"]["attempts"]) == 1, "首组被重跑或队列未恢复")
plan = read(base / "evaluation/plan.json")
outer = verify_context(plan["jobs"][0]["arguments"], plan)
command = f'Get-CimInstance Win32_Process | Where-Object {{ $_.ProcessId -eq {queue["pid"]} }} | Select-Object ProcessId,CreationDate,CommandLine | ConvertTo-Json'
process = json.loads(subprocess.run(["pwsh", "-NoProfile", "-Command", command], stdout=subprocess.PIPE, encoding="utf-8", check=True).stdout)
require(process["ProcessId"] == queue["pid"] and "ContinueTraceComparison.py" in process["CommandLine"], "缺少实际恢复进程")
doc = "tools/仿真说明.md"
head = git("show", "HEAD:" + doc)
require(git("show", ":" + doc) == head, "说明索引已有其他修改")
working = (repo / doc).read_bytes()
marker = "#### 监控评价锁字段缺失的原始记录保留与接续（2026-10-12）"
require(marker.encode("utf-8") not in working, "说明已存在")
target.mkdir()
for folder in base.glob("lock-compatibility*"):
    shutil.copytree(folder, target / folder.name)
for name in ("TraceMonitorLockCompatibility.py", "ContinueTraceComparison.py", "CheckTraceLockCompatibility.py",
    "CompareTraceResults.py", "SaveTraceComparison.py", "SaveTraceLockCompatibility.py"):
    shutil.copyfile(workspace / "work" / name, target / name)
now = datetime.now().astimezone().isoformat()
record = dict(saved_at=now, actual_process=process, current_job=queue["current_job"],
    first_policy_completed_without_rerun=True, first_result_files_unchanged=True,
    outer_lock_proof=outer, original_internal_lock_fields_kept_null=True,
    boundary="只修复监控与队列之间的锁记录兼容，不改冻结程序、模型、场景、指标、预算或期限。")
(target / "兼容接续记录.json").write_bytes((json.dumps(record, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
p = repo / "tools/mujoco/learning/研究进度.json"
old = p.read_bytes()
progress = json.loads(old.decode("utf-8"))
progress.update(updated_at=now, status="comparison_running",
    active_process=dict(kind="EF_new24_comparison", queue_pid=queue["pid"], current_job=queue["current_job"]))
progress["long_trace_F"].update(stage="four_policy_comparison_running", queue_pid=queue["pid"],
    lock_compatibility_record=(target / "兼容接续记录.json").relative_to(repo).as_posix())
progress["completed"].append("旧评价器对监控集未填内部锁，首组48条记录完成后队列拒绝。以事前模型提交、原启动命令绑定及四模型锁完成严格外层核验，错锁/改计划/改启动绑定等检查均拒绝；原始结果字节不变，首组未重跑，其余三组按原队列接续。")
progress["next"]="等待原唯一新24例队列完成，使用已保存外层锁兼容与新版CompareTraceResults.py复算全部192条完成记录；不改原始结果内部锁字段，不重跑训练或打开最终60例。"
require(p.read_bytes() == old, "进度文件被其他任务修改")
p.write_bytes((json.dumps(progress, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
appendix = f"""{marker}

E41在新24例上完整完成后，原队列因锁字段不符而停止。实际原因是冻结评价入口只对`final`读取`--lock`，对`trace-monitor`留下`policy_lock_sha256=null`；物理执行与模型、场景及指标不受影响。48条原控制/候选完成记录、运行和汇总文件全部保留原字节，没有事后改写成有内部锁，也没有重跑或挑选结果。

接续入口仅接受原唯一四任务计划，对四份完整模型、单模型锁、场景、基线、冻结源码、实际启动命令与输入哈希逐项核验，再复用原队列其余检查。四份最终模型在12:32:20的`ecbabfc`已保存；四模型锁实际于12:32:55生成，E41于12:33:14启动，锁与计划于首组运行中的12:34:48提交为`1fdb9e1`。程序核对两个固定Git提交的实际字节，并核对队列启动前计算的锁/模型绑定；不把后一个提交冒称评价前提交。

真实首组48条记录完整复算通过，错种子锁、计划外输出、改期限、缺锁、改启动锁哈希及改实际命令均被拒绝。证据及对应程序见`learning/results/study-20261012/trace-monitor-lock-compatibility/`。第一份已由队列识别为完成且仍只有一次实际尝试，其余三份沿用原输出和原计划接续。评价标准与最终集边界未改变。
"""
require((repo / doc).read_bytes() == working, "说明被其他任务修改")
(repo / doc).write_bytes(working.rstrip(b"\r\n") + b"\n\n" + appendix.encode("utf-8"))
blob = git("hash-object", "-w", "--stdin", input=head.rstrip(b"\r\n") + b"\n\n" + appendix.encode("utf-8")).decode().strip()
git("update-index", "--cacheinfo", f"100644,{blob},{doc}")
paths = [v.relative_to(repo).as_posix() for v in target.rglob("*") if v.is_file()] + [p.relative_to(repo).as_posix()]
git("add", "--pathspec-from-file=-", "--pathspec-file-nul", input=("\0".join(paths) + "\0").encode("utf-8"))
actual = set(git("diff", "--cached", "--name-only", "-z").decode("utf-8").strip("\0").split("\0"))
require(actual == {doc, *paths}, "暂存范围不符")
git("-c", "core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof", "diff", "--cached", "--check")
print(git("-c", "core.quotepath=false", "diff", "--cached", "--stat").decode("utf-8"))
