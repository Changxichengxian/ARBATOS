"""用首组真实完整结果核验兼容入口，保留原文件字节，并拒绝错锁与变更计划。"""
import copy
from datetime import datetime
import json
from pathlib import Path
import shutil
from TraceMonitorLockCompatibility import base, queue, read, sha, require, install, verify_context, verify_report_lock

target = base / "lock-compatibility-v2"
require(not target.exists(), "兼容核验记录已经存在")
plan = read(base / "evaluation/plan.json")
state = read(base / "evaluation/queue-state.json")
job = state["jobs"]["E-41"]
require(state["status"] == "needs_attention" and job["attempts"][-1]["exit_code"] == 0, "不是首组已完成但锁字段缺失的停止状态")
directory = base / "evaluation/E-41"
arguments = plan["jobs"][0]["arguments"]
payload = job["job_binding"]["payload"]
captured = {str(p): sha(p) for p in directory.rglob("*") if p.is_file()}
require(len(list((directory / "completed").rglob("*.json"))) == 48, "原始完成记录不是48条")
try:
    queue.is_complete("evaluate", directory, arguments, payload["source"], payload["script_sha256"], payload["source_hashes"])
except RuntimeError as error:
    require("评价锁绑定不匹配" in str(error), "旧程序失败原因不符")
else:
    raise RuntimeError("未复现旧内部锁记录问题")
install()
require(queue.is_complete("evaluate", directory, arguments, payload["source"], payload["script_sha256"], payload["source_hashes"]), "真实完整结果未通过兼容核验")
require(queue.make_job_binding("evaluate", arguments, plan) == job["job_binding"], "兼容改变了原任务绑定")
from CompareTraceResults import verify_evaluation, verify_snapshot
manifest = verify_snapshot(base / "frozen")
cases = Path(queue.option(arguments, "--cases"))
policy = Path(queue.option(arguments, "--policy"))
lock = Path(queue.option(arguments, "--lock"))
report, completed = verify_evaluation(directory, cases, base / "baseline/summary.json", policy, manifest, lock)
require(len(completed) == 48, "完整评价记录未复核")
proof = verify_report_lock(directory, cases, policy, lock)
negative_checks = {}
for name, action in (
    ("wrong_seed_lock", lambda: verify_report_lock(directory, cases, policy, base / "comparison-locks/E-73.json")),
    ("unregistered_output", lambda: verify_context(arguments, plan, base / "evaluation/replacement")),
    ("changed_plan", lambda: verify_context(arguments, dict(plan, deadline="2026-10-15T02:10:00+08:00"))),
    ("missing_lock_argument", lambda: verify_context([v for i, v in enumerate(arguments) if i not in (arguments.index("--lock"), arguments.index("--lock")+1)], plan)),
):
    try:
        action()
    except RuntimeError:
        negative_checks[name] = True
    else:
        raise RuntimeError("错误输入未被拒绝：" + name)
require(captured == {str(p): sha(p) for p in directory.rglob("*") if p.is_file()}, "核验改变了原始首组结果")
target.mkdir()
shutil.copyfile(base / "evaluation/queue-state.json", target / "原队列停止记录.json")
record = dict(schema=1, checked_at=datetime.now().astimezone().isoformat(), status="passed",
    original_inner_lock_rejection_reproduced=True, actual_complete_records_checked=48,
    original_job_binding_unchanged=True, original_result_files_unchanged=True,
    original_result_files=captured, outer_lock_proof=proof, negative_checks=negative_checks,
    source_files={name: sha(Path(__file__).parent / name) for name in
        ("CheckTraceLockCompatibility.py", "TraceMonitorLockCompatibility.py", "ContinueTraceComparison.py", "CompareTraceResults.py")},
    boundary="实际首组原始结果逐条核验，内部锁null保持原样；其余三份尚未评价，未改模型、场景、指标或期限。")
(target / "外层锁兼容核验.json").write_bytes((json.dumps(record, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
print(json.dumps({k: record[k] for k in ("status", "actual_complete_records_checked", "original_result_files_unchanged", "negative_checks")}, ensure_ascii=False))
