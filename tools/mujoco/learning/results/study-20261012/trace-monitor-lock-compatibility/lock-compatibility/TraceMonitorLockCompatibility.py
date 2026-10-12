"""仅兼容本次已事前锁定的四任务：旧监控评价不填写内部锁，外层严格核验。"""
from datetime import datetime
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import subprocess
import sys

workspace = Path(__file__).resolve().parents[1]
base = workspace / "work/study-20261012/long-trace"
frozen = base / "frozen"
sys.path.insert(0, str(frozen / "tools/mujoco/learning"))
import RunWbrStudyQueue as queue
from EvaluateWbrStudy import load_cases, verify_lock
from WbrLearningEnv import verify_snapshot, experiment_identity

read = lambda p: json.loads(Path(p).read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
PLAN_SHA = "a7769c31de98ab522eca3622a13c611b229b349cb8eade74cd357a37c4b9b459"
LOCK_SHA = "11f9a1072ee59ebc4834ca29b563ac9b946900b4810d8a6edd7174658806a1b8"
original_make_binding = queue.make_job_binding
original_validate = queue.validate_output_binding


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


@lru_cache(maxsize=1)
def committed_evidence():
    repo = Path(r"D:\ARBATOS")
    def blob(commit, relative):
        return subprocess.run(["git", "-C", str(repo), "show", commit + ":" + relative],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout
    prefix = "tools/mujoco/learning/results/study-20261012/"
    commit = "1fdb9e1b9209442836eb429c63628ae7c450c334"
    record = blob(commit, prefix + "trace-F-four-policy-locks/四策略锁定.json")
    plan = blob(commit, prefix + "trace-F-four-policy-locks/评价计划.json")
    require(hashlib.sha256(record).hexdigest() == LOCK_SHA and hashlib.sha256(plan).hexdigest() == PLAN_SHA,
        "固定Git提交中的原始锁或计划不一致")
    before = "ecbabfc9ea69f2888e9c02eb280fd4f1f8a458ec"
    for model in json.loads(record.decode("utf-8"))["models"]:
        directory = ("aligned-E-" if model["route"] == "E" else "trace-F-") + str(model["seed"]) + "-trained/"
        for name, value in model["files"].items():
            require(hashlib.sha256(blob(before, prefix + directory + name)).hexdigest() == value,
                "评价前提交中的最终模型或完成证据不一致")
    return dict(model_commit_before_evaluation=before, locks_and_plan_commit_during_first_evaluation=commit,
        boundary="四模型在评价前已提交；锁实际生成早于启动，锁和计划的Git保存发生于首组运行中。没有将后一次提交冒称评价前提交。")


def verify_context(arguments, plan=None, directory=None):
    committed = committed_evidence()
    plan_path = base / "evaluation/plan.json"
    record_path = base / "四策略锁定.json"
    require(sha(plan_path) == PLAN_SHA and sha(record_path) == LOCK_SHA, "原唯一计划或事前四模型锁改变")
    registered_plan, record = read(plan_path), read(record_path)
    require(plan is None or plan == registered_plan, "不是原唯一评价计划")
    require(registered_plan["all_four_policy_lock_sha256"] == LOCK_SHA and record["all_four_completed_and_checked"], "原计划没有绑定完整四模型锁")
    jobs = [j for j in registered_plan["jobs"] if list(map(str, arguments)) == j["arguments"]]
    require(len(jobs) == 1, "命令不属于原唯一四任务，拒绝兼容")
    label = jobs[0]["id"]
    require({j["id"] for j in registered_plan["jobs"]} == {"E-41", "E-73", "F-41", "F-73"}, "任务集合不符")
    require(directory is None or Path(directory).resolve() == (base / "evaluation" / label).resolve(), "评价输出目录错配")
    manifest = verify_snapshot(frozen)
    identity = experiment_identity(manifest)
    cases_path = Path(queue.option(arguments, "--cases"))
    cases = load_cases(cases_path)
    require(cases["split"] == "trace-monitor" and cases["cases_sha256"] == record["cases_sha256"] ==
        "a935e32db4cf96af9fe74013be09cc718a60b9686646b4983c8691f6130b4226", "不是登记的新24例")
    require(record["source_sha256"] == sha(workspace / "work/LockTraceComparison.py")
        and record["completion_checker_sha256"] == sha(workspace / "work/WbrFinalModelChecks.py"), "事前模型核验入口改变")
    require(len(record["models"]) == 4, "缺少完整模型")
    selected = None
    for model in record["models"]:
        model_label = f"{model['route']}-{model['seed']}"
        model_dir = Path(model["directory"])
        lock_path = base / "comparison-locks" / (model_label + ".json")
        require(all(sha(model_dir / name) == value for name, value in model["files"].items()), "锁定后模型或运行记录改变")
        require(model["completion"]["natural_budget_completion"] and model["actual_steps"] == 262144, "不是完整固定预算模型")
        require(sha(lock_path) == model["lock_file_sha256"], "单模型锁文件改变")
        lock = verify_lock(lock_path, cases_path, cases, model_dir, read(model_dir / "policy.json"), manifest, identity)
        require(lock["lock_sha256"] == model["lock_sha256"], "单模型锁内容改变")
        if model_label == label:
            selected = model
    require(selected is not None and Path(queue.option(arguments, "--policy")).resolve() == Path(selected["directory"]).resolve(), "实际策略不属于对应锁")
    require(Path(queue.option(arguments, "--lock")).resolve() == (base / "comparison-locks" / (label + ".json")).resolve(), "对应锁路径错配")
    require(sha(base / "baseline/summary.json") == record["baseline_summary_sha256"], "共同原控制改变")
    state_path = base / "evaluation/queue-state.json"
    if state_path.exists():
        state = read(state_path)
        require(state["plan_sha256"] == PLAN_SHA, "实际队列不是原计划")
        locked = datetime.fromisoformat(record["locked_at"])
        for job in state["jobs"].values():
            for attempt in job["attempts"]:
                require(locked < datetime.fromisoformat(attempt["started_at"]), "模型锁不是评价前生成")
    return dict(label=label, plan_sha256=PLAN_SHA, four_model_lock_sha256=LOCK_SHA,
        policy_lock_sha256=selected["lock_sha256"], verifier_sha256=sha(Path(__file__)),
        committed_evidence=committed,
        inner_lock_field_expected=None,
        boundary="旧监控评价内部锁字段为空；原始记录不改写，由原唯一计划、事前四模型锁、完整模型哈希与原队列绑定共同核验。")


def verify_report_lock(directory, cases_path, policy_directory, lock_path):
    plan = read(base / "evaluation/plan.json")
    jobs = [j for j in plan["jobs"] if Path(queue.option(j["arguments"], "--output")).resolve() == Path(directory).resolve()]
    require(len(jobs) == 1, "报告不属于原唯一评价输出")
    args = jobs[0]["arguments"]
    for flag, path in (("--cases", cases_path), ("--policy", policy_directory), ("--lock", lock_path)):
        require(Path(queue.option(args, flag)).resolve() == Path(path).resolve(), "报告外层锁输入错配：" + flag)
    proof = verify_context(args, plan, directory)
    run = read(Path(directory) / "run.json")
    require(run["binding"]["policy_lock_sha256"] is None, "旧评价原始内部锁字段意外改变")
    summary_path = Path(directory) / "summary.json"
    if summary_path.exists():
        require(read(summary_path)["policy_lock_sha256"] is None, "旧汇总内部锁字段意外改变")
    return proof


def applies(arguments):
    cases = queue.option(arguments, "--cases")
    lock = queue.option(arguments, "--lock")
    return bool((cases and Path(cases).resolve() == (frozen / "tools/mujoco/learning/cases/trace-20261012/monitor.json").resolve())
        or (lock and Path(lock).resolve().parent == (base / "comparison-locks").resolve()))


def install():
    if getattr(queue, "trace_lock_compatibility_installed", False):
        return
    def make_binding(kind, arguments, plan):
        if kind == "evaluate" and applies(arguments):
            verify_context(arguments, plan)
        return original_make_binding(kind, arguments, plan)
    def validate(kind, directory, arguments, source, script_sha256, source_hashes=None):
        if kind != "evaluate" or not applies(arguments):
            return original_validate(kind, directory, arguments, source, script_sha256, source_hashes)
        verify_report_lock(directory, queue.option(arguments, "--cases"), queue.option(arguments, "--policy"), queue.option(arguments, "--lock"))
        # 原始报告没有内部锁；只在已完整核验外层锁的唯一计划内，移除该单个检查输入。
        reduced = list(arguments)
        index = reduced.index("--lock")
        del reduced[index:index+2]
        return original_validate(kind, directory, reduced, source, script_sha256, source_hashes)
    queue.make_job_binding = make_binding
    queue.validate_output_binding = validate
    queue.trace_lock_compatibility_installed = True

