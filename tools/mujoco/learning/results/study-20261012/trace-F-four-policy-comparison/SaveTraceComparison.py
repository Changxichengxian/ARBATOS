"""保存唯一新24例四策略对照及逐场景证据，只暂存本任务结果。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from CompareTraceResults import verify_evaluation, gate, verify_snapshot

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
base = workspace / "work/study-20261012/long-trace"
results = repo / "tools/mujoco/learning/results/study-20261012"
read = lambda path: json.loads(path.read_text(encoding="utf-8"))
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def git(*args, input=None):
    return subprocess.run(["git", "-C", str(repo), *args], input=input,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout


def save(path, value):
    path.write_bytes((json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))


require(git("branch", "--show-current").strip() == b"main", "分支不符")
require(not git("diff", "--cached", "--name-only").strip(), "索引已有其他工作")
conclusion_path = base / "四策略完整对照结论.json"
require(conclusion_path.is_file(), "新24例完整对照尚未核验，不能保存结论")
conclusion = read(conclusion_path)
require(conclusion["completed_records_checked"] == 192 and conclusion["all_four_locked_before_evaluation"], "对照证据不完整")
require(not conclusion["expansion_authorized"] and not conclusion["final_dataset_used"], "阶段边界改变")
require(conclusion["source_script_sha256"] == sha(workspace / "work/CompareTraceResults.py"), "结论读取程序改变")
require(all(sha(Path(path)) == value for path, value in conclusion["source_files"].items()), "原对照证据改变")
queue = read(base / "evaluation/queue-state.json")
require(queue["status"] == "completed" and read(base / "training/queue-state.json")["status"] == "completed", "原队列尚未结束")
frozen = base / "frozen"
manifest = verify_snapshot(frozen)
rule_path = frozen / "tools/mujoco/learning/results/study-20261012/长程反馈单变量训练登记.json"
rule = read(rule_path)
cases_path = frozen / "tools/mujoco/learning/cases/trace-20261012/monitor.json"
reports = {}
require(len(conclusion["rows"]) == 4 and {f"{r['route']}-{r['seed']}" for r in conclusion["rows"]} == {"E-41", "E-73", "F-41", "F-73"}, "策略集合不符")
for row in conclusion["rows"]:
    label = f"{row['route']}-{row['seed']}"
    report, completed = verify_evaluation(base / "evaluation" / label, cases_path,
        base / "baseline/summary.json", Path(row["policy_directory"]), manifest,
        base / "comparison-locks" / (label + ".json"))
    checks = report["acceptance"]["checks"]
    require(len(completed) == row["completed_records_checked"] == 48, "逐场景数量不符")
    require(gate(report["acceptance"], rule["screening_first"]) == row["gate"], "登记筛选不能复算")
    require(row["failed_original_cases"] == [r for r in report["acceptance"]["cases"] if not r["pass"]]
        and row["failed_motion_cases"] == [r for r in checks["motion_quality"]["cases"] if not r["pass"]], "失败场景集合不符")
    reports[label] = report
f_pass = all(row["gate"]["pass_all"] for row in conclusion["rows"] if row["route"] == "F")
require(f_pass == conclusion["F_both_seed_screen_pass"], "F双种子结论不符")

target = results / "trace-F-four-policy-comparison"
require(not target.exists(), "本次对照已保存")
doc = "tools/仿真说明.md"
head = git("show", "HEAD:" + doc)
require(git("show", ":" + doc) == head, "说明索引已有其他修改")
working = (repo / doc).read_bytes()
marker = "#### 四份完整策略的新场景统一对照（2026-10-12）"
require(marker.encode("utf-8") not in working, "本次说明已存在")

files = {str(p.relative_to(base)).replace("\\", "/"): p
    for name in ("evaluation", "baseline", "comparison-locks")
    for p in (base / name).rglob("*.json")}
for name in ("四策略完整对照结论.json", "四策略锁定.json", "training/plan.json", "training/queue-state.json"):
    files[name] = base / name
files["长程反馈单变量训练登记.json"] = rule_path
files["新监控场景.json"] = cases_path
for name in ("SaveTraceComparison.py", "CompareTraceResults.py", "LockTraceComparison.py", "WbrFinalModelChecks.py",
    "TraceMonitorLockCompatibility.py", "ContinueTraceComparison.py", "CheckTraceLockCompatibility.py"):
    files[name] = workspace / "work" / name
for directory in base.glob("lock-compatibility*"):
    for path in directory.rglob("*"):
        if path.is_file():
            files[path.relative_to(base).as_posix()] = path
target.mkdir()
for relative, source in files.items():
    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
    require(sha(destination) == sha(source), "保存副本不一致")
now = datetime.now().astimezone().isoformat()
save(target / "保存核验.json", dict(schema=1, saved_at=now, completed_records=192,
    gates_recomputed=True, F_both_seed_screen_pass=f_pass,
    files={relative: sha(target / relative) for relative in files},
    boundary="完整保留原24例四策略配对结果和失败样本；未授权追加训练或使用最终60例。"))

lines = ["轮腿四策略统一对照", f"保存时间：{now}", "",
    "E/F均为两个独立种子，每组完整262144步；全部四份最终模型锁定后才开始这次新24例评价。",
    "停车与受推均值各来自8个单项场景；8个组合场景只参加逐例保护；热指标用全部24例。",
    "热指标是四髋力矩平方积分，不是实测温度。下列收益均相对同场景原控制，负值表示误差增加。", "",
    "方案 / 种子 / 停车误差降低 / 受推误差降低 / 热指标增加 / 原逐例保护失败 / 运动质量失败 / 筛选通过"]
for row in conclusion["rows"]:
    g = row["gate"]
    def percent(value):
        return "不可比较" if value is None else f"{100*value:+.4f}%"
    lines.append(f"{row['route']} / {row['seed']} / {percent(g.get('stop_improvement'))} / {percent(g.get('push_improvement'))} / {percent(g.get('heat_change'))} / {len(row['failed_original_cases'])}/24 / {len(row['failed_motion_cases'])}/24 / {g['pass_all']}")
lines += ["", "F双种子筛选：" + ("通过；还需另行登记后续研究，不能视为正式采用。" if f_pass else "未通过，停止此固定配置。"),
    "E在原监控集未通过的决定保留，新24例仅用于登记的E/F对照。",
    "192条原控制/候选完成记录逐条核对，原验收和运动质量完整复算。物理完成与控制质量达标分别判断。",
    "旧监控评价程序忽略内部锁字段，首组完成后队列因此停下；原结果保持不变，经事前唯一计划和四模型锁的外层严格核验后接续，其余模型与场景、评分均未改变。",
    "改变长程反馈权重也会改变其他评分的传播与训练波动，不能据此单独归因某项末尾惩罚。",
    "正式目标仍是三个独立训练种子各60个封存场景、两项均值各改善至少10%，并满足全部逐例、热和物理边界。最终60例未使用。",
    "此结果只适用于冻结的参考机械模型与控制版本，尚无实车效果结论。", ""]
report_text = "\n".join(lines)
(target / "对照结果说明.txt").write_bytes(report_text.encode("utf-8"))
(workspace / "outputs/轮腿四策略统一对照.txt").write_bytes(report_text.encode("utf-8"))
progress_path = repo / "tools/mujoco/learning/研究进度.json"
progress_bytes = progress_path.read_bytes()
progress = json.loads(progress_bytes.decode("utf-8"))
progress.update(updated_at=now, status="comparison_complete",
    active_process=dict(kind="none", queue_pid=None, trainer_pid=None, updated_at=now))
progress["long_trace_F"].update(stage="comparison_complete", both_seed_screen_pass=f_pass,
    independent_performance_verified=True, expansion_authorized=False,
    queue_pid=None, trainer_pid=None,
    conclusion=(target / "四策略完整对照结论.json").relative_to(repo).as_posix())
progress["next"] = ("F通过本次筛选，先另行登记后续研究，保留正式三种子和60例门槛。" if f_pass else
    "六类方案均未通过各自筛选；完成跨方案负结果、瓶颈和可复现交付核验，再按原目标负结果分支结束研究。")
progress["completed"].append("E/F四份完整模型已事前锁定并完成唯一新24例对照，192条配对记录、原验收和运动质量完整复算；F双种子筛选" + ("通过。" if f_pass else "未通过，不扩大此配置。"))
require(progress_path.read_bytes() == progress_bytes, "进度文件被其他任务修改，停止写入")
save(progress_path, progress)
appendix = marker + "\n\n" + "\n\n".join([lines[3],
    "新24例的四策略结果如下，均相对同一原控制。表中热指标指四髋力矩平方积分，不是温度。",
    "\n".join(lines[7:12]), lines[13],
    "完整192条原控制/候选完成记录、逐例失败、四模型锁定和实际队列已保存，原筛选与运动质量复算一致，见 `learning/results/study-20261012/trace-F-four-policy-comparison/`。",
    "E的原筛选失败决定保留。这里只比较登记的两个种子和有限预算，不能证明强化学习无效或训练已收敛。正式三种子及每种子60例验收仍未执行；未将策略接入正式控制或连接实车。"] ) + "\n"
expected = working.rstrip(b"\r\n") + b"\n\n" + appendix.encode("utf-8")
require((repo / doc).read_bytes() == working, "写入前说明被其他任务修改")
(repo / doc).write_bytes(expected)
blob = git("hash-object", "-w", "--stdin", input=head.rstrip(b"\r\n") + b"\n\n" + appendix.encode("utf-8")).decode().strip()
git("update-index", "--cacheinfo", f"100644,{blob},{doc}")
paths = [p.relative_to(repo).as_posix() for p in target.rglob("*") if p.is_file()] + [progress_path.relative_to(repo).as_posix()]
git("add", "--pathspec-from-file=-", "--pathspec-file-nul", input=("\0".join(paths) + "\0").encode("utf-8"))
actual = set(git("diff", "--cached", "--name-only", "-z").decode("utf-8").strip("\0").split("\0"))
require(actual == {doc, *paths}, "索引混入其他工作")
require(all(git("show", ":" + p) == (repo / p).read_bytes() for p in paths), "暂存文件不同")
git("-c", "core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof", "diff", "--cached", "--check")
print(git("-c", "core.quotepath=false", "diff", "--cached", "--stat").decode("utf-8"))
