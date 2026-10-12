"""冻结一次只改变优势估计lambda的有限对照；不修改E的任何文件。"""
from copy import deepcopy
from datetime import datetime
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import sys

workspace = Path(__file__).resolve().parents[1]
study = workspace / "work/study-20261012"
source = study / "aligned-reference/training-frozen"
base = study / "long-trace"
repo = Path(r"D:\ARBATOS")
results = repo / "tools/mujoco/learning/results/study-20261012"
prefix = "tools/mujoco/learning/"
sys.path.insert(0,str(source/prefix))
from WbrLearningEnv import verify_snapshot, experiment_identity
from BuildWbrAlignedCases import build
from BuildWbrAlignedBank import build as build_bank
from EvaluateWbrStudy import _completed_path

read = lambda p: json.loads(p.read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok, message):
    if not ok:
        raise RuntimeError(message)

def save(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

manifest = verify_snapshot(source)
require(sha(source/"manifest.json") == "1d04fb34dd8ac26f497746ad5f2871c501025504424c5fe8cb207b94305684c2", "E冻结版本改变")
e_rule_relative = prefix + "results/study-20261012/对齐评分参考策略训练登记.json"
rule_relative = prefix + "results/study-20261012/长程反馈单变量训练登记.json"
lock_relative = prefix + "results/study-20261012/trace-F-admitted/训练库许可.json"
monitor_relative = prefix + "cases/trace-20261012/monitor.json"
e_rule = read(source/e_rule_relative)
bank_relative = prefix + "results/study-20261012/aligned-training-baseline/training-bank.json"
bank = read(source/bank_relative)
original_baseline = study / "aligned-reference/baselines/training/summary.json"
rebuilt = build_bank(source/(prefix+"cases/aligned-20261012/training.json"),original_baseline)
require(rebuilt == bank, "原训练库无法完整复建")
summary,run = read(original_baseline),read(original_baseline.parent/"run.json")
for row in summary["baseline_results"]:
    item = read(_completed_path(original_baseline.parent,"baseline",row["id"]))
    require(item["row"] == row and item["binding_sha256"] == run["binding_sha256"], "原训练基线完成记录不一致")
monitor = build("trace-monitor",8,91000,95417,False)
used = set()
for name in ("development","validation","final","pressure"):
    used.update(row["seed"] for row in read(source/(prefix+f"cases/study-20261012/{name}.json"))["cases"])
for name in ("training","monitor"):
    used.update(row["seed"] for row in read(source/(prefix+f"cases/aligned-20261012/{name}.json"))["cases"])
require(not used & {row["seed"] for row in monitor["cases"]}, "新监控种子重叠")
rule = dict(schema=1,route="F-long-trace",registered_at=datetime.now().astimezone().isoformat(),
    deadline=e_rule["deadline"],screening_seeds=[41,73],formal_seeds=[],smoke_seed=9001,smoke_steps=[1024,2048],
    training_steps=[262144],envs=2,checkpoint_every=16384,training_cases_sha256=e_rule["training_cases_sha256"],
    training_bank_lock=lock_relative,training_case_count=144,monitor_cases_sha256=monitor["cases_sha256"],
    monitor_case_count=24,monitor_generation=dict(per_kind=8,seed_base=91000,generator_seed=95417,all_randomized=False),
    reward_source_sha256=e_rule["reward_source_sha256"],weights=e_rule["weights"],algorithm=deepcopy(e_rule["algorithm"]),
    initialization=e_rule["initialization"],screening_first=deepcopy(e_rule["screening_first"]),
    parent_registration_sha256=sha(source/e_rule_relative),parent_snapshot_sha256=sha(source/"manifest.json"),
    hypothesis="只检验本预算下GAE lambda从0.97改为1能否改善整体筛选；此参数影响所有奖励项的优势估计和方差，不只作用于末尾姿态罚分。",
    diagnostics={name:sha(study/"aligned-reference"/name) for name in
        ("credit-timing-diagnostic.json","pitch-timing-diagnostic/时序核验.json")},
    comparison=dict(routes=["E","F"],seeds=[41,73],steps_each=262144,
        all_four_final_weights_locked_before_any_policy_evaluation=True,
        baseline_may_run_before_training=True,only_registered_final_weights=True),
    stop_rules=["先完成真实1024至2048步续训验证；新监控原控制必须全部物理通过，否则停止本登记，不能替换场景。",
        "两个F种子均按固定预算执行，全部训练完后锁定E/F四个最终策略才统一评价，不读中途监控、不择优检查点。",
        "F任一种子物理、原逐例、运动质量或平均热量保护失败，或双种子不能同时达到两项至少1%收益，本路线停止。",
        "这次登记仅允许双种子262144步；不允许扩大训练、正式种子、旧45验证或最终60例。若通过，后续阶段必须另行事前登记并保持正式目标不变。",
        "新24例只使用一次；读完不能继续改lambda或换监控场景。E在此仅作固定预算方法对照，不因此恢复已失败的E晋级。"],
    boundaries=["144训练库与E逐字节相同；不使用独立评价场景训练或校准。",
        "除lambda及路线许可/路径外，训练入口算法执行部分、评分/模型/动作/93维观测/初始化/优化器/预算保持一致。",
        "新监控变化只在评价侧；所有四个候选与同一组原控制配对，不能与旧监控均值直接作因果比较。",
        "两种子和24例仅作有限筛选，无统计普遍性或实车改善承诺；不打开最终集。"])
rule["algorithm"]["gae_lambda"] = 1.
rule["screening_first"]["dataset"] = "trace-monitor"
differences = {key:(e_rule["algorithm"][key],value) for key,value in rule["algorithm"].items() if e_rule["algorithm"][key] != value}
require(differences == {"gae_lambda":(.97,1.)}, "不止一个算法参数改变")
require(not base.exists() and not (repo/rule_relative).exists(), "已有F登记或目录，拒绝覆盖")
base.mkdir()
frozen = base / "frozen"
frozen.mkdir()
for relative,expected in {**manifest["files"],manifest["library"]:manifest["library_sha256"]}.items():
    require(sha(source/relative) == expected, "原冻结内容改变")
    target=frozen/relative
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source/relative,target)
trainer_relative=prefix+"TrainWbrAligned.py"
original=(source/trainer_relative).read_text(encoding="utf-8")
trainer=original.replace('registration["route"] != "E-aligned-reference"','registration["route"] != "F-long-trace"')
trainer=trainer.replace('registered_name = "'+e_rule_relative+'"','registered_name = "'+rule_relative+'"')
start=trainer.index('    if not smoke and args.steps == max(registration["training_steps"]):')
end=trainer.index('    manifest = verify_snapshot()',start)
trainer=trainer[:start]+'    if args.promotion is not None:\n        raise ValueError("F有限对照不接受晋级或扩展训练")\n'+trainer[end:]
require(trainer != original and trainer[trainer.index('    bank = read(args.bank)'):] == original[original.index('    bank = read(args.bank)'):], "训练执行部分改变")
(frozen/trainer_relative).write_bytes(trainer.encode("utf-8"))
save(frozen/rule_relative,rule)
save(frozen/monitor_relative,monitor)
e_lock=read(source/e_rule["training_bank_lock"])
lock=deepcopy(e_lock)
lock.update(verified_at=datetime.now().astimezone().isoformat(),registration_sha256=sha(frozen/rule_relative),
    original_permission_sha256=sha(source/e_rule["training_bank_lock"]),
    note="F专用许可：同一训练库逐字节复用，已从原汇总复建并重新逐项核对144条完成记录；物理身份和评价器保持不变。")
save(frozen/lock_relative,lock)
new_manifest=deepcopy(manifest)
new_manifest.update(reference_snapshot=str(source),derived_for="F-long-trace",created_at=rule["registered_at"],
    parent_manifest_sha256=sha(source/"manifest.json"))
for relative in (trainer_relative,rule_relative,monitor_relative,lock_relative):
    new_manifest["files"][relative]=sha(frozen/relative)
save(frozen/"manifest.json",new_manifest)
verify_snapshot(frozen)
require(experiment_identity(new_manifest) == experiment_identity(manifest), "物理或策略接口身份改变")
require(sha(frozen/bank_relative) == sha(source/bank_relative), "训练库字节改变")
proof=dict(schema=1,created_at=rule["registered_at"],parameters_changed=differences,
    algorithm_execution_identical=True,original_144_baselines_reverified=True,training_bank_bytes_identical=True,
    physical_identity_unchanged=True,experiment_identity=experiment_identity(new_manifest),
    library_sha256=manifest["library_sha256"],parent_manifest_sha256=sha(source/"manifest.json"),
    manifest_sha256=sha(frozen/"manifest.json"),registration_sha256=sha(frozen/rule_relative),
    bank_sha256=sha(frozen/bank_relative),monitor_cases_sha256=monitor["cases_sha256"],
    builder_sha256=sha(Path(__file__)),changed_existing_snapshot_files=[trainer_relative],
    added_snapshot_files=[rule_relative,monitor_relative,lock_relative],long_training_started=False)
save(base/"准备核验.json",proof)
(base/"训练入口差异.patch").write_bytes("".join(difflib.unified_diff(original.splitlines(True),trainer.splitlines(True),fromfile="E/TrainWbrAligned.py",tofile="F/TrainWbrAligned.py")).encode("utf-8"))
smoke_root=base/"smoke"
common=["--bank",str(frozen/bank_relative),"--registration",str(frozen/rule_relative),"--seed","9001","--envs","1"]
jobs=[]
for name,steps in (("first",1024),("resumed",2048)):
    arguments=["--output",str(smoke_root/name),"--steps",str(steps),*common]
    if name == "resumed":
        arguments += ["--resume","{job:first}"]
    jobs.append(dict(id=name,kind="aligned_train",arguments=arguments))
save(smoke_root/"plan.json",dict(schema=1,stage="F_trace_smoke",deadline=rule["deadline"],
    python=str(workspace/"work/rl-env/Scripts/python.exe"),learning_directory=str(frozen/prefix),jobs=jobs))
admitted=results/"trace-F-admitted"
admitted.mkdir(exist_ok=False)
for source_path,destination in ((frozen/rule_relative,repo/rule_relative),(frozen/monitor_relative,repo/monitor_relative),
    (frozen/lock_relative,admitted/"训练库许可.json"),(frozen/"manifest.json",admitted/"manifest.json"),
    (base/"准备核验.json",admitted/"准备核验.json"),(base/"训练入口差异.patch",admitted/"训练入口差异.patch"),
    (frozen/trainer_relative,admitted/"TrainWbrAligned.py"),(Path(__file__),admitted/"PrepareTraceTrial.py")):
    require(not destination.exists(), "目标已存在")
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source_path,destination)
print(json.dumps(proof,ensure_ascii=False,indent=2))
