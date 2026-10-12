"""只从训练日志机械选例，重放最终策略并定位可见的姿态超限时刻。"""
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

workspace = Path(__file__).resolve().parents[1]
base = workspace / "work/study-20261012/aligned-reference"
frozen = base / "training-frozen"
learning = frozen / "tools/mujoco/learning"
sys.path.insert(0,str(learning))
from WbrLearningEnv import verify_snapshot, experiment_identity
from StudyWbrPolicy import StudyPolicy
from EvaluateWbrStudy import run_case, acceptance
from WbrAlignedReward import guard_cost

read = lambda p: json.loads(p.read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def save(path, value):
    path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

def require(ok, message):
    if not ok:
        raise RuntimeError(message)

manifest = verify_snapshot(frozen)
bank_path = learning / "results/study-20261012/aligned-training-baseline/training-bank.json"
bank = read(bank_path)
require(sha(bank_path) == "fe2eab74ac5d7d23bb11eb5a175fd93c2d26ffd77f25b9281cbb8005a143f3b2", "训练库改变")
train = base / "screening/41/train"
require(read(train/"status.json")["status"] == "completed", "训练尚未完成")
log = train / "aligned-episodes.jsonl"
episodes = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines() if line.strip()]
eligible = [row for row in episodes if row["case"] == "push" and row["violations"].get("control/peak_pitch_deg",0)>0]
require(eligible, "没有可选择的受推姿态退步训练记录")
selected = sorted(eligible,key=lambda row:(-row["violations"]["control/peak_pitch_deg"],row["id"],row["step"]))[0]
item = next(row for row in bank["rows"] if row["scenario"]["id"] == selected["id"])
scenario = item["scenario"]
require(scenario["id"].startswith("aligned-training-"), "不得使用独立评价数据选例")
rule_path = learning / "results/study-20261012/对齐评分参考策略训练登记.json"
rule = read(rule_path)
policy = StudyPolicy(train)
target = base / "pitch-timing-diagnostic"
target.mkdir(exist_ok=False)
sources = {str(p):sha(p) for p in (bank_path,rule_path,log,train/"policy.json",train/"policy.npz",
    frozen/"manifest.json", learning/"EvaluateWbrStudy.py",learning/"WbrLearningEnv.py",Path(__file__))}
registration = dict(created_at=datetime.now().astimezone().isoformat(),
    selection="在E41已完成训练日志的push回合中，选control/peak_pitch_deg归一化超限最大者；并列按场景编号、训练步排序。",
    selected_training_log_row=selected, scenario=scenario, source_files=sources,
    experiment_identity=experiment_identity(manifest), library_sha256=manifest["library_sha256"],
    boundary="只定位一个训练场景最终确定性策略的15ms姿态采样；不是重现训练随机动作，不是独立性能验证。")
save(target/"选例登记.json",registration)
print(json.dumps(dict(selected=scenario["id"],logged_violation=selected["violations"]["control/peak_pitch_deg"]),ensure_ascii=False),flush=True)
baseline = run_case(scenario,None,bank["env_config"],record=True)
reference = {key:value for key,value in item["baseline"].items() if key != "controller"}
require({key:value for key,value in baseline.items() if key != "trace"} == reference, "实际基线与封存训练库不一致")
save(target/"原控制重放.json",baseline)
candidate = run_case(scenario,policy,bank["env_config"],record=True)
save(target/"最终策略重放.json",candidate)
checks = acceptance([baseline],[candidate])["cases"][0]["checks"]
pitch = checks["peak_pitch_deg"]
threshold = pitch["baseline"]+pitch["allowed_increase"]
trace = candidate["trace"]
witnesses = [(index,row) for index,row in enumerate(trace) if abs(math.degrees(row["pitch_rad"])) > threshold]
peak_index = max(range(len(trace)),key=lambda index:abs(trace[index]["pitch_rad"]))
gamma,lam = rule["algorithm"]["gamma"],rule["algorithm"]["gae_lambda"]

def time_record(index):
    row = trace[index]
    lag = len(trace)-1-index
    return dict(index=index,time_s=row["time_s"],absolute_pitch_deg=abs(math.degrees(row["pitch_rad"])),
        steps_before_terminal_penalty=lag, seconds_before_terminal_penalty=lag*.015,
        isolated_direct_terminal_coefficient=(gamma*lam)**lag)

guard,violations = guard_cost(baseline,candidate)
result = dict(checked_at=datetime.now().astimezone().isoformat(),case_id=scenario["id"],
    baseline_replay_exact=True, policy_steps=len(trace),terminal_time_s=trace[-1]["time_s"],
    physical_pass=candidate["metrics"]["passed"], pitch_guard=pitch, sampled_threshold_deg=threshold,
    first_sampled_exceedance=time_record(witnesses[0][0]) if witnesses else None,
    maximum_sampled_pitch=time_record(peak_index), guard_cost=guard,violations=violations,
    first_witness_is_not_exact_1ms_time=True, independent_evaluation=False,
    evidence_sha256={p.name:sha(p) for p in target.iterdir() if p.is_file()},
    interpretation="1ms峰值仍裁决原保护；15ms轨迹只定位可见超限。指数项只是隔离的末尾误差直接贡献，未考虑价值函数和其他奖励，不能证明其导致训练失败。")
require(all(sha(Path(p)) == digest for p,digest in sources.items()), "诊断期间来源发生变化")
save(target/"时序核验.json",result)
print(json.dumps(result,ensure_ascii=False,indent=2))
