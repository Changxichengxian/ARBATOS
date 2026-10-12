"""六类方案均未通过筛选后，生成带来源的负结果总表和下一步说明。"""
from datetime import datetime
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
results = repo / "tools/mujoco/learning/results/study-20261012"
study = workspace / "work/study-20261012"
read = lambda p: json.loads(p.read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


abcd_path = results / "四路线完整验证/四路线完整对照核验.json"
e_path = results / "aligned-E-both-completed/双种子筛选结论.json"
f_path = results / "trace-F-four-policy-comparison/四策略完整对照结论.json"
require(f_path.is_file(), "F完整对照尚未保存，不能形成六方案负结果")
abcd, e, f = (read(p) for p in (abcd_path, e_path, f_path))
require(not e["both_seed_gate_pass"] and not f["F_both_seed_screen_pass"]
    and all(not r["screening_pass"] for r in abcd["rows"]), "至少一个方案需继续，不能按全部负结果收尾")
require(abcd["completed_record_rows_checked"] == 720 and e["completed_evaluation_records_checked"] == 96
    and f["completed_records_checked"] == 192 and f["all_four_locked_before_evaluation"], "核验范围不完整")
require(not e["final_dataset_used"] and not f["final_dataset_used"], "最终集边界改变")
sources = {str(p): sha(p) for p in (abcd_path, e_path, f_path)}
for mapping in [r["source"] for r in abcd["rows"]] + [r["sources"] for r in e["rows"]] + [f["source_files"]]:
    require(all(sha(Path(path)) == value for path, value in mapping.items()), "原结果来源改变")
    sources.update(mapping)
rows = []
for original in abcd["rows"]:
    row = dict(original)
    row.update(evaluation_set="A至D共同45例", cases_sha256=abcd["validation_cases_sha256"],
        evaluation_kind="original_screening", final_test=False)
    rows.append(row)


def add_monitor(route, seed, summary_path, screening_gate, name, kind):
    report = read(summary_path)
    a = report["acceptance"]
    c = a["checks"]
    sources[str(summary_path)] = sha(summary_path)
    failures = [r for r in a["cases"] if not r["pass"]]
    quality_failures = [r for r in c["motion_quality"]["cases"] if not r["pass"]]
    require(len(report["baseline_results"]) == len(report["learned_results"]) == 24, "监控报告不完整")
    rows.append(dict(route=route, label="对齐评分" if route == "E" else "长程反馈", seed=seed,
        evaluation_set=name, cases_sha256=report["scenarios_sha256"], evaluation_kind=kind,
        validation_pairs=24, stop_improvement=screening_gate.get("stop_improvement"),
        push_improvement=screening_gate.get("push_improvement"), hip_heat_change=screening_gate.get("heat_change"),
        mean_case_counts=dict(stop=8, push=8, heat=24), physical_complete=c["physical_complete"]["pass"],
        original_guard_failing_cases=len(failures), failed_case_ids=[r["id"] for r in failures],
        original_guard_counts=dict(Counter(key for r in failures for key, check in r["checks"].items() if not check["pass"])),
        extra_motion_quality="passed" if c["motion_quality"]["pass"] else "failed",
        motion_quality_failing_cases=len(quality_failures), failed_motion_case_ids=[r["id"] for r in quality_failures],
        screening_pass=screening_gate["pass_all"], budget=dict(kind="PPO_steps", count=262144),
        source={str(summary_path): sha(summary_path)}, final_test=False))


for original in e["rows"]:
    add_monitor("E", original["seed"], results / f"aligned-E-{original['seed']}-monitor/summary.json",
        original["gate"], "E原24例", "original_screening")
for original in f["rows"]:
    label = f"{original['route']}-{original['seed']}"
    add_monitor(original["route"], original["seed"], results / "trace-F-four-policy-comparison/evaluation" / label / "summary.json",
        original["gate"], "E/F统一新24例", "registered_comparison")
require(len(rows) == 14 and sum(r["validation_pairs"] for r in rows) == 504, "比较数量不符")
final_path = study / "frozen/tools/mujoco/learning/cases/study-20261012/final.json"
final = read(final_path)
require(final["split"] == "final" and final["case_count"] == 60
    and final["cases_sha256"] == "4bab9e7570e467f921e6cd280fe5c1e63d8b04d94a47970445357a4c062307bb", "封存最终集改变")
sources[str(final_path)] = sha(final_path)
training_proofs = {}
locked_models = read(results / "trace-F-four-policy-comparison/四策略锁定.json")["models"]
for seed in (41, 73):
    path = results / f"trace-F-{seed}-trained/训练完成核验.json"
    proof = read(path)
    locked_model = next(r for r in locked_models if r["route"] == "F" and r["seed"] == seed)
    require(proof["actual_model_steps"] == 262144 and locked_model["completion"]["natural_budget_completion"]
        and all(proof["files"][name] == value == sha(path.parent / name) for name, value in locked_model["files"].items()), "F训练完成边界不符")
    training_proofs[str(seed)] = dict(path=str(path), sha256=sha(path), source=proof["source"],
        completed_training_episodes=proof["completed_training_episodes"], completion=locked_model["completion"],
        completion_source="四策略锁定时对实际最终模型的末批检查，保留较早单种子证明原样")
    sources[str(path)] = sha(path)
restart_path = results / "trace-F-73-resume/用户恢复与续训记录.json"
restart = read(restart_path)
require(restart["action"] == "restart_seed_73_from_zero" and restart["old_saved_steps"] == 42496, "暂停后重跑来源不符")
sources[str(restart_path)] = sha(restart_path)
target = results / "六方案筛选汇总"
require(not target.exists(), "总表已经保存，拒绝覆盖")
now = datetime.now().astimezone().isoformat()
record = dict(schema=1, created_at=now, conclusion="六类有限配置的筛选阶段负结果",
    distinct_routes=6, final_policies=12, completed_pair_groups=14, candidate_evaluations=504,
    completed_control_records=1008, rows=rows, final_dataset_used=False,
    final_cases_sha256=final["cases_sha256"], formal_training_seeds_not_run=[101, 211, 307],
    F_training_proofs=training_proofs, F73_extra_interrupted_steps=42496,
    source_files=sources, source_script_sha256=sha(Path(__file__)),
    negative_branch_supported=True, delivery_verification="单独记录于最终交付的已执行核验/汇总.json及实际执行环境.json，本统计表不替代运行核验",
    boundary="12份策略共14组筛选/比较结果，三种评价场景集分别统计。没有执行最终验收，不能称为最终验收失败。")
target.mkdir()
(target / "六方案筛选汇总.json").write_bytes((json.dumps(record, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))

lines = ["轮腿强化学习研究：结论与下一步", f"结果保存时间：{now}", "",
    "本轮没有得到可采用的新控制策略。六类方案已完成登记的筛选或对照；这是有完整证据的筛选阶段负结果。",
    "研究目标允许在至少三种有区别的方案未达标后交付负结果。此次已有六类，因此可在完成最后复现交付后结束，无需凑满两天。", "",
    "研究顺序：先接通小幅控制修正与物理检查，再冻结机械模型、控制版本、场景和采用标准；依次比较六维动作、对称动作、线性反馈搜索、常数参考参数搜索、对齐评分的学习策略，以及长程反馈权重变体。",
    "正式筛选前还完成过32768步的首轮试跑和12组配对评价，用于检验流程；收益不足，结果独立保留，不计入下表六路线统计。",
    "原正式标准保留：101、211、307三个独立训练种子各跑60个封存场景，停车及受推均值各改善至少10%，逐例误差、姿态、运动质量、髋热和物理边界全部满足。",
    "没有路线通过筛选，所以未训练三个正式种子、未使用最终60例，也未进入起身或落地学习。", "",
    "以下百分比均相对同场景原控制。停车/受推正数为误差降低，热指标正数为负担增加。",
    "热指标为四髋力矩平方的积分，不是实测温度、电量或电池功耗。",
    "A至D用同45例；E原24例与E/F新24例分开。预算不同，不是等算力排名。", ""]
for dataset in ("A至D共同45例", "E原24例", "E/F统一新24例"):
    lines += [dataset, "方案/种子 | 预算 | 停车改善 | 受推改善 | 热增加 | 原逐例失败 | 运动质量 | 本次门槛"]
    for row in (r for r in rows if r["evaluation_set"] == dataset):
        def pct(x):
            return "不可比较" if x is None else f"{100*x:+.3f}%"
        quality = {"passed": "通过", "failed": "失败", "not_evaluated": "未评价"}[row["extra_motion_quality"]]
        budget = f"{row['budget']['count']}" + ("步" if row["budget"]["kind"] == "PPO_steps" else "候选")
        lines.append(f"{row['route']}/{row['seed']} | {budget} | {pct(row['stop_improvement'])} | {pct(row['push_improvement'])} | {pct(row['hip_heat_change'])} | {row['original_guard_failing_cases']}/{row['validation_pairs']} | {quality} | {'通过' if row['screening_pass'] else '未通过'}")
    lines.append("")
lines += ["F所有逐例保护失败项（未省略失败项）："]
for row in (r for r in f["rows"] if r["route"] == "F"):
    for case in row["failed_original_cases"]:
        for name, check in case["checks"].items():
            if not check["pass"]:
                label = {"peak_pitch_deg": "前后倾角峰值，度", "hip_heat_case": "四髋力矩平方积分"}.get(name, name)
                lines.append(f"F/{row['seed']}，{case['id']}，{label}：原控制{check['baseline']:.6g} → 新策略{check['learned']:.6g}；增加{check['delta']:.6g}，允许增加{check['allowed_increase']:.6g}。")
lines.append("")
physical_count = sum(r["validation_pairs"] for r in rows if r["physical_complete"])
lines += [f"共12份最终策略、14组比较、504次候选策略的场景评价，对应1008条原控制/候选完成记录。这些评价重复使用各自登记的场景集，不是504个独立场景。整组物理通过的候选评价合计{physical_count}次；个别物理失败若存在，详见完整原报告，不用剩余成功场景替代整体。",
    "A/B/C在更早的必要门槛已失败，没有追加运动质量评价；表中未评价不能解读为通过。新24例上的E只作为固定对照，不改变它在原24例已被淘汰的决定。", "",
    "已经确认的瓶颈：",
    "1. A/B的直接修正收益很小，部分种子增加髋部用力或姿态摆动；减少动作自由度没有使这两组预算达到筛选标准。",
    "2. C/D能改善部分均值，但组合场景和姿态保护仍有退步。只看停车、受推平均误差会漏掉这些代价。",
    "3. E在原监控集获得约4.6%至8.3%的单项均值收益，但仍有俯仰和组合场景退步，训练评分尚未可靠保证所有约束。",
    "4. F只改变长程反馈权重，保留原动作、评分、训练库和预算；新24例结果仍未让两个种子同时通过，不能据此把单一惩罚时序解释成唯一原因。", "",
    "目前无法证明：训练已经收敛、强化学习整体无效、更大预算必然无效，或某种方案具有普遍优势。每类只有两个筛选种子和有限场景，结果依赖当前接口和预算。",
    "当前机械对象是冻结的约16 kg RM WBR参考模型与ARBATOS控制核心，并非实际MINIWHEELEG-M整车模型。传感器和参数扰动只覆盖登记范围，没有实测校准或真实CAN链路、温升、地形和硬件验收。", "",
    "下一步建议按顺序做：",
    "1. 先确认实际轮腿尺寸、质量与重心、传动和电机参数，用真实可追溯数据校准模型；本轮不连接或控制实车。",
    "2. 分析停车参考速度和受推反馈各自造成的回摆、俯仰及组合退步，先构造可解释的控制改动和约束，再开展新研究。",
    "3. 若继续学习，另行登记约束处理、训练预算和独立测试集，重点检验逐例退步，而不是重复当前失败配置或只延长训练。",
    "4. 主电脑修好后再实测同一工作负载速度。当前流程主要消耗CPU，不能按显卡型号直接承诺训练提速倍数。", "",
    "恢复记录：F73曾按暂停请求在42496步保存并退出；继续后保留旧模型，从原seed73重新完成262144步，未按性能换种子或挑选中途策略。旧42496步只计额外计算，不计入最终模型训练步数。",
    "交付保存全部最终策略、配置、冻结源码/模型/动态库、完整评价报告和关键完成记录。原A–E十份策略已实际重放60条轨迹，全新Python环境代表策略18条也一致；最终含F的新包是否完成独立目录实跑，以包内已执行核验/汇总.json和实际执行环境.json为准，结论总表本身不替代该验证。",
    "最终60例SHA-256：" + final["cases_sha256"], ""]
text = "\n".join(lines)
(target / "研究结论与下一步.txt").write_bytes(text.encode("utf-8"))
(workspace / "outputs/轮腿研究完整结论.txt").write_bytes(text.encode("utf-8"))
shutil.copyfile(Path(__file__), target / Path(__file__).name)
print(json.dumps(dict(directory=str(target), routes=6, policies=12, comparisons=14,
    completed_control_records=1008, final_dataset_used=False), ensure_ascii=False))
