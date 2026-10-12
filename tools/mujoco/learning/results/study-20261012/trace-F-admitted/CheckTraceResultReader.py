"""用已完成的真实E评价检查新结果读取器；不读取F候选表现。"""
from datetime import datetime
import json
from pathlib import Path
import shutil
import CompareTraceResults as reader

workspace=Path(__file__).resolve().parents[1]
base=workspace/"work/study-20261012/aligned-reference"
frozen=base/"training-frozen"
manifest=reader.verify_snapshot(frozen)
cases=frozen/"tools/mujoco/learning/cases/aligned-20261012/monitor.json"
baseline=base/"baselines/monitor/summary.json"
actual=[]
for seed in (41,73):
    directory=base/f"screening/{seed}/monitor"
    report,completed=reader.verify_evaluation(directory,cases,baseline,base/f"screening/{seed}/train",manifest)
    saved=reader.read(Path(r"D:\ARBATOS")/f"tools/mujoco/learning/results/study-20261012/aligned-E-{seed}-monitor/单种子监控核验.json")
    rule=reader.read(frozen/"tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json")
    reader.require(reader.gate(report["acceptance"],rule["screening_first"])==saved["single_seed_gate"] and len(completed)==48,"真实E结果与已保存结论不同")
    actual.append(dict(seed=seed,completed_records=len(completed),original_gate_exact=True,summary_sha256=reader.sha(directory/"summary.json")))

def rejected(name,call,expected):
    try:
        call()
    except RuntimeError as error:
        reader.require(expected in str(error),"意外失败位置："+str(error))
        return dict(test=name,rejected=True,reason=str(error))
    raise RuntimeError("应拒绝而未拒绝："+name)

tests=[]
tests.append(rejected("错误策略来源",lambda:reader.verify_evaluation(base/"screening/41/monitor",cases,baseline,base/"screening/73/train",manifest),"评价策略不是登记模型"))
tests.append(rejected("错误测试集",lambda:reader.verify_evaluation(base/"screening/41/monitor",reader.learning/"cases/trace-20261012/monitor.json",baseline,base/"screening/41/train",manifest),"评价来源改变"))
fixture=workspace/"work/trace-result-reader-fixture"
fixture.mkdir(exist_ok=False)
source=base/"screening/41/monitor"
for name in ("summary.json","run.json"):
    shutil.copyfile(source/name,fixture/name)
shutil.copytree(source/"completed",fixture/"completed")
report=reader.read(fixture/"summary.json")
report["learned_results"][0]["metrics"]["passed"]=not report["learned_results"][0]["metrics"]["passed"]
(fixture/"summary.json").write_text(json.dumps(report,ensure_ascii=False),encoding="utf-8")
tests.append(rejected("汇总与真实完成记录不一致",lambda:reader.verify_evaluation(fixture,cases,baseline,base/"screening/41/train",manifest),"实际完成记录与汇总不一致"))
tests.append(rejected("评价尚未开始时禁止形成对照结论",reader.compare,"唯一新监控队列尚未开始"))
reader.require(not (reader.base/"四策略完整对照结论.json").exists(),"提前形成新监控结论")
record=dict(schema=1,checked_at=datetime.now().astimezone().isoformat(),actual_old_E_evaluations=actual,negative_checks=tests,
    new_F_candidate_results_read=False,all_four_comparison_positive_path_executed=False,
    reader_sha256=reader.sha(Path(reader.__file__)),checker_sha256=reader.sha(Path(__file__)),
    boundary="真实旧E的96条评价记录用于检查公共读取器及拒绝路径；四个新24例结果的完整正向检查仍待原训练和评价完成。")
target=reader.base/"对照结果读取检查.json"
reader.require(not target.exists(),"已有检查记录")
target.write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(json.dumps(record,ensure_ascii=False,indent=2))
