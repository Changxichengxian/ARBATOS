"""将已完成候选的冻结环境、最终模型、完整配对报告和复现入口组成独立目录。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

workspace=Path(__file__).resolve().parents[1]
study=workspace/"work/study-20261012"
results=Path(r"D:\ARBATOS\tools\mujoco\learning\results\study-20261012")
sys.path.insert(0,str(study/"long-trace/frozen/tools/mujoco/learning"))
from EvaluateWbrStudy import policy_artifacts,load_cases
from WbrLearningEnv import verify_snapshot

read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda data:hashlib.sha256(data).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output",type=Path,required=True)
parser.add_argument("--include-trace",action="store_true")
args=parser.parse_args()
target=args.output.resolve()
require(target.is_relative_to(workspace/"work") and not target.exists(),"需使用工作目录内新的打包目录")
snapshots=dict(ABC=study/"frozen",D=study/"reference-search/frozen",E=study/"aligned-reference/training-frozen")
specs=[]
for seed in (41,73):
    for route,name in (("A",f"screening-A-full-{seed}-evaluated"),("B",f"screening-B-symmetric-{seed}")):
        specs.append(dict(label=f"{route}-{seed}",profile="ABC",policy=results/name,summary=results/name/"summary.json",run=results/name/"evaluation-run.json",cases="tools/mujoco/learning/cases/study-20261012/validation.json"))
    for route,name,profile in (("C",f"linear-C-{seed}","ABC"),("D",f"reference-D-{seed}","D")):
        specs.append(dict(label=f"{route}-{seed}",profile=profile,policy=results/(name+"-search-completed"),summary=results/(name+"-validation")/"summary.json",run=results/(name+"-validation")/"run.json",cases="tools/mujoco/learning/cases/study-20261012/validation.json"))
    specs.append(dict(label=f"E-{seed}",profile="E",policy=results/f"aligned-E-{seed}-trained",summary=results/f"aligned-E-{seed}-monitor/summary.json",run=results/f"aligned-E-{seed}-monitor/run.json",cases="tools/mujoco/learning/cases/aligned-20261012/monitor.json"))
if args.include_trace:
    comparison=read(study/"long-trace/四策略完整对照结论.json")
    require(comparison["completed_records_checked"]==192 and comparison["all_four_locked_before_evaluation"],"新24例对照没有完整核验")
    require(all(sha(Path(name).read_bytes())==value for name,value in comparison["source_files"].items()),"新监控对照来源改变")
    snapshots["F"]=study/"long-trace/frozen"
    for seed in (41,73):
        specs.append(dict(label=f"F-{seed}",profile="F",policy=results/f"trace-F-{seed}-trained",summary=study/f"long-trace/evaluation/F-{seed}/summary.json",run=study/f"long-trace/evaluation/F-{seed}/run.json",cases="tools/mujoco/learning/cases/trace-20261012/monitor.json"))
inventory={}
policies=[]
manifests={label:verify_snapshot(snapshot) for label,snapshot in snapshots.items()}
for spec in specs:
    binding=read(spec["run"])["binding"]
    require(binding["policy_artifacts"]==policy_artifacts(spec["policy"]),"保存的最终模型与实际评价不符")
    summary=read(spec["summary"])
    cases=load_cases(snapshots[spec["profile"]]/spec["cases"])
    require(summary["scenarios_sha256"]==cases["cases_sha256"] and len(summary["learned_results"])==cases["case_count"],"配对报告不完整或场景不同")
target.mkdir()

def put(relative,data):
    path=(target/relative).resolve()
    require(path.is_relative_to(target) and relative not in inventory,"包内路径越界或重复")
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(data)
    require(sha(path.read_bytes())==sha(data),"打包写入字节不同")
    inventory[relative]=sha(data)

for label,snapshot in snapshots.items():
    manifest=manifests[label]
    put(f"snapshots/{label}/manifest.json",(snapshot/"manifest.json").read_bytes())
    for relative,expected in {**manifest["files"],manifest["library"]:manifest["library_sha256"]}.items():
        data=(snapshot/relative).read_bytes()
        require(sha(data)==expected,"冻结源文件改变")
        put(f"snapshots/{label}/{relative}",data)
for spec in specs:
    label=spec["label"]
    for name in ("policy.json","policy.npz","checkpoint.zip","search-state.json","run.json","status.json","aligned-episodes.jsonl","progress.jsonl"):
        source=spec["policy"]/name
        if source.is_file():
            put(f"models/{label}/{name}",source.read_bytes())
    put(f"reports/{label}/summary.json",spec["summary"].read_bytes())
    put(f"reports/{label}/evaluation-run.json",spec["run"].read_bytes())
    cases=load_cases(snapshots[spec["profile"]]/spec["cases"])
    selected=[]
    for kind in ("stop","push","combined"):
        candidates=sorted([row for row in cases["cases"] if row["options"]["case"]==kind],key=lambda row:row["id"])
        randomized=[row for row in candidates if row["randomize"]]
        selected.append((randomized or candidates)[0]["id"])
    policies.append(dict(label=label,snapshot=f"snapshots/{spec['profile']}",policy=f"models/{label}",
        summary=f"reports/{label}/summary.json",cases=f"snapshots/{spec['profile']}/{spec['cases']}",
        replay_case_ids=selected,source_policy=str(spec["policy"]),source_summary=str(spec["summary"])))
put("检查并复现.py",(workspace/"work/ReproduceStudyPackage.py").read_bytes())
versions=read(results/"aligned-E-73-trained/policy.json")["versions"]
put("requirements.txt",("\n".join(f"{name}=={version}" for name,version in versions.items())+"\n").encode("utf-8"))
description="""轮腿离线研究复现包

这里保存已完成方案的最终模型、优化器或搜索状态、各自冻结的完整源码/模型/Windows动态库，以及原配对评价报告。
历史报告中的绝对路径用于记录来源，不应直接运行其中的旧队列或依赖原机器目录。

在较短的新目录解压，使用装有requirements.txt所列版本的Python执行：
python 检查并复现.py

入口先检查包内全部文件，然后为每份已完成策略重跑停车、受推、组合各一个场景及原控制。
选例只按场景输入排序：各类首个随机场景；不存在随机场景时才用首个场景，不依据策略结果挑选。
每个场景逐字段比较原报告与真实重放，并记录实际网络载入、13组优化器状态和128组输入的导出推理误差。
结果写入复现输出。已有结果不会覆盖；再次运行时另选新解压目录。
也可用 --one A-41 只核验一份策略；完整汇总只由未运行过的完整入口产生。

此复现不会训练、选取新检查点、打开最终60例评价或连接实车，也不授权扩大训练。
三例重放只是迁移目录的运行验证；完整45例或24例的性能结论以包内原报告为准。
同机逐字段一致不能保证另一CPU/软件版本逐位一致；跨电脑重放仍须实际核验并保留差异。
动态库属于Windows，不宣称跨系统运行。若重新编译或修改源文件，原文件清单将拒绝通过，不应把修改后运行称为原实验。
PPO网络的checkpoint.zip包含优化器；C/D的search-state.json包含搜索与随机状态。当前固定预算已经完成，不建议继续这些失败配置。
本包阶段是否含F由交付清单include_trace决定；未包含时，不代表F已经完成或已得到性能结论。
"""
put("使用说明.txt",description.encode("utf-8"))
record=dict(schema=1,created_at=datetime.now().astimezone().isoformat(),include_trace=args.include_trace,
    policies=policies,files=inventory,builder_sha256=sha(Path(__file__).read_bytes()),
    boundary="只证明这些最终文件完整保存；另运行包内入口，才能确认换目录实际重放是否一致。未声明整个研究目标完成。")
(target/"交付清单.json").write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(json.dumps(dict(directory=str(target),policies=len(policies),files=len(inventory),include_trace=args.include_trace),ensure_ascii=False))
