"""保存F首个完整16384步检查点、实际预检模型和全部冻结源文件。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

workspace=Path(__file__).resolve().parents[1]
base=workspace/"work/study-20261012/long-trace"
frozen=base/"frozen"
repo=Path(r"D:\ARBATOS")
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda data:hashlib.sha256(data).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

admission=read(base/"长训练准入.json")
require(admission["eligible"] and admission["all_baseline_physical_pass"],"未通过实际预检")
require(all(sha(Path(p).read_bytes())==digest for p,digest in admission["source_files"].items()),"预检来源变化")
require(all(sha(Path(p).read_bytes())==digest for p,digest in admission["source_model_files"].items()),"实际短训模型变化")
queue=read(base/"training/queue-state.json")
job=queue["jobs"]["F-trace-41-train"]
require(job["status"] in ("running","completed"),"首种子没有启动")
train=Path(job["actual_output"])
require(train.resolve()==(base/"training/41/train").resolve(),"训练目录改变")
checkpoint=train/"checkpoints/step-000016384"
contract=read(checkpoint/"policy.json")
require(contract["seed"]==41 and contract["envs"]==2 and contract["training_steps"]==16384
    and contract["requested_total_steps"]==262144,"不是登记的首个保存点")
require(contract["training_protocol"]["algorithm"]["gae_lambda"]==1.
    and contract["training_protocol"]["registration_sha256"]==admission["registration_sha256"],"保存点协议不符")
manifest=read(frozen/"manifest.json")
package=base/"recovery-package-first"
package.mkdir(exist_ok=False)
inventory={}

def put(relative,data):
    path=package/relative
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(data)
    inventory[relative]=sha(data)

put("snapshot/manifest.json",(frozen/"manifest.json").read_bytes())
for relative,expected in {**manifest["files"],manifest["library"]:manifest["library_sha256"]}.items():
    data=(frozen/relative).read_bytes()
    require(sha(data)==expected,"冻结源改变："+relative)
    put("snapshot/"+relative,data)
for name,key in (("checkpoint.zip","checkpoint_sha256"),("policy.npz","policy_sha256"),("policy.json",None)):
    data=(checkpoint/name).read_bytes()
    require(key is None or sha(data)==contract[key],"检查点未完整保存")
    put("recovery/F-trace-41/"+name,data)
for name in ("first","resumed"):
    for path in (base/"smoke"/name).iterdir():
        if path.is_file() and path.suffix in (".json",".jsonl",".npz",".zip"):
            put("smoke/"+name+"/"+path.name,path.read_bytes())
for path,name in ((base/"长训练准入.json","长训练准入.json"),(base/"实际续训核验.json","实际续训核验.json"),
    (base/"参数及权限核验.json","参数及权限核验.json"),(base/"准备核验.json","准备核验.json"),
    (base/"训练入口差异.patch","训练入口差异.patch"),(base/"baseline/summary.json","monitor-baseline-summary.json"),
    (base/"baseline/run.json","monitor-baseline-run.json"),(train/"run.json","F41-run.json"),
    (base/"training/plan.json","原训练计划.json")):
    put("evidence/"+name,path.read_bytes())
put("检查恢复包.py",(workspace/"work/ProbeTraceRecovery.py").read_bytes())
text="""轮腿反馈时间权重单变量对照恢复包

snapshot是F的完整冻结环境，recovery/F-trace-41为种子41的16384步实际网络及优化器。
smoke保存1024至2048步真实续训模型；evidence保存24例新监控原控制及预检来源。
本包尚无F独立控制性能结论；新监控候选评价须待E/F两个种子共四个最终策略全部锁定。
原登记仅允许41、73各262144步；不允许扩展预算、旧45评价或最终60例。

迁移前确认原机器任务已退出，勿把复制的进程号当成新任务。新目录续训会保留网络、优化器，重新初始化物理回合和场景洗牌，非逐步重放。
安装policy.json的versions列明依赖后，用该Python运行本包的检查恢复包.py；先验证全部哈希、真实模型和一条原控制重放，再考虑续训。
在较短目录解压，避免Windows长路径问题。动态库是本机Windows产物，不宣称跨系统可用。

以本包目录为当前目录，以下为种子41的续训命令，输出必须是新的空目录：
python snapshot/tools/mujoco/learning/TrainWbrAligned.py --output recovery-run/F41 --bank snapshot/tools/mujoco/learning/results/study-20261012/aligned-training-baseline/training-bank.json --registration snapshot/tools/mujoco/learning/results/study-20261012/长程反馈单变量训练登记.json --seed 41 --envs 2 --steps 262144 --resume recovery/F-trace-41

不直接运行evidence中的原队列计划：其中绝对路径和状态属于原机器。训练仍受北京时间2026-10-14 02:10截止时间约束。
"""
put("恢复说明.txt",text.encode("utf-8"))
record=dict(schema=1,created_at=datetime.now().astimezone().isoformat(),steps=16384,seed=41,
    experiment_identity=contract["experiment_identity"],library_sha256=manifest["library_sha256"],
    source=str(checkpoint),manifest_sha256=sha((frozen/"manifest.json").read_bytes()),
    files=inventory,builder_sha256=sha(Path(__file__).read_bytes()),
    boundary="只证明完整保存，不代表迁移或性能已通过；另跑包内真实检查。")
(package/"恢复包清单.json").write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
archive=base/"F首段恢复包.zip"
require(not archive.exists(),"恢复包已存在")
with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED,compresslevel=9) as z:
    for path in sorted(package.rglob("*")):
        if path.is_file():
            z.write(path,path.relative_to(package).as_posix())
with zipfile.ZipFile(archive) as z:
    require(z.testzip() is None,"恢复包损坏")
    require(all(sha(z.read(name))==digest for name,digest in inventory.items()),"打包字节不符")
    extraction=workspace/"work/f-recovery-check"
    require(not extraction.exists(),"恢复核验目录已存在")
    extraction.mkdir()
    z.extractall(extraction)
print(json.dumps(dict(archive=str(archive),sha256=sha(archive.read_bytes()),size=archive.stat().st_size,
    files=len(inventory)+1,extraction=str(extraction)),ensure_ascii=False))
