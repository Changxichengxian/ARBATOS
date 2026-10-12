"""新依赖环境和新解压目录实际复现三种已完成的冻结控制环境。"""
from datetime import datetime
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile

workspace=Path(__file__).resolve().parents[1]
base=workspace/"work/study-20261012/long-trace"
destination=workspace/"work/study-package-clean-check"
archive=workspace/"outputs/轮腿已完成策略复现包.zip"
requirements=base/"完整环境依赖.txt"
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
normalize=lambda s:re.sub(r"[-_.]+","-",s).lower()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

require(Path(sys.prefix).resolve()==(workspace/"work/package-clean-env").resolve(),"必须使用新建的独立环境")
require(sys.version_info[:2]==(3,12),"Python主次版本改变")
expected=dict(line.split("==") for line in requirements.read_text(encoding="utf-8").splitlines() if line.strip())
expected={normalize(k):v for k,v in expected.items()}
distributions=list(metadata.distributions())
actual={normalize(d.metadata["Name"]):d.version for d in distributions}
require(actual==expected and len(actual)==35,"独立环境和原完整依赖不一致")
require(all(Path(d.locate_file("")).resolve().is_relative_to(Path(sys.prefix).resolve()) for d in distributions),"使用了新环境以外的包")
require(sha(archive)=="b186e70f2a7ee3ad7ddf8fd5cf26bdf96f34914ea8b38447b8b313136d7b36af","已验证的旧复现包改变")
require(not destination.exists(),"新解压目录已存在")
with zipfile.ZipFile(archive) as package:
    require(package.testzip() is None,"复现包损坏")
    for entry in package.infolist():
        require((destination/entry.filename).resolve().is_relative_to(destination.resolve()),"压缩包路径越界")
        require((entry.external_attr>>16)&0o170000 != 0o120000,"不允许符号链接")
    package.extractall(destination)
require(not (destination/"复现输出").exists(),"复现包带有活动运行结果")
inventory=read(destination/"交付清单.json")
require(all(sha(destination/name)==value for name,value in inventory["files"].items()),"解压文件与原清单不符")
chosen=[]
for profile in ("ABC","D","E"):
    candidates=sorted(row["label"] for row in inventory["policies"] if row["snapshot"]=="snapshots/"+profile)
    require(bool(candidates),"缺少已完成冻结环境")
    chosen.append(candidates[0])
require(chosen==["A-41","D-41","E-41"],"未按每种环境首份策略选择")
started=datetime.now().astimezone().isoformat()
for label in chosen:
    subprocess.run([sys.executable,str(destination/"检查并复现.py"),"--one",label],cwd=destination,check=True)
rows={label:read(destination/"复现输出"/(label+".json")) for label in chosen}
require(all(row["status"]=="passed" and row["trajectories"]==6
    and row["package_inventory_sha256"]==sha(destination/"交付清单.json") for row in rows.values()),"新环境实际重放未全部通过")
require(all(sha(destination/name)==value for name,value in inventory["files"].items()),"运行修改了包内已封存文件")
record=dict(schema=1,status="passed",started_at=started,checked_at=datetime.now().astimezone().isoformat(),
    python_version=sys.version,python_executable=sys.executable,independent_prefix=sys.prefix,
    system_site_packages=False,packages=actual,requirements_sha256=sha(requirements),
    archive_sha256=sha(archive),package_inventory_sha256=sha(destination/"交付清单.json"),
    selected_policies=chosen,selection_rule="每种旧冻结控制环境按名称排序的第一份已完成策略；每种任务取原输入排序的首个随机场景。",
    trajectories=sum(row["trajectories"] for row in rows.values()),all_fields_exact=True,
    results={label:dict(path=str(destination/"复现输出"/(label+".json")),sha256=sha(destination/"复现输出"/(label+".json"))) for label in rows},
    source_sha256=sha(Path(__file__)),
    boundary="同机新建独立Python依赖环境、新目录解压的18条历史轨迹实跑；不是另一电脑或全体十份策略的第二次完整验收，不包含尚未完成的F，也未运行最终60例。")
(base/"独立依赖环境复现.json").write_text(json.dumps(record,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(record,ensure_ascii=False),flush=True)
