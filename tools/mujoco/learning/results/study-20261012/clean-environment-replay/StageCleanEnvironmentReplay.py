"""保存全新依赖环境的实际重放证据，并检查补全依赖后的交付目录。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

workspace=Path(__file__).resolve().parents[1]
repo=Path(r"D:\ARBATOS")
base=workspace/"work/study-20261012/long-trace"
target=repo/"tools/mujoco/learning/results/study-20261012/clean-environment-replay"
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def write(path,value):
    path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

def git(*args,input=None):
    return subprocess.run(["git","-C",str(repo),*args],input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout

require(git("branch","--show-current").strip()==b"main" and not git("diff","--cached","--name-only").strip(),"分支或索引需要核查")
proof=read(base/"独立依赖环境复现.json")
require(proof["status"]=="passed" and proof["trajectories"]==18 and proof["all_fields_exact"],"新环境重放未完成")
require(proof["requirements_sha256"]==sha(base/"完整环境依赖.txt") and len(proof["packages"])==35,"依赖来源不符")
require(proof["source_sha256"]==sha(workspace/"work/CheckCleanStudyPackage.py"),"实际重放入口改变")
require(proof["archive_sha256"]==sha(workspace/"outputs/轮腿已完成策略复现包.zip"),"旧交付包改变")
require(proof["selected_policies"]==["A-41","D-41","E-41"],"不是三种环境各首份策略")
for label,source in proof["results"].items():
    require(sha(Path(source["path"]))==source["sha256"],"实际重放记录改变")
    row=read(Path(source["path"]))
    require(row["status"]=="passed" and row["trajectories"]==len(row["results"])==6
        and all(item["all_fields_exact"] and item["actual"]==item["expected"] for item in row["results"]),"实际逐字段重放未通过")
target.mkdir(exist_ok=False)
for name in ("独立依赖环境复现.json","完整环境依赖.txt"):
    shutil.copyfile(base/name,target/name)
for label,source in proof["results"].items():
    shutil.copyfile(source["path"],target/(label+".json"))
for name in ("CheckCleanStudyPackage.py","BuildStudyDeliveryPackage.py",Path(__file__).name):
    shutil.copyfile(workspace/"work"/name,target/name)
package=workspace/"work/package-full-dependency-check"
subprocess.run([sys.executable,str(workspace/"work/BuildStudyDeliveryPackage.py"),"--output",str(package)],check=True)
old=read(workspace/"work/study-package-clean-check/交付清单.json")
new=read(package/"交付清单.json")
require(new["policies"]==old["policies"] and not new["include_trace"],"候选或选例变化")
require(set(new["files"])-set(old["files"])=={"依赖环境实际核验.json"} and not(set(old["files"])-set(new["files"])),"包内文件意外增删")
changed=[name for name,value in old["files"].items() if new["files"][name]!=value]
require(set(changed)=={"requirements.txt","使用说明.txt"},"运行文件发生变化")
require(all(sha(package/name)==value for name,value in new["files"].items()),"新目录文件不符")
packing=dict(status="passed",new_package=str(package),new_inventory_sha256=sha(package/"交付清单.json"),
    old_inventory_sha256=proof["package_inventory_sha256"],unchanged_runtime_files=len(old["files"])-len(changed),
    changed_files=changed,added_files=["依赖环境实际核验.json"],packages=len(proof["packages"]),
    builder_sha256=sha(workspace/"work/BuildStudyDeliveryPackage.py"),simulation_repeated=False,
    boundary="新目录实际生成且清单核对通过。运行输入字节与刚完成18条重放的旧包相同；没有声称新目录又实跑了全部十份策略，旧发布压缩包保留原样。")
write(target/"打包器完整依赖核验.json",packing)
progress_path=repo/"tools/mujoco/learning/研究进度.json"
progress=read(progress_path)
now=datetime.now().astimezone().isoformat()
progress["updated_at"]=now
progress["completed_strategy_replay"]["clean_environment"]=dict(python="3.12.14",packages=35,policies=proof["selected_policies"],
    trajectories=18,all_fields_exact=True,proof=(target/"独立依赖环境复现.json").relative_to(repo).as_posix(),
    package_builder=(target/"BuildStudyDeliveryPackage.py").relative_to(repo).as_posix())
progress["completed"].append("全新独立Python3.12.14环境安装原完整35个依赖后，从旧交付包重新解压，三种冻结控制环境各首份策略共18条历史轨迹逐字段一致。补全依赖的新包目录实际生成，运行输入字节与已验证旧包一致；未评价F控制性能。")
write(progress_path,progress)
doc="tools/仿真说明.md"
head=git("show","HEAD:"+doc)
require(git("show",":"+doc)==head,"说明有其他暂存修改")
working=(repo/doc).read_bytes()
marker="#### 独立依赖环境复现与完整版本清单（2026-10-12）"
require(marker.encode("utf-8") not in working,"本阶段说明已存在")
appendix=f"""{marker}

另建不包含系统site-packages的Python3.12.14环境，按原训练环境固定的35个依赖安装；torch仍为2.9.1+cpu，未改动原训练环境。安装后逐项核对所有包的版本及实际位置，再从已发布十策略压缩包解压至新目录。每种旧冻结控制环境按名称选择首份策略A41/D41/E41，按原登记选择停车、受推、组合各首个随机场景及原控制，共18条历史轨迹全部逐字段一致。

这证明同机新依赖环境与新目录可重放上述三份策略，不代表另一电脑，也不替代原45/24例验证。原交付包的5个顶层版本已扩成完整35包清单用于后续打包，并写明CPU版安装命令。补全依赖的新包目录已实际生成：{packing['unchanged_runtime_files']}个共用运行文件字节完全相同，更新依赖清单和使用说明并增加环境核验证据；没有重复宣称全部十份模型在新环境都实跑。

完整版本清单、三个逐条重放记录、环境检查及打包器见 `learning/results/study-20261012/clean-environment-replay/`。本阶段没有评价F策略或使用最终60例。
"""
expected=working.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")
(repo/doc).write_bytes(expected)
blob=git("hash-object","-w","--stdin",input=head.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")).decode().strip()
git("update-index","--cacheinfo",f"100644,{blob},{doc}")
paths=[p.relative_to(repo).as_posix() for p in target.iterdir() if p.is_file()]+[progress_path.relative_to(repo).as_posix()]
git("add","--",*paths)
actual=set(git("diff","--cached","--name-only","-z").decode("utf-8").strip("\0").split("\0"))
require(actual=={doc,*paths},"混入其他暂存修改")
require(all(git("show",":"+p)==(repo/p).read_bytes() for p in paths),"暂存文件不一致")
require((repo/doc).read_bytes()==expected,"说明工作区改变")
git("-c","core.whitespace=cr-at-eol","diff","--cached","--check")
print(json.dumps(dict(target=str(target),staged_files=len(actual),trajectories=18,new_package_files=len(new["files"])),ensure_ascii=False))
