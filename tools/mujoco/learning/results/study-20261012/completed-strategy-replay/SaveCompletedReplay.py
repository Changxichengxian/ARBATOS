"""只有十份已完成策略真实重放通过后，才打包并暂存本阶段交付。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

workspace=Path(__file__).resolve().parents[1]
repo=Path(r"D:\ARBATOS")
learning=repo/"tools/mujoco/learning"
results=learning/"results/study-20261012"
package=workspace/"work/study-package-check"
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def git(*args,input=None):
    return subprocess.run(["git","-C",str(repo),*args],input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout

require(Path(git("rev-parse","--show-toplevel").decode().strip()).resolve()==repo,"仓库不符")
require(git("branch","--show-current").strip()==b"main" and not git("diff","--cached","--name-only").strip(),"分支或索引需要核查")
inventory=read(package/"交付清单.json")
proof=read(package/"复现输出/汇总.json")
require(proof["status"]=="passed" and proof["all_fields_exact"] and proof["trajectories"]==60,"十份策略重放未全部通过")
require(proof["inventory_sha256"]==sha(package/"交付清单.json") and not inventory["include_trace"],"实际核验包来源不符")
require({row["label"] for row in inventory["policies"]}=={f"{route}-{seed}" for route in "ABCDE" for seed in (41,73)},"核验策略集合不符")
require(all(sha(package/name)==value for name,value in inventory["files"].items()),"核验后包内文件改变")
require(inventory["builder_sha256"]==sha(workspace/"work/BuildStudyDeliveryPackage.py")
    and sha(package/"检查并复现.py")==sha(workspace/"work/ReproduceStudyPackage.py"),"打包或复现程序改变")
for name,value in proof["result_files"].items():
    p=package/"复现输出"/name
    row=read(p)
    require(sha(p)==value and row["package_inventory_sha256"]==sha(package/"交付清单.json")
        and row["status"]=="passed" and row["trajectories"]==6
        and all(item["actual"]==item["expected"] and item["all_fields_exact"] for item in row["results"]),"重放记录不能复核")
target=results/"completed-strategy-replay"
archive=workspace/"work/已完成策略复现包.zip"
output_archive=workspace/"outputs/轮腿已完成策略复现包.zip"
require(not target.exists() and not archive.exists() and not output_archive.exists(),"本阶段已经保存")
doc="tools/仿真说明.md"
head=git("show","HEAD:"+doc)
require(git("show",":"+doc)==head,"说明索引已有其他修改")
working=(repo/doc).read_bytes()
marker="#### 十份已完成策略换目录真实重放（2026-10-12）"
require(marker.encode("utf-8") not in working,"说明已存在")
entry_files={name:package/name for name in inventory["files"]}
entry_files["交付清单.json"]=package/"交付清单.json"
entry_files.update({"已执行核验/"+p.name:p for p in (package/"复现输出").glob("*.json")})
with zipfile.ZipFile(archive,"x",zipfile.ZIP_DEFLATED,compresslevel=9) as z:
    for name,source in sorted(entry_files.items()):
        z.write(source,name)
with zipfile.ZipFile(archive) as z:
    require(z.testzip() is None and len(z.namelist())==len(entry_files),"交付归档损坏")
    require(all(hashlib.sha256(z.read(name)).hexdigest()==sha(source) for name,source in entry_files.items()),"归档字节不对应实测目录")
target.mkdir()
for source in (workspace/"work/BuildStudyDeliveryPackage.py",workspace/"work/ReproduceStudyPackage.py",Path(__file__)):
    shutil.copyfile(source,target/source.name)
for source in (package/"复现输出").glob("*.json"):
    shutil.copyfile(source,target/source.name)
shutil.copyfile(package/"交付清单.json",target/"交付清单.json")
shutil.copyfile(archive,target/"已完成策略复现包.zip")
shutil.copyfile(archive,output_archive)
require(sha(archive)==sha(target/"已完成策略复现包.zip")==sha(output_archive),"保存后的交付包不一致")
now=datetime.now().astimezone().isoformat()
record=dict(schema=1,saved_at=now,policy_count=10,trajectories=60,all_fields_exact=True,
    actual_runtime_directory=str(package),actual_proof_sha256=sha(package/"复现输出/汇总.json"),
    archive_sha256=sha(archive),archive_bytes=archive.stat().st_size,archive_entries=len(entry_files),
    inventory_sha256=sha(package/"交付清单.json"),source_script_sha256=sha(Path(__file__)),
    evidence={p.name:sha(p) for p in target.iterdir() if p.suffix==".json"},
    archive=(target/"已完成策略复现包.zip").relative_to(repo).as_posix(),F_included=False,
    boundary="同机独立目录里实际重放A至E十份最终策略，逐份三种场景及原控制共60条轨迹逐字段一致。归档字节与实测目录完全相同。不是另一电脑验证，不代表独立性能通过或整个目标完成。")
(target/"交付包保存核验.json").write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
progress_path=learning/"研究进度.json"
progress=read(progress_path)
progress["updated_at"]=now
progress["completed_strategy_replay"]={key:record[key] for key in ("policy_count","trajectories","all_fields_exact","archive","archive_sha256","F_included")}
progress["completed"].append("A至E十份最终策略已在独立目录实际重放停车/受推/组合及对应原控制，共60条轨迹逐字段一致；实际网络/13组优化器及导出推理通过，保存含599份源文件与模型报告的可运行复现包。F尚未包含。")
progress_path.write_bytes((json.dumps(progress,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
appendix=f"""{marker}

在独立目录重新组成A至E的十份最终模型及各自冻结环境，逐份实际重跑停车、受推、组合三例及对应原控制，共60条轨迹。选例只按输入排序取各类首个随机场景，不看性能结果；所有已保存指标逐字段一致，包括原有控制指标退步。六份神经网络均实际载入固定预算模型和13组优化器状态，128组输入与导出推理一致；四份搜索策略从已保存参数运行。此为同机换目录复现，未验证另一电脑，且不把三例重放当成全45例或24例独立验收。

`learning/results/study-20261012/completed-strategy-replay/已完成策略复现包.zip` 包含599份列入清单的源码/模型/报告/运行入口、清单及11份已执行核验记录；共{len(entry_files)}项、{archive.stat().st_size}字节，SHA256 `{sha(archive)}`。归档每一项与实测目录字节相同。解压到较短的新目录、使用包内requirements所列依赖后，执行 `python 检查并复现.py` 即可从包内相对路径重跑；结果写入新的 `复现输出`，包内旧核验另存于 `已执行核验`，不会覆盖。只重放已评价场景，不运行最终60例或连接实车。

本阶段包只包含A至E；F原固定训练与新24例对照仍未完成。归档和复现成功仅证明保存与重现可靠，不改变各路线未通过的性能结论。程序、逐策略实际重放结果和交付核验一并保存于同目录。
"""
expected=working.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")
(repo/doc).write_bytes(expected)
blob=git("hash-object","-w","--stdin",input=head.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")).decode().strip()
git("update-index","--cacheinfo",f"100644,{blob},{doc}")
paths=[p.relative_to(repo).as_posix() for p in target.iterdir() if p.is_file()]+[progress_path.relative_to(repo).as_posix()]
git("add","--",*paths)
actual=set(git("diff","--cached","--name-only","-z").decode("utf-8").strip("\0").split("\0"))
require(actual=={doc,*paths},"索引混入其他工作")
require(all(git("show",":"+p)==(repo/p).read_bytes() for p in paths),"暂存字节不符")
require((repo/doc).read_bytes()==expected,"说明工作区发生改变")
git("-c","core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof","diff","--cached","--check")
print(git("-c","core.quotepath=false","diff","--cached","--stat").decode("utf-8"))
