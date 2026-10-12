from datetime import datetime
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import uuid

workspace=Path(__file__).resolve().parents[1]
base=workspace/"work/study-20261012/long-trace"
script=workspace/"work/LockTraceComparison.py"
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
target=base/"评价前置检查-v2.json"
if target.exists():
    raise FileExistsError(target)
guard=subprocess.run([sys.executable,str(script)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",timeout=60)
if guard.returncode==0 or "所有原定任务尚未完成" not in guard.stderr:
    raise RuntimeError("未完成模型没有被正确拒绝")
smoke=subprocess.run([sys.executable,str(script),"--check-smoke-lineage"],stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",timeout=60)
if smoke.returncode:
    raise RuntimeError(smoke.stderr)
chain=json.loads(smoke.stdout)
if not chain["actual_resume_chain_checked"] or len(chain["chain"])!=1 or chain["chain"][0]["initial_steps"]!=1024:
    raise RuntimeError("实际恢复链核验不符")
if (base/"comparison-locks").exists() or (base/"evaluation").exists() or (base/"四策略锁定.json").exists():
    raise RuntimeError("前置拒绝检查不应产生评价许可")
# 隔离执行同一源文件的实际发布函数，检查中断补齐及并发写入冲突；不伪造策略许可。
test_root=workspace/"work/trace-publication-tests-v2"
test_root.mkdir(exist_ok=False)
tree=ast.parse(script.read_text(encoding="utf-8"))
definitions=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ("require","save")]
namespace=dict(json=json,os=os,uuid=uuid)
exec(compile(ast.Module(body=definitions,type_ignores=[]),str(script),"exec"),namespace)
publish=namespace["save"]
first=test_root/"record.json"
second=test_root/"plan.json"
publish(first,dict(test_record=1))
first_bytes=first.read_bytes()
publish(first,dict(test_record=1))
publish(second,dict(test_plan=1))
second_bytes=second.read_bytes()
try:
    publish(second,dict(test_plan=2))
except RuntimeError:
    pass
else:
    raise RuntimeError("不同的已有计划被覆盖")
race=test_root/"race.json"
original_link=os.link
def competing_link(source,destination):
    Path(destination).write_bytes(b"another writer\n")
    original_link(source,destination)
namespace["os"]=SimpleNamespace(fsync=os.fsync,link=competing_link)
try:
    publish(race,dict(our_record=1))
except RuntimeError:
    pass
else:
    raise RuntimeError("并发不同内容没有拒绝")
if first.read_bytes()!=first_bytes or second.read_bytes()!=second_bytes or race.read_bytes()!=b"another writer\n":
    raise RuntimeError("发布过程中已有内容变化")
if list(test_root.glob("*.tmp")):
    raise RuntimeError("发布临时文件未清理")
record=dict(schema=1,checked_at=datetime.now().astimezone().isoformat(),
    pending_training_rejected=True,no_evaluation_artifacts_created=True,actual_smoke_resume_chain=chain,
    locker_sha256=sha(script),checker_sha256=sha(Path(__file__)),
    publication_checks=dict(same_record_reused=True,missing_plan_completed=True,different_plan_rejected=True,
        competing_writer_preserved=True,temporary_files_cleaned=True),
    boundary="验证未完成时阻止评价和真实短训恢复来源；四个最终模型完成后的正向锁定仍待执行。")
target.write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(json.dumps(record,ensure_ascii=False))
