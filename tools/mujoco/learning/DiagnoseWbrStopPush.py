"""只取消已登记组合场景的外推力，分离停车回摆与受推后的配对差值。"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

from AuditWbrReference import audit


def encoded(value):
    return json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(",", ":")).encode("utf-8")


def content_hash(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_verified(path,expected):
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected:
        raise RuntimeError("读取的原轨迹与历史文件哈希不同")
    return json.loads(raw.decode("utf-8"))


def write(path,value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    temporary.replace(path)


def no_push_case(original):
    if original["options"]["case"] != "combined" or not original["options"]["push"]:
        raise ValueError("只接受有外推力的组合开发场景")
    scenario = deepcopy(original)
    scenario["id"] += "-no-push"
    scenario["options"]["push"] = 0.
    return scenario


def check_cache(saved,binding,scenario):
    if saved.get("binding") != binding or saved.get("sha256") != content_hash(
            {key:value for key,value in saved.items() if key != "sha256"}):
        raise RuntimeError("无推力缓存身份或内容不匹配")
    result = saved["result"]
    expected = dict(id=scenario["id"],case="combined",seed=scenario["seed"],
                    randomize=scenario["randomize"],options=scenario["options"])
    if any(result.get(key) != value for key,value in expected.items()):
        raise RuntimeError("无推力缓存场景字段不匹配")
    return result


def pair_metrics(forced,unforced):
    if not forced["metrics"]["passed"] or not unforced["metrics"]["passed"]:
        return dict(complete=False,forced_failure=forced["metrics"].get("failure"),
                    unforced_failure=unforced["metrics"].get("failure"))
    a,b = forced["trace"],unforced["trace"]
    if not a or not b or [row["time_s"] for row in a] != [row["time_s"] for row in b]:
        raise RuntimeError("有推/无推轨迹的采样时刻不同")
    before_a = [row for row in a if row["time_s"] < 5.-1e-9]
    before_b = [row for row in b if row["time_s"] < 5.-1e-9]
    if not before_a or before_a != before_b:
        raise RuntimeError("推力发生前轨迹不完全相同，不能作为单变量配对")
    indices = [i for i,row in enumerate(a) if 5.-1e-9 <= row["time_s"] <= 7.2+1e-9]
    if not indices:
        raise RuntimeError("缺少受推窗口")
    va = np.array([a[i]["true_v_mps"] for i in indices])
    vb = np.array([b[i]["true_v_mps"] for i in indices])
    delta = va-vb
    peak_index = int(np.argmax(np.abs(va)))
    sample = a[indices[peak_index]]
    return dict(complete=True,pre_push_exact=True,pre_push_samples=len(before_a),
        before_push_time_s=before_a[-1]["time_s"],before_push_velocity_mps=before_a[-1]["true_v_mps"],
        forced_peak_abs_mps=float(np.max(np.abs(va))),unforced_peak_abs_mps=float(np.max(np.abs(vb))),
        forced_rms_mps=float(np.sqrt(np.mean(va**2))),unforced_rms_mps=float(np.sqrt(np.mean(vb**2))),
        paired_delta_peak_abs_mps=float(np.max(np.abs(delta))),paired_delta_rms_mps=float(np.sqrt(np.mean(delta**2))),
        forced_peak_sample=dict(time_s=sample["time_s"],forced_v_mps=float(va[peak_index]),
                                unforced_v_mps=float(vb[peak_index]),delta_v_mps=float(delta[peak_index])),
        forced_hip_heat_nm2s=forced["metrics"]["hip_heat_nm2s"],
        unforced_hip_heat_nm2s=unforced["metrics"]["hip_heat_nm2s"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot",required=True,type=Path)
    parser.add_argument("--source-results",required=True,type=Path)
    parser.add_argument("--registration",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    parser.add_argument("--resume",action="store_true")
    args = parser.parse_args()
    args.snapshot,args.source_results,args.output = (path.resolve() for path in (args.snapshot,args.source_results,args.output))
    sys.path.insert(0,str(args.snapshot/"tools/mujoco/learning"))
    from WbrLearningEnv import verify_snapshot,experiment_identity
    from EvaluateWbrStudy import run_case,EvaluationInterrupted
    manifest = verify_snapshot()
    registration = read(args.registration)
    audited = audit(args.snapshot,args.source_results)
    if audited["checked_cases"] != 48 or digest(args.source_results/"summary.json") != registration["source_summary_sha256"]:
        raise RuntimeError("原固定参数诊断不完整或与登记不符")
    if registration["experiment_identity"] != experiment_identity(manifest):
        raise RuntimeError("登记的物理身份不匹配")
    cases = read(args.snapshot/"tools/mujoco/learning/cases/study-20261012/development.json")
    if cases["split"] != "development" or cases["cases_sha256"] != registration["development_cases_sha256"]:
        raise RuntimeError("只允许登记的开发场景")
    selected = [row for row in cases["cases"] if row["id"] in registration["case_ids"]]
    if {row["id"] for row in selected} != set(registration["case_ids"]) or len(selected) != 2:
        raise RuntimeError("必须完整选择两个组合开发场景")
    registered_modes = {row["name"]:row["action"] for row in read(args.source_results/"run.json")["binding"]["modes"]}
    modes = registration["modes"]
    if len(modes) != 4 or len({row["name"] for row in modes}) != 4:
        raise RuntimeError("必须完整选择四个固定参数模式")
    for mode in modes:
        if registered_modes.get(mode["name"]) != mode["action"]:
            raise RuntimeError("动作参数与原诊断不同")
    binding = dict(script_sha256=digest(Path(__file__)),audit_script_sha256=digest(Path(__file__).with_name("AuditWbrReference.py")),
        snapshot_manifest_sha256=digest(args.snapshot/"manifest.json"),
        evaluator_sha256=digest(args.snapshot/"tools/mujoco/learning/EvaluateWbrStudy.py"),
        registration_sha256=digest(args.registration),experiment_identity=experiment_identity(manifest),
        library_sha256=manifest["library_sha256"],source_summary_sha256=registration["source_summary_sha256"],
        source_files={f"{mode['name']}-{case['id']}.json":audited["result_files"][f"{mode['name']}-{case['id']}.json"]
                      for mode in modes for case in selected})
    if binding["source_files"] != registration["source_files"]:
        raise RuntimeError("原轨迹与登记的历史文件哈希不同")
    binding_sha256 = content_hash(binding)
    args.output.mkdir(parents=True,exist_ok=True)
    lock = (args.output/"diagnostic.lock").open("a+b")
    lock.seek(0)
    if not lock.read(1):
        lock.write(b"0")
        lock.flush()
    lock.seek(0)
    if os.name == "nt":
        import msvcrt
        msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    else:
        import fcntl
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    run_path = args.output/"run.json"
    if run_path.exists():
        if not args.resume or read(run_path) != dict(binding=binding,binding_sha256=binding_sha256):
            raise RuntimeError("必须使用原登记和版本显式恢复")
        if (args.output/"summary.json").exists():
            raise RuntimeError("诊断已完成，无需重复")
    else:
        if args.resume or list(args.output.glob("*.json")):
            raise RuntimeError("新诊断拒绝预存结果")
        write(run_path,dict(binding=binding,binding_sha256=binding_sha256))
    deadline = datetime.fromisoformat(registration["deadline"]).timestamp()
    should_stop = lambda: time.time() >= deadline or (args.output/"stop.request").exists()
    pairs = []
    class FixedPolicy:
        def __init__(self,action):
            self.value = np.asarray(action,dtype=np.float32)
        def predict(self,observation):
            return self.value.copy()
    try:
        for mode in modes:
            for original in selected:
                if should_stop():
                    raise EvaluationInterrupted("诊断停止")
                scenario = no_push_case(original)
                source_name = f"{mode['name']}-{original['id']}.json"
                forced = read_verified(args.source_results/source_name,binding["source_files"][source_name])
                cache_binding = dict(run=binding_sha256,mode=mode,scenario=scenario,
                                     source_sha256=binding["source_files"][source_name])
                path = args.output/f"{mode['name']}-{scenario['id']}.json"
                if path.exists():
                    if not args.resume:
                        raise RuntimeError("新诊断出现预存单例")
                    unforced = check_cache(read(path),cache_binding,scenario)
                else:
                    unforced = run_case(scenario,FixedPolicy(mode["action"]),
                        dict(reward_mode="dense",action_mode="reference",gas_force=450.),record=True,stop_requested=should_stop)
                    saved = dict(binding=cache_binding,result=unforced)
                    saved["sha256"] = content_hash(saved)
                    write(path,saved)
                pair = dict(mode=mode,case=original,source_file=source_name,unforced_file=path.name,
                            unforced_sha256=digest(path),comparison=pair_metrics(forced,unforced))
                pairs.append(pair)
                write(args.output/"status.json",dict(status="running",pid=os.getpid(),completed_pairs=len(pairs),total_pairs=8))
                print(json.dumps(dict(mode=mode["name"],case=original["id"],comparison=pair["comparison"]),ensure_ascii=False),flush=True)
        if digest(Path(__file__)) != binding["script_sha256"]:
            raise RuntimeError("运行期间诊断脚本被改动")
        if (digest(Path(__file__).with_name("AuditWbrReference.py")) != binding["audit_script_sha256"] or
                digest(args.registration) != binding["registration_sha256"] or
                digest(args.snapshot/"manifest.json") != binding["snapshot_manifest_sha256"]):
            raise RuntimeError("运行期间登记、审计代码或冻结清单被改动")
        verify_snapshot()
        for name,expected in binding["source_files"].items():
            if digest(args.source_results/name) != expected:
                raise RuntimeError("运行期间原轨迹被改动")
        write(args.output/"summary.json",dict(schema=1,binding=binding,binding_sha256=binding_sha256,
            completed_at=datetime.now(timezone.utc).isoformat(),pairs=pairs,
            all_physical_complete=all(row["comparison"]["complete"] for row in pairs),
            scope="只用既有两个组合开发场景；每种固定控制保持同种子和条件，只取消推力。差值含闭环控制响应，不能假设线性叠加；不修改正式验收，也不计作独立性能验证。"))
        write(args.output/"status.json",dict(status="completed",pid=os.getpid(),completed_pairs=8,total_pairs=8))
    except EvaluationInterrupted:
        write(args.output/"status.json",dict(status="interrupted",pid=os.getpid(),completed_pairs=len(pairs),total_pairs=8))
    except Exception as error:
        write(args.output/"status.json",dict(status="failed",pid=os.getpid(),completed_pairs=len(pairs),error=repr(error)))
        raise
    finally:
        lock.close()


if __name__ == "__main__":
    main()
