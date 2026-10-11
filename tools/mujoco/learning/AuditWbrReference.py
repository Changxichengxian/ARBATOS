"""核对冻结参考诊断的场景、源代码和实际动作轨迹，不运行物理仿真。"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def audit(snapshot,output):
    read = lambda path: json.loads(path.read_text(encoding="utf-8"))
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = read(snapshot/"manifest.json")
    for name,expected in manifest["files"].items():
        if digest(snapshot/name) != expected:
            raise RuntimeError(f"冻结源码被改动：{name}")
    if digest(snapshot/manifest["library"]) != manifest["library_sha256"]:
        raise RuntimeError("冻结动态库不匹配")
    run = read(output/"run.json")["binding"]
    script = snapshot/"tools/mujoco/learning/DiagnoseWbrReference.py"
    if run["script_sha256"] != digest(script) or run["library_sha256"] != manifest["library_sha256"]:
        raise RuntimeError("运行代码或动态库身份不匹配")
    registration = snapshot/"tools/mujoco/learning/results/study-20261012/参考减速诊断登记.json"
    if digest(registration) != run["registration_sha256"]:
        raise RuntimeError("诊断登记文件不匹配")
    registered = read(registration)
    cases = read(snapshot/"tools/mujoco/learning/cases/study-20261012/development.json")
    if registered["modes"] != run["modes"] or cases["cases_sha256"] != run["cases_sha256"]:
        raise RuntimeError("模式或场景清单不匹配")
    checked = {}
    complete_modes = []
    for mode in run["modes"]:
        count = 0
        for scenario in cases["cases"]:
            path = output/f"{mode['name']}-{scenario['id']}.json"
            if not path.exists():
                continue
            saved = read(path)
            row = saved.get("result",saved)
            expected = dict(id=scenario["id"],seed=scenario["seed"],randomize=scenario["randomize"],
                            case=scenario["options"]["case"],options=scenario["options"])
            if any(row.get(key) != value for key,value in expected.items()):
                raise RuntimeError(f"实际场景字段不匹配：{path.name}")
            target = np.concatenate((np.asarray(mode["action"],dtype=np.float32).astype(float),np.zeros(3)))
            filtered = np.zeros(6)
            if not row.get("trace"):
                raise RuntimeError(f"缺少实际动作轨迹：{path.name}")
            for sample in row["trace"]:
                filtered += np.clip(target-filtered,-.2,.2)
                if not np.array_equal(filtered,np.asarray(sample["action"])):
                    raise RuntimeError(f"动作轨迹与登记参数不同：{path.name}")
            checked[path.name] = digest(path)
            count += 1
        if count == len(cases["cases"]):
            complete_modes.append(mode["name"])
    actual = {path.name for path in output.glob("*-development-*.json")}
    if actual != set(checked):
        raise RuntimeError("目录含未登记场景文件")
    return dict(schema=1,checked_cases=len(checked),complete_modes=complete_modes,
                expected_cases=len(run["modes"])*len(cases["cases"]),
                original_run_sha256=digest(output/"run.json"), result_files=checked,
                note="只确认记录的场景/动作/冻结身份，不替代性能和物理验收")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot",required=True,type=Path)
    parser.add_argument("--results",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args = parser.parse_args()
    result = audit(args.snapshot,args.results)
    args.output.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps({key:value for key,value in result.items() if key != "result_files"},ensure_ascii=False))
