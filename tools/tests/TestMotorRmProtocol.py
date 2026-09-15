"""按实际电机表和发送函数检查 RM 模式、组帧及停机路由，无硬件访问。"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def function(path, name):
    text = (ROOT / path).read_text(encoding="utf-8")
    match = re.search(r"(?m)^(?:static\s+)?(?:inline\s+)?[\w*]+\s+" + name + r"\s*\(", text)
    if match is None:
        raise ValueError(f"找不到函数 {name}")
    start = text.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end] + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定 Zig 编译器")
    out = ROOT / "local/build/host-tests/rm-protocol"
    out.mkdir(parents=True, exist_ok=True)
    python = ROOT / "local/cache/zephyrproject/.venv/Scripts/python.exe"
    task = (ROOT / "shared/application/comm/can/CanTxTask.c").read_text(encoding="utf-8")
    declarations = task[task.index("enum { CanTxRmGroupCount"):task.index("static uint32_t s_can_tx_mit_budget_tick_ms")]
    parts = [declarations]
    base = "shared/application/comm/can/"
    for path, names in [
        ("CanCommandTxCommonHelpers.inc", ["CanTxBusIndex", "CanTxRmGroupIndex", "CanTxMarkRmGroupConfigured"]),
        ("CanCommandTxRouteHelpers.inc", ["CanTxClearRmFrames", "CanTxStoreRmCurrent"]),
        ("CanReceive.c", ["CanCmdRmGroupSend", "CAN_cmd_rm_group", "CAN_cmd_rm_group_tracked"]),
    ]:
        parts.extend(function(base + path, name) for name in names)
    parts.append(function(base + "CanCommandTxRouteHelpers.inc", "CanTxRefreshRmGroup"))
    parts.append((ROOT / (base + "CanCommandTxEmitHelpers.inc")).read_text(encoding="utf-8"))
    parts.append(function(base + "CanTxTask.c", "CanTxEmergencyStopNow"))
    (out / "ExtractedRm.inc").write_text("\n".join(parts), encoding="utf-8")
    for target in ["INFANTRY-TIAN-A", "HERO-M", "SENTINEL-M"]:
        config = out / target
        subprocess.run([str(python), "-X", "utf8", str(ROOT / "tools/config/RobotConfigGen.py"),
                        "generate", "--target", target, "--out", str(config)], check=True)
        exe = out / (target + ".exe")
        includes = [out, ROOT / "tools/tests/stubs", ROOT / "Robotconfig" / target, config,
                    ROOT / "shared/application/robot", ROOT / "shared/application/motors",
                    ROOT / "shared/components/support", ROOT / "shared/components/controller"]
        subprocess.run([args.zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        *(["-DRM_TEST_INFANTRY=1"] if target == "INFANTRY-TIAN-A" else []),
                        *["-I" + str(p) for p in includes],
                        str(ROOT / "tools/tests/MotorRmProtocolRegression.c"),
                        str(ROOT / "shared/application/motors/MotorModelDb.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
