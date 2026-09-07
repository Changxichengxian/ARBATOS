"""仅用于已获准、已固定的准备固件台架加热；结束或检测异常时关加热。"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--elf", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--seconds", type=int, default=600)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
probe = Path(__file__).with_name("PreflightProbe.py")


def request(action, name):
    path = a.output / (name + ".json")
    for attempt in range(3):
        run = subprocess.run([sys.executable, "-X", "utf8", str(probe), action,
                              "--elf", str(a.elf), "--output", str(path)],
                             capture_output=True, text=True, encoding="utf-8", timeout=25)
        if run.returncode == 0 or "WAIT ACK" not in run.stderr:
            break
        # 校准扇区擦除期间单 Bank Flash 读取会等待；只对这一错误有限重试。
        (a.output / f"{name}-wait-{attempt}.txt").write_text(run.stderr, encoding="utf-8")
        time.sleep(1)
    if run.returncode:
        raise RuntimeError(run.stdout + run.stderr)
    return json.loads(path.read_text(encoding="utf-8"))


start = time.monotonic()
saved_at = None
try:
    initial = request("status", "initial")
    if not initial["imuValid"] or initial["imuAgeMs"] > 50 or initial["spiErrors"]:
        raise RuntimeError("IMU is not ready")
    request("warm", "enabled")
    number = 0
    while time.monotonic() - start < a.seconds:
        state = request("status", f"sample-{number:04}")
        number += 1
        elapsed = time.monotonic() - start
        print(json.dumps({"elapsed": round(elapsed, 1), "temperature": state["temperature"],
                          "heater": state["heater"], "calibrated": state["calibrated"],
                          "store": state["ImuCalStoreDiag"], "bias": state["bias"],
                          "spiErrors": state["spiErrors"]}), flush=True)
        if (not math.isfinite(state["temperature"]) or state["temperature"] >= 43
                or state["heater"] > 200 or not state["imuValid"]
                or state["imuAgeMs"] > 100 or state["spiErrors"]):
            raise RuntimeError("temperature, heater or IMU safety check failed")
        store = state["ImuCalStoreDiag"]
        if state["calibrated"] and store[1] > 0 and store[3] == 0:
            if saved_at is None:
                saved_at = elapsed
            if elapsed - saved_at >= 60:
                print("PASS: calibration saved; another 60 seconds observed", flush=True)
                break
        time.sleep(5)
    else:
        raise TimeoutError("warm calibration did not complete before the deadline")
finally:
    # 每个请求独占调试器；即使采样出错也尝试显式关闭。
    stopped = request("cool", "cooled")
    time.sleep(0.1)
    stopped = request("status", "final")
    print(json.dumps({"heatEnabled": stopped["heatEnabled"], "heater": stopped["heater"],
                      "temperature": stopped["temperature"]}), flush=True)
    if stopped["heatEnabled"] or stopped["heater"]:
        raise RuntimeError("heater shutdown not confirmed")
