"""小轮腿网页驾驶入口；沿用大轮腿的按住操作与独立物理/渲染线程方式。"""

import argparse
import io
import json
import math
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import webbrowser

import mujoco
import numpy as np
from PIL import Image

import RunMeasuredWheelLeg as sim


class LiveSimulation:
    def __init__(self, output, profile):
        self.output, self.profile = output, profile
        output.mkdir(parents=True, exist_ok=False)
        self.xml = sim.model_xml(profile)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.core = sim.Core(profile, sim.ROOT / "local/cache/measured-wheelleg-native")
        self.cfg = sim.make_config(profile)
        self.designed = sim.design(self.model, self.cfg, self.core, profile, profile["control"]["leg_length_m"])
        self.schedule = sim.design_modes(self.model, self.cfg, self.core, profile)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.keys = set()
        self.drive_speed, self.turn_speed = .8, 1.2
        self.mode = "vmc"
        self.target_length = profile["control"]["leg_length_m"]
        self.input_time = 0.0
        self.reset_requested = False
        self.motor_stop_requested = False
        self.frame = b""
        self.frame_seq = 0
        self.camera = "follow"
        self.state = {"status": "正在启动", "running": False}
        self.render_qpos = np.array(self.designed["qpos"])
        self.reset()
        (output / "model.xml").write_text(self.xml, encoding="utf-8")
        sim.write_json(output / "parameters.json", profile)
        sim.write_json(output / "controller.json", self.designed)
        sim.write_json(output / "height_gains.json", self.schedule)
        sim.write_json(output / "sources.json", {p.name: sim.hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (Path(__file__), sim.HERE / "RunMeasuredWheelLeg.py", sim.HERE / "MeasuredWheelLegViewer.html", sim.PROFILE)})

    def reset(self, mode="vmc"):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.designed["qpos"]
        mujoco.mj_forward(self.model, self.data)
        self.controller = sim.Controller(self.model, self.core, self.profile, self.designed, mode, self.schedule)

    def command(self, value):
        keys = value.get("keys", [])
        if not isinstance(keys, list) or any(not isinstance(k, str) or k not in ("w", "a", "s", "d", "q", "e", " ") for k in keys):
            raise ValueError("无效按键")
        speed, turn = value.get("drive_speed", .8), value.get("turn_speed", 1.2)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (speed, turn)):
            raise ValueError("速度必须为有限数值")
        mode, length = value.get("mode"), value.get("leg_length")
        if mode is not None and (not isinstance(mode, str) or mode not in sim.MODES):
            raise ValueError("无效控制模式")
        if length is not None and (isinstance(length, bool) or not isinstance(length, (int, float)) or not math.isfinite(length)):
            raise ValueError("腿长必须为有限数值")
        with self.lock:
            self.keys = set(keys)
            self.drive_speed = float(np.clip(speed, .1, self.profile["control"]["drive_speed_limit_m_s"]))
            self.turn_speed = float(np.clip(turn, .2, self.profile["control"]["yaw_rate_limit_rad_s"]))
            self.input_time = time.monotonic()
            if length is not None:
                self.target_length = float(np.clip(length, *self.profile["control"]["leg_range_m"]))
            if mode is not None and mode != self.mode:
                self.mode = mode
                self.reset_requested = True
            if value.get("reset") is True or self.reset_requested:
                self.keys.clear()
                self.target_length = self.profile["control"]["leg_length_m"]
            self.reset_requested |= value.get("reset") is True
            self.motor_stop_requested |= value.get("motor_stop") is True

    def physics(self):
        started = time.monotonic()
        dt = self.profile["control"]["physics_dt_s"]
        stride = round(self.profile["control"]["control_dt_s"] / dt)
        step = 0
        last_status = 0.0
        try:
            with (self.output / "驾驶记录.csv").open("w", encoding="utf-8", buffering=1) as log:
                log.write("time,mode,target_length,applied_length,right_length,left_length,target_v,target_yaw,body_v,yaw_rate,pitch_deg,roll_deg,split_deg,rf_tau,rb_tau,rw_tau,lf_tau,lb_tau,lw_tau\n")
                while not self.stop.is_set():
                    now = time.monotonic()
                    with self.lock:
                        reset, motor_stop = self.reset_requested, self.motor_stop_requested
                        self.reset_requested = self.motor_stop_requested = False
                        keys = self.keys.copy() if now - self.input_time < .35 else set()
                        speed, turn = self.drive_speed, self.turn_speed
                        mode, length = self.mode, self.target_length
                    if reset:
                        self.reset(mode)
                        keys = set()
                    if motor_stop:
                        self.controller.enabled = False
                    target_v = speed * (int("w" in keys) - int("s" in keys))
                    target_yaw = turn * (int("a" in keys) - int("d" in keys))
                    if " " in keys:
                        target_v = target_yaw = 0.0
                    if step % stride == 0:
                        self.controller.step(self.data, target_v, target_yaw, length)
                    mujoco.mj_step(self.model, self.data)
                    step += 1
                    if step % 30 == 0:
                        roll, pitch, yaw = sim.legacy.quat_to_euler_wxyz(self.data.qpos[3:7])
                        velocity = self.data.xmat[self.model.body("base").id].reshape(3, 3).T @ self.data.qvel[:3]
                        split = math.degrees(float(self.controller.leg_angles[0] - self.controller.leg_angles[1]))
                        state = {"running": self.controller.enabled, "time": float(self.data.time),
                                 "speed": float(velocity[0]), "target_speed": target_v, "yaw_rate": float(self.data.qvel[5]),
                                 "target_yaw": target_yaw, "pitch": math.degrees(pitch), "roll": math.degrees(roll),
                                 "yaw": math.degrees(yaw), "split": split, "legs": (1000 * self.controller.leg_lengths).tolist(),
                                 "torques": self.data.ctrl.tolist(), "rtf": step * dt / max(.001, now - started),
                                 "mode": self.controller.mode, "target_length": self.controller.target_length,
                                 "applied_length": self.controller.length, "height_range": self.profile["control"]["leg_range_m"],
                                 "drive_speed": speed, "turn_speed": turn,
                                 "status": sim.MODES[self.controller.mode] if self.controller.enabled else "电机已停止，按 T 复位",
                                 "keys": sorted(keys)}
                        with self.lock:
                            self.render_qpos = self.data.qpos.copy()
                            self.state = state
                        log.write(",".join(str(v) for v in [self.data.time, self.controller.mode, self.controller.target_length,
                                  self.controller.length, *self.controller.leg_lengths, target_v, target_yaw, velocity[0], self.data.qvel[5],
                                  state["pitch"], state["roll"], split, *self.data.ctrl]) + "\n")
                        if now - last_status > 1:
                            sim.write_json(self.output / "live_state.json", state)
                            last_status = now
                    if step % 6 == 0:
                        self.stop.wait(max(0.0, started + step * dt - time.monotonic()))
        except Exception as exc:
            with self.lock:
                self.state.update(status=f"物理运行失败：{exc}", running=False)
            self.stop.set()
            raise

    def render(self):
        # 与大轮腿入口一样，渲染使用独立数据，浏览器取图不阻塞控制周期。
        model = mujoco.MjModel.from_xml_string(self.xml)
        model.vis.headlight.ambient[:] = [.45, .45, .45]
        data = mujoco.MjData(model)
        camera = mujoco.MjvCamera()
        try:
            with mujoco.Renderer(model, height=600, width=960) as renderer:
                while not self.stop.is_set():
                    started = time.monotonic()
                    with self.lock:
                        data.qpos[:] = self.render_qpos
                        mode = self.camera
                    mujoco.mj_forward(model, data)
                    _, _, yaw = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
                    camera.distance, camera.elevation = .62, -22
                    camera.azimuth = math.degrees(yaw) + (25 if mode == "follow" else 130)
                    camera.lookat[:] = [data.qpos[0], data.qpos[1], .10]
                    renderer.update_scene(data, camera=camera)
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                    buffer = io.BytesIO()
                    Image.fromarray(renderer.render()).save(buffer, "JPEG", quality=90)
                    with self.lock:
                        self.frame = buffer.getvalue()
                        self.frame_seq += 1
                    self.stop.wait(max(0, .05 - (time.monotonic() - started)))
        except Exception as exc:
            with self.lock:
                self.state.update(status=f"渲染失败：{exc}", running=False)
            self.stop.set()
            raise


def make_handler(live):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, code, content, kind):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                # 共享已有大轮腿页面的基础样式，单独列出小车当前可用功能。
                reference = (sim.HERE.parent / "reference/OwnWbrViewer.html").read_text(encoding="utf-8")
                style = reference.split("<style>", 1)[1].split("</style>", 1)[0]
                html = (sim.HERE / "MeasuredWheelLegViewer.html").read_text(encoding="utf-8").replace("{{REFERENCE_STYLE}}", style)
                self.send(200, html.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/state":
                with live.lock:
                    value = dict(live.state, frame_seq=live.frame_seq)
                self.send(200, json.dumps(value).encode(), "application/json")
            elif path == "/frame.jpg":
                with live.lock:
                    frame = live.frame
                self.send(200 if frame else 503, frame, "image/jpeg")
            else:
                self.send(404, b"Not found", "text/plain")

        def do_POST(self):
            origin = self.headers.get("Origin")
            host = self.headers.get("Host", "")
            if origin and origin != "http://" + host:
                self.send(403, b"Origin rejected", "text/plain")
                return
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 4096:
                    raise ValueError("Invalid size")
                value = json.loads(self.rfile.read(size))
                if not isinstance(value, dict):
                    raise ValueError("Expected object")
                if self.path == "/command":
                    live.command(value)
                elif self.path == "/camera" and value.get("mode") in ("follow", "oblique"):
                    with live.lock:
                        live.camera = value["mode"]
                else:
                    raise ValueError("Invalid request")
                self.send(200, b'{"ok":true}', "application/json")
            except (ValueError, TypeError) as exc:
                self.send(400, json.dumps({"error": str(exc)}).encode(), "application/json")
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--output", type=Path, default=sim.ROOT / "local/cache" / ("wheelleg-web-" + time.strftime("%Y%m%d-%H%M%S")))
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()
    live = LiveSimulation(args.output.resolve(), sim.load_profile())
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(live))
    threads = [threading.Thread(target=live.physics, name="small-physics"), threading.Thread(target=live.render, name="small-render")]
    for thread in threads:
        thread.start()
    url = f"http://127.0.0.1:{args.port}"
    print(url, flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        live.stop.set()
        server.server_close()
        for thread in threads:
            thread.join()


if __name__ == "__main__":
    main()
