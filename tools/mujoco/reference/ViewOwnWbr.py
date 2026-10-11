"""本机浏览器交互：MuJoCo 实时物理、ARBATOS 控制核心、滑行与反向制动。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import mujoco
import numpy as np
from PIL import Image

from RunOwnWbr import load_core
from RunWbr import ROOT, SOURCE, REVISION, MOTORS, load_bridge
from WbrGasSpring import load_model
from WbrGuideRollers import guide_roller_info
from WbrKneeRange import knee_range_info
from WbrLinkCollision import link_collision_info
from WbrTerrain import ground_geom_ids, ground_height, terrain_info, scenario_pose
from WbrMotorTelemetry import WbrMotorTelemetry
from WbrRecorder import WbrRecorder

RENDER_QUALITIES = {"standard": (800, 500), "high": (1280, 800), "ultra": (1920, 1200)}
HIP_CONTINUOUS_NM = 20.0
HIP_PEAK_NM = 40.0


def steering_direction(estimated_speed, drive_request, previous):
    """按实际行驶方向反转倒车转向，制动过零附近保留迟滞。"""
    if estimated_speed < -.08:
        return -1
    if estimated_speed > .08:
        return 1
    if abs(estimated_speed) < .03:
        return -1 if drive_request < 0 else 1
    return previous


class JumpDiagnostics:
    """仅观察真实离地段，不把真值传给控制器。反弹分别计量。"""
    def __init__(self, model, radius):
        self.model = model
        self.radius = radius
        self.ground = ground_geom_ids(model)
        self.wheels = {model.body(name).id for name in ("lwlink", "rwlink")}
        self.base = model.body("base_link").id
        self.reset()

    def reset(self):
        self.total = self.longest = 0.0
        self.run = self.best = None
        self.support_height = None

    def observe(self, data):
        supported = False
        support_heights = []
        for contact in data.contact:
            if contact.geom1 in self.ground or contact.geom2 in self.ground:
                supported = True
                if abs(float(contact.frame[2])) > .5:
                    support_heights.append(float(contact.pos[2]))
        if supported:
            self.run = None
            if support_heights:
                self.support_height = max(support_heights)
            return
        com_z = float(data.subtree_com[self.base, 2])
        wheel_ground = {wheel: ground_height(self.model, float(data.xpos[wheel, 0]),
                                             float(data.xpos[wheel, 1])) for wheel in self.wheels}
        if not all(math.isfinite(height) for height in wheel_ground.values()):
            self.run = None
            return
        if self.run is None:
            jacobian = np.zeros((3, self.model.nv))
            mujoco.mj_jacSubtreeCom(self.model, data, jacobian, self.base)
            # 越过台阶边沿后，脚下高度会变化；净空固定使用最后支撑面的高度。
            ground_start = self.support_height if self.support_height is not None else max(wheel_ground.values())
            self.run = dict(duration=0.0, clearance=0.0, local_clearance=0.0,
                            ground_start=ground_start, com_start=com_z,
                            com_peak=com_z, vz=float((jacobian @ data.qvel)[2]))
        self.run["duration"] += self.model.opt.timestep
        self.total += self.model.opt.timestep
        self.longest = max(self.longest, self.run["duration"])
        self.run["clearance"] = max(self.run["clearance"],
            min(float(data.xpos[wheel, 2]) for wheel in self.wheels) - self.radius - self.run["ground_start"])
        self.run["local_clearance"] = max(self.run["local_clearance"], min(
            float(data.xpos[wheel, 2]) - self.radius - wheel_ground[wheel] for wheel in self.wheels))
        self.run["com_peak"] = max(self.run["com_peak"], com_z)
        if self.best is None or self.run["clearance"] >= self.best["clearance"]:
            self.best = self.run

    def summary(self):
        return dict(clearance_mm=self.best["clearance"] * 1000 if self.best else 0.0,
                    local_clearance_mm=self.best["local_clearance"] * 1000 if self.best else 0.0,
                    no_contact_total_ms=self.total * 1000,
                    no_contact_longest_ms=self.longest * 1000,
                    com_rise_mm=(self.best["com_peak"] - self.best["com_start"]) * 1000
                        if self.best else None,
                    takeoff_vz_mps=self.best["vz"] if self.best else None)

    @staticmethod
    def empty():
        return dict(clearance_mm=None, local_clearance_mm=None, no_contact_total_ms=None,
                    no_contact_longest_ms=None, com_rise_mm=None, takeoff_vz_mps=None)


class LiveSimulation:
    def __init__(self, output: Path, correct_link_collisions: bool = True, terrain: str = "flat",
                 guide_rollers: str = "passive", knee_range: str = "stop180"):
        self.output = output
        if terrain not in ("flat", "course"):
            raise ValueError("未知场地")
        self.terrain = terrain
        self.noslip_iterations = 10 if terrain == "course" else 0
        self.correct_link_collisions = correct_link_collisions
        self.guide_rollers = guide_rollers
        self.knee_range = knee_range
        output.mkdir(parents=True, exist_ok=True)
        profile = json.loads(Path(__file__).with_name("OwnWbrProfile.json").read_text(encoding="utf-8"))
        self.profile = profile
        self.config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
            profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
            profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
        self.own_lib, own_build = load_core(output)
        self.upstream_lib = load_bridge(SOURCE, output, correct_spring_derivatives=True)
        pointer = ctypes.POINTER(ctypes.c_float)
        self.own_lib.OwnWbrCommandStep.argtypes = [ctypes.c_void_p, pointer, pointer, pointer]
        self.own_lib.OwnWbrCommandStep.restype = ctypes.c_int
        self.own_lib.OwnWbrSetJointPeak.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self.own_lib.OwnWbrSetJointPeak.restype = ctypes.c_int
        self.joint_limits_api = hasattr(self.own_lib, "OwnWbrSetJointLimits")
        if self.joint_limits_api:
            self.own_lib.OwnWbrSetJointLimits.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
            self.own_lib.OwnWbrSetJointLimits.restype = ctypes.c_int
        self.own_lib.OwnWbrEnableRise.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.own_lib.OwnWbrEnableRise.restype = ctypes.c_int
        self.own_lib.OwnWbrRequestRecovery.argtypes = [ctypes.c_void_p]
        self.own_lib.OwnWbrRequestRecovery.restype = ctypes.c_int
        self.own_lib.OwnWbrSetGasSpring.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self.own_lib.OwnWbrSetGasSpring.restype = ctypes.c_int
        self.own_lib.OwnWbrSetJumpProfile.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.own_lib.OwnWbrSetJumpProfile.restype = ctypes.c_int
        self.own_lib.OwnWbrSetStepJump.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.own_lib.OwnWbrSetStepJump.restype = ctypes.c_int
        self.own_lib.OwnWbrSetStepClimb.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.own_lib.OwnWbrSetStepClimb.restype = ctypes.c_int
        self.climb_fold_api = hasattr(self.own_lib, "OwnWbrSetClimbFold")
        if self.climb_fold_api:
            self.own_lib.OwnWbrSetClimbFold.argtypes = [ctypes.c_void_p, ctypes.c_int]
            self.own_lib.OwnWbrSetClimbFold.restype = ctypes.c_int
        elif knee_range == "cad120":
            raise RuntimeError("CAD 短腿模型需要匹配的收腿控制器")
        self.climb_assist_api = hasattr(self.own_lib, "OwnWbrSetClimbAssist")
        if self.climb_assist_api:
            self.own_lib.OwnWbrSetClimbAssist.argtypes = [ctypes.c_void_p, ctypes.c_int]
            self.own_lib.OwnWbrSetClimbAssist.restype = ctypes.c_int
        self.own_lib.OwnWbrGetStepClimbPhase.argtypes = [ctypes.c_void_p]
        self.own_lib.OwnWbrGetStepClimbPhase.restype = ctypes.c_int
        self.own_lib.OwnWbrGetJumpProfile.argtypes = [ctypes.c_void_p]
        self.own_lib.OwnWbrGetJumpProfile.restype = ctypes.c_int
        self.own_lib.OwnWbrGetJumpPreparationStatus.argtypes = [ctypes.c_void_p]
        self.own_lib.OwnWbrGetJumpPreparationStatus.restype = ctypes.c_int
        self.own_lib.OwnWbrGetJumpSpeedLimit.argtypes = [ctypes.c_void_p]
        self.own_lib.OwnWbrGetJumpSpeedLimit.restype = ctypes.c_float
        self.own_lib.OwnWbrGetJumpReverseSpeedLimit.argtypes = [ctypes.c_void_p]
        self.own_lib.OwnWbrGetJumpReverseSpeedLimit.restype = ctypes.c_float
        self.own_lib.OwnWbrSetStage.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.own_lib.OwnWbrSetStage.restype = ctypes.c_int
        self.own_lib.OwnWbrSetLegLength.argtypes = [ctypes.c_void_p, ctypes.c_float]
        self.own_lib.OwnWbrSetLegLength.restype = ctypes.c_int
        self.own_lib.OwnWbrSetDriveMode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.own_lib.OwnWbrSetDriveMode.restype = ctypes.c_int
        self.model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=450.0,
                                correct_link_collisions=correct_link_collisions, terrain=terrain,
                                noslip_iterations=self.noslip_iterations, guide_rollers=guide_rollers,
                                knee_range=knee_range)
        self.guide_roller_info = guide_roller_info(self.model)
        self.knee_range_info = knee_range_info(self.model)
        self.link_collision_info = link_collision_info(self.model)
        self._clearance_rollers = np.array([geom.id for geom in
            (self.model.geom(i) for i in range(self.model.ngeom))
            if geom.name.startswith("guide_roller_geom_")], dtype=int)
        wheel_bodies = {self.model.body(name).id for name in ("lwlink", "rwlink")}
        self._clearance_wheels = []
        for geom in range(self.model.ngeom):
            if (self.model.geom_bodyid[geom] in wheel_bodies and
                    self.model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH):
                mesh = self.model.geom_dataid[geom]
                start = self.model.mesh_vertadr[mesh]
                count = self.model.mesh_vertnum[mesh]
                self._clearance_wheels.append((geom, self.model.mesh_vert[start:start+count].copy()))
        self.profile_pose = self.model.key("home").qpos.copy()
        self.profile_pose[:len(profile["qpos"])] = profile["qpos"]
        self.ground_geoms = ground_geom_ids(self.model)
        self.terrain_info = terrain_info(self.model)
        segments = self.terrain_info["main_course"]
        self.course_bounds = ((min(segment["x_m"][0] for segment in segments),
                               max(segment["x_m"][1] for segment in segments),
                               *self.terrain_info["width_y_m"]) if segments else None)
        if self.terrain_info.get("bounds_m"):
            self.course_bounds = (*self.terrain_info["bounds_m"]["x"],
                                  *self.terrain_info["bounds_m"]["y"])
        self.model.opt.timestep = .001
        self.model.vis.global_.offwidth = 800
        self.model.vis.global_.offheight = 500
        self.model.vis.quality.offsamples = 0
        self.data = mujoco.MjData(self.model)
        self._physics_forward_noslip = None
        self.motor_telemetry = WbrMotorTelemetry(self.model)
        self.recorder = WbrRecorder(output / "recordings")
        self.record_actions = []
        self.motor_state = self.motor_telemetry.snapshot(self.data)
        self._hip_actuators = np.array([self.model.actuator(name).id for name in
            ("ljoint1_actuator", "ljoint4_actuator", "rjoint1_actuator", "rjoint4_actuator")])
        self._hip_joints = self.model.actuator_trnid[self._hip_actuators, 0].copy()
        self._hip_dofs = self.model.jnt_dofadr[self._hip_joints].copy()
        if not np.array_equal(self._hip_actuators, [0, 1, 3, 4]):
            raise RuntimeError(f"WBR 髋关节执行器顺序不符：{self._hip_actuators.tolist()}")
        self._base_ctrlrange = self.model.actuator_ctrlrange.copy()
        self._base_forcerange = self.model.actuator_forcerange.copy()
        self._base_joint_frcrange = self.model.jnt_actfrcrange.copy()
        self.render_model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=450.0, enabled=False,
                                       correct_link_collisions=correct_link_collisions, terrain=terrain,
                                       noslip_iterations=self.noslip_iterations, guide_rollers=guide_rollers,
                                       knee_range=knee_range)
        self.render_model.opt.timestep = .001
        self.render_model.vis.global_.offwidth = 1920
        self.render_model.vis.global_.offheight = 1200
        self.render_model.vis.quality.offsamples = 0
        names = ["imu_quat", "imu_gyro", "imu_accel"]
        names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
        self.indices = np.concatenate([np.arange(self.model.sensor(n).adr[0],
            self.model.sensor(n).adr[0] + self.model.sensor(n).dim[0]) for n in names])
        self.lock = threading.Lock()
        self.input_lock = threading.Lock()
        self.keys = set()
        self.controller = "own"
        self.stage = 3
        self.location = "start"
        self.jump_profile = 0
        self.drive_speed = 1.0
        self.turn_speed = 1.2
        self.raw_turn = 0
        self.climb_assist_requested = False
        self.climb_assist_applied = False
        self.stop_requested = False
        self.leg_length = .20
        self.own_ready = False
        self.entry_height_pending = False
        self.rise_seen_active = False
        self.leg_length_changed = False
        self.controller_changed = False
        self.input_at = 0.0
        self.control_owner = None
        self.jump_until = 0.0
        self.reset_requested = False
        self.reset_in_progress = False
        self.reset_mode = "global"
        self.motor_stop_latched = False
        self.motor_stop_generation = 0
        self.last_reset = None
        self.last_valid_qpos = self.profile_pose.copy()
        self.stop_event = threading.Event()
        self.frame = b""
        self.frame_seq = 0
        self.render_qpos = self.profile_pose.copy()
        self.camera_mode = "follow"
        self.render_quality = "high"
        self.frame_width, self.frame_height = RENDER_QUALITIES[self.render_quality]
        self.state = dict(running=False, status="正在初始化", frame_seq=0,
                          controller="own", controller_id="own", controller_label="我的 ARBATOS",
                          controller_phase="初始化", control_dt_s=.003, command_status={},
                          limited_recovery_supported=True,
                          climb_assist_supported=self.climb_assist_api,
                          climb_assist_requested=False,
                          own_ready=False,
                          stage=3, selected_stage=3, stage_label="VMC + 髋平衡",
                          rise_phase=None, rise_label="等待起身", rise_elapsed_s=None,
                          command_inhibited="起身完成前暂不响应移动和跳跃",
                          hip_model="8009", hip_continuous_nm=HIP_CONTINUOUS_NM,
                          hip_peak_nm=HIP_PEAK_NM, hip_rated_rpm=None,
                          hip_speed_max_rpm=0.0,
                          recent_jump_clearance_mm=None, jump_no_contact_total_ms=None,
                          jump_no_contact_longest_ms=None, jump_com_rise_mm=None,
                          jump_takeoff_vz_mps=None)
        self.started = time.monotonic()
        self.handle = None
        (output / "session.json").write_text(json.dumps(dict(
            controllers={"own": "ARBATOS WheelLegCore", "upstream": "CosmosMount WBR"}, source_revision=REVISION,
            terrain=terrain,
            noslip_iterations=self.noslip_iterations,
            upstream_rise_noslip_iterations=0,
            upstream_spring_derivatives_corrected=True,
            upstream_leg_length_command_m=.21 if correct_link_collisions else .16,
            upstream_first_normal_peak_s=.5 if correct_link_collisions else 0,
            upstream_rise_min_length_m=.20 if correct_link_collisions else .16,
            upstream_start_wait_s=.05 if correct_link_collisions else .5,
            normal_command_acceleration_mps2={"own": 1.0, "upstream": .4},
            reverse_command_deceleration_mps2=2.0, own_coast_deceleration_mps2=.12,
            own_bridge_sha256=hashlib.sha256(
                Path(__file__).with_name("OwnWbrBridge.c").read_bytes()).hexdigest().upper(),
            simulation_source_sha256={name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest().upper()
                for name in ("ViewOwnWbr.py", "WbrTerrain.py", "WbrGasSpring.py", "WbrGuideRollers.py", "WbrKneeRange.py", "WbrLinkCollision.py", "assets/WbrLinkCollision.json", "WbrMotorTelemetry.py", "WbrRecorder.py")},
            model="CosmosMount/wbr_mujoco", own_control_dt_s=.003, upstream_control_dt_s=.001, physics_dt_s=.001,
            physics_step_mode="guarded Euler step2 + full post-forward; other cases use full step",
            hip_motor_model="8009", hip_motor_continuous_nm=HIP_CONTINUOUS_NM,
            hip_motor_peak_nm=HIP_PEAK_NM, hip_motor_rated_rpm=None,
            gas_spring_n_each=450, gas_spring_count=2, gas_spring_source="upstream reference geometry; internal passive force",
            guide_rollers=self.guide_roller_info,
            knee_range=self.knee_range_info,
            link_collision=self.link_collision_info,
            correct_link_collisions=correct_link_collisions,
            input_timeout_s=.35, own_build_command=own_build), ensure_ascii=False, indent=2), encoding="utf-8")
        self.record_metadata = json.loads((output / "session.json").read_text(encoding="utf-8"))
        self.record_metadata.update(motor_model=self.motor_telemetry.metadata(), terrain_info=self.terrain_info,
                                    nq=self.model.nq, nv=self.model.nv,
                                    actuator_names=[self.model.actuator(i).name for i in range(self.model.nu)])
        self.record_metadata["control_sensor_order"] = (
            [f"imu_quat.{axis}" for axis in "wxyz"] +
            [f"imu_gyro.{axis}" for axis in "xyz"] +
            [f"imu_accel.{axis}" for axis in "xyz"] +
            [f"{motor}.{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")])
        self.record_metadata["control_input_nonfinite"] = "null 表示该输入为 NaN 或无穷值"
        self.record_metadata["climb_assist_input"] = (
            "input.climb_assist_requested 是按键请求；control_input.climb_assist 是该控制周期实际送入接口的 G 参数。"
            "动作触发后锁存本次序列，松 G 不会半途卸力；X 可中止并锁住电机输出。")

    def request_record(self, action):
        if action not in ("start", "stop"):
            raise ValueError("invalid record action")
        with self.input_lock:
            if len(self.record_actions) >= 8:
                raise ValueError("too many pending record actions")
            self.record_actions.append(action)

    def chassis_wheel_clearance(self):
        """仅显示车底与最低轮底的垂直差；平地双轮着地时才等于离地间隙。"""
        ids = self._clearance_rollers
        if not len(ids) or not self._clearance_wheels:
            return None
        axis_z = self.data.geom_xmat[ids].reshape(-1, 3, 3)[:, 2, 2]
        size = self.model.geom_size[ids]
        extent = size[:, 0]*np.sqrt(np.maximum(0.0, 1.0-axis_z**2)) + size[:, 1]*np.abs(axis_z)
        chassis_bottom = float(np.min(self.data.geom_xpos[ids, 2]-extent))
        wheel_bottom = min(float(np.min(vertices @ self.data.geom_xmat[geom].reshape(3, 3)[2]) +
                                 self.data.geom_xpos[geom, 2]) for geom, vertices in self._clearance_wheels)
        return chassis_bottom-wheel_bottom

    def record_frame(self, controller, stage, values, out, failed, stopped):
        """真值只进记录器；平衡控制仍只接收原有传感器。"""
        contacts = []
        for index, contact in enumerate(self.data.contact):
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            contacts.append(dict(geom1=self.model.geom(contact.geom1).name,
                geom2=self.model.geom(contact.geom2).name, distance_m=float(contact.dist),
                normal_force_n=float(force[0])))
        return dict(controller=controller, stage=stage, location=self.active_location,
            control_input=self.last_control_input,
            chassis_wheel_clearance_m=self.chassis_wheel_clearance(),
            input=dict(drive_mps=values[0], yaw_radps=values[1], jump=bool(values[2]),
                       stopped=stopped, turn_speed_radps=self.turn_speed,
                       raw_turn=self.raw_turn, leg_length_m=self.leg_length,
                       climb_assist_requested=self.climb_assist_requested,
                       climb_assist=self.climb_assist_applied,
                       drive_speed_requested_mps=self.drive_speed, jump_profile=self.active_jump_profile),
            roll_deg=math.degrees(out[6]), pitch_deg=math.degrees(out[7]), yaw_deg=math.degrees(out[8]),
            estimated_forward_mps=float(out[10]), step_climb_phase=self.step_climb_phase,
            step_climb_mode=self.step_climb_mode,
            jump_phase=int(out[29]), jump_count=int(out[30]), running=not bool(failed),
            rise_phase=int(out[34]) if controller == "own" else None,
            status=failed or self.contact_notice or "running", qpos=self.data.qpos.tolist(),
            qvel=self.data.qvel.tolist(), ctrl=self.data.ctrl.tolist(),
            actuator_force=self.data.actuator_force.tolist(), motor_telemetry=self.motor_state,
            contacts=contacts)

    def command(self, keys, reset=False, controller=None, stage=None, leg_length=None,
                jump_profile=None, drive_speed=None, client_id=None, claim_control=False, stop=False,
                location=None, turn_speed=None, reset_mode="global", emergency_stop=False,
                clear_emergency_stop=False, stop_generation=None):
        with self.input_lock:
            if not isinstance(emergency_stop, bool) or not isinstance(clear_emergency_stop, bool):
                raise ValueError("invalid output stop request")
            if not isinstance(reset, bool):
                raise ValueError("invalid reset")
            if reset_mode not in ("recover", "local", "global"):
                raise ValueError("invalid reset mode")
            # 旧页面的 local 请求也只能重新接管当前姿态，不能再瞬移扶正。
            if reset_mode == "local":
                reset_mode = "recover"
            selected = self.controller if controller is None else controller
            if selected not in ("own", "upstream"):
                raise ValueError("invalid controller")
            if reset and reset_mode == "recover" and selected != "own":
                raise ValueError("本轮受限自起仅支持 ARBATOS 控制器；请按 T 强制复位")
            if clear_emergency_stop and not reset:
                raise ValueError("解除停止必须同时明确请求 R 或 T")
            if client_id is not None and (not isinstance(client_id, str) or
                    not 1 <= len(client_id) <= 64 or
                    any(not (char.isascii() and (char.isalnum() or char == "-")) for char in client_id)):
                raise ValueError("invalid client id")
            if not isinstance(claim_control, bool) or (claim_control and client_id is None):
                raise ValueError("invalid control claim")
            if emergency_stop:
                self.motor_stop_generation += 1
                self.motor_stop_latched = True
                self.keys.clear()
                self.jump_until = 0.0
                self.reset_requested = False
                self.stop_requested = True
                return True
            if self.motor_stop_latched and reset and not clear_emergency_stop:
                raise ValueError("输出已锁住，请通过新版页面 R/T 明确恢复")
            if clear_emergency_stop and (isinstance(stop_generation, bool) or
                    not isinstance(stop_generation, int) or stop_generation != self.motor_stop_generation):
                raise ValueError("停止状态已更新，请重新确认后按 R/T")
            next_location = self.location if location is None else location
            if next_location not in ("start", "large_steps", "step_jump200", "step_climb", "step_auto", "climb200", "slope_course", "cross10", "cross15",
                                     "playground", "stairs", "ramps", "rough", "rmul2026"):
                raise ValueError("invalid location")
            if next_location != "start" and self.terrain != "course":
                raise ValueError("location requires course terrain")
            if next_location != "start" and selected != "own":
                raise ValueError("terrain presets require own controller")
            if next_location in ("step_jump200", "step_climb", "step_auto", "climb200") and not self.correct_link_collisions:
                raise ValueError("step climb requires corrected link collisions")
            next_stage = self.stage if stage is None else stage
            if isinstance(next_stage, bool) or not isinstance(next_stage, int) or next_stage not in range(4):
                raise ValueError("invalid stage")
            if next_location in ("step_jump200", "step_climb", "step_auto", "climb200") and next_stage != 3:
                raise ValueError("step climb requires stage 3")
            if not isinstance(stop, bool):
                raise ValueError("invalid stop")
            if not isinstance(keys, list) or any(not isinstance(key, str) or
                    key not in {"w", "a", "s", "d", " ", "g"} for key in keys):
                raise ValueError("invalid keys")
            next_jump = self.jump_profile if jump_profile is None else jump_profile
            if isinstance(next_jump, bool) or not isinstance(next_jump, int) or next_jump not in (0, 1):
                raise ValueError("invalid jump profile")
            if next_location == "step_jump200" and next_jump != 1:
                raise ValueError("200mm跳台入口需要选择大跳动作")
            next_speed = self.drive_speed if drive_speed is None else drive_speed
            if (isinstance(next_speed, bool) or not isinstance(next_speed, (int, float)) or
                    not math.isfinite(next_speed) or not .25 <= next_speed <= 3.0):
                raise ValueError("invalid drive speed")
            next_turn = self.turn_speed if turn_speed is None else turn_speed
            if (isinstance(next_turn, bool) or not isinstance(next_turn, (int, float)) or
                    not math.isfinite(next_turn) or not .3 <= next_turn <= 3.0):
                raise ValueError("invalid turn speed")
            next_length = self.leg_length
            if leg_length is not None:
                if (isinstance(leg_length, bool) or not isinstance(leg_length, (int, float)) or
                        not math.isfinite(leg_length) or not .18 <= leg_length <= .32):
                    raise ValueError("invalid leg length")
                next_length = float(leg_length)
            if reset and reset_mode == "recover" and (
                    selected != self.controller or next_stage != self.stage or next_location != self.location):
                raise ValueError("自起仅作用于当前控制器、阶段和场地；请先同步当前状态")
            # 所有字段先验证，再一次性提交，保证 400 不会留下半更新状态。
            # 旧标签页的定时请求不能反复切模式，只有实际操作才接管。
            if ((self.control_owner is not None or client_id is not None) and
                    client_id != self.control_owner and not claim_control):
                return False
            if claim_control:
                self.control_owner = client_id
            if reset and clear_emergency_stop:
                self.motor_stop_latched = False
            changed = selected != self.controller
            stage_changed = next_stage != self.stage
            location_changed = next_location != self.location
            length_changed = abs(next_length - self.leg_length) > 1e-6
            self.controller = selected
            self.stage = next_stage
            self.location = next_location
            self.jump_profile = next_jump
            self.drive_speed = float(next_speed)
            self.turn_speed = float(next_turn)
            self.stop_requested = stop
            self.leg_length = next_length
            self.leg_length_changed |= length_changed
            new_keys = set(keys) & {"w", "a", "s", "d", " ", "g"}
            if (not self.climb_assist_api or not self.correct_link_collisions or
                    selected != "own" or self.stage != 3 or
                    not self.own_ready or next_location in ("step_climb", "large_steps", "step_jump200")):
                new_keys.discard("g")
            if selected == "upstream" or self.stage != 3 or not self.own_ready:
                new_keys.discard(" ")
            if next_location == "step_climb":
                new_keys.discard(" ")
            if "g" in new_keys:
                new_keys.discard(" ")
                self.jump_until = 0.0
            if selected == "own" and not self.own_ready:
                new_keys.clear()
            if changed or stage_changed or location_changed:
                self.keys.clear()
                self.jump_until = 0
                self.controller_changed = True
                self.reset_requested = True
                self.reset_mode = "global"
                new_keys.clear()
            now = time.monotonic()
            if " " in new_keys and " " not in self.keys:
                # 保留短按到至少一个 3 ms 控制周期，按住时仍只有一个上升沿。
                self.jump_until = now + .05
            self.keys = new_keys
            self.input_at = now
            self.reset_requested |= stage_changed or reset
            if reset:
                self.reset_mode = "global" if changed or stage_changed or location_changed else reset_mode
                self.keys.clear()
                self.jump_until = 0
            if stop:
                self.keys.clear()
                self.jump_until = 0
            return True

    def inputs(self):
        with self.input_lock:
            controller = self.controller
            stage = self.stage
            jump_profile = self.jump_profile
            drive_speed = self.drive_speed
            turn_speed = self.turn_speed
            leg_length = self.leg_length
            location = self.location
            changed, self.controller_changed = self.controller_changed, False
            length_changed, self.leg_length_changed = self.leg_length_changed, False
            stopped = (self.stop_requested or self.motor_stop_latched or
                       time.monotonic() - self.input_at >= .35)
            if stopped:
                self.keys.clear()
                self.jump_until = 0.0
            keys = self.keys.copy()
            if not stopped and time.monotonic() < self.jump_until:
                keys.add(" ")
            reset, self.reset_requested = self.reset_requested, False
            reset_mode = self.reset_mode
            if reset:
                self.reset_in_progress = True
        if controller == "own" and stage == 3 and location in ("large_steps", "step_jump200") and jump_profile == 1:
            drive_speed = min(drive_speed, max(0., self.own_lib.OwnWbrGetJumpSpeedLimit(self.handle)))
        if controller == "own" and location == "step_climb":
            drive_speed = min(drive_speed, .5)
        drive = drive_speed * (("w" in keys) - ("s" in keys))
        self.raw_turn = ("a" in keys) - ("d" in keys)
        self.climb_assist_requested = "g" in keys and not reset and not self.motor_stop_latched
        # 这里只收集按键；物理线程再按观察器速度决定倒车转向，避免读取位置真值。
        values = [drive, turn_speed * self.raw_turn, float(" " in keys)]
        return controller, stage, leg_length, jump_profile, changed, length_changed, values, reset, stopped, location, reset_mode

    def reset(self, controller, stage, leg_length, jump_profile=0, location="start", reset_mode="global"):
        reset_mode = "recover" if reset_mode == "local" else reset_mode
        recovering = reset_mode == "recover"
        if recovering and controller != "own":
            raise ValueError("本轮受限自起仅支持 ARBATOS 控制器；请按 T 强制复位")
        self._physics_forward_noslip = None
        if recovering and (not np.isfinite(self.data.qpos).all() or
                not np.isfinite(self.data.qvel).all() or any(w.number for w in self.data.warning)):
            raise ValueError("物理状态异常，无法从当前姿态自起；请按 T 强制复位")
        origin = self.data.qpos.copy() if np.isfinite(self.data.qpos).all() else self.last_valid_qpos.copy()
        origin_velocity, origin_time = self.data.qvel.copy(), float(self.data.time)
        if self.handle:
            if self.active_controller == "own":
                self.own_lib.OwnWbrDestroy(self.handle)
            else:
                self.upstream_lib.WbrDestroy(self.handle)
        self.active_controller = controller
        self.step_climb_mode = 0
        self.last_control_input = None
        # 原作者放平起身依赖机身在地面滑动；站稳后再使用场地的防滑求解。
        self.model.opt.noslip_iterations = 0 if controller == "upstream" else self.noslip_iterations
        if controller == "own":
            self.handle = self.own_lib.OwnWbrCreate(self.config)
            fold_ok = (self.own_lib.OwnWbrSetClimbFold(
                self.handle, int(self.knee_range == "cad120"))
                if self.handle and self.climb_fold_api else self.knee_range in ("upstream", "stop180"))
            if not self.handle:
                limits_ok = 0
            elif self.joint_limits_api:
                limits_ok = self.own_lib.OwnWbrSetJointLimits(self.handle, HIP_CONTINUOUS_NM, HIP_PEAK_NM)
            else:
                limits_ok = 0
            stage_ok = self.own_lib.OwnWbrSetStage(self.handle, stage) if self.handle else 0
            length_ok = (self.own_lib.OwnWbrSetLegLength(self.handle, leg_length)
                         if self.handle and stage != 0 else 1)
            rise_ok = self.own_lib.OwnWbrEnableRise(self.handle,
                0 if reset_mode == "global" and (location.startswith("cross") or location == "step_jump200") else 1) if self.handle else 0
            spring_ok = self.own_lib.OwnWbrSetGasSpring(self.handle, 450.0) if self.handle else 0
            core_jump_profile = jump_profile + (2 if self.correct_link_collisions else 0)
            jump_ok = self.own_lib.OwnWbrSetJumpProfile(self.handle, core_jump_profile) if self.handle else 0
            step_jump_ok = (self.own_lib.OwnWbrSetStepJump(self.handle,
                            int(location in ("large_steps", "step_jump200") and self.correct_link_collisions))
                            if self.handle and stage == 3 else 1)
            # 专用入口保留慢速搭台，普通驾驶使用 IMU 碰撞识别；越台大跳单独控制。
            step_climb_mode = (0 if not self.correct_link_collisions or location in ("large_steps", "step_jump200") else
                               1 if location == "step_climb" else 2)
            step_climb_ok = (self.own_lib.OwnWbrSetStepClimb(self.handle, step_climb_mode)
                             if self.handle and stage == 3 else 1)
            recovery_ok = (self.own_lib.OwnWbrRequestRecovery(self.handle)
                           if recovering and self.handle else 1)
            if not (limits_ok and stage_ok and length_ok and rise_ok and spring_ok and
                    jump_ok and step_jump_ok and step_climb_ok and fold_ok and recovery_ok):
                self.own_lib.OwnWbrDestroy(self.handle)
                self.handle = None
                raise RuntimeError("ARBATOS 控制器参数设置失败")
            self.step_climb_mode = step_climb_mode if stage == 3 else 0
            self._set_hip_limits(HIP_PEAK_NM)
        else:
            self.handle = (self.upstream_lib.WbrCreateWithRiseLength(.20)
                           if self.correct_link_collisions else self.upstream_lib.WbrCreate())
            self._set_hip_limits(HIP_PEAK_NM)
        if not self.handle:
            raise RuntimeError("控制核心创建失败")
        if not recovering:
            mujoco.mj_resetData(self.model, self.data)
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
            source_pose = self.profile_pose if location.startswith("cross") or location == "step_jump200" else self.data.qpos
            self.data.qpos[:] = scenario_pose(self.model, source_pose, location)
            mujoco.mj_forward(self.model, self.data)
        # R 仅清除旧电机命令；包括倒地姿态、运动速度和时间在内的物理状态均保留。
        self.data.ctrl[:] = 0
        self.last_valid_qpos = self.data.qpos.copy()
        self.last_reset = dict(mode=reset_mode, origin_qpos=origin.tolist(), qpos=self.data.qpos.tolist(),
            origin_qvel=origin_velocity.tolist(), qvel=self.data.qvel.tolist(),
            origin_time=origin_time, time=float(self.data.time))
        self.recovery_started_at = float(self.data.time) if recovering else None
        self.active_stage = stage
        self.active_jump_profile = jump_profile
        self.active_leg_length = leg_length
        self.active_location = location
        self.step_climb_phase = 0
        self.step_climb_started_at = None
        self.unexpected_contact_since = None
        self.contact_notice = ""
        self.slope_stable_since = None
        self.slope_ready_at = None
        self.own_ready = False if controller == "own" else True
        self.entry_height_pending = (controller == "own" and
            location in ("climb200", "rmul2026") and leg_length >= .30 and not recovering)
        self.rise_seen_active = False
        self.upstream_normal_started = None
        self.upstream_stable_since = None
        self.upstream_ready_at = None

    def _confirm_upstream_ready(self, sensor, out):
        # 起身确认只看控制器估计与传感器，不读 MuJoCo 的位置、速度或接触真值。
        if self.upstream_ready_at is not None:
            return
        length = .21 if self.correct_link_collisions else .16
        stable = (int(out[24]) == 4 and max(abs(out[6]), abs(out[7])) < math.radians(3) and
                  max(abs(sensor[4]), abs(sensor[5]), abs(sensor[6])) < .3 and
                  abs(out[10]) < .08 and max(abs(out[12] - length), abs(out[18] - length)) < .015 and
                  max(abs(out[13]), abs(out[19])) < .15)
        if not stable:
            self.upstream_stable_since = None
        elif self.upstream_stable_since is None:
            self.upstream_stable_since = float(self.data.time)
        elif self.data.time - self.upstream_stable_since >= .25:
            self.upstream_ready_at = float(self.data.time)
            self.model.opt.noslip_iterations = self.noslip_iterations

    def _set_hip_limits(self, peak):
        self.model.actuator_ctrlrange[self._hip_actuators] = (-peak, peak)
        self.model.actuator_forcerange[self._hip_actuators] = (-peak, peak)
        self.model.jnt_actfrcrange[self._hip_joints] = (-peak, peak)

    def _restore_hip_limits(self):
        self.model.actuator_ctrlrange[:] = self._base_ctrlrange
        self.model.actuator_forcerange[:] = self._base_forcerange
        self.model.jnt_actfrcrange[:] = self._base_joint_frcrange

    def _step_physics(self):
        # 停止请求和实际积分共用锁：停止响应发出后，下一步不再施加电机力矩。
        with self.input_lock:
            if self.motor_stop_latched:
                self.data.ctrl[:] = 0
            self._step_physics_locked()

    def _step_physics_locked(self):
        """复用上次完整更新的运动学；保留原传感器时点和数值异常检查。"""
        model, data = self.model, self.data
        bad_pos = data.warning[mujoco.mjtWarning.mjWARN_BADQPOS]
        bad_vel = data.warning[mujoco.mjtWarning.mjWARN_BADQVEL]
        if (model.opt.integrator != mujoco.mjtIntegrator.mjINT_EULER or
                self._physics_forward_noslip is None or
                self._physics_forward_noslip != model.opt.noslip_iterations or
                mujoco.get_mjcb_control() is not None or bad_pos.number or bad_vel.number):
            mujoco.mj_step(model, data)
        else:
            mujoco.mj_checkPos(model, data)
            mujoco.mj_checkVel(model, data)
            # 检查可能重置物理数据，此时必须重建缓存；旧警告也不走快速路径。
            if bad_pos.number or bad_vel.number:
                mujoco.mj_step(model, data)
            else:
                mujoco.mj_step2(model, data)
        mujoco.mj_forward(model, data)
        self._physics_forward_noslip = int(model.opt.noslip_iterations)

    def _has_nonwheel_floor_contact(self):
        wheel_bodies = {self.model.body("lwlink").id, self.model.body("rwlink").id}
        for contact in self.data.contact:
            if contact.geom1 not in self.ground_geoms and contact.geom2 not in self.ground_geoms:
                continue
            other = contact.geom2 if contact.geom1 in self.ground_geoms else contact.geom1
            if int(self.model.geom_bodyid[other]) not in wheel_bodies:
                ground = contact.geom1 if contact.geom1 in self.ground_geoms else contact.geom2
                # 已验证的机身搭台接触不提示；其它接触仍显示，给控制器恢复机会。
                allowed_support = (self.active_controller == "own" and
                    self.active_stage == 3 and 1 <= self.step_climb_phase <= 6 and
                    self.step_climb_started_at is not None and
                    self.data.time - self.step_climb_started_at <= 2.6 and
                    self.model.geom(ground).name == "terrain_large100" and
                    self.model.body(int(self.model.geom_bodyid[other])).name == "base_link")
                if allowed_support:
                    continue
                return True
        return False

    def _wheel_support_surfaces(self):
        """只用于显示实际落脚位置；接触真值不参与控制。"""
        wheels = {self.model.body("lwlink").id: 0, self.model.body("rwlink").id: 1}
        surfaces = [set(), set()]
        for contact in self.data.contact:
            ground = next((geom for geom in (contact.geom1, contact.geom2)
                           if geom in self.ground_geoms), None)
            if ground is None or abs(float(contact.frame[2])) < .5:
                continue
            other = contact.geom2 if contact.geom1 == ground else contact.geom1
            side = wheels.get(int(self.model.geom_bodyid[other]))
            if side is not None:
                surfaces[side].add(self.model.geom(ground).name)
        return [sorted(side) for side in surfaces]

    def physics(self):
        sensor = (ctypes.c_float * 28)()
        own_command, upstream_command = (ctypes.c_float * 3)(), (ctypes.c_float * 7)()
        out = (ctypes.c_float * 40)()
        phase_names = ["就绪", "下蹲蓄力", "蹬地", "收腿卸载", "落地缓冲", "恢复"]
        fields = ["sim_time", "pitch_deg", "roll_deg", "yaw_deg", "v_mps", "command_v", "command_yaw", "jump_phase", "jump_count"]
        try:
            with self.input_lock:
                initial_controller, initial_stage, initial_length = self.controller, self.stage, self.leg_length
                initial_jump = self.jump_profile
                initial_location = self.location
            self.reset(initial_controller, initial_stage, initial_length, initial_jump, initial_location)
            tick, failed = 0, None
            physics_paused = False
            hip_limit = HIP_PEAK_NM
            x_ref = yaw_ref = 0.0
            reference_v = 0.0
            turn_direction = 1
            upstream_drive_started = False
            command_status = {"move": False, "spin": False, "jump": False}
            jump_count = 0
            jump_active = False
            jump_diagnostics = JumpDiagnostics(self.model, self.profile["radius_m"])
            recent_jump = JumpDiagnostics.empty()
            deadline = time.monotonic()
            rate_at, rate_sim, real_rate = deadline, 0.0, 1.0
            last_record_event = None
            with (self.output / "interactive.csv").open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fields)
                writer.writeheader()
                while not self.stop_event.is_set():
                    controller, stage, leg_length, jump_profile, changed, length_changed, values, reset, stopped, location, reset_mode = self.inputs()
                    physics_stepped = False
                    with self.input_lock:
                        record_actions, self.record_actions = self.record_actions, []
                        motor_stop = self.motor_stop_latched
                    for action in record_actions:
                        if action == "start":
                            self.recorder.start(dict(self.record_metadata, controller=controller, stage=stage,
                                location=location, leg_length_m=leg_length, drive_speed_mps=self.drive_speed,
                                turn_speed_radps=self.turn_speed), float(self.data.time))
                        else:
                            self.recorder.stop(float(self.data.time))
                    reset = (reset or changed) and not motor_stop
                    if reset:
                        self.recorder.event("recovery_start" if reset_mode == "recover" else "scene_reset", float(self.data.time),
                            dict(controller=controller, stage=stage, location=location, mode=reset_mode))
                        try:
                            self.reset(controller, stage, leg_length, jump_profile, location, reset_mode)
                        except (ValueError, RuntimeError) as exc:
                            failed = f"复位未完成：{exc}"
                            self.data.ctrl[:] = 0
                            with self.lock:
                                self.state.update(running=False, status=failed)
                            with self.input_lock:
                                self.reset_in_progress = False
                            continue
                        out[:] = [0.0] * len(out)
                        tick, failed = 0, None
                        physics_paused = False
                        x_ref = yaw_ref = 0.0
                        reference_v = 0.0
                        turn_direction = 1
                        upstream_drive_started = False
                        command_status = {"move": False, "spin": False, "jump": False}
                        jump_count = 0
                        jump_active = False
                        jump_diagnostics.reset()
                        recent_jump = JumpDiagnostics.empty()
                        rate_at, rate_sim = time.monotonic(), float(self.data.time)
                        deadline = time.monotonic()
                        with self.lock:
                            self.state = dict(self.state, controller=controller,
                                running=True, physics_running=True, sim_time=float(self.data.time),
                                last_reset=self.last_reset,
                                controller_id=controller,
                                controller_label="原作者控制 + 模型起身适配" if controller == "upstream" else "我的 ARBATOS",
                                controller_phase="原作者起身" if controller == "upstream" else "起身中",
                                control_dt_s=.001 if controller == "upstream" else .003,
                                own_ready=self.own_ready if controller == "own" else None,
                                upstream_ready=False,
                                rise_phase=(11 if reset_mode == "recover" else 1) if controller == "own" else None,
                                rise_label=("从当前姿态尝试自起" if reset_mode == "recover" else "保持初始姿态") if controller == "own" else "原作者起身中",
                                rise_elapsed_s=0.0 if controller == "own" else None,
                                command_inhibited="起身完成前暂不响应移动和跳跃" if controller == "own" else None,
                                stage=stage if controller == "own" else None, selected_stage=stage,
                                stage_label=(["关节抱姿 + 轮 LQR", "腿位置环 + 轮 LQR",
                                    "VMC 腿高 + 轮 LQR", "VMC + 髋平衡"][stage]
                                    if controller == "own" else "原作者状态机"),
                                leg_length_target_m=leg_length if controller != "own" or stage != 0 else None,
                                status=("从当前姿态尝试自起" if reset_mode == "recover" else "已返回场地入口"),
                                command_status=command_status.copy())
                        with self.input_lock:
                            self.reset_in_progress = False
                    elif length_changed and self.active_controller == "own" and self.active_stage != 0:
                        if not self.own_lib.OwnWbrSetLegLength(self.handle, leg_length):
                            failed = "目标腿长设置失败"
                        else:
                            self.active_leg_length = leg_length
                    self.climb_assist_applied = False
                    if motor_stop:
                        failed = ("已锁住六路电机输出；确认周围无障碍后按 R 受限自起或 T 回入口"
                                  if controller == "own" else
                                  "已锁住六路电机输出；原作者控制器不支持本轮受限自起，请按 T 回入口")
                        self.data.ctrl[:] = 0
                        with self.input_lock:
                            self.reset_in_progress = False
                    turn_direction = steering_direction(float(out[10]), values[0], turn_direction)
                    values[1] *= turn_direction
                    if self.handle is None:
                        failed = failed or "控制器未创建，按 T 强制复位重试"
                        self.data.ctrl[:] = 0
                        command_status = {"move": False, "spin": False, "jump": False}
                    elif controller == "own":
                        if jump_profile != self.active_jump_profile:
                            profile = jump_profile + (2 if self.correct_link_collisions else 0)
                            if not self.own_lib.OwnWbrSetJumpProfile(self.handle, profile):
                                failed = "跳跃动作设置失败"
                            else:
                                self.active_jump_profile = jump_profile
                        if not self.own_lib.OwnWbrSetDriveMode(self.handle, 0 if stopped else 1):
                            failed = "行驶模式设置失败"
                        if self.climb_assist_api:
                            climb_assist = (self.climb_assist_requested and self.own_ready and
                                            stage == 3 and self.step_climb_mode == 2)
                            if not self.own_lib.OwnWbrSetClimbAssist(self.handle, int(climb_assist)):
                                failed = "上台准备设置失败"
                            else:
                                self.climb_assist_applied = bool(climb_assist)
                        applied_values = values if self.own_ready else [0.0, 0.0, 0.0]
                        own_command[:] = applied_values
                        command_status = {"move": abs(applied_values[0]) > 0,
                                          "spin": abs(applied_values[1]) > 0,
                                          "jump": applied_values[2] > 0}
                    else:
                        upstream_ready = self.upstream_ready_at is not None
                        target_v, yaw_rate = values[:2] if upstream_ready else (0.0, 0.0)
                        # 松键随估计速度滑行；反向键快速制动。失焦或失联仍显式停稳。
                        measured_v = float(out[10])
                        if stopped or not upstream_ready or (target_v == 0.0 and abs(measured_v) < .05):
                            upstream_drive_started = False
                        elif target_v != 0.0:
                            upstream_drive_started = True
                        coast = upstream_drive_started and not stopped and target_v == 0.0
                        reversing = target_v * measured_v < -.01
                        if coast:
                            reference_v = measured_v
                            x_ref = float(out[9])
                        else:
                            acceleration = 2.0 if stopped or reversing else .4
                            reference_v += float(np.clip(target_v - reference_v,
                                -acceleration * .001, acceleration * .001))
                        v_cmd = reference_v if upstream_ready else 0.0
                        move_enabled = self.data.time >= (.05 if self.correct_link_collisions else .5)
                        if out[31] > .5:
                            if not coast:
                                x_ref += v_cmd * .001
                            yaw_ref += yaw_rate * .001
                        else:
                            x_ref, yaw_ref = float(out[9]), float(out[8])
                        # 原目标 .16 m 低于本 CAD 机构可达下限，会持续压在膝限位上。
                        reference_length = leg_length if upstream_ready else (.21 if self.correct_link_collisions else .16)
                        upstream_command[:] = [x_ref, v_cmd, reference_length, yaw_ref, yaw_rate,
                                               float(move_enabled), 0.0]
                        command_status = {"move": abs(v_cmd) > 0, "spin": abs(yaw_rate) > 0, "jump": False}
                    if not failed:
                        control_period = 3 if controller == "own" else 1
                        if tick % control_period == 0:
                            sensor[:] = self.data.sensordata[self.indices]
                            # 保存送进控制器的原始输入及时点，避免回放时把重算加速度当成当时测量。
                            self.last_control_input = dict(time_s=float(self.data.time),
                                climb_assist=self.climb_assist_applied,
                                sensor=[value if math.isfinite(value) else None for value in sensor],
                                command=[value if math.isfinite(value) else None for value in
                                    (own_command if controller == "own" else upstream_command)])
                            if controller == "own":
                                error = self.own_lib.OwnWbrCommandStep(self.handle, sensor, own_command, out)
                            else:
                                error = self.upstream_lib.WbrStep(self.handle, sensor, upstream_command, out)
                            if error:
                                climb_fault = {10: "关节峰值用力时间达到上限",
                                    11: "动作超时", 12: "车身姿态偏离过大",
                                    13: "送轮时轮速估计过大"}.get(error)
                                if controller == "own" and climb_fault:
                                    failed = f"上台已停止：{climb_fault}；六路输出归零，按 T 返回入口"
                                else:
                                    failed = (f"控制计算失败 {error}，按 R 尝试受限自起" if controller == "own" else
                                              f"控制计算失败 {error}，原作者控制器不支持本轮受限自起，请按 T 强制复位")
                            elif controller == "own":
                                self.step_climb_phase = self.own_lib.OwnWbrGetStepClimbPhase(self.handle)
                                if self.step_climb_phase == 1 and self.step_climb_started_at is None:
                                    self.step_climb_started_at = float(self.data.time)
                                elif self.step_climb_phase == 0:
                                    self.step_climb_started_at = None
                                rise_phase = int(out[34])
                                if rise_phase < 0:
                                    reason = {-15: "髋关节超速", -16: "扫腿行程达到上限",
                                        -17: "车轮超速", -18: "尚未落稳或姿态没有继续改善",
                                        -19: "单次翻身超时", -20: "机身转动过快"}.get(rise_phase, "未达到站立条件")
                                    failed = f"自起已停止：{reason}；六路输出归零"
                                elif rise_phase > 0:
                                    self.rise_seen_active = True
                                    self.own_ready = False
                                elif self.rise_seen_active:
                                    self.own_ready = True
                                    if self.entry_height_pending:
                                        # 专用高台入口先达到所选腿高再接受驾驶，不用接触或地形真值判定。
                                        height_ready = (min(out[12], out[18]) > min(.30, leg_length - .012) and
                                                        abs(out[7]) < .0524 and abs(sensor[5]) < .35)
                                        self.entry_height_pending = not height_ready
                                        self.own_ready = height_ready
                                elif (self.active_location.startswith("cross") or self.active_location == "step_jump200") and not self.own_ready:
                                    roll_limit = 20 if self.active_location.startswith("cross") else 5
                                    stable = (abs(out[6]) < math.radians(roll_limit) and abs(out[7]) < math.radians(5) and
                                              max(abs(sensor[4]), abs(sensor[5]), abs(sensor[6])) < .35 and
                                              abs(out[10]) < .08 and max(abs(out[13]), abs(out[19])) < .15)
                                    if not stable:
                                        self.slope_stable_since = None
                                    elif self.slope_stable_since is None:
                                        self.slope_stable_since = float(self.data.time)
                                    elif self.data.time - self.slope_stable_since >= .5:
                                        self.own_ready = True
                                        self.slope_ready_at = float(self.data.time)
                            else:
                                self._confirm_upstream_ready(sensor, out)
                            fall_check = (self.own_ready if controller == "own" else
                                self.recovery_started_at is None or self.upstream_ready_at is not None)
                            if not failed and fall_check and max(abs(out[6]), abs(out[7])) > math.radians(60):
                                failed = ("已倒地，按 R 尝试受限自起" if controller == "own" else
                                          "已倒地，原作者控制器不支持本轮受限自起，请按 T 强制复位")
                            if (not failed and controller == "upstream" and self.recovery_started_at is not None and
                                    self.upstream_ready_at is None and self.data.time-self.recovery_started_at > 8):
                                failed = "原作者控制器起身超时，请按 T 强制复位"
                            if failed:
                                self.data.ctrl[:] = 0
                            else:
                                torques = np.asarray(out[:6], dtype=float).copy()
                                if controller == "own":
                                    hip_limit = min(HIP_PEAK_NM, float(out[31])) if self.joint_limits_api else HIP_CONTINUOUS_NM
                                else:
                                    if int(out[24]) == 4 and self.upstream_normal_started is None:
                                        self.upstream_normal_started = float(self.data.time)
                                    # 刚进入平衡状态仍在完成起身，保留短时动作峰值。
                                    startup_peak = (self.correct_link_collisions and
                                        self.upstream_normal_started is not None and
                                        self.data.time - self.upstream_normal_started < .5)
                                    hip_limit = HIP_CONTINUOUS_NM if int(out[24]) == 4 and not startup_peak else HIP_PEAK_NM
                                for actuator_id in self._hip_actuators:
                                    torques[actuator_id] = np.clip(torques[actuator_id], -hip_limit, hip_limit)
                                self.data.ctrl[:] = torques
                        if not failed:
                            self._step_physics()
                            physics_stepped = True
                            if controller == "own":
                                current_count = int(out[30])
                                if current_count > jump_count:
                                    jump_count = current_count
                                    jump_active = True
                                    jump_diagnostics.reset()
                                if jump_active:
                                    jump_diagnostics.observe(self.data)
                                    if int(out[29]) == 0:
                                        recent_jump = jump_diagnostics.summary()
                                        jump_active = False
                            if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                                    or any(w.number for w in self.data.warning)):
                                failed = "物理计算异常，按 T 强制复位"
                                physics_paused = True
                                self.data.ctrl[:] = 0
                            # 场地地面连续覆盖矩形区域；越界检查无需每毫秒重算所有坡面高度。
                            elif self.course_bounds is not None and not (
                                    self.course_bounds[0] - 1e-9 <= self.data.qpos[0] <= self.course_bounds[1] + 1e-9 and
                                    self.course_bounds[2] - 1e-9 <= self.data.qpos[1] <= self.course_bounds[3] + 1e-9):
                                failed = "已驶出场地边界，按 T 返回测试位置"
                                self.data.ctrl[:] = 0
                            else:
                                contact = ((controller == "own" and self.own_ready or
                                    controller == "upstream" and self.upstream_ready_at is not None) and
                                    self._has_nonwheel_floor_contact())
                                if contact:
                                    if self.unexpected_contact_since is None:
                                        self.unexpected_contact_since = float(self.data.time)
                                    elapsed = float(self.data.time) - self.unexpected_contact_since
                                    self.contact_notice = f"机身或腿部碰撞，控制器正在调整（{elapsed:.2f} s）"
                                    # 接触时长不能证明已经倒地；保留真实碰撞和姿态停机判据。
                                    # 轻微擦碰继续输出平衡力矩，避免先停电机再人为制造失衡。
                                else:
                                    self.unexpected_contact_since = None
                                    self.contact_notice = ""
                    if failed and not physics_paused and not physics_stepped:
                        # 控制失效后让刚体继续落地；电机归零，不把倒地画面冻在半空。
                        self.data.ctrl[:] = 0
                        self._step_physics()
                        if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                                or any(w.number for w in self.data.warning)):
                            physics_paused = True
                            failed = "物理计算异常，仿真暂停；按 T 强制复位返回入口"
                    if not physics_paused:
                        self.last_valid_qpos = self.data.qpos.copy()
                    if failed and not physics_paused:
                        sensor[:] = self.data.sensordata[self.indices]
                        w, x, y, z = sensor[:4]
                        out[6] = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
                        out[7] = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
                        out[8] = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                    if tick % 10 == 0 and np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all():
                        self.motor_state = self.motor_telemetry.snapshot(self.data)
                        event_state = (failed, bool(self.contact_notice), self.step_climb_phase, int(out[29]), stopped)
                        if event_state != last_record_event:
                            self.recorder.event("control_state", float(self.data.time), dict(
                                failure=failed, contact=bool(self.contact_notice), step_phase=self.step_climb_phase,
                                jump_phase=int(out[29]), input_stopped=stopped))
                            last_record_event = event_state
                        if self.recorder.status()["active"]:
                            self.recorder.sample(float(self.data.time), self.record_frame(
                                controller, stage, values, out, failed, stopped))
                    now = time.monotonic()
                    if now - rate_at > 1:
                        real_rate = (self.data.time - rate_sim) / (now - rate_at)
                        rate_at, rate_sim = now, self.data.time
                        stream.flush()
                    if tick % 30 == 0:
                        # 真值仅用于界面和日志；绝不传给 OwnWbrCommandStep。
                        jac = np.zeros((3, self.model.nv))
                        mujoco.mj_jacBody(self.model, self.data, jac, None, self.model.body("base_link").id)
                        velocity = jac @ self.data.qvel
                        forward = velocity[0] * math.cos(out[8]) + velocity[1] * math.sin(out[8])
                        phase = int(out[29])
                        rise_phase = int(out[34]) if controller == "own" else None
                        rise_labels = {1: "保持初始姿态", 2: "抬升中", 3: "平衡稳定观察", 4: "站立确认",
                            11: "等待落稳并判断姿态", 12: "慢速建立支点", 13: "限行程尝试翻身", 14: "恢复站姿", 15: "自起稳定确认"}
                        rise_failure_labels = {-2: "起身目标计算失败", -3: "起身抬升超时", -4: "站立稳定性未达标",
                            -11: "当前姿态或传感器不支持自起", -12: "自起关节目标计算失败", -13: "翻身超时", -14: "自起稳定超时",
                            -15: "髋关节超速，输出已停止", -16: "扫腿行程到限，输出已停止",
                            -17: "车轮超速，输出已停止", -18: "落稳或姿态进展不足，输出已停止",
                            -19: "单次翻身超时，输出已停止", -20: "机身转动过快，输出已停止"}
                        rise_label = (rise_failure_labels.get(rise_phase, "起身失败") if rise_phase is not None and rise_phase < 0 else
                                      rise_labels.get(rise_phase, f"起身阶段 {rise_phase}") if rise_phase is not None and rise_phase > 0 else
                                      "抬高到越台站姿" if controller == "own" and self.entry_height_pending else
                                      "起身完成" if controller == "own" and self.own_ready else
                                      "等待起身启动" if controller == "own" else
                                      "原作者起身中" if self.upstream_ready_at is None else "起身完成")
                        hip_speed_rpm = float(np.max(np.abs(self.data.qvel[self._hip_dofs])) * 60.0 / (2.0 * math.pi))
                        actual_jump_profile = self.own_lib.OwnWbrGetJumpProfile(self.handle) if controller == "own" else None
                        jump_prepare_status = self.own_lib.OwnWbrGetJumpPreparationStatus(self.handle) if controller == "own" else None
                        jump_speed_limit = self.own_lib.OwnWbrGetJumpSpeedLimit(self.handle) if controller == "own" else None
                        jump_reverse_limit = self.own_lib.OwnWbrGetJumpReverseSpeedLimit(self.handle) if controller == "own" else None
                        jump_notice = "原作者控制没有跳跃映射"
                        if controller == "own":
                            if phase > 0:
                                jump_notice = ("准备大跳：调整腿高与车速" if actual_jump_profile == 3 and phase == 1 else
                                               "大跳执行中" if actual_jump_profile in (1, 3) else "小跳执行中")
                            else:
                                jump_notice = (f"越台大跳 · 调试中 · 起跳速度目标 ±{jump_speed_limit:.2f} m/s" if self.active_jump_profile == 1 else
                                               f"小跳目标 100 mm · 前进保留至 {jump_speed_limit:.0f}，后退降至 {jump_reverse_limit:.0f} m/s")
                            if self.active_location == "large_steps" and actual_jump_profile == 3:
                                jump_notice = ("越台大跳执行中" if phase > 0 else
                                    f"100mm旧场景当前未复验 · 当前控制器速度上限 {jump_speed_limit:.2f} m/s")
                            if self.active_location == "step_jump200":
                                jump_notice = ("200mm带速试验执行中" if phase > 0 else
                                    f"200mm带速试验 · 当前控制器速度上限 {jump_speed_limit:.2f} m/s · 空格起跳")
                            if self.active_location == "step_climb":
                                jump_notice = "磕台使用搭台支撑、收腿和回站，空格暂不触发跳跃。"
                            if jump_prepare_status == -1:
                                jump_notice = "预蹲未就绪，本次跳跃取消；保持平衡，可松开空格后重试。"
                        state = dict(running=not bool(failed), sim_time=float(self.data.time),
                            physics_running=not physics_paused, last_reset=self.last_reset,
                            terrain=self.terrain, terrain_info=self.terrain_info, location=self.active_location,
                            guide_rollers=self.guide_roller_info,
                            knee_range=self.knee_range_info,
                            link_collision=self.link_collision_info,
                            chassis_wheel_clearance_m=self.chassis_wheel_clearance(),
                            control_owner=self.control_owner, drive_speed_requested_mps=self.drive_speed,
                            turn_speed_requested_radps=self.turn_speed, yaw_rate_radps=float(sensor[6]),
                            steering_reverse=turn_direction < 0,
                            motor_stop_latched=motor_stop,
                            motor_stop_supported=True, limited_recovery_supported=True,
                            climb_assist_supported=self.climb_assist_api,
                            climb_assist_requested=self.climb_assist_requested,
                            climb_assist_applied=self.climb_assist_applied,
                            motor_stop_generation=self.motor_stop_generation,
                            motor_telemetry=self.motor_state,
                            contact_notice=self.contact_notice,
                            step_climb_phase=self.step_climb_phase,
                            step_climb_mode=self.step_climb_mode,
                            step_climb_available=self.correct_link_collisions,
                            step_climb_label={0: ("抬高腿部，等待前进触台" if self.active_location == "step_climb" else
                                                  "普通驾驶，自动识别触台"), 1: "机身搭台", 2: "保持机身支撑",
                                3: "收腿抬轮", 4: "前送轮子", 5: "恢复腿高", 6: "站起",
                                7: "回站驻车，松 W 后再按可继续", 8: "动作结束，可继续驾驶"}
                                .get(self.step_climb_phase, "状态异常"),
                            noslip_iterations=int(self.model.opt.noslip_iterations),
                            course_x_m=float(self.data.xpos[self.model.body("base_link").id, 0]),
                            course_y_m=float(self.data.xpos[self.model.body("base_link").id, 1]),
                            wheel_support_surfaces=self._wheel_support_surfaces(),
                            controller=controller, controller_id=controller,
                            controller_label="原作者控制 + 模型起身适配" if controller == "upstream" else "我的 ARBATOS",
                            controller_phase=("原作者起身" if controller == "upstream" and self.upstream_ready_at is None else
                                              ({0: "放松", 1: "恢复", 2: "放平", 3: "姿态调整",
                                                4: "平衡运行", 5: "腾空", 6: "旋转", 7: "上台阶", 8: "跳跃"}
                                               .get(int(out[24]), "未知状态")) if controller == "upstream" else
                                              "起身中" if not self.own_ready else
                                              "磕台动作中" if 1 <= self.step_climb_phase <= 6 else "平衡运行"),
                            control_dt_s=.001 if controller == "upstream" else .003,
                            command_status=command_status.copy(),
                            stage=self.active_stage if controller == "own" else None,
                            selected_stage=stage,
                            jump_profile=self.active_jump_profile if controller == "own" else None,
                            jump_notice=jump_notice, actual_jump_profile=actual_jump_profile,
                            jump_prepare_status=jump_prepare_status,
                            jump_speed_limit_mps=jump_speed_limit,
                            jump_reverse_speed_limit_mps=jump_reverse_limit,
                            stage_label=((["关节抱姿 + 轮 LQR", "腿位置环 + 轮 LQR",
                                          "VMC 腿高 + 轮 LQR", "VMC + 髋平衡"][self.active_stage])
                                         if controller == "own" else "原作者状态机"),
                            leg_length_target_m=leg_length if controller != "own" or self.active_stage != 0 else None,
                            leg_length_applied_m=float(out[33]) if controller == "own" and self.active_stage != 0 else
                                reference_length if controller == "upstream" else None,
                            drive_mode="磕台动作中" if 1 <= self.step_climb_phase <= 6 else
                                "安全停稳" if stopped else "松键滑行 / 反向制动",
                            input_stopped=stopped,
                            joint_peak_limit_nm=(min(HIP_PEAK_NM, float(out[31])) if self.joint_limits_api else
                                HIP_CONTINUOUS_NM) if controller == "own" else
                                hip_limit,
                            rise_phase=rise_phase, rise_label=("直立姿态已确认" if self.own_ready else "直立姿态确认中")
                                if (self.active_location.startswith("cross") or self.active_location == "step_jump200") and self.recovery_started_at is None else rise_label,
                            rise_elapsed_s=(self.slope_ready_at if self.slope_ready_at is not None else float(self.data.time))
                                if (self.active_location.startswith("cross") or self.active_location == "step_jump200") and self.recovery_started_at is None else float(out[35]) if controller == "own" else
                                self.upstream_ready_at if self.upstream_ready_at is not None else float(self.data.time),
                            own_ready=self.own_ready if controller == "own" else None,
                            upstream_ready=self.upstream_ready_at is not None if controller == "upstream" else None,
                            upstream_ready_at_s=self.upstream_ready_at if controller == "upstream" else None,
                            command_inhibited=("起身失败，按 R 重试" if controller == "own" and rise_phase is not None and rise_phase < 0 else
                                               "姿态确认前暂不响应移动和跳跃" if controller == "own" and not self.own_ready else
                                               "原作者起身稳定确认中" if controller == "upstream" and self.upstream_ready_at is None else None),
                            hip_model="8009", hip_continuous_nm=HIP_CONTINUOUS_NM,
                            hip_peak_nm=HIP_PEAK_NM, hip_rated_rpm=None,
                            gas_spring_n_each=450.0, gas_spring_count=2,
                            correct_link_collisions=self.correct_link_collisions,
                            hip_speed_max_rpm=hip_speed_rpm,
                            recent_jump_clearance_mm=recent_jump["clearance_mm"],
                            recent_jump_local_clearance_mm=recent_jump["local_clearance_mm"],
                            jump_no_contact_total_ms=recent_jump["no_contact_total_ms"],
                            jump_no_contact_longest_ms=recent_jump["no_contact_longest_ms"],
                            jump_com_rise_mm=recent_jump["com_rise_mm"],
                            jump_takeoff_vz_mps=recent_jump["takeoff_vz_mps"],
                            real_time_factor=real_rate, pitch_deg=math.degrees(out[7]),
                            roll_deg=math.degrees(out[6]), yaw_deg=math.degrees(out[8]),
                            v_mps=float(forward), command_v=values[0], command_yaw=values[1],
                            leg_m=.5*(out[12]+out[18]), left_leg_m=float(out[12]), right_leg_m=float(out[18]),
                            jump_phase=(phase_names[phase] if controller == "own" and 0 <= phase < 6 else "无映射" if controller == "upstream" else "未知"),
                            jump_count=int(out[30]), status=failed or
                                ("正在起身" if controller == "own" and not self.own_ready else
                                 "磕台动作中" if 1 <= self.step_climb_phase <= 6 else "平衡控制运行中"),
                            wheel_z_m=float(.5*(self.data.body("lwlink").xpos[2]+self.data.body("rwlink").xpos[2])))
                        with self.lock:
                            self.state = state
                            self.render_qpos = self.data.qpos.copy()
                        writer.writerow({k:state[k] for k in fields})
                    tick += 1
                    deadline += .001
                    remaining = deadline - time.monotonic()
                    if remaining > 0:
                        self.stop_event.wait(remaining)
                    elif remaining < -.1:
                        # 机器忙时显示真实倍率，不累积大量补帧后突然加速。
                        deadline = time.monotonic()
        except Exception as exc:
            with self.lock:
                self.state.update(running=False, status=f"仿真异常：{exc}")
            raise
        finally:
            self.recorder.stop(float(self.data.time))
            self.recorder.wait(5.0)
            if self.handle:
                if self.active_controller == "own":
                    self.own_lib.OwnWbrDestroy(self.handle)
                else:
                    self.upstream_lib.WbrDestroy(self.handle)
                self.handle = None

    def set_camera(self, mode):
        if mode not in ("follow", "oblique", "overview"):
            raise ValueError("invalid camera mode")
        with self.lock:
            self.camera_mode = mode

    def set_quality(self, quality):
        if quality not in RENDER_QUALITIES:
            raise ValueError("invalid render quality")
        with self.lock:
            self.render_quality = quality

    def _update_camera(self, data, camera, mode):
        ground_z = ground_height(self.render_model, float(data.qpos[0]), float(data.qpos[1]))
        height = .26 + (ground_z if math.isfinite(ground_z) else 0.0)
        if mode == "follow":
            w, x, y, z = data.qpos[3:7]
            yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
            # 实际渲染核对：azimuth=0 位于 -X，看向 +X；只跟偏航，保持世界竖直。
            camera.distance, camera.azimuth, camera.elevation = 2.4, math.degrees(yaw), -20
            camera.lookat[:] = [data.qpos[0] + .55 * math.cos(yaw),
                                data.qpos[1] + .55 * math.sin(yaw), data.qpos[2] + .12]
        elif mode == "overview":
            zone = next((zone for zone in self.terrain_info.get("zones", [])
                         if zone["location"] == self.active_location), None)
            if zone is not None:
                camera.lookat[:] = [(zone["x_m"][0]+zone["x_m"][1])/2,
                                    (zone["y_m"][0]+zone["y_m"][1])/2, 0]
                camera.distance = 1.15*max(zone["x_m"][1]-zone["x_m"][0],
                                           zone["y_m"][1]-zone["y_m"][0])
            else:
                camera.lookat[:] = [float(data.qpos[0]), float(data.qpos[1]), 0]
                camera.distance = 12.0
            camera.azimuth, camera.elevation = 130, -55
        else:
            camera.distance, camera.azimuth, camera.elevation = (2.6 if self.terrain == "course" else 1.45), 130, -22
            camera.lookat[:] = [data.qpos[0] + (.45 if self.terrain == "course" else 0.0),
                                data.qpos[1], height]

    def render(self):
        # OpenGL 上下文和渲染数据只由本线程使用，物理积分不等待浏览器取图。
        data = mujoco.MjData(self.render_model)
        camera = mujoco.MjvCamera()
        renderer = None
        render_size = None
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                with self.lock:
                    data.qpos[:] = self.render_qpos
                    camera_mode = self.camera_mode
                    quality = self.render_quality
                width, height = RENDER_QUALITIES[quality]
                if render_size != (width, height):
                    if renderer is not None:
                        renderer.close()
                    renderer = mujoco.Renderer(self.render_model, height=height, width=width)
                    render_size = (width, height)
                mujoco.mj_forward(self.render_model, data)
                self._update_camera(data, camera, camera_mode)
                renderer.update_scene(data, camera=camera)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                buffer = io.BytesIO()
                Image.fromarray(renderer.render()).save(buffer, "JPEG", quality=93)
                with self.lock:
                    self.frame = buffer.getvalue()
                    self.frame_seq += 1
                    self.frame_width, self.frame_height = width, height
                self.stop_event.wait(max(0, .05 - (time.monotonic()-started)))
        except Exception as exc:
            with self.lock:
                self.state.update(running=False, status=f"渲染异常：{exc}")
            raise
        finally:
            if renderer is not None:
                renderer.close()


def make_handler(simulation):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, code, content, kind, frame_seq=None):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            if frame_seq is not None:
                self.send_header("X-Frame-Seq", str(frame_seq))
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self.send(200, Path(__file__).with_name("OwnWbrViewer.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/state":
                with simulation.input_lock:
                    reset_pending = simulation.reset_requested or simulation.reset_in_progress
                    motor_stop_latched = simulation.motor_stop_latched
                    motor_stop_generation = simulation.motor_stop_generation
                with simulation.lock:
                    state = dict(simulation.state, frame_seq=simulation.frame_seq,
                                 reset_pending=reset_pending,
                                 motor_stop_latched=motor_stop_latched,
                                 motor_stop_generation=motor_stop_generation,
                                 control_owner=simulation.control_owner, camera_mode=simulation.camera_mode,
                                 render_quality=simulation.render_quality, frame_width=simulation.frame_width,
                                 frame_height=simulation.frame_height, recorder=simulation.recorder.status())
                self.send(200, json.dumps(state, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")
            elif path.startswith("/recordings/"):
                try:
                    recording = simulation.recorder.downloadPath(unquote(path.removeprefix("/recordings/")))
                except (FileNotFoundError, ValueError):
                    self.send(404, b"Recording not ready", "text/plain")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header("Content-Length", str(recording.stat().st_size))
                self.send_header("Content-Disposition", f'attachment; filename="{recording.name}"')
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    with recording.open("rb") as source:
                        while block := source.read(65536):
                            self.wfile.write(block)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            elif path == "/frame.jpg":
                with simulation.lock:
                    frame = simulation.frame
                    seq = simulation.frame_seq
                self.send(200 if frame else 503, frame, "image/jpeg", seq)
            else:
                self.send(404, b"Not found", "text/plain")

        def do_POST(self):
            origin = self.headers.get("Origin")
            if origin and origin != f"http://{self.headers.get('Host')}":
                self.send(403, b"Same origin required", "text/plain")
                return
            if self.path not in ("/command", "/camera", "/quality", "/record"):
                self.send(404, b"Not found", "text/plain")
                return
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 1024:
                    raise ValueError("invalid size")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError("invalid body")
                if self.path == "/camera":
                    if set(body) != {"mode"} or not isinstance(body["mode"], str):
                        raise ValueError("invalid camera request")
                    simulation.set_camera(body["mode"])
                    self.send(200, json.dumps({"ok": True, "camera_mode": body["mode"]}).encode(), "application/json")
                    return
                if self.path == "/quality":
                    if set(body) != {"quality"} or not isinstance(body["quality"], str):
                        raise ValueError("invalid quality request")
                    simulation.set_quality(body["quality"])
                    self.send(200, b'{"ok":true}', "application/json")
                    return
                if self.path == "/record":
                    if set(body) != {"action"} or not isinstance(body["action"], str):
                        raise ValueError("invalid record request")
                    simulation.request_record(body["action"])
                    self.send(202, b'{"ok":true}', "application/json")
                    return
                keys = body.get("keys", [])
                if not isinstance(keys, list) or any(not isinstance(k, str) for k in keys):
                    raise ValueError("invalid keys")
                controller = body.get("controller")
                stage = body.get("stage")
                leg_length = body.get("leg_length")
                if controller is not None and controller not in ("own", "upstream"):
                    raise ValueError("invalid controller")
                if stage is not None and (isinstance(stage, bool) or not isinstance(stage, int)):
                    raise ValueError("invalid stage")
                applied = simulation.command(keys, body.get("reset", False), controller, stage, leg_length,
                    body.get("jump_profile"), body.get("drive_speed"), body.get("client_id"),
                    body.get("claim_control", False), body.get("stop", False), body.get("location"),
                    body.get("turn_speed"), body.get("reset_mode", "global"),
                    body.get("emergency_stop", False), body.get("clear_emergency_stop", False),
                    body.get("stop_generation"))
                self.send(200, json.dumps({"ok": True, "applied": applied,
                    "motor_stop_generation": simulation.motor_stop_generation}).encode(), "application/json")
            except (ValueError, TypeError):
                self.send(400, b"Invalid command", "text/plain")
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-interactive-20261003")
    parser.add_argument("--terrain", choices=("flat", "course"), default="course")
    parser.add_argument("--original-link-collisions", action="store_true",
                        help="仅用于历史对照，恢复原 CAD 凸包之间的错误自碰撞")
    parser.add_argument("--guide-rollers", choices=("legacy", "fixed", "passive"), default="passive",
                        help="底部机械对照：原凸包、固定导轮或自由转动导轮")
    parser.add_argument("--knee-range", choices=("upstream", "cad120", "stop180"), default="stop180",
                        help="默认180mm实体挡块；另保留上游范围与CAD120参考对照")
    args = parser.parse_args()
    revision = subprocess.check_output(["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip()
    changed_upstream = subprocess.run(["git", "-C", str(SOURCE), "diff", "--quiet", "HEAD", "--",
                                       "controller", "mjcf", "config"]).returncode != 0
    if revision != REVISION or changed_upstream:
        parser.error("需要固定版本且未修改的 WBR 控制器、模型和配置")
    simulation = LiveSimulation(args.output.resolve(), not args.original_link_collisions, args.terrain,
                                args.guide_rollers, args.knee_range)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(simulation))
    physics_thread = threading.Thread(target=simulation.physics, name="wbr-physics", daemon=True)
    render_thread = threading.Thread(target=simulation.render, name="wbr-render", daemon=True)
    physics_thread.start()
    render_thread.start()
    print(f"http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        simulation.stop_event.set()
        server.server_close()
        # 正常退出时等记录文件写完，避免守护线程随主进程中断。
        physics_thread.join()
        simulation.recorder.wait(None)
        render_thread.join()


if __name__ == "__main__":
    main()
