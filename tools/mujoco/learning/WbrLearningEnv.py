"""平地停车与纵向抗扰的 MuJoCo 学习环境；观测仅来自传感器和控制器状态。"""
from __future__ import annotations

from collections import deque
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys

import gymnasium as gym
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent/"reference"))
from WbrGasSpring import load_model

MOTORS = ("ljoint1", "ljoint4", "lwheel", "rjoint1", "rjoint4", "rwheel")
FRAME_SIZE = 31
HISTORY = 3
POLICY_DT = .015
LIMITS = np.array([20., 20., 5.2, 20., 20., 5.2])


def verify_snapshot(root=ROOT):
    manifest = json.loads((root/"manifest.json").read_text(encoding="utf-8"))
    for relative, expected in manifest["files"].items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"实验快照发生变化：{relative}")
    library = root/manifest["library"]
    if hashlib.sha256(library.read_bytes()).hexdigest() != manifest["library_sha256"]:
        raise RuntimeError("控制动态库与实验快照不匹配")
    return manifest


def experiment_identity(manifest):
    # 评价脚本可改善统计，但物理模型、环境和控制代码必须与训练一致。
    selected = {k:v for k,v in manifest["files"].items()
                if not k.startswith("tools/mujoco/learning/") or
                k in ("tools/mujoco/learning/WbrLearningEnv.py", "tools/mujoco/learning/WbrLearningBridge.c",
                      "tools/mujoco/learning/StudyWbrPolicy.py")}
    return hashlib.sha256(json.dumps(selected, sort_keys=True).encode()).hexdigest()


def load_library():
    manifest = json.loads((ROOT/"manifest.json").read_text(encoding="utf-8"))
    lib = ctypes.CDLL(str(ROOT/manifest["library"]))
    ptr = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCreate.argtypes, lib.OwnWbrCreate.restype = [ptr], ctypes.c_void_p
    lib.OwnWbrDestroy.argtypes = [ctypes.c_void_p]
    lib.OwnWbrSetGasSpring.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.OwnWbrSetJointLimits.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.OwnWbrCommandStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr]
    lib.WbrLearningStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr]
    lib.WbrLearningReferenceStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr]
    return lib


class WbrLearningEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, *, randomize=True, strict=False, record=False, gas_force=450.,
                 reward_mode="legacy", action_mode="full"):
        super().__init__()
        if reward_mode not in ("legacy", "dense") or action_mode not in ("full", "symmetric", "reference"):
            raise ValueError("未知的奖励或动作模式")
        self.reward_mode, self.action_mode = reward_mode, action_mode
        self.randomize, self.strict, self.record = randomize, strict, record
        self.gas_force = float(gas_force)
        if not math.isfinite(self.gas_force) or not 0 <= self.gas_force <= 600:
            raise ValueError("气簧推力必须在 0～600 N 内")
        self.model = load_model(gas_force_n=self.gas_force, correct_link_collisions=True,
                               guide_rollers="passive", knee_range="stop180", noslip_iterations=10)
        self.model.opt.timestep = .001
        self.data = mujoco.MjData(self.model)
        self.lib = load_library()
        self.handle = None
        self.profile = json.loads((HERE.parent/"reference/OwnWbrProfile.json").read_text(encoding="utf-8"))
        self.base = self.model.body("base_link").id
        self.floor = self.model.geom("floor").id
        self.wheels = {self.model.body("lwlink").id, self.model.body("rwlink").id}
        self.closure = [(self.model.site(f"{s}link4_site{i}").id,
                         self.model.site(f"{s}link1_site{i}").id) for s in ("l", "r") for i in (1, 2)]
        self.aids = np.array([self.model.actuator(m+"_actuator").id for m in MOTORS])
        self.dofs = np.array([self.model.jnt_dofadr[self.model.actuator_trnid[a, 0]] for a in self.aids])
        if not np.array_equal(self.aids, np.arange(6)):
            raise RuntimeError("执行器顺序变化")
        for aid, limit in zip(self.aids, LIMITS):
            self.model.actuator_ctrlrange[aid] = [-limit, limit]
            self.model.actuator_forcerange[aid] = [-limit, limit]
            self.model.jnt_actfrcrange[self.model.actuator_trnid[aid, 0]] = [-limit, limit]
        self.base_mass = self.model.body_mass.copy()
        self.base_inertia = self.model.body_inertia.copy()
        self.base_friction = self.model.geom_friction.copy()
        names = ["imu_quat", "imu_gyro", "imu_accel"] + [f"{m}_{k}" for m in MOTORS for k in ("pos", "vel", "tor")]
        self.sensor_indices = np.concatenate([np.arange(self.model.sensor(n).adr[0],
            self.model.sensor(n).adr[0]+self.model.sensor(n).dim[0]) for n in names])
        self.sensor, self.command = (ctypes.c_float*28)(), (ctypes.c_float*3)()
        self.output, self.action_buffer = (ctypes.c_float*40)(), (ctypes.c_float*6)()
        action_size = {"full":6, "symmetric":3, "reference":3}[action_mode]
        action_low = np.full(action_size, -1., dtype=np.float32)
        if action_mode == "reference":
            action_low[0] = 0.
        self.action_space = gym.spaces.Box(action_low, np.ones(action_size, dtype=np.float32))
        self.observation_space = gym.spaces.Box(-20., 20., (FRAME_SIZE*HISTORY,), dtype=np.float32)
        self.jac = np.zeros((3, self.model.nv))
        self.history = deque(maxlen=HISTORY)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        self.case = options.get("case", "stop" if self.np_random.random() < .6 else "push")
        if self.case not in ("stop", "push", "combined"):
            raise ValueError("未知场景")
        self.speed = float(options.get("speed", self.np_random.choice([-1., 1.]) * self.np_random.uniform(.35, .75)))
        self.push = float(options.get("push", self.np_random.choice([-1., 1.]) * self.np_random.uniform(12., 35.)))
        self.duration = float(options.get("duration", 8.))
        if not all(math.isfinite(v) for v in (self.speed, self.push, self.duration)) or self.duration <= 0:
            raise ValueError("无效的场景参数")
        self.delay = int(options.get("delay", self.np_random.integers(0, 2) if self.randomize else 0))
        if not 0 <= self.delay <= 3:
            raise ValueError("控制延迟限定为 0～3 个控制周期")
        self.strength = self.np_random.uniform(.95, 1.05, 6) if self.randomize else np.ones(6)
        mass_scale = float(self.np_random.uniform(.95, 1.05)) if self.randomize else 1.
        friction_scale = float(self.np_random.uniform(.85, 1.15)) if self.randomize else 1.
        self.model.body_mass[:] = self.base_mass
        self.model.body_inertia[:] = self.base_inertia
        self.model.body_mass[self.base] *= mass_scale
        self.model.body_inertia[self.base] *= mass_scale
        self.model.geom_friction[:] = self.base_friction
        self.model.geom_friction[:, 0] *= friction_scale
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_resetData(self.model, self.data)
        qpos = self.profile["qpos"]
        self.data.qpos[:len(qpos)] = qpos
        mujoco.mj_forward(self.model, self.data)
        self.neutral_angles = self.data.sensordata[self.sensor_indices][[10, 13, 19, 22]].copy()
        if self.handle:
            self.lib.OwnWbrDestroy(self.handle)
        p = self.profile
        config = (ctypes.c_float*20)(*np.asarray(p["negative_k"]).flatten(), p["theta0"], p["tp0"],
            p["wheel0"], p["support_n"], .2, p["radius_m"], p["length_kp"], p["length_kd"])
        self.handle = self.lib.OwnWbrCreate(config)
        if not self.handle:
            raise RuntimeError("创建控制器失败")
        if self.lib.OwnWbrSetGasSpring(self.handle, self.gas_force) != 1:
            raise RuntimeError("气簧补偿配置失败")
        if self.lib.OwnWbrSetJointLimits(self.handle, 20., 40.) != 1:
            raise RuntimeError("电机配置失败")
        self.tick, self.elapsed, self.command_v, self.error = 0, 0., 0., None
        self.filtered_action = np.zeros(6)
        self.delay_queue = deque([np.zeros(6) for _ in range(self.delay)])
        self.action_buffer[:] = np.zeros(6)
        self.feedback = self.data.sensordata[self.sensor_indices].copy()
        self.trace, self.poses = [], []
        self.completed = False
        # 先用零修正自由站立 0.6 s；没有外力固定和位置重置。
        for _ in range(600):
            self._physics_step(warmup=True)
            if self.error:
                raise RuntimeError(f"初始站立失败：{self.error}")
        self.origin_time = float(self.data.time)
        self.elapsed = 0.
        self.history.clear()
        frame = self._frame()
        for _ in range(HISTORY):
            self.history.append(frame.copy())
        truth = self._truth()
        self.start_x = truth["x_m"]
        self.stop_x = None
        self.metrics = dict(samples=0, velocity_error_sq=0., pitch_sq=0., peak_pitch_deg=0.,
            peak_roll_deg=0., peak_pitch_1ms_deg=0., peak_roll_1ms_deg=0., max_closure_1ms_m=0.,
            max_closure_m=0., hip_heat_nm2s=0., peak_hip_nm=0., peak_wheel_nm=0.,
            stop_velocity_sq=0., stop_samples=0, peak_reverse_mps=0.,
            stop_travel_m=0., reward=0., mass_scale=mass_scale, friction_scale=friction_scale,
            delay_control_steps=self.delay, model_strength=self.strength.tolist())
        return self._observation(), {"case": self.case}

    def _physics_step(self, warmup=False):
        self.elapsed = 0. if warmup else float(self.data.time-self.origin_time)
        self.command_v = self.speed if not warmup and self.case != "push" and 1. <= self.elapsed < 4. else 0.
        self.data.xfrc_applied[:] = 0.
        push_time = 2.5 if self.case == "push" else 5.
        if not warmup and self.case != "stop" and push_time <= self.elapsed < push_time+.2:
            self.data.xfrc_applied[self.base, 0] = self.push
        if self.tick % 3 == 0:
            mujoco.mj_forward(self.model, self.data)
            self.feedback = self.data.sensordata[self.sensor_indices].copy()
            if self.randomize:
                self.feedback[:4] += self.np_random.normal(0., .00015, 4)
                self.feedback[:4] /= np.linalg.norm(self.feedback[:4])
                self.feedback[4:7] += self.np_random.normal(0., .004, 3)
                self.feedback[[10,13,16,19,22,25]] += self.np_random.normal(0., .0003, 6)
                self.feedback[[11,14,17,20,23,26]] += self.np_random.normal(0., .008, 6)
            self.sensor[:] = self.feedback
            self.command[:] = (self.command_v, 0., 0.)
            controller = (self.lib.WbrLearningReferenceStep if self.action_mode == "reference"
                          else self.lib.WbrLearningStep)
            code = controller(self.handle, self.sensor, self.command, self.action_buffer, self.output)
            if code:
                self.error = f"controller_error_{code}"
                self.data.ctrl[:] = 0.
                return
            request = np.array(self.output[:6]) * self.strength
            self.delay_queue.append(np.clip(request, -LIMITS, LIMITS))
            self.data.ctrl[:] = self.delay_queue.popleft()
        mujoco.mj_step(self.model, self.data)
        self.tick += 1
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all() or any(w.number for w in self.data.warning):
            self.error = "mujoco_warning_or_nonfinite"
        torque = self.data.actuator_force[self.aids]
        if not np.isfinite(torque).all() or np.any(np.abs(torque) > LIMITS+1e-5):
            self.error = "applied_torque_limit"
        if not warmup:
            self.metrics["hip_heat_nm2s"] += float(np.square(torque[[0,1,3,4]]).sum())*.001
            self.metrics["peak_hip_nm"] = max(self.metrics["peak_hip_nm"], float(np.max(np.abs(torque[[0,1,3,4]]))))
            self.metrics["peak_wheel_nm"] = max(self.metrics["peak_wheel_nm"], float(np.max(np.abs(torque[[2,5]]))))
        if self.strict:
            mujoco.mj_forward(self.model, self.data)
            truth = self._truth()
            if not warmup:
                self.metrics["peak_pitch_1ms_deg"] = max(self.metrics["peak_pitch_1ms_deg"], abs(math.degrees(truth["pitch_rad"])))
                self.metrics["peak_roll_1ms_deg"] = max(self.metrics["peak_roll_1ms_deg"], abs(math.degrees(truth["roll_rad"])))
                self.metrics["max_closure_1ms_m"] = max(self.metrics["max_closure_1ms_m"], truth["closure_m"])

    def _truth(self):
        rotation = self.data.xmat[self.base].reshape(3, 3)
        pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        mujoco.mj_jacBody(self.model, self.data, self.jac, None, self.base)
        velocity = self.jac @ self.data.qvel
        forward = float(np.dot(velocity[:2], rotation[:2, 0])/max(np.linalg.norm(rotation[:2, 0]), 1e-6))
        closure = max(float(np.linalg.norm(self.data.site_xpos[a]-self.data.site_xpos[b])) for a,b in self.closure)
        other = 0
        for contact in self.data.contact:
            if self.floor in (contact.geom1, contact.geom2):
                gid = contact.geom2 if contact.geom1 == self.floor else contact.geom1
                other += int(self.model.geom_bodyid[gid] not in self.wheels)
        if max(abs(pitch), abs(roll)) > math.radians(35):
            self.error = self.error or "tilt_over_35deg"
        if closure > .001:
            self.error = self.error or "closure_over_1mm"
        if other:
            self.error = self.error or "nonwheel_floor_contact"
        if np.any(self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
            self.error = self.error or "joint_limit_contact"
        return dict(pitch_rad=pitch, roll_rad=roll, true_v_mps=forward, closure_m=closure,
                    x_m=float(self.data.xpos[self.base,0]), nonwheel_contacts=other)

    def _frame(self):
        sensor = self.feedback
        w,x,y,z = sensor[:4]
        gravity = np.array([2*(w*y-x*z), -2*(y*z+w*x), -1+2*(x*x+y*y)])
        values = np.concatenate([sensor[4:7]*.25, gravity, [self.command_v, .2],
            sensor[[10,13,19,22]]-self.neutral_angles, sensor[[11,14,17,20,23,26]]*.05,
            [self.output[10], self.output[25]-self.output[9]],
            (np.array([self.output[12],self.output[18]])-.2)*10.,
            [self.output[13],self.output[19]], [self.output[26]], self.filtered_action])
        if len(values) != FRAME_SIZE or not np.isfinite(values).all():
            raise RuntimeError("观测维度或数值异常")
        return np.clip(values, -20, 20).astype(np.float32)

    def _observation(self):
        return np.concatenate(self.history).astype(np.float32)

    def step(self, action):
        if self.completed:
            raise RuntimeError("场景结束后必须先 reset")
        action = np.asarray(action, dtype=float)
        if action.shape != self.action_space.shape or not np.isfinite(action).all():
            self.data.ctrl[:] = 0.
            raise ValueError("学习动作维度不匹配或包含非有限数")
        if self.action_mode == "symmetric":
            action = np.tile(action, 2)
        elif self.action_mode == "reference":
            action = np.concatenate(([np.clip(action[0],0.,1.)],action[1:],np.zeros(3)))
        previous = self.filtered_action.copy()
        self.filtered_action += np.clip(np.clip(action, -1., 1.)-self.filtered_action, -.2, .2)
        self.action_buffer[:] = self.filtered_action
        for _ in range(15):
            self._physics_step()
            if self.error:
                break
        mujoco.mj_forward(self.model, self.data)
        self.elapsed = float(self.data.time-self.origin_time)
        truth = self._truth()
        speed_error = truth["true_v_mps"]-self.command_v
        reward = (1. + 2.*math.exp(-speed_error**2/.04)
                  - 8.*truth["pitch_rad"]**2 - 4.*truth["roll_rad"]**2
                  - .01*float(np.square(self.data.actuator_force[self.aids]/LIMITS).sum())
                  - .015*float(np.square(self.filtered_action).sum())
                  - .03*float(np.square(self.filtered_action-previous).sum()))
        if self.reward_mode == "dense":
            focused = ((self.case == "stop" and self.elapsed >= 4.) or
                       (self.case == "push" and 2.5 <= self.elapsed <= 4.7))
            # 二次误差让大误差阶段也有明显评分差；权重固定后再做独立比较。
            reward = (1. - (8. if focused else 3.)*(speed_error/.5)**2
                      - .12*(truth["pitch_rad"]/.1)**2 - .25*(truth["roll_rad"]/.1)**2
                      - .015*float(np.square(self.data.actuator_force[self.aids]/LIMITS).sum())
                      - .005*float(np.square(self.filtered_action).sum())
                      - .02*float(np.square(self.filtered_action-previous).sum()))
        if self.error:
            reward -= 100.
        m = self.metrics
        m["samples"] += 1
        m["velocity_error_sq"] += speed_error**2
        m["pitch_sq"] += truth["pitch_rad"]**2
        m["peak_pitch_deg"] = max(m["peak_pitch_deg"], abs(math.degrees(truth["pitch_rad"])))
        m["peak_roll_deg"] = max(m["peak_roll_deg"], abs(math.degrees(truth["roll_rad"])))
        m["max_closure_m"] = max(m["max_closure_m"], truth["closure_m"])
        m["reward"] += reward
        if self.elapsed >= 4.:
            if self.stop_x is None:
                self.stop_x = truth["x_m"]
            m["stop_velocity_sq"] += truth["true_v_mps"]**2
            m["stop_samples"] += 1
            m["peak_reverse_mps"] = max(m["peak_reverse_mps"], -math.copysign(1., self.speed)*truth["true_v_mps"])
            m["stop_travel_m"] = max(m["stop_travel_m"], abs(truth["x_m"]-self.stop_x))
        if self.record:
            self.trace.append(dict(time_s=self.elapsed, command_v_mps=self.command_v,
                **truth, hip_heat_nm2s=m["hip_heat_nm2s"], action=self.filtered_action.tolist()))
            self.poses.append(self.data.qpos.copy())
        self.history.append(self._frame())
        terminated = self.error is not None
        truncated = self.elapsed >= self.duration
        self.completed = terminated or truncated
        info = {"failure": self.error}
        if self.completed:
            info["measurements"] = self.summary()
        return self._observation(), float(reward), terminated, truncated, info

    def summary(self):
        m = dict(self.metrics)
        count = max(m.pop("samples"), 1)
        m["velocity_rmse_mps"] = math.sqrt(m.pop("velocity_error_sq")/count)
        m["pitch_rms_deg"] = math.degrees(math.sqrt(m.pop("pitch_sq")/count))
        stop_count = m.pop("stop_samples")
        m["stop_velocity_rms_mps"] = math.sqrt(m.pop("stop_velocity_sq")/stop_count) if stop_count else None
        m.update(case=self.case, speed_mps=self.speed, push_n=self.push, simulated_s=self.elapsed,
                 failure=self.error, passed=self.error is None and self.elapsed >= self.duration,
                 gas_force_n=self.gas_force, strict_physics_checks=self.strict)
        return m

    def close(self):
        if self.handle:
            self.lib.OwnWbrDestroy(self.handle)
            self.handle = None
