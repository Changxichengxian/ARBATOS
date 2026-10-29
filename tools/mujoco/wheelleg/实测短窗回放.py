"""在明确的双轮接地假设下，用日志状态及电机命令进行短窗开环回放。"""
import importlib.util
import json
import math
import pickle
import sys
from pathlib import Path

import mujoco
import numpy as np
import RunMeasuredWheelLeg as sim

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('sdviewer', ROOT / 'tools/sdlog/SdLogViewer.py')
viewer = importlib.util.module_from_spec(spec)
sys.modules['sdviewer'] = viewer
spec.loader.exec_module(viewer)
log = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'local/cache/wheelleg-joystick-20261029/日志0046/解析数据.pkl'
with log.open('rb') as stream:
    series = pickle.load(stream).series['WHEELLEG_BENCH_FRAME']
t = np.asarray(series.ticks_ms) * .001
f = {k: np.asarray(v) for k, v in series.fields.items()}
profile = sim.load_profile()
model = mujoco.MjModel.from_xml_string(sim.model_xml(profile))
model.actuator_ctrlrange[[2, 5]] = [-1., 1.]
cfg = sim.make_config(profile)
radius = float(cfg.wheel_radius_m)

def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]

def leg(data, side, front, back):
    sim.legacy.initialize_leg_pose(mujoco, model, data, side, cfg, front, back)

def initialize(i):
    data = mujoco.MjData(model)
    data.qpos[3:7] = quat(f['roll'][i], f['pitch'][i], f['yaw'][i])
    data.qvel[3:6] = [f['gyro_x'][i], f['gyro_y'][i], f['gyro_z'][i]]
    for side, prefix in (('right', 'R'), ('left', 'L')):
        q = np.array([f['joint_'+prefix+end+'_rad'][i] for end in ('F', 'B')])
        dq = np.array([f['joint_'+prefix+end+'_radps'][i] for end in ('F', 'B')])
        leg(data, side, *q)
        temp = mujoco.MjData(model)
        leg(temp, side, *(q + dq * 1e-5))
        for end in ('front', 'back'):
            node, shifted = data.joint(side+'_'+end+'_knee'), temp.joint(side+'_'+end+'_knee')
            node.qvel[0] = sim.legacy.wrap_pi(shifted.qpos[0]-node.qpos[0]) / 1e-5
        for j, end in enumerate(('front', 'back')):
            data.joint(side+'_'+end+'_hinge').qvel[0] = dq[j]
        data.joint(side+'_wheel_hinge').qvel[0] = f['wheel_'+prefix+'_rate'][i]
    mujoco.mj_forward(model, data)
    centers = [data.geom(side+'_wheel_geom').xpos.copy() for side in sim.SIDES]
    data.qpos[2] += radius - np.mean([p[2] for p in centers])
    mujoco.mj_forward(model, data)
    # 接地点速度置零只是一种初态假设；日志没有独立地速和支撑力。
    residual = []
    for side in sim.SIDES:
        geom = data.geom(side+'_wheel_geom')
        point = geom.xpos.copy(); point[2] -= radius
        jac = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jac, None, point, int(model.geom_bodyid[geom.id]))
        residual.append(jac @ data.qvel)
    data.qvel[:3] = -np.mean(residual, axis=0)
    mismatch = float(np.linalg.norm(residual[0]-residual[1]))
    mujoco.mj_forward(model, data)
    return data, mismatch, float(centers[0][2]-centers[1][2])

def command(data, i):
    for side_index, (side, prefix) in enumerate((('right', 'R'), ('left', 'L'))):
        for j, end in enumerate(('front', 'back')):
            role = prefix+('F' if j == 0 else 'B')
            node = data.joint(side+'_'+end+'_hinge')
            data.ctrl[side_index*3+j] = (f['out_torque_'+role][i] + f['joint_kp'][i] *
                (f['joint_target_'+role][i]-node.qpos[0]) - f['joint_kd'][i]*node.qvel[0])
        node = data.joint(side+'_wheel_hinge')
        data.ctrl[side_index*3+2] = (f['out_torque_'+prefix+'W'][i] + f['wheel_kd'][i] *
            (f['wheel_target_'+prefix][i]-node.qvel[0]))

def run(start):
    i = int(np.searchsorted(t, start)); data, mismatch, gap = initialize(i)
    command(data, i)
    # 初始电机内部状态不可观测；假设短窗开始前已达到当前命令。
    for actuator in (2, 5):
        adr = int(model.actuator_actadr[actuator])
        if adr >= 0:
            data.act[adr] = np.clip(data.ctrl[actuator], -1, 1)
    errors = []
    for step in range(100):
        j = min(len(t)-1, int(np.searchsorted(t, t[i]+step*.001, side='right')-1))
        command(data, j); mujoco.mj_step(model, data)
        k = min(len(t)-1, int(np.searchsorted(t, t[i]+(step+1)*.001)))
        roll, pitch, yaw = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
        errors.append([pitch-f['pitch'][k], roll-f['roll'][k],
            data.joint('right_wheel_hinge').qvel[0]-f['wheel_R_rate'][k],
            data.joint('left_wheel_hinge').qvel[0]-f['wheel_L_rate'][k],
            *[data.joint(side+'_'+end+'_hinge').qpos[0]-f['joint_'+role+'_rad'][k]
              for side, end, role in (('right','front','RF'),('right','back','RB'),('left','front','LF'),('left','back','LB'))]])
    errors = np.asarray(errors)
    return dict(start=float(t[i]), end=float(t[i]+.1), initial_contact_velocity_mismatch_mps=mismatch,
                wheel_height_difference_m=gap, rmse=np.sqrt(np.mean(errors**2, axis=0)).tolist(),
                final_error=errors[-1].tolist())

def quiet_window(start, end):
    # 选择整段低速度窗口作为静态近似，仍可能受到外力。
    candidates = np.flatnonzero((t >= start) & (t < end-.1) & (f['phase'] == 3))
    raw = (abs(f['gyro_x']) + abs(f['gyro_y']) + abs(f['gyro_z']) +
           .05*(abs(f['wheel_R_rate']) + abs(f['wheel_L_rate'])))
    cumulative = np.r_[0., np.cumsum(raw)]
    ends = np.searchsorted(t, t[candidates]+.1)
    score = (cumulative[ends]-cumulative[candidates]) / np.maximum(ends-candidates, 1)
    return float(t[candidates[np.argmin(score)]])

windows = [('fit_quiet', quiet_window(70, 120)), ('fit', 70.203), ('fit', 81.651), ('fit', 118.413),
           ('holdout_quiet', quiet_window(272, 285)), ('holdout', 272.559), ('holdout', 283.644)]
print(json.dumps(dict(columns=['pitch_rad','roll_rad','wheel_R_radps','wheel_L_radps',
                               'joint_RF_rad','joint_RB_rad','joint_LF_rad','joint_LB_rad'],
                      results=[dict(group=group, **run(start)) for group, start in windows]),
                 ensure_ascii=False, indent=2))
