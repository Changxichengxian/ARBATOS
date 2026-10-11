"""仿真原地复位姿态；只用于用户复位，不参与行驶控制。"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from WbrTerrain import ground_geom_ids, ground_height


def local_reset_pose(model, home, current):
    """保留车身 XY 和朝向，恢复 home 关节，向上避开当前位置的地形。"""
    current = np.asarray(current, dtype=float)
    if current.shape != (model.nq,) or not np.isfinite(current).all():
        raise ValueError("原地复位需要有效的当前位置")
    pose = np.array(home, dtype=float, copy=True)
    w, x, y, z = current[3:7]
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    rotation = np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
    mujoco.mju_mulQuat(pose[3:7], rotation, pose[3:7].copy())
    pose[:2] = current[:2]
    height = ground_height(model, *pose[:2])
    pose[2] += height if math.isfinite(height) else 0.0
    data = mujoco.MjData(model)
    grounds = ground_geom_ids(model)
    # 台阶边缘可能横向穿入底板；仅沿 Z 抬升，绝不悄悄挪回入口。
    for _ in range(160):
        data.qpos[:] = pose
        mujoco.mj_forward(model, data)
        depth = min((float(c.dist) for c in data.contact
                     if (c.geom1 in grounds) != (c.geom2 in grounds)), default=.001)
        if depth >= -1e-6:
            return pose
        pose[2] += max(.005, min(.02, .001-depth))
    raise ValueError("当前位置无法腾出复位空间，请按 T 返回场地入口")
