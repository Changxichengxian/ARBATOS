"""检查扩大场地的真实碰撞高度、原路线和所有入口；不验收新增障碍的控制效果。"""
from __future__ import annotations

import json
import math
import time
import weakref
import gc
from pathlib import Path

import mujoco
import numpy as np

from RunWbr import SOURCE
from WbrGasSpring import load_model
from WbrTerrain import (
    BOUNDS_M, COURSE_SEGMENTS, CROSS_SLOPES, EXPLORATION_ZONES, LARGE_STEPS,
    STANDARD_CLIMB_PLATFORMS, apply_terrain, cross_edge_height, cross_transition_height,
    ground_geom_ids, ground_height, scenario_pose, terrain_info,
)


def main() -> None:
    spec = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><geom name="floor" type="plane" size="0 0 .1" '
        'friction=".8 .005 .0001"/></worldbody></mujoco>')
    apply_terrain(spec, "course")
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geom = np.zeros(1, dtype=np.int32)

    def check(x, y, expected=None):
        height = ground_height(model, x, y)
        distance = mujoco.mj_ray(model, data, np.array([x, y, 4.0]),
            np.array([0.0, 0.0, -1.0]), None, 1, -1, geom)
        actual = 4-distance if distance >= 0 else math.nan
        assert (math.isnan(height) and math.isnan(actual)) or abs(height-actual) < 2e-7, (x, y, height, actual)
        if expected is not None:
            assert abs(height-expected) < 2e-7, (x, y, height, expected)

    for _, x0, x1, z0, z1 in COURSE_SEGMENTS:
        for t in (.1, .5, .9):
            check(x0+(x1-x0)*t, 0, z0+(z1-z0)*t)
    for _, x0, x1, y, height in LARGE_STEPS:
        check((x0+x1)/2, y, height)
    for _, x0, x1, y0, y1, degrees in CROSS_SLOPES:
        check((x0+x1)/2, (y0+y1)/2, cross_edge_height(degrees, (y0+y1)/2))
    for x in np.linspace(5.510003, 15.990003, 87):
        for y in (1.05, 1.5, 1.95):
            expected = cross_transition_height(x, y)
            if math.isfinite(expected):
                check(x, y, expected)
    check(21, -9, -.08)
    check(25.25, 9, .5)
    check(22.5, 12.5, .8)
    for index, height in enumerate((.04, .06, .08, .10)):
        check(-.4+index*2.5, -9, height)
    for name, x0, x1, y0, y1, height in STANDARD_CLIMB_PLATFORMS:
        check((x0+x1)/2, (y0+y1)/2, height)
        platform = model.geom(name)
        assert np.allclose(platform.size[:2], [(x1-x0)/2, (y1-y0)/2])
        assert abs(platform.pos[2]+platform.size[2]-height) < 2e-7
        check(x0-.1, (y0+y1)/2, 0.0)
        check(x1+.1, (y0+y1)/2, 0.0)
    for index, height in enumerate((.06, .12, .18, .24, .24, .18, .12, .06)):
        check(index+.5, -12.5, height)
    # 随机点避开浮点几何边界；遍历整场地确认没有遗漏地板的孔洞。
    random = np.random.default_rng(20261003)
    points = random.uniform([-30, -20], [50, 20], (6000, 2))
    for x, y in points:
        check(x, y)
        assert math.isfinite(ground_height(model, x, y)), (x, y)
    for x, y in ((-30.1, 0), (50.1, 0), (0, -20.1), (0, 20.1)):
        check(x, y)
        assert math.isnan(ground_height(model, x, y))

    # 模型销毁后缓存应释放；不能把地址相同的后续模型当成旧模型。
    disposable = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="floor" type="plane" size="0 0 .1"/></worldbody></mujoco>')
    assert ground_height(disposable, 0, 0) == 0
    reference = weakref.ref(disposable)
    del disposable
    gc.collect()
    assert reference() is None
    changed = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="floor" type="plane" pos="0 0 .7" size="0 0 .1"/></worldbody></mujoco>')
    assert ground_height(changed, 0, 0) == .7
    robot = load_model(SOURCE / "mjcf/scene.xml", correct_link_collisions=True, terrain="course")
    robot_data = mujoco.MjData(robot)
    mujoco.mj_resetDataKeyframe(robot, robot_data, robot.key("home").id)
    home = robot_data.qpos.copy()
    profile = json.loads(Path(__file__).with_name("OwnWbrProfile.json").read_text(encoding="utf-8"))
    entries = {}
    for location in ("start", "large_steps", "step_climb", "step_auto", "slope_course",
                     "cross10", "cross15", *(zone["location"] for zone in EXPLORATION_ZONES)):
        source = profile["qpos"] if location in ("cross10", "cross15") else home
        robot_data.qpos[:] = scenario_pose(robot, source, location)
        mujoco.mj_forward(robot, robot_data)
        penetration = []
        grounds = ground_geom_ids(robot)
        for contact in robot_data.contact:
            if (contact.geom1 in grounds) != (contact.geom2 in grounds):
                penetration.append(float(contact.dist))
        tolerance = -.002 if location in {zone["location"] for zone in EXPLORATION_ZONES} else -.015
        assert min(penetration, default=0) >= tolerance, (location, penetration)
        entries[location] = dict(xy_m=robot_data.qpos[:2].tolist(),
            min_contact_distance_m=min(penetration, default=0))
    info = terrain_info(robot)
    assert info["bounds_m"] == BOUNDS_M
    assert info["large_steps"][2]["x_m"] == [9.0, 9.5]
    assert info["standard_climb_platforms"] == [dict(
        name="terrain_climb200_platform", x_m=[9.0, 11.0],
        y_m=[-18.5, -15.5], height_m=.20)]
    started = time.perf_counter()
    for x, y in points:
        ground_height(model, x, y)
    elapsed = time.perf_counter()-started
    print(json.dumps(dict(passed=True, random_ray_samples=len(points), bounds_m=BOUNDS_M,
        ground_queries_per_second=len(points)/elapsed, entries=entries), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
