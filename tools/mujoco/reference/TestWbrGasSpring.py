"""独立验证 WBR 物理气弹簧的内力、虚功方向和电机隔离。"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np

from WbrGasSpring import (DEFAULT_GAS_FORCE_N, LINK_COLLISION_EXCLUDES, MOTOR_NAMES,
                          gas_generalized_force, gas_constraint_projected_motor_torque,
                          gas_motor_torque_from_raw, gas_potential_energy,
                          gas_spring_state, load_model)
from WbrLinkCollision import link_collision_info


def finite_difference_energy(model, data, qpos_index: int, step: float = 1.0e-6) -> float:
    original = float(data.qpos[qpos_index])
    data.qpos[qpos_index] = original + step
    mujoco.mj_forward(model, data)
    plus = gas_potential_energy(model, data)
    data.qpos[qpos_index] = original - step
    mujoco.mj_forward(model, data)
    minus = gas_potential_energy(model, data)
    data.qpos[qpos_index] = original
    mujoco.mj_forward(model, data)
    return (plus - minus) / (2.0 * step)


def contact_body_pairs(model, data) -> set[tuple[str, str]]:
    pairs = set()
    for contact in data.contact:
        body1 = model.body(int(model.geom_bodyid[contact.geom1])).name or "world"
        body2 = model.body(int(model.geom_bodyid[contact.geom2])).name or "world"
        pairs.add(tuple(sorted((body1, body2))))
    return pairs


def main() -> int:
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    qfrc = gas_generalized_force(model, data)
    springs = gas_spring_state(model, data)

    net_force = sum((item["upper_force_n"] + item["lower_force_n"] for item in springs),
                    start=np.zeros(3))
    net_moment = sum((np.cross(item["upper_m"], item["upper_force_n"]) +
                      np.cross(item["lower_m"], item["lower_force_n"]) for item in springs),
                     start=np.zeros(3))
    joint_names = tuple(model.joint(index).name for index in range(1, model.njnt))
    virtual_work_errors = {}
    passive_torque = {}
    for name in joint_names:
        joint = model.joint(name)
        dof = int(joint.dofadr[0])
        derivative = finite_difference_energy(model, data, int(joint.qposadr[0]))
        virtual_work_errors[name] = float(qfrc[dof] + derivative)
        passive_torque[name] = float(qfrc[dof])

    equivalent_motor_torque = {}
    constrained_motor_torque = {}
    constraint_residual = {}
    extension_work = {}
    jacobian = np.asarray(data.efc_J).reshape(data.nefc, model.nv)
    projected_by_helper = gas_constraint_projected_motor_torque(model, data)
    for side, q1_name, q4_name in (("left", "ljoint1", "ljoint4"),
                                    ("right", "rjoint1", "rjoint4")):
        q1 = float(data.qpos[model.joint(q1_name).qposadr[0]])
        q4 = float(data.qpos[model.joint(q4_name).qposadr[0]])
        equivalent_motor_torque[side] = list(gas_motor_torque_from_raw(q1, q4, side))
        if side == "left":
            rows = list(range(0, 6))
            passive_names = ("llink1child1joint", "llink4child1joint",
                             "llink4child2joint", "llink4child3joint")
        else:
            rows = list(range(6, 12))
            passive_names = ("rlink1child1joint", "rlink4child1joint",
                             "rlink4child2joint", "rlink4child3joint")
        active = [int(model.joint(name).dofadr[0]) for name in (q1_name, q4_name)]
        passive = [int(model.joint(name).dofadr[0]) for name in passive_names]
        active_jacobian = jacobian[np.ix_(rows, active)]
        passive_jacobian = jacobian[np.ix_(rows, passive)]
        passive_per_active = -np.linalg.pinv(passive_jacobian) @ active_jacobian
        allowed_velocity = np.zeros((model.nv, 2))
        allowed_velocity[active, :] = np.eye(2)
        allowed_velocity[passive, :] = passive_per_active
        constrained_motor_torque[side] = list(allowed_velocity.T @ qfrc)
        constraint_residual[side] = float(np.max(np.abs(jacobian[rows] @ allowed_velocity)))
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        wheel_body = model.body("lwlink" if side == "left" else "rwlink").id
        mujoco.mj_jacBody(model, data, jacp, jacr, wheel_body)
        hip = np.asarray(data.xanchor[model.joint(q1_name).id], dtype=float)
        wheel = np.asarray(data.xpos[wheel_body], dtype=float)
        direction_xz = (wheel - hip)[[0, 2]]
        direction_xz /= np.linalg.norm(direction_xz)
        length_gradient = direction_xz @ jacp[[0, 2], :] @ allowed_velocity
        extension_work[side] = float(np.dot(constrained_motor_torque[side], length_gradient))

    # MuJoCo 的回调应把同一广义力加入 qfrc_passive，且不得增加 actuator。
    callback_error = float(np.max(np.abs(data.qfrc_passive - qfrc)))
    base_generalized_force_max = float(np.max(np.abs(qfrc[:6])))
    original_contact_pairs = contact_body_pairs(model, data)
    corrected_model = load_model(correct_link_collisions=True)
    corrected_data = mujoco.MjData(corrected_model)
    mujoco.mj_resetDataKeyframe(corrected_model, corrected_data,
                                corrected_model.key("home").id)
    mujoco.mj_forward(corrected_model, corrected_data)
    corrected_contact_pairs = contact_body_pairs(corrected_model, corrected_data)
    excluded_pairs = {tuple(sorted(pair)) for pair in LINK_COLLISION_EXCLUDES}
    floor_pairs_original = {pair for pair in original_contact_pairs if "world" in pair}
    floor_pairs_corrected = {pair for pair in corrected_contact_pairs if "world" in pair}
    structural_arrays_equal = bool(
        (model.nq, model.nv, model.nu, model.nbody, model.njnt) ==
        (corrected_model.nq, corrected_model.nv, corrected_model.nu,
         corrected_model.nbody, corrected_model.njnt) and
        np.array_equal(model.body_mass, corrected_model.body_mass) and
        np.array_equal(model.body_inertia, corrected_model.body_inertia) and
        np.array_equal(model.jnt_type, corrected_model.jnt_type) and
        np.array_equal(model.qpos0, corrected_model.qpos0))
    collision_info = link_collision_info(corrected_model)
    corrected_gas_error = float(np.max(np.abs(
        gas_generalized_force(corrected_model, corrected_data) - qfrc)))
    render_model = load_model(enabled=False)
    render_data = mujoco.MjData(render_model)
    mujoco.mj_resetDataKeyframe(render_model, render_data, render_model.key("home").id)
    mujoco.mj_forward(render_model, render_data)
    render_passive_max = float(np.max(np.abs(render_data.qfrc_passive)))
    mujoco.mj_forward(model, data)
    gas_after_second_load_error = float(np.max(np.abs(data.qfrc_passive - qfrc)))
    report = {
        "passed": bool(
            model.nu == 6 and tuple(model.actuator(i).name for i in range(model.nu)) == MOTOR_NAMES and
            np.linalg.norm(net_force) < 1.0e-10 and np.linalg.norm(net_moment) < 1.0e-10 and
            base_generalized_force_max < 1.0e-10 and
            max(abs(value) for value in virtual_work_errors.values()) < 2.0e-5 and
            callback_error < 1.0e-10 and
            render_model.nu == 6 and render_model.ntendon == 2 and
            np.max(np.abs(render_model.tendon_stiffness)) == 0.0 and
            render_passive_max < 1.0e-10 and gas_after_second_load_error < 1.0e-10 and
            max(constraint_residual.values()) < 1.0e-8 and
            max(abs(np.asarray(constrained_motor_torque[side]) - projected_by_helper[side]).max()
                for side in ("left", "right")) < 1.0e-10 and
            max(abs(np.asarray(constrained_motor_torque[side]) - equivalent_motor_torque[side]).max()
                for side in ("left", "right")) < 2.0 and
            min(extension_work.values()) > 0.0 and
            excluded_pairs <= original_contact_pairs and
            not (excluded_pairs & corrected_contact_pairs) and
            original_contact_pairs - excluded_pairs == corrected_contact_pairs and
            floor_pairs_original == floor_pairs_corrected and floor_pairs_corrected and
            corrected_model.nexclude == model.nexclude + len(LINK_COLLISION_EXCLUDES) and
            corrected_model.ngeom == model.ngeom + collision_info["added_collision_geoms"] and
            collision_info["enabled"] and collision_info["added_collision_geoms"] == 74 and
            structural_arrays_equal and corrected_gas_error < 1.0e-10
        ),
        "gas_force_n": DEFAULT_GAS_FORCE_N,
        "force_model": "constant reference force; real stroke and force curve unknown",
        "nu": model.nu,
        "motor_order": list(MOTOR_NAMES),
        "spring_lengths_m": {item["side"]: item["length_m"] for item in springs},
        "joint_passive_torque_nm": passive_torque,
        "equivalent_motor_torque_nm": equivalent_motor_torque,
        "constraint_projected_motor_torque_nm": constrained_motor_torque,
        "constraint_velocity_residual": constraint_residual,
        "extension_virtual_work_n": extension_work,
        "ideal_four_bar_vs_cad_max_error_nm": max(
            abs(np.asarray(constrained_motor_torque[side]) - equivalent_motor_torque[side]).max()
            for side in ("left", "right")),
        "net_force_norm_n": float(np.linalg.norm(net_force)),
        "net_moment_norm_nm": float(np.linalg.norm(net_moment)),
        "base_generalized_force_max": base_generalized_force_max,
        "virtual_work_error_nm": virtual_work_errors,
        "passive_callback_error_nm": callback_error,
        "render_model_passive_max_nm": render_passive_max,
        "gas_after_second_load_error_nm": gas_after_second_load_error,
        "link_collision_correction": {
            "default_enabled": False,
            "excluded_body_pairs": [list(pair) for pair in LINK_COLLISION_EXCLUDES],
            "original_contact_pairs": [list(pair) for pair in sorted(original_contact_pairs)],
            "corrected_contact_pairs": [list(pair) for pair in sorted(corrected_contact_pairs)],
            "floor_contacts_preserved": floor_pairs_original == floor_pairs_corrected,
            "other_contacts_preserved": original_contact_pairs - excluded_pairs == corrected_contact_pairs,
            "structure_unchanged": structural_arrays_equal,
            "refinement": collision_info,
            "gas_generalized_force_error_nm": corrected_gas_error,
        },
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
