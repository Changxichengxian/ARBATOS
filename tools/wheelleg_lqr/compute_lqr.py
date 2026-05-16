#!/usr/bin/env python3
"""Compute wheel-leg LQR gains for the small 3510 direct-drive prototype.

The model is a Python port of the reference MATLAB script:
local/reference/wheel-legged-master/打印件版/仿真/simulation/get_k_length.m

It solves the same continuous-time three-body wheel-leg model and fits each
of the 12 LQR gains as a cubic polynomial of leg length.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class WheelLegParams:
    # Five-bar geometry for VMC, meters.
    upper_link: float = 0.05000
    lower_link: float = 0.11404
    hip_spacing: float = 0.06000

    # Linear model parameters, meters / kg / kg*m^2.
    wheel_radius: float = 0.03275
    total_mass: float = 2.25
    wheel_end_mass_total: float = 0.320
    leg_mass_total: float = 0.120
    body_length: float = 0.115
    body_height: float = 0.055
    body_com_to_hip: float = -0.004

    # LQR generation range, meters.
    leg_min: float = 0.085
    leg_max: float = 0.120
    leg_step: float = 0.005

    # Motor torque limits, N*m. Used for reporting only.
    wheel_torque_peak: float = 0.45
    wheel_torque_continuous: float = 0.18


def continuous_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Solve continuous LQR without scipy, using the Hamiltonian eigenspace."""
    n = A.shape[0]
    H = np.block(
        [
            [A, -B @ np.linalg.inv(R) @ B.T],
            [-Q, -A.T],
        ]
    )
    eigvals, eigvecs = np.linalg.eig(H)
    stable = np.where(np.real(eigvals) < 0.0)[0]
    if len(stable) != n:
        raise RuntimeError(f"Expected {n} stable eigenvalues, got {len(stable)}")

    V = eigvecs[:, stable]
    U1 = V[:n, :]
    U2 = V[n:, :]
    P = np.real_if_close(U2 @ np.linalg.inv(U1), tol=1000).real
    P = 0.5 * (P + P.T)
    return np.linalg.inv(R) @ B.T @ P


def linear_model(params: WheelLegParams, leg_length: float) -> tuple[np.ndarray, np.ndarray]:
    """Build the A/B matrices from the reference MATLAB model."""
    Rw = params.wheel_radius
    L = leg_length / 2.0
    LM = leg_length / 2.0
    # Reference variable l: body center of mass distance from hip axis.
    # Only the magnitude enters the linearized equations.
    com_distance = abs(params.body_com_to_hip)
    mw = params.wheel_end_mass_total
    mp = params.leg_mass_total
    M = params.total_mass - mw - mp
    g = 9.8

    Iw = 0.5 * mw * Rw**2
    Ip = mp * (leg_length**2 + params.hip_spacing**2) / 12.0
    IM = M * (params.body_length**2 + params.body_height**2) / 12.0

    wheel_den = Iw / Rw + mw * Rw
    accel_theta_coeff = M * (L + LM) + mp * L

    solve_mat = np.array(
        [
            [
                Rw * accel_theta_coeff,
                wheel_den + Rw * (M + mp),
                -Rw * M * com_distance,
            ],
            [
                Ip + L * accel_theta_coeff + LM * M * (L + LM),
                L * (M + mp) + LM * M,
                -M * com_distance * (L + LM),
            ],
            [
                -com_distance * M * (L + LM),
                -com_distance * M,
                IM + M * com_distance**2,
            ],
        ],
        dtype=float,
    )

    gravity_theta = (M + mp) * g * L + M * g * LM

    state_rhs = np.zeros((3, 6), dtype=float)
    state_rhs[1, 0] = gravity_theta
    state_rhs[2, 4] = M * g * com_distance

    input_rhs = np.array(
        [
            [1.0, 0.0],
            [-1.0, 1.0],
            [0.0, 1.0],
        ],
        dtype=float,
    )

    accel_state = np.linalg.solve(solve_mat, state_rhs)
    accel_input = np.linalg.solve(solve_mat, input_rhs)

    A = np.zeros((6, 6), dtype=float)
    B = np.zeros((6, 2), dtype=float)
    A[0, 1] = 1.0
    A[2, 3] = 1.0
    A[4, 5] = 1.0
    A[[1, 3, 5], :] = accel_state
    B[[1, 3, 5], :] = accel_input
    return A, B


def leg_lengths(params: WheelLegParams) -> np.ndarray:
    count = int(round((params.leg_max - params.leg_min) / params.leg_step)) + 1
    return params.leg_min + params.leg_step * np.arange(count)


def generate(params: WheelLegParams, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lengths = leg_lengths(params)
    gains = []
    for length in lengths:
        A, B = linear_model(params, length)
        gains.append(continuous_lqr(A, B, Q, R).reshape(-1))

    gains_arr = np.asarray(gains)
    coeffs = np.asarray([np.polyfit(lengths, gains_arr[:, i], 3) for i in range(12)])
    return lengths, gains_arr, coeffs


def c_array(name: str, coeffs: np.ndarray) -> str:
    lines = [f"float {name}[12][4] = {{"]
    for row in coeffs:
        lines.append("    {" + ", ".join(f"{v:.6g}f" for v in row) + "},")
    lines.append("};")
    return "\n".join(lines)


def torque_diagnostics(
    lengths: np.ndarray,
    gains: np.ndarray,
    params: WheelLegParams,
) -> list[str]:
    """Human-readable static torque checks at the shortest leg length."""
    length = float(lengths[0])
    wheel = gains[0, :6]
    hip = gains[0, 6:]
    deg10 = np.deg2rad(10.0)

    def limit_angle(limit: float, coeff: float) -> float:
        return float(np.rad2deg(limit / abs(coeff))) if abs(coeff) > 1e-9 else float("inf")

    lines = [
        "## Short-leg static torque checks",
        "",
        f"These checks use the shortest fitted leg length, L0={length:.3f} m.",
        "They are only first-instant linear estimates, not a guarantee that the real robot can recover.",
        "",
        "| Static error | wheel torque T | hip virtual torque Tp |",
        "| --- | ---: | ---: |",
        f"| body pitch phi = 10 deg | {wheel[4] * deg10:.4f} N*m | {hip[4] * deg10:.4f} N*m |",
        f"| leg angle theta = 10 deg | {wheel[0] * deg10:.4f} N*m | {hip[0] * deg10:.4f} N*m |",
        f"| wheel position x = 10 cm | {wheel[2] * 0.10:.4f} N*m | {hip[2] * 0.10:.4f} N*m |",
        f"| wheel speed dx = 0.5 m/s | {wheel[3] * 0.50:.4f} N*m | {hip[3] * 0.50:.4f} N*m |",
        "",
        "| Wheel torque limit check | body pitch only | leg angle only |",
        "| --- | ---: | ---: |",
        f"| continuous {params.wheel_torque_continuous:.2f} N*m | {limit_angle(params.wheel_torque_continuous, wheel[4]):.1f} deg | {limit_angle(params.wheel_torque_continuous, wheel[0]):.1f} deg |",
        f"| peak {params.wheel_torque_peak:.2f} N*m | {limit_angle(params.wheel_torque_peak, wheel[4]):.1f} deg | {limit_angle(params.wheel_torque_peak, wheel[0]):.1f} deg |",
        "",
    ]
    return lines


def report_case(case_name: str, params: WheelLegParams, Q: np.ndarray, R: np.ndarray) -> str:
    lengths, gains, coeffs = generate(params, Q, R)

    body_mass = params.total_mass - params.wheel_end_mass_total - params.leg_mass_total
    IM = body_mass * (params.body_length**2 + params.body_height**2) / 12.0
    continuous_accel = 2.0 * params.wheel_torque_continuous / (
        params.total_mass * params.wheel_radius
    )
    peak_accel = 2.0 * params.wheel_torque_peak / (params.total_mass * params.wheel_radius)

    lines = [
        f"# {case_name}",
        "",
        "## Assumed parameters",
        f"- total_mass = {params.total_mass:.3f} kg",
        f"- body_mass = {body_mass:.3f} kg",
        f"- wheel_radius = {params.wheel_radius:.5f} m",
        f"- wheel_end_mass_total = {params.wheel_end_mass_total:.3f} kg",
        f"- leg_mass_total = {params.leg_mass_total:.3f} kg",
        f"- body_com_to_hip = {params.body_com_to_hip:.5f} m",
        f"- body_pitch_inertia_rough = {IM:.6g} kg*m^2",
        f"- leg_range = {params.leg_min:.3f}..{params.leg_max:.3f} m, step {params.leg_step:.3f} m",
        f"- continuous horizontal accel estimate = {continuous_accel:.3f} m/s^2",
        f"- peak horizontal accel estimate = {peak_accel:.3f} m/s^2",
        "",
        "## Q/R",
        f"- Q = diag({np.diag(Q).tolist()})",
        f"- R = {R.tolist()}",
        "",
        "## K samples",
    ]

    for idx in [0, len(lengths) // 2, len(lengths) - 1]:
        lines.append(f"- L0={lengths[idx]:.3f}: " + ", ".join(f"{v:.4f}" for v in gains[idx]))

    lines += [""] + torque_diagnostics(lengths, gains, params) + [
        "",
        "## C coefficients",
        "```c",
        c_array(f"Poly_Coefficient_{case_name}", coeffs),
        "```",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    params = WheelLegParams()
    cases = {
        "soft_R40": (
            np.diag([1.0, 0.07, 3.0, 2.0, 300.0, 0.6]),
            np.diag([40.0, 1.0]),
        ),
        "baseline_R60": (
            np.diag([1.0, 0.07, 3.0, 2.0, 300.0, 0.6]),
            np.diag([60.0, 1.0]),
        ),
        "gentle_R80": (
            np.diag([1.0, 0.07, 3.0, 2.0, 300.0, 0.6]),
            np.diag([80.0, 1.0]),
        ),
    }

    output = [
        "# Small 3510 Wheel-Leg LQR Run",
        "",
        "This file is generated by `tools/wheelleg_lqr/compute_lqr.py`.",
        "",
        "## VMC geometry",
        "```c",
        f"vmc->l1 = {params.upper_link:.5f}f;",
        f"vmc->l2 = {params.lower_link:.5f}f;",
        f"vmc->l3 = {params.lower_link:.5f}f;",
        f"vmc->l4 = {params.upper_link:.5f}f;",
        f"vmc->l5 = {params.hip_spacing:.5f}f;",
        "```",
        "",
    ]

    for name, (Q, R) in cases.items():
        output.append(report_case(name, params, Q, R))

    out_path = Path(__file__).with_name("small_3510_lqr_report.md")
    out_path.write_text("\n".join(output), encoding="utf-8")
    print(out_path)


if __name__ == "__main__":
    main()
