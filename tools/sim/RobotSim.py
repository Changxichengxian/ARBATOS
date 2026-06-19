#!/usr/bin/env python3
"""Static pressure simulator for ARBATOS robot profiles.

The tool reads the checked-in project configuration and estimates CAN traffic,
receive backlog, and CPU budget pressure. It is intentionally conservative and
does not require flashing firmware.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]

CAN_BITRATE_BPS = 1_000_000
DEFAULT_CAN_BITS_PER_FRAME = 135
DEFAULT_DURATION_MS = 1000
DEFAULT_RX_US_PER_FRAME = 6.0
DEFAULT_TX_US_PER_FRAME = 8.0
DEFAULT_WHEELLEG_MIT_BUDGET_US = 1500.0
DEFAULT_SHOOT_BUDGET_US = 500.0
CAN_WARN_UTIL = 0.70
CAN_FAIL_UTIL = 1.00

RM_GROUP_PROTOCOL = "MOTOR_PROTOCOL_RM_GROUP"
MIT_PROTOCOLS = {
    "MOTOR_PROTOCOL_DM_3MODE",
    "MOTOR_PROTOCOL_DM_EXT_V1",
    "MOTOR_PROTOCOL_DM_EXT_V2",
}
RS485_PROTOCOLS = {
    "MOTOR_PROTOCOL_UNITREE_RS485",
    "MOTOR_PROTOCOL_N6014B_RS485",
}

MOTOR_MODELS: dict[str, dict[str, Any]] = {
    "MOTOR_MODEL_3508": {"base": 0x200, "protocol": RM_GROUP_PROTOCOL, "transport": "MOTOR_TRANSPORT_CAN"},
    "MOTOR_MODEL_3510": {"base": 0x200, "protocol": RM_GROUP_PROTOCOL, "transport": "MOTOR_TRANSPORT_CAN"},
    "MOTOR_MODEL_2006": {"base": 0x200, "protocol": RM_GROUP_PROTOCOL, "transport": "MOTOR_TRANSPORT_CAN"},
    "MOTOR_MODEL_6020": {"base": 0x204, "protocol": RM_GROUP_PROTOCOL, "transport": "MOTOR_TRANSPORT_CAN"},
    "MOTOR_MODEL_6623": {"base": 0x204, "protocol": RM_GROUP_PROTOCOL, "transport": "MOTOR_TRANSPORT_CAN"},
    "MOTOR_MODEL_DM_J4310_2EC_V11": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_3MODE",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
    "MOTOR_MODEL_DM_J4310_2EC_V12": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_EXT_V2",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
    "MOTOR_MODEL_DM_J8009_2EC_V10": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_3MODE",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
    "MOTOR_MODEL_DM_J8006_2EC_V11": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_3MODE",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
    "MOTOR_MODEL_DM_J8006_2EC_V10": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_EXT_V1",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
    "MOTOR_MODEL_UNITREE_GO_M8010_6": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_UNITREE_RS485",
        "transport": "MOTOR_TRANSPORT_RS485",
    },
    "MOTOR_MODEL_N6014B": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_N6014B_RS485",
        "transport": "MOTOR_TRANSPORT_RS485",
    },
    "MOTOR_MODEL_DM_H3510_V10": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_EXT_V2",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
    "MOTOR_MODEL_DM_6215": {
        "base": 0x000,
        "protocol": "MOTOR_PROTOCOL_DM_3MODE",
        "transport": "MOTOR_TRANSPORT_CAN",
    },
}

MOTOR_ID_BY_NAME: dict[str, int] = {
    "Motor0": 0,
    "Motor1": 1,
    "Motor2": 2,
    "Motor3": 3,
    "Motor4": 4,
    "Motor5": 5,
    "Motor6": 6,
    "Motor7": 7,
    "Motor8": 8,
    "Motor9": 9,
    "Motor10": 10,
    "Motor11": 11,
    "Motor12": 12,
    "Motor13": 13,
    "Motor14": 14,
    "Motor15": 15,
    "Motor16": 16,
    "Motor17": 17,
}
MOTOR_NAME_BY_ID = {value: key for key, value in MOTOR_ID_BY_NAME.items()}

PROTOCOL_NAMES = {
    "MOTOR_PROTOCOL_INHERIT",
    "MOTOR_PROTOCOL_RM_GROUP",
    "MOTOR_PROTOCOL_DM_3MODE",
    "MOTOR_PROTOCOL_DM_EXT_V1",
    "MOTOR_PROTOCOL_DM_EXT_V2",
    "MOTOR_PROTOCOL_UNITREE_RS485",
    "MOTOR_PROTOCOL_N6014B_RS485",
}
CONTROL_MODE_NAMES = {
    "MOTOR_CONTROL_MODE_INHERIT",
    "MOTOR_CONTROL_MODE_CURRENT",
    "MOTOR_CONTROL_MODE_MIT",
    "MOTOR_CONTROL_MODE_SPEED",
    "MOTOR_CONTROL_MODE_POS_VEL",
    "MOTOR_CONTROL_MODE_FORCE_POS",
}
TRANSPORT_NAMES = {
    "MOTOR_TRANSPORT_INHERIT",
    "MOTOR_TRANSPORT_CAN",
    "MOTOR_TRANSPORT_RS485",
}

ROLE_SPECS = [
    ("chassis", "motor.chassis", 4, 0, 1),
    ("friction", "motor.friction", 4, 8, 2),
    ("yaw", "motor.yaw", 1, 4, 1),
    ("yaw_upper", "motor.yaw_upper", 1, 5, 1),
    ("pitch", "motor.pitch", 1, 6, 1),
    ("trigger", "motor.trigger", 1, 7, 1),
    ("arm", "motor.arm", 6, 12, 0),
]


@dataclass
class MotorNode:
    name: str
    role: str
    index: int
    actuator_id: int
    fallback_bus: int
    model: str = "MOTOR_MODEL__NONE"
    can_id: int = 0
    can_bus: int = 0
    protocol: str = "MOTOR_PROTOCOL_INHERIT"
    control_mode: str = "MOTOR_CONTROL_MODE_INHERIT"
    master_id: int = 0
    transport: str = "MOTOR_TRANSPORT_INHERIT"
    feedback_id: int = 0
    feedback_id_enable: int = 0

    @property
    def enabled(self) -> bool:
        return self.can_id != 0 and self.model in MOTOR_MODELS

    @property
    def bus(self) -> int:
        return self.can_bus if self.can_bus in (1, 2, 3) else self.fallback_bus

    @property
    def default(self) -> dict[str, Any]:
        return MOTOR_MODELS.get(self.model, {})

    @property
    def resolved_protocol(self) -> str:
        if self.protocol != "MOTOR_PROTOCOL_INHERIT":
            return self.protocol
        return str(self.default.get("protocol", RM_GROUP_PROTOCOL))

    @property
    def resolved_transport(self) -> str:
        if self.transport != "MOTOR_TRANSPORT_INHERIT":
            return self.transport
        return str(self.default.get("transport", "MOTOR_TRANSPORT_CAN"))

    @property
    def std_id(self) -> int:
        if not self.enabled or self.resolved_transport != "MOTOR_TRANSPORT_CAN":
            return 0
        return int(self.default.get("base", 0)) + self.can_id

    @property
    def feedback_std_id(self) -> int:
        if self.feedback_id_enable:
            return self.feedback_id
        if self.master_id:
            return self.master_id
        return self.std_id

    @property
    def is_can(self) -> bool:
        return self.enabled and self.resolved_transport == "MOTOR_TRANSPORT_CAN"

    @property
    def is_rm_group(self) -> bool:
        return self.is_can and self.resolved_protocol == RM_GROUP_PROTOCOL

    @property
    def is_mit_can(self) -> bool:
        return self.is_can and self.resolved_protocol in MIT_PROTOCOLS


@dataclass
class OperationConfig:
    mode: str = "ROBOT_RUN_MODE_FULL"
    target_task: str = "ROBOT_TASK_MODULE_NONE"
    target_motor: int = 255
    variant: str = "ROBOT_RUN_VARIANT_NORMAL"


@dataclass
class ProjectConfig:
    project: str
    config_dir: Path
    macros: dict[str, str]
    modules: list[str]
    motors: list[MotorNode]
    periods_ms: dict[str, int]
    wheel_actuator_ids: set[int] = field(default_factory=set)
    project_defines: dict[str, str] = field(default_factory=dict)
    operation: OperationConfig = field(default_factory=OperationConfig)

    def macro_text(self, name: str, fallback: str = "") -> str:
        return self.macros.get(name, fallback)

    def macro_int(self, name: str, fallback: int) -> int:
        return parse_int_expr(self.macro_text(name, str(fallback)), self.macros, fallback)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def read_text_with_local_config_includes(path: Path, stack: tuple[Path, ...] = ()) -> str:
    path = path.resolve()
    if path in stack:
        chain = " -> ".join(str(item) for item in (*stack, path))
        raise ValueError(f"recursive config include: {chain}")

    text = read_text(path)
    include_re = re.compile(r'(?m)^\s*#\s*include\s+"(Config[A-Za-z0-9_]+\.inc)"\s*$')

    def expand(match: re.Match[str]) -> str:
        include_path = path.parent / match.group(1)
        if not include_path.exists():
            raise FileNotFoundError(f"missing config include: {include_path}")
        return read_text_with_local_config_includes(include_path, (*stack, path))

    return include_re.sub(expand, text)


def strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    return text


def parse_define_text(text: str) -> dict[str, str]:
    macros: dict[str, str] = {}
    for match in re.finditer(r"(?m)^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)\s*(.*?)\s*$", text):
        name, value = match.group(1), match.group(2).strip()
        if not value:
            value = "1"
        macros[name] = value
    return macros


def parse_uvprojx_defines(project: str) -> dict[str, str]:
    project_root = REPO_ROOT / "projects" / project
    defines: dict[str, str] = {}
    for uvprojx in project_root.rglob("*.uvprojx"):
        try:
            root = ET.parse(uvprojx).getroot()
        except ET.ParseError:
            continue

        for define_node in root.iter("Define"):
            if not define_node.text:
                continue
            for raw_token in re.split(r"[,;\s]+", define_node.text):
                token = raw_token.strip()
                if not token:
                    continue
                if "=" in token:
                    name, value = token.split("=", 1)
                else:
                    name, value = token, "1"
                if name:
                    defines[name] = value

        for misc_node in root.iter("MiscControls"):
            if not misc_node.text:
                continue
            for match in re.finditer(r"(?:-D|--define)\s*([A-Za-z_][A-Za-z0-9_]*)(?:=([^\s]+))?", misc_node.text):
                defines[match.group(1)] = match.group(2) or "1"
    return defines


def find_matching_brace(text: str, brace_index: int) -> int:
    depth = 0
    in_string = False
    escaped = False
    for i in range(brace_index, len(text)):
        ch = text[i]
        if in_string:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                in_string = False
            continue
        if ch == '"':
            in_string = True
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError("unmatched brace")


def extract_initializer(text: str, field_name: str) -> str | None:
    match = re.search(r"\." + re.escape(field_name) + r"\s*=", text)
    if not match:
        return None
    brace_index = text.find("{", match.end())
    if brace_index < 0:
        return None
    end = find_matching_brace(text, brace_index)
    return text[brace_index + 1 : end]


def split_top_level(text: str) -> list[str]:
    items: list[str] = []
    depth = 0
    start = 0
    in_string = False
    escaped = False
    for i, ch in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                in_string = False
            continue
        if ch == '"':
            in_string = True
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
        elif ch == "," and depth == 0:
            item = text[start:i].strip()
            if item:
                items.append(item)
            start = i + 1
    tail = text[start:].strip()
    if tail:
        items.append(tail)
    return items


def strip_outer_braces(text: str) -> str:
    text = text.strip()
    if text.startswith("{") and text.endswith("}"):
        try:
            if find_matching_brace(text, 0) == len(text) - 1:
                return text[1:-1].strip()
        except ValueError:
            return text
    return text


def parse_int_expr(expr: str | None, macros: dict[str, str], fallback: int = 0) -> int:
    if expr is None:
        return fallback
    cleaned = str(expr).strip()
    cleaned = re.sub(r"\([A-Za-z_][A-Za-z0-9_\s\*]*\)", "", cleaned)
    cleaned = cleaned.strip()
    while cleaned.startswith("(") and cleaned.endswith(")"):
        try:
            if find_matching_brace(cleaned.replace("(", "{").replace(")", "}"), 0) == len(cleaned) - 1:
                cleaned = cleaned[1:-1].strip()
            else:
                break
        except ValueError:
            break
    cleaned = re.sub(r"(?<=\d)[uUlLfF]+", "", cleaned)

    if "<<" in cleaned:
        left, right = cleaned.split("<<", 1)
        return parse_int_expr(left, macros, fallback) << parse_int_expr(right, macros, 0)

    token_match = re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", cleaned)
    if token_match:
        token = token_match.group(0)
        if token in MOTOR_ID_BY_NAME:
            return MOTOR_ID_BY_NAME[token]
        if token in macros and macros[token] != cleaned:
            return parse_int_expr(macros[token], macros, fallback)
        return fallback

    token_match = re.search(r"[A-Za-z_][A-Za-z0-9_]*", cleaned)
    if token_match:
        token = token_match.group(0)
        if token in MOTOR_ID_BY_NAME:
            return MOTOR_ID_BY_NAME[token]
        if token in macros:
            return parse_int_expr(macros[token], macros, fallback)

    number_match = re.search(r"0x[0-9A-Fa-f]+|\d+", cleaned)
    if not number_match:
        return fallback
    return int(number_match.group(0), 0)


def parse_string_macro(value: str | None, fallback: str = "") -> str:
    if value is None:
        return fallback
    match = re.search(r'"([^"]*)"', value)
    if match:
        return match.group(1)
    return value.strip() or fallback


def extract_named_value(init: str, field_name: str) -> str | None:
    match = re.search(r"\." + re.escape(field_name) + r"\s*=\s*([^,}\n]+)", init)
    if not match:
        return None
    return match.group(1).strip()


def extract_named_enum(init: str, field_name: str, allowed: set[str]) -> str | None:
    value = extract_named_value(init, field_name)
    if not value:
        return None
    match = re.search(r"[A-Za-z_][A-Za-z0-9_]*", value)
    if not match:
        return None
    token = match.group(0)
    return token if token in allowed else None


def extract_named_symbol(init: str, field_name: str, fallback: str) -> str:
    value = extract_named_value(init, field_name)
    if not value:
        return fallback
    for token in re.findall(r"[A-Za-z_][A-Za-z0-9_]*", value):
        if token not in {"uint8_t", "uint16_t", "uint32_t", "int8_t", "int16_t", "int32_t"}:
            return token
    return fallback


def parse_model_expr(expr: str | None) -> str | None:
    if not expr:
        return None
    match = re.search(r"MOTOR_MODEL_[A-Za-z0-9_]+", expr)
    return match.group(0) if match else None


def parse_node_initializer(
    init: str | None,
    *,
    name: str,
    role: str,
    index: int,
    actuator_id: int,
    fallback_bus: int,
    macros: dict[str, str],
) -> MotorNode:
    node = MotorNode(name=name, role=role, index=index, actuator_id=actuator_id, fallback_bus=fallback_bus)
    if init is None:
        return node

    body = strip_outer_braces(init).strip()
    if body in ("", "0", "0u"):
        return node

    if "." in body:
        node.model = parse_model_expr(extract_named_value(body, "model")) or node.model
        node.can_id = parse_int_expr(extract_named_value(body, "can_id"), macros, node.can_id)
        node.can_bus = parse_int_expr(extract_named_value(body, "can_bus"), macros, node.can_bus)
        node.protocol = extract_named_enum(body, "protocol", PROTOCOL_NAMES) or node.protocol
        node.control_mode = extract_named_enum(body, "control_mode", CONTROL_MODE_NAMES) or node.control_mode
        node.master_id = parse_int_expr(extract_named_value(body, "master_id"), macros, node.master_id)
        node.transport = extract_named_enum(body, "transport", TRANSPORT_NAMES) or node.transport
        node.feedback_id = parse_int_expr(extract_named_value(body, "feedback_id"), macros, node.feedback_id)
        node.feedback_id_enable = parse_int_expr(
            extract_named_value(body, "feedback_id_enable"), macros, node.feedback_id_enable
        )
        return node

    values = split_top_level(body)
    if values:
        node.model = parse_model_expr(values[0]) or node.model
    if len(values) > 1:
        node.can_id = parse_int_expr(values[1], macros, node.can_id)
    if len(values) > 2:
        node.can_bus = parse_int_expr(values[2], macros, node.can_bus)
    if len(values) > 3:
        protocol = parse_model_expr(values[3])
        if protocol is None:
            match = re.search(r"MOTOR_PROTOCOL_[A-Za-z0-9_]+", values[3])
            protocol = match.group(0) if match else None
        node.protocol = protocol if protocol in PROTOCOL_NAMES else node.protocol
    if len(values) > 4:
        match = re.search(r"MOTOR_CONTROL_MODE_[A-Za-z0-9_]+", values[4])
        node.control_mode = match.group(0) if match and match.group(0) in CONTROL_MODE_NAMES else node.control_mode
    if len(values) > 5:
        node.master_id = parse_int_expr(values[5], macros, node.master_id)
    if len(values) > 6:
        match = re.search(r"MOTOR_TRANSPORT_[A-Za-z0-9_]+", values[6])
        node.transport = match.group(0) if match and match.group(0) in TRANSPORT_NAMES else node.transport
    if len(values) > 10:
        node.feedback_id = parse_int_expr(values[10], macros, node.feedback_id)
    if len(values) > 11:
        node.feedback_id_enable = parse_int_expr(values[11], macros, node.feedback_id_enable)
    return node


def default_actuator_id(base_id: int, index: int) -> int:
    motor_id = base_id + index
    return motor_id if motor_id in MOTOR_NAME_BY_ID else 255


def parse_motor_nodes(config_c: str, macros: dict[str, str]) -> list[MotorNode]:
    motors: list[MotorNode] = []
    motor_body = extract_initializer(config_c, "motor") or ""

    for role, base_name, count, motor_base_id, fallback_bus in ROLE_SPECS:
        if role == "arm":
            effective_fallback = lambda idx: 1 if idx == 0 else 2
        else:
            effective_fallback = lambda _idx, bus=fallback_bus: bus

        field_body = extract_initializer(motor_body, role)
        if count == 1:
            items = [field_body] if field_body is not None else [None]
        else:
            raw_items = split_top_level(field_body or "")
            items = raw_items[:count] + [None] * max(0, count - len(raw_items))

        for index in range(count):
            if role in ("yaw", "yaw_upper", "pitch", "trigger"):
                motor_name = base_name
                actuator_id = default_actuator_id(motor_base_id, index)
            else:
                motor_name = f"{base_name}{index}"
                actuator_id = default_actuator_id(motor_base_id, index)
            motors.append(
                parse_node_initializer(
                    items[index],
                    name=motor_name,
                    role=role,
                    index=index,
                    actuator_id=actuator_id,
                    fallback_bus=effective_fallback(index),
                    macros=macros,
                )
            )
    return motors


def parse_modules(config_c: str) -> list[str]:
    profile_body = extract_initializer(config_c, "profile") or ""
    task_body = extract_initializer(profile_body, "task_modules") or ""
    return list(dict.fromkeys(re.findall(r"ROBOT_TASK_MODULE_[A-Z0-9_]+", task_body)))


def parse_operation(config_c: str, macros: dict[str, str]) -> OperationConfig:
    body = extract_initializer(config_c, "operation") or ""
    return OperationConfig(
        mode=extract_named_symbol(body, "mode", "ROBOT_RUN_MODE_FULL"),
        target_task=extract_named_symbol(body, "target_task", "ROBOT_TASK_MODULE_NONE"),
        target_motor=parse_int_expr(extract_named_value(body, "target_motor"), macros, 255),
        variant=extract_named_symbol(body, "variant", "ROBOT_RUN_VARIANT_NORMAL"),
    )


def parse_periods(config_c: str, macros: dict[str, str]) -> dict[str, int]:
    defaults = {
        "gimbal": parse_int_expr(macros.get("ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS"), macros, 1),
        "chassis": parse_int_expr(macros.get("ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS"), macros, 2),
        "can_tx": parse_int_expr(macros.get("ROBOT_PROFILE_CAN_COMMAND_TX_PERIOD_MS"), macros, 1),
        "watch": parse_int_expr(macros.get("ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS"), macros, 10),
        "WheelLegMit": 3,
        "shoot": 1,
    }
    periods: dict[str, int] = {}
    for block, fallback in defaults.items():
        if block == "can_tx" or block == "watch":
            periods[block] = max(1, fallback)
            continue
        body = extract_initializer(config_c, block)
        raw = extract_named_value(body or "", "control_period_ms")
        periods[block] = max(1, parse_int_expr(raw, macros, fallback))
    return periods


def parse_wheel_actuators(config_c: str, macros: dict[str, str]) -> set[int]:
    body = extract_initializer(config_c, "WheelLegMit") or ""
    ids: set[int] = set()
    for field_name in ("left_wheel_actuator", "right_wheel_actuator"):
        raw = extract_named_value(body, field_name)
        if raw is not None:
            ids.add(parse_int_expr(raw, macros, 255))
    return {item for item in ids if item != 255}


def load_project(project: str) -> ProjectConfig:
    config_dir = REPO_ROOT / "Robotconfig" / project
    config_h_path = config_dir / "RobotConfig.h"
    config_c_path = config_dir / "RobotConfig.c"
    if not config_h_path.exists() or not config_c_path.exists():
        raise FileNotFoundError(f"missing Robotconfig/{project}/config.[ch]")

    profile_defaults = parse_define_text(strip_c_comments(read_text(REPO_ROOT / "shared/application/robot/RobotTaskProfile.h")))
    can_tx_source_macros = parse_define_text(
        strip_c_comments(read_text(REPO_ROOT / "shared/application/comm/can/CanTxTask.c"))
    )
    project_defines = parse_uvprojx_defines(project)
    project_macros = parse_define_text(strip_c_comments(read_text(config_h_path)))
    macros = dict(profile_defaults)
    macros.update(project_defines)
    macros.update(project_macros)
    macros.update(can_tx_source_macros)

    config_c = strip_c_comments(read_text_with_local_config_includes(config_c_path))
    return ProjectConfig(
        project=project,
        config_dir=config_dir,
        macros=macros,
        modules=parse_modules(config_c),
        motors=parse_motor_nodes(config_c, macros),
        periods_ms=parse_periods(config_c, macros),
        wheel_actuator_ids=parse_wheel_actuators(config_c, macros),
        project_defines=project_defines,
        operation=parse_operation(config_c, macros),
    )


def project_module_enabled(project: ProjectConfig, module: str) -> bool:
    return module in set(project.modules)


def project_has_gimbal_task(project: ProjectConfig) -> bool:
    return (
        project_module_enabled(project, "ROBOT_TASK_MODULE_SINGLE_GIMBAL")
        or project_module_enabled(project, "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL")
    )


def project_has_arm_route_owner(project: ProjectConfig) -> bool:
    return (
        project_module_enabled(project, "ROBOT_TASK_MODULE_ARM")
        or project_module_enabled(project, "ROBOT_TASK_MODULE_WHEELLEG_MIT")
    )


def project_shoot_runtime_enabled(project: ProjectConfig) -> bool:
    return project.macro_int("ROBOT_TASK_BUILD_SHOOT_RM", 1) != 0 and project_has_gimbal_task(project)


def motor_route_role_active(project: ProjectConfig, motor: MotorNode) -> bool:
    if motor.role == "arm":
        return project_has_arm_route_owner(project)
    return True


def motor_feedback_active(project: ProjectConfig, motor: MotorNode) -> bool:
    if motor.role == "arm":
        return project_has_arm_route_owner(project)
    return True


def motor_control_task_active(project: ProjectConfig, motor: MotorNode) -> bool:
    if motor.role == "chassis":
        return project_module_enabled(project, "ROBOT_TASK_MODULE_CLASSIC_CHASSIS")
    if motor.role in ("yaw", "pitch"):
        return project_has_gimbal_task(project)
    if motor.role == "yaw_upper":
        return project_module_enabled(project, "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL")
    if motor.role in ("trigger", "friction"):
        return project_shoot_runtime_enabled(project)
    if motor.role == "arm":
        return project_has_arm_route_owner(project)
    return False


def rm_group_id(std_id: int) -> int | None:
    if 0x201 <= std_id <= 0x204:
        return 0x200
    if 0x205 <= std_id <= 0x208:
        return 0x1FF
    return None


def motor_feedback_hz(motor: MotorNode, default_feedback_hz: float) -> float:
    if not motor.is_can:
        return 0.0
    # Current supported CAN motors in this repo are expected to publish at 1 kHz.
    # DJI/RM manuals use 1 kHz feedback; DM MIT motors are modeled the same here.
    return default_feedback_hz


def simulate_mit_tx(
    project: ProjectConfig,
    motors: list[MotorNode],
    duration_ms: int,
    wake_period_ms: int,
) -> dict[str, Any]:
    mit_motors = [motor for motor in motors if motor.is_mit_can]
    if not mit_motors:
        return {
            "fps_by_bus": {},
            "sent_frames": 0,
            "skipped_due_to_budget": 0,
            "max_frames_per_wake": 0,
            "wake_period_ms": max(1, wake_period_ms),
            "motors": [],
        }

    max_per_ms = parse_int_expr(project.macros.get("CAN_TX_MIT_MAX_FRAMES_PER_MS"), project.macros, 3)
    if max_per_ms <= 0:
        max_per_ms = 3
    wheel_period = parse_int_expr(project.macros.get("CAN_TX_MIT_WHEEL_CMD_PERIOD_MS"), project.macros, 2)
    joint_period = parse_int_expr(project.macros.get("CAN_TX_MIT_JOINT_CMD_PERIOD_MS"), project.macros, 5)

    motor_states: list[dict[str, Any]] = []
    for motor in mit_motors:
        period = wheel_period if motor.actuator_id in project.wheel_actuator_ids else joint_period
        motor_states.append({"motor": motor, "period_ms": max(1, period), "last_sent_ms": None, "sent": 0, "skipped": 0})

    fps_by_bus: dict[int, float] = {}
    max_seen = 0
    skipped = 0
    sent_total = 0
    wake_period_ms = max(1, wake_period_ms)
    for now_ms in range(0, duration_ms, wake_period_ms):
        used = 0
        for state in motor_states:
            last = state["last_sent_ms"]
            if last is None or now_ms - int(last) >= int(state["period_ms"]):
                if used < max_per_ms:
                    used += 1
                    sent_total += 1
                    state["sent"] += 1
                    state["last_sent_ms"] = now_ms
                    motor = state["motor"]
                    fps_by_bus[motor.bus] = fps_by_bus.get(motor.bus, 0.0) + 1000.0 / float(duration_ms)
                else:
                    skipped += 1
                    state["skipped"] += 1
        max_seen = max(max_seen, used)

    return {
        "fps_by_bus": {str(bus): fps for bus, fps in sorted(fps_by_bus.items())},
        "sent_frames": sent_total,
        "skipped_due_to_budget": skipped,
        "max_frames_per_wake": max_seen,
        "max_allowed_frames_per_wake": max_per_ms,
        "wake_period_ms": wake_period_ms,
        "motors": [
            {
                "name": state["motor"].name,
                "period_ms": state["period_ms"],
                "sent_frames": state["sent"],
                "skipped_due_to_budget": state["skipped"],
            }
            for state in motor_states
        ],
    }


def simulate_rx_queue(total_rx_fps: float, duration_ms: int, frames_per_wake: int) -> dict[str, Any]:
    backlog = 0.0
    carry = 0.0
    max_backlog = 0.0
    processed_total = 0
    arrived_total = 0
    per_ms = total_rx_fps / 1000.0
    for _ in range(duration_ms):
        carry += per_ms
        arrivals = int(math.floor(carry))
        carry -= arrivals
        arrived_total += arrivals
        backlog += arrivals
        processed = min(int(backlog), frames_per_wake)
        processed_total += processed
        backlog -= processed
        max_backlog = max(max_backlog, backlog)
    return {
        "arrived_frames": arrived_total,
        "processed_frames": processed_total,
        "end_backlog_frames": round(backlog, 3),
        "max_backlog_frames": round(max_backlog, 3),
        "frames_per_wake": frames_per_wake,
    }


def add_cpu_item(items: list[dict[str, Any]], name: str, budget_us: float, period_ms: float, source: str) -> None:
    if budget_us <= 0.0 or period_ms <= 0.0:
        return
    calls_per_s = 1000.0 / period_ms
    us_per_s = budget_us * calls_per_s
    items.append(
        {
            "name": name,
            "budget_us": budget_us,
            "period_ms": period_ms,
            "calls_per_s": calls_per_s,
            "cpu_percent": us_per_s / 10000.0,
            "source": source,
        }
    )


def build_cpu_report(
    project: ProjectConfig,
    total_rx_fps: float,
    total_tx_fps: float,
    rx_us_per_frame: float,
    tx_us_per_frame: float,
) -> dict[str, Any]:
    budget_items: list[dict[str, Any]] = []
    modules = set(project.modules)
    macros = project.macros

    if "ROBOT_TASK_MODULE_SINGLE_GIMBAL" in modules or "ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL" in modules:
        add_cpu_item(
            budget_items,
            "gimbal_control",
            parse_int_expr(macros.get("ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US"), macros, 700),
            project.periods_ms["gimbal"],
            "RtProf budget",
        )
    if "ROBOT_TASK_MODULE_CLASSIC_CHASSIS" in modules:
        add_cpu_item(
            budget_items,
            "classic_chassis",
            parse_int_expr(macros.get("ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US"), macros, 1200),
            project.periods_ms["chassis"],
            "RtProf budget",
        )
    if "ROBOT_TASK_MODULE_WHEELLEG_MIT" in modules:
        add_cpu_item(
            budget_items,
            "WheelLegMit",
            DEFAULT_WHEELLEG_MIT_BUDGET_US,
            project.periods_ms["WheelLegMit"],
            "sim default until profiler budget is added",
        )
    if project_shoot_runtime_enabled(project):
        add_cpu_item(
            budget_items,
            "shoot_control",
            DEFAULT_SHOOT_BUDGET_US,
            project.periods_ms["shoot"],
            "sim default until profiler budget is added",
        )
    if "ROBOT_TASK_MODULE_CAN_COMMAND_TX" in modules:
        add_cpu_item(
            budget_items,
            "can_command_tx",
            parse_int_expr(macros.get("ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US"), macros, 300),
            project.periods_ms["can_tx"],
            "RtProf budget",
        )
    if "ROBOT_TASK_MODULE_CAN_FEEDBACK_RX" in modules:
        add_cpu_item(
            budget_items,
            "can_feedback_rx",
            parse_int_expr(macros.get("ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US"), macros, 300),
            1.0,
            "RtProf budget",
        )
    if "ROBOT_TASK_MODULE_SDLOG" in modules:
        add_cpu_item(
            budget_items,
            "SdLogWrite_fast_path",
            parse_int_expr(macros.get("ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US"), macros, 50),
            1.0,
            "RtProf write budget estimate",
        )
    add_cpu_item(
        budget_items,
        "watch_task_beat",
        parse_int_expr(macros.get("ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US"), macros, 10),
        project.periods_ms["watch"],
        "RtProf budget",
    )

    traffic_items = [
        {
            "name": "can_rx_frame_handling",
            "frames_per_s": total_rx_fps,
            "us_per_frame": rx_us_per_frame,
            "cpu_percent": (total_rx_fps * rx_us_per_frame) / 10000.0,
        },
        {
            "name": "can_tx_frame_build_send",
            "frames_per_s": total_tx_fps,
            "us_per_frame": tx_us_per_frame,
            "cpu_percent": (total_tx_fps * tx_us_per_frame) / 10000.0,
        },
    ]

    return {
        "traffic_estimate": {
            "items": traffic_items,
            "cpu_percent": sum(item["cpu_percent"] for item in traffic_items),
        },
        "task_budget_envelope": {
            "items": budget_items,
            "cpu_percent": sum(item["cpu_percent"] for item in budget_items),
        },
    }


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    project = load_project(args.project)
    duration_ms = max(1, args.duration_ms)
    if args.can_tx_period_ms is not None:
        project.periods_ms["can_tx"] = max(1, int(args.can_tx_period_ms))
    can_bus_count = project.macro_int("ROBOT_BOARD_CAN_BUS_COUNT", 2)
    bus_ids = list(range(1, max(1, can_bus_count) + 1))

    rx_can_motors = [motor for motor in project.motors if motor.is_can and motor_feedback_active(project, motor)]
    rm_tx_motors = [motor for motor in project.motors if motor.is_rm_group]
    active_tx_route_motors = [
        motor for motor in project.motors if motor.is_can and motor_route_role_active(project, motor)
    ]
    rx_fps_by_bus: dict[int, float] = {bus: 0.0 for bus in bus_ids}
    for motor in rx_can_motors:
        rx_fps_by_bus[motor.bus] = rx_fps_by_bus.get(motor.bus, 0.0) + motor_feedback_hz(
            motor, args.motor_feedback_hz
        )

    rm_tx_groups: dict[int, set[int]] = {bus: set() for bus in bus_ids}
    for motor in rm_tx_motors:
        group_id = rm_group_id(motor.std_id)
        if group_id is not None:
            rm_tx_groups.setdefault(motor.bus, set()).add(group_id)

    can_tx_period_ms = project.periods_ms["can_tx"]
    tx_fps_by_bus: dict[int, float] = {bus: 0.0 for bus in bus_ids}
    rm_loop_fps = 1000.0 / float(can_tx_period_ms)
    for bus, groups in rm_tx_groups.items():
        tx_fps_by_bus[bus] = tx_fps_by_bus.get(bus, 0.0) + len(groups) * rm_loop_fps

    mit_tx = simulate_mit_tx(project, active_tx_route_motors, duration_ms, can_tx_period_ms)
    for bus_text, fps in mit_tx["fps_by_bus"].items():
        bus = int(bus_text)
        tx_fps_by_bus[bus] = tx_fps_by_bus.get(bus, 0.0) + float(fps)

    can_buses: list[dict[str, Any]] = []
    risk_level = "ok"
    risk_notes: list[str] = []
    for bus in sorted(set(bus_ids) | set(rx_fps_by_bus) | set(tx_fps_by_bus)):
        rx_fps = rx_fps_by_bus.get(bus, 0.0)
        tx_fps = tx_fps_by_bus.get(bus, 0.0)
        total_fps = rx_fps + tx_fps
        utilization = total_fps * float(args.can_bits_per_frame) / float(CAN_BITRATE_BPS)
        level = "ok"
        if utilization >= CAN_FAIL_UTIL:
            level = "fail"
            risk_level = "fail"
            risk_notes.append(f"CAN{bus} utilization is above 100%.")
        elif utilization >= CAN_WARN_UTIL:
            level = "warn"
            if risk_level != "fail":
                risk_level = "warn"
            risk_notes.append(f"CAN{bus} utilization is above {int(CAN_WARN_UTIL * 100)}%.")
        can_buses.append(
            {
                "bus": bus,
                "rx_fps": rx_fps,
                "tx_fps": tx_fps,
                "total_fps": total_fps,
                "utilization_percent": utilization * 100.0,
                "risk": level,
                "rm_groups": sorted(hex(group_id) for group_id in rm_tx_groups.get(bus, set())),
            }
        )

    if int(mit_tx.get("skipped_due_to_budget", 0)) > 0:
        if risk_level == "ok":
            risk_level = "warn"
        risk_notes.append("MIT command scheduler skipped due frames because of the per-wake frame cap.")

    total_rx_fps = sum(rx_fps_by_bus.values())
    total_tx_fps = sum(tx_fps_by_bus.values())
    rx_queue = simulate_rx_queue(
        total_rx_fps,
        duration_ms,
        max(1, parse_int_expr(project.macros.get("ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE"), project.macros, 32)),
    )
    if rx_queue["end_backlog_frames"] > 0:
        if risk_level != "fail":
            risk_level = "warn"
        risk_notes.append("CAN feedback receive work cannot drain all simulated frames within the configured per-wake cap.")

    cpu = build_cpu_report(project, total_rx_fps, total_tx_fps, args.rx_us_per_frame, args.tx_us_per_frame)
    if cpu["traffic_estimate"]["cpu_percent"] >= 30.0:
        if risk_level != "fail":
            risk_level = "warn"
        risk_notes.append("Estimated CAN frame handling alone is above 30% CPU.")
    if cpu["task_budget_envelope"]["cpu_percent"] >= 100.0:
        if risk_level == "ok":
            risk_level = "warn"
        risk_notes.append("Configured task budget envelope is above 100%; this is an upper bound, not measured CPU.")

    simulated_motors = [
        {
            "name": motor.name,
            "actuator": MOTOR_NAME_BY_ID.get(motor.actuator_id, str(motor.actuator_id)),
            "model": motor.model,
            "bus": motor.bus,
            "can_id": motor.can_id,
            "std_id": hex(motor.std_id) if motor.std_id else "0x0",
            "feedback_id": hex(motor.feedback_std_id) if motor.feedback_std_id else "0x0",
            "protocol": motor.resolved_protocol,
            "transport": motor.resolved_transport,
            "feedback_active": motor_feedback_active(project, motor),
            "tx_route_active": motor_route_role_active(project, motor),
            "control_task_active": motor_control_task_active(project, motor),
        }
        for motor in project.motors
        if motor.is_can
        and (
            motor_feedback_active(project, motor)
            or motor_route_role_active(project, motor)
            or motor.is_rm_group
        )
    ]

    assumptions = [
        "CAN bus bitrate is modeled as 1 Mbps classical CAN.",
        f"CAN frame cost uses {args.can_bits_per_frame} bits/frame including arbitration and overhead.",
        f"Profile-active CAN motor feedback is modeled at {args.motor_feedback_hz:g} Hz.",
        "MIT TX simulation only includes profile-active routes and assumes steady-state command pressure.",
        "RM group TX pressure counts configured groups because firmware emits zero-current frames for configured RM groups.",
        "CPU traffic estimate uses CLI frame costs; task budget envelope uses profiler budgets and sim defaults.",
    ]

    return {
        "project": {
            "name": project.project,
            "target_name": parse_string_macro(project.macro_text("ARBATOS_TARGET_NAME"), project.project),
            "profile_kind": project.macro_text("ROBOT_PROFILE_KIND"),
            "board_name": parse_string_macro(project.macro_text("ARBATOS_BOARD_NAME"), "unknown"),
            "board_kind": project.macro_text("ROBOT_BOARD_KIND"),
            "cpu_hz": project.macro_int("ROBOT_BOARD_CPU_HZ", 0),
            "can_bus_count": can_bus_count,
            "can_tx_period_ms": can_tx_period_ms,
            "can_tx_period_ms_source": "cli override" if args.can_tx_period_ms is not None else "code",
            "motor_feedback_hz": args.motor_feedback_hz,
            "duration_ms": duration_ms,
            "operation": {
                "mode": project.operation.mode,
                "target_task": project.operation.target_task,
                "target_motor": project.operation.target_motor,
                "variant": project.operation.variant,
            },
            "project_define_overrides": {
                key: value
                for key, value in sorted(project.project_defines.items())
                if key.startswith("ROBOT_PROFILE_") or key.startswith("CAN_TX_")
            },
        },
        "modules": project.modules,
        "periods_ms": project.periods_ms,
        "motors": simulated_motors,
        "can": {
            "bits_per_frame": args.can_bits_per_frame,
            "buses": can_buses,
            "rx_queue": rx_queue,
            "mit_tx_scheduler": mit_tx,
        },
        "cpu": cpu,
        "risk": {"level": risk_level, "notes": risk_notes},
        "assumptions": assumptions,
    }


def format_percent(value: float) -> str:
    return f"{value:.1f}%"


def format_fps(value: float) -> str:
    if abs(value - round(value)) < 0.01:
        return str(int(round(value)))
    return f"{value:.1f}"


def print_report(report: dict[str, Any]) -> None:
    project = report["project"]
    print(f"ARBATOS simulation: {project['name']}")
    print(
        f"board: {project['board_name']} {project['board_kind']} "
        f"cpu={project['cpu_hz']}Hz can_buses={project['can_bus_count']}"
    )
    print(f"CAN TX period: {project['can_tx_period_ms']} ms ({project['can_tx_period_ms_source']})")
    print(f"motor feedback: {format_fps(project['motor_feedback_hz'])} Hz")
    operation = project.get("operation", {})
    if operation:
        print(
            "operation: "
            f"{operation.get('mode', 'unknown')} "
            f"target_task={operation.get('target_task', 'unknown')} "
            f"target_motor={operation.get('target_motor', 'unknown')} "
            f"variant={operation.get('variant', 'unknown')}"
        )
    if project["project_define_overrides"]:
        print(f"project define overrides: {project['project_define_overrides']}")
    print()

    print("simulated motor traffic:")
    for motor in report["motors"]:
        status = ",".join(
            name
            for name, active in (
                ("rx", motor.get("feedback_active", False)),
                ("tx", motor.get("tx_route_active", False)),
                ("ctrl", motor.get("control_task_active", False)),
            )
            if active
        )
        print(
            f"  CAN{motor['bus']} {motor['name']:<16} {motor['model']:<30} "
            f"cmd_id={motor['std_id']:<6} fb_id={motor['feedback_id']:<6} "
            f"{motor['protocol']} [{status or 'configured'}]"
        )
    if not report["motors"]:
        print("  none")
    print()

    print("CAN pressure:")
    print("  bus   rx_fps  tx_fps  total_fps  util   risk   rm_groups")
    for bus in report["can"]["buses"]:
        print(
            f"  CAN{bus['bus']:<2} {format_fps(bus['rx_fps']):>7} "
            f"{format_fps(bus['tx_fps']):>7} {format_fps(bus['total_fps']):>10} "
            f"{format_percent(bus['utilization_percent']):>7} {bus['risk']:<6} "
            f"{','.join(bus['rm_groups']) if bus['rm_groups'] else '-'}"
        )
    queue = report["can"]["rx_queue"]
    print(
        "  rx queue: "
        f"cap={queue['frames_per_wake']} frames/wake, "
        f"max_backlog={queue['max_backlog_frames']}, "
        f"end_backlog={queue['end_backlog_frames']}"
    )
    mit = report["can"]["mit_tx_scheduler"]
    if mit["motors"]:
        print(
            "  MIT tx scheduler: "
            f"sent={mit['sent_frames']} frames/{project['duration_ms']}ms, "
            f"wake={mit['wake_period_ms']}ms, "
            f"max_per_wake={mit['max_frames_per_wake']}/{mit['max_allowed_frames_per_wake']}, "
            f"skipped_due={mit['skipped_due_to_budget']}"
        )
        for motor in mit["motors"]:
            if motor["skipped_due_to_budget"]:
                print(
                    f"    {motor['name']}: period={motor['period_ms']}ms "
                    f"sent={motor['sent_frames']} skipped={motor['skipped_due_to_budget']}"
                )
    print()

    traffic = report["cpu"]["traffic_estimate"]
    envelope = report["cpu"]["task_budget_envelope"]
    print(f"CPU estimate: CAN traffic {traffic['cpu_percent']:.1f}%")
    for item in traffic["items"]:
        print(
            f"  {item['name']:<24} {format_fps(item['frames_per_s']):>7} fps "
            f"* {item['us_per_frame']:.1f} us = {item['cpu_percent']:.1f}%"
        )
    print(f"CPU budget envelope: {envelope['cpu_percent']:.1f}%")
    for item in envelope["items"]:
        print(
            f"  {item['name']:<24} {item['budget_us']:>7.1f} us / "
            f"{item['period_ms']:>4.1f} ms = {item['cpu_percent']:.1f}% ({item['source']})"
        )
    print()

    print(f"risk: {report['risk']['level']}")
    for note in report["risk"]["notes"]:
        print(f"  - {note}")
    print("assumptions:")
    for item in report["assumptions"]:
        print(f"  - {item}")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulate ARBATOS project CAN and CPU pressure.")
    parser.add_argument("--project", required=True, help="Robotconfig/project name, e.g. HERO-C")
    parser.add_argument("--duration-ms", type=int, default=DEFAULT_DURATION_MS)
    parser.add_argument("--can-tx-period-ms", type=int, default=None, help="Override CAN command task period in sim only.")
    parser.add_argument("--motor-feedback-hz", type=float, default=1000.0, help="Default enabled CAN motor feedback rate.")
    parser.add_argument("--can-bits-per-frame", type=int, default=DEFAULT_CAN_BITS_PER_FRAME)
    parser.add_argument("--rx-us-per-frame", type=float, default=DEFAULT_RX_US_PER_FRAME)
    parser.add_argument("--tx-us-per-frame", type=float, default=DEFAULT_TX_US_PER_FRAME)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    parser.add_argument("--fail-on-risk", action="store_true", help="Exit non-zero when risk level is warn/fail.")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    try:
        report = build_report(args)
    except Exception as exc:  # pragma: no cover - CLI boundary
        print(f"RobotSim: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print_report(report)

    if args.fail_on_risk and report["risk"]["level"] != "ok":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
