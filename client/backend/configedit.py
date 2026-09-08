"""车型 C 初始化配置的结构化读取和最小范围编辑。"""

from __future__ import annotations

import hashlib
import math
import os
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path


CONFIG_FILES = ("ConfigTuning.inc", "ConfigHardware.inc", "ConfigInput.inc", "ConfigOperation.inc")
NUMBER_RE = re.compile(r"[+-]?(?:(?:\d+\.?(?:\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?(?:[fFlLuU]*)\Z")
INTEGER_RE = re.compile(r"[+-]?(?:0[xX][0-9a-fA-F]+|\d+)(?:[uUlL]*)\Z")
FP32_MAX = 3.402823466e38

MODEL_LABELS = {
    "MOTOR_MODEL_3508": "DJI M3508",
    "MOTOR_MODEL_3510": "DJI M3510",
    "MOTOR_MODEL_2006": "DJI M2006",
    "MOTOR_MODEL_6020": "DJI GM6020",
    "MOTOR_MODEL_6623": "DJI GM6623",
    "MOTOR_MODEL_DM_J4310_2EC_V11": "达妙 J4310-2EC V1.1",
    "MOTOR_MODEL_DM_J4310_2EC_V12": "达妙 J4310-2EC V1.2",
    "MOTOR_MODEL_DM_J8009_2EC_V10": "达妙 J8009-2EC V1.0",
    "MOTOR_MODEL_DM_J8006_2EC_V11": "达妙 J8006-2EC V1.1",
    "MOTOR_MODEL_DM_J8006_2EC_V10": "达妙 J8006-2EC V1.0",
    "MOTOR_MODEL_UNITREE_GO_M8010_6": "宇树 GO-M8010-6",
    "MOTOR_MODEL_N6014B": "N6014B",
    "MOTOR_MODEL_DM_H3510_V10": "达妙 H3510 V1.0",
    "MOTOR_MODEL_DM_6215": "达妙 6215",
}
RS485_MODELS = {"MOTOR_MODEL_UNITREE_GO_M8010_6", "MOTOR_MODEL_N6014B"}
RM_MODELS = {
    "MOTOR_MODEL_3508", "MOTOR_MODEL_3510", "MOTOR_MODEL_2006", "MOTOR_MODEL_6020", "MOTOR_MODEL_6623"
}
BUS_OPTIONS = [
    {"value": "CAN1", "label": "CAN1"},
    {"value": "CAN2", "label": "CAN2"},
    {"value": "RS485_1", "label": "RS485-1"},
    {"value": "RS485_2", "label": "RS485-2"},
]


def _motor_id_max(model: str) -> int:
    # 当前 RM 组帧只发 0x200/0x1FF：普通 RM 覆盖 0x201..0x208，GM 仅覆盖 0x205..0x208。
    if model in {"MOTOR_MODEL_6020", "MOTOR_MODEL_6623"}:
        return 4
    if model in RM_MODELS:
        return 8
    if model == "MOTOR_MODEL_N6014B":
        return 15
    return 255

ENUM_OPTIONS = {
    "wheel_type": [
        ("CHASSIS_WHEEL_TYPE_MECANUM", "麦克纳姆轮"),
        ("CHASSIS_WHEEL_TYPE_XDRIVE", "X 型全向轮"),
    ],
    "fusion_mode": [
        ("IMU_FUSION_MAHONY_6AXIS", "Mahony 六轴"),
        ("IMU_FUSION_AHRS_9AXIS", "AHRS 九轴"),
    ],
    "active_source": [
        ("MANUAL_INPUT_SRC_AUTO", "自动选择"),
        ("MANUAL_INPUT_SRC_DBUS", "SBUS/DBUS"),
        ("MANUAL_INPUT_SRC_ELRS", "ELRS"),
        ("MANUAL_INPUT_SRC_IMAGE", "图传遥控"),
        ("MANUAL_INPUT_SRC_USB", "USB"),
    ],
    "mix_mode": [
        ("MANUAL_INPUT_MIX_SELECT_LATEST", "使用最近一帧"),
        ("MANUAL_INPUT_MIX_MERGE", "合并辅助请求"),
        ("MANUAL_INPUT_MIX_SELECT_STICKY", "保持当前健康来源"),
    ],
    "switch_pos": [
        ("MANUAL_INPUT_SWITCH_POS_UP", "上"),
        ("MANUAL_INPUT_SWITCH_POS_MID", "中"),
        ("MANUAL_INPUT_SWITCH_POS_DOWN", "下"),
    ],
    "run_mode": [
        ("ROBOT_RUN_MODE_FULL", "正常运行"),
        ("ROBOT_RUN_MODE_SINGLE_TASK", "单任务"),
        ("ROBOT_RUN_MODE_SINGLE_MOTOR", "单电机"),
        ("ROBOT_RUN_MODE_CALIBRATION", "校准"),
        ("ROBOT_RUN_MODE_ENTERTAIN", "娱乐演示"),
    ],
}

GROUP_LABELS = {
    "gimbal": "云台参数", "dual_gimbal": "双云台参数", "chassis": "底盘参数",
    "shoot": "发射参数", "power": "功率参数", "detect": "离线检测",
    "imu": "IMU 与温控", "voltage": "电压参数", "buzzer": "蜂鸣器",
    "led": "指示灯", "WheelLegMit": "轮腿 MIT", "WheelLegServo": "轮腿舵机",
    "ArmJ0Unitree": "机械臂 J0", "manual_input": "手动输入", "operation": "运行模式",
    "powerMeter": "功率计", "sdlog": "SD 日志", "AuxTelem": "辅助遥测",
}
PID_LABELS = ("比例 P（kp）", "积分 I（ki）", "微分 D（kd）", "输出上限（max_out）", "积分上限（max_iout）")

AXIS_LABELS = {
    "chassis[0]": "底盘左前", "chassis[1]": "底盘右前", "chassis[2]": "底盘左后",
    "chassis[3]": "底盘右后", "friction[0]": "摩擦轮 1", "friction[1]": "摩擦轮 2",
    "friction[2]": "摩擦轮 3", "friction[3]": "摩擦轮 4", "yaw": "云台 Yaw",
    "yaw_upper": "上层云台 Yaw", "pitch": "云台 Pitch", "trigger": "拨盘",
    "arm[0]": "机械臂 J0", "arm[1]": "机械臂 J1", "arm[2]": "机械臂 J2",
    "arm[3]": "机械臂 J3", "arm[4]": "机械臂 J4", "arm[5]": "机械臂 J5",
}

INPUT_AXIS_LABELS = {
    "INPUT_AXIS_CHASSIS_X": "底盘前后", "INPUT_AXIS_CHASSIS_Y": "底盘左右",
    "INPUT_AXIS_CHASSIS_WZ": "底盘旋转", "INPUT_AXIS_GIMBAL_YAW": "云台 Yaw",
    "INPUT_AXIS_GIMBAL_PITCH": "云台 Pitch", "INPUT_AXIS_CALIB_0": "校准杆 1",
    "INPUT_AXIS_CALIB_1": "校准杆 2", "INPUT_AXIS_CALIB_2": "校准杆 3",
    "INPUT_AXIS_CALIB_3": "校准杆 4",
}
INPUT_SWITCH_LABELS = {
    "INPUT_SW_GIMBAL_MODE": "云台模式", "INPUT_SW_CHASSIS_MODE": "底盘模式",
    "INPUT_SW_SHOOT_MODE": "发射模式", "INPUT_SW_CALIB_L": "校准左拨杆",
    "INPUT_SW_CALIB_R": "校准右拨杆",
}


@dataclass
class Node:
    key: str
    value_start: int
    value_end: int
    brace_start: int | None = None
    brace_end: int | None = None
    children: list["Node"] = field(default_factory=list)


@dataclass
class InternalField:
    public: dict
    filename: str
    start: int | None = None
    end: int | None = None
    original: str | None = None
    scale: int = 0
    motor: str | None = None
    member: str | None = None
    node: Node | None = None
    virtual: str | None = None


def _mask(text: str) -> str:
    chars = list(text)
    i = 0
    state = "code"
    while i < len(chars):
        c = chars[i]
        n = chars[i + 1] if i + 1 < len(chars) else ""
        if state == "code":
            if c == "/" and n == "/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "line"
                continue
            if c == "/" and n == "*":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "block"
                continue
            if c in {'"', "'"}:
                chars[i] = " "
                state = "string" if c == '"' else "char"
        elif state == "line":
            if c == "\n":
                state = "code"
            else:
                chars[i] = " "
        elif state == "block":
            if c == "*" and n == "/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "code"
                continue
            if c not in "\r\n":
                chars[i] = " "
        else:
            chars[i] = " "
            if c == "\\" and i + 1 < len(chars):
                chars[i + 1] = " "
                i += 2
                continue
            if (state == "string" and c == '"') or (state == "char" and c == "'"):
                state = "code"
        i += 1
    return "".join(chars)


def _matching(mask: str, start: int, opening: str = "{", closing: str = "}") -> int:
    depth = 0
    for i in range(start, len(mask)):
        if mask[i] == opening:
            depth += 1
        elif mask[i] == closing:
            depth -= 1
            if depth == 0:
                return i
    raise ValueError("配置初始化器的大括号不完整")


def _value_end(mask: str, start: int, limit: int) -> int:
    braces = parens = brackets = 0
    for i in range(start, limit):
        c = mask[i]
        if c == "{": braces += 1
        elif c == "}":
            if braces == 0 and parens == 0 and brackets == 0: return i
            braces -= 1
        elif c == "(": parens += 1
        elif c == ")": parens -= 1
        elif c == "[": brackets += 1
        elif c == "]": brackets -= 1
        elif c == "," and braces == 0 and parens == 0 and brackets == 0:
            return i
    return limit


def _nodes(text: str, mask: str, start: int = 0, end: int | None = None) -> list[Node]:
    end = len(text) if end is None else end
    result = []
    i = start
    depth = 0
    pattern = re.compile(r"(?:\.([A-Za-z_]\w*)|\[([A-Za-z_]\w*)\])\s*=")
    while i < end:
        c = mask[i]
        if c in "{(" or (c == "[" and depth != 0):
            depth += 1
            i += 1
            continue
        if c in "})]":
            depth -= 1
            i += 1
            continue
        if depth != 0 or c not in ".[":
            i += 1
            continue
        match = pattern.match(mask, i)
        if match is None:
            if c in "{([":
                depth += 1
            i += 1
            continue
        key = match.group(1) or f"[{match.group(2)}]"
        value_start = match.end()
        while value_start < end and mask[value_start].isspace():
            value_start += 1
        value_end = _value_end(mask, value_start, end)
        node = Node(key, value_start, value_end)
        if value_start < end and mask[value_start] == "{":
            brace_end = _matching(mask, value_start)
            if brace_end > value_end:
                raise ValueError("配置初始化器范围异常")
            node.brace_start = value_start
            node.brace_end = brace_end
            node.children = _nodes(text, mask, value_start + 1, brace_end)
        result.append(node)
        i = value_end + 1
    return result


def _child(node: Node, key: str) -> Node | None:
    return next((item for item in node.children if item.key == key), None)


def _literal(text: str):
    raw = text.strip()
    if INTEGER_RE.fullmatch(raw):
        clean = re.sub(r"[uUlL]+$", "", raw)
        return int(clean, 0), "integer"
    if NUMBER_RE.fullmatch(raw):
        clean = re.sub(r"[fFlLuU]+$", "", raw)
        value = float(clean)
        if not math.isfinite(value):
            raise ValueError("数值必须是有限值")
        return value, "number"
    return raw, "text"


def _format_number(value, original: str, integer: bool) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("数值字段必须提供有限数字")
    try:
        numeric = float(value)
    except (OverflowError, ValueError):
        raise ValueError("数值字段必须提供有限数字") from None
    if not math.isfinite(numeric):
        raise ValueError("数值字段必须提供有限数字")
    suffix_match = re.search(r"[fFlLuU]+$", original.strip())
    suffix = suffix_match.group(0) if suffix_match else ""
    if integer:
        if int(value) != value:
            raise ValueError("整数字段不能填写小数")
        number = int(value)
        suffix_lower = suffix.lower()
        bits = 64 if "ll" in suffix_lower else 32
        if "u" in suffix_lower:
            if number < 0 or number > (1 << bits) - 1:
                raise ValueError(f"无符号 {bits} 位整数超出范围")
        elif number < -(1 << (bits - 1)) or number > (1 << (bits - 1)) - 1:
            raise ValueError(f"有符号 {bits} 位整数超出范围")
        if re.match(r"[+-]?0[xX]", original.strip()):
            prefix = "0X" if "0X" in original else "0x"
            return ("-" if number < 0 else "") + prefix + format(abs(number), "X" if prefix == "0X" else "x") + suffix
        return f"{number}{suffix}"
    number = float(value)
    if "f" in suffix.lower() and abs(number) > FP32_MAX:
        raise ValueError("单精度浮点数超出范围")
    rendered = format(number, ".15g")
    if ("." in original or "e" in original.lower()) and "." not in rendered and "e" not in rendered.lower():
        rendered += ".0"
    return rendered + suffix


def _top_items(text: str, mask: str, start: int, end: int) -> list[tuple[int, int]]:
    result = []
    item_start = start
    braces = parens = brackets = 0
    for i in range(start, end):
        c = mask[i]
        if c == "{": braces += 1
        elif c == "}": braces -= 1
        elif c == "(": parens += 1
        elif c == ")": parens -= 1
        elif c == "[": brackets += 1
        elif c == "]": brackets -= 1
        elif c == "," and braces == 0 and parens == 0 and brackets == 0:
            a, b = item_start, i
            while a < b and text[a].isspace(): a += 1
            while b > a and text[b - 1].isspace(): b -= 1
            if a < b: result.append((a, b))
            item_start = i + 1
    a, b = item_start, end
    while a < b and text[a].isspace(): a += 1
    while b > a and text[b - 1].isspace(): b -= 1
    if a < b: result.append((a, b))
    return result


def _array_literals(text: str, mask: str, start: int, end: int, prefix=()):
    result = []
    for index, (a, b) in enumerate(_top_items(text, mask, start, end)):
        if mask[a] == "{" and _matching(mask, a) == b - 1:
            nested = _array_literals(text, mask, a + 1, b - 1, prefix + (index,))
            if nested is None:
                return None
            result.extend(nested)
        else:
            value, kind = _literal(text[a:b])
            if kind == "text":
                return None
            result.append((prefix + (index,), a, b, value, kind))
    return result


def _description(text: str, end: int) -> str | None:
    line_end = text.find("\n", end)
    line_end = len(text) if line_end < 0 else line_end
    tail = text[end:line_end]
    marker = tail.find("//")
    if marker < 0:
        return None
    value = re.sub(r"^\s*\[\d+(?:\.\.\d+)?\]\s*", "", tail[marker + 2:].strip())
    return value or None


def _label(name: str, description: str | None, suffix: str = "") -> str:
    if description and re.search(r"[\u4e00-\u9fff]", description):
        short = re.split(r"[：:；;|]", description, maxsplit=1)[0].strip()
        if 1 <= len(short) <= 32:
            return short + suffix
    return name + suffix


def _enum_options(path: str):
    name = path.rsplit(".", 1)[-1]
    if name == "wheel_type": key = "wheel_type"
    elif name == "fusion_mode": key = "fusion_mode"
    elif name == "active_source": key = "active_source"
    elif name == "mix_mode": key = "mix_mode"
    elif name.endswith("Pos") or name.endswith("_pos"): key = "switch_pos"
    elif name == "mode" and path.startswith("operation."): key = "run_mode"
    else: return None
    return [{"value": value, "label": label} for value, label in ENUM_OPTIONS[key]]


class ConfigEditor:
    def __init__(self, workspace):
        self.workspace = workspace

    def _sources(self, target):
        sources = {}
        target_name = None
        for filename in CONFIG_FILES:
            path, target_name = self.workspace._file_path(target, filename)
            data = path.read_bytes()
            text, bom, newline = self.workspace._text_info(data)
            sources[filename] = {"path": path, "data": data, "text": text, "bom": bom, "newline": newline,
                                 "mask": _mask(text), "nodes": None}
            sources[filename]["nodes"] = _nodes(text, sources[filename]["mask"])
        return target_name, sources

    @staticmethod
    def _revision(sources):
        digest = hashlib.sha256()
        for filename in CONFIG_FILES:
            digest.update(filename.encode("utf-8") + b"\0" + sources[filename]["data"] + b"\0")
        return digest.hexdigest()

    def get(self, target):
        target_name, sources = self._sources(target)
        groups, fields = self._build(sources)
        return {"target": target_name, "revision": self._revision(sources), "groups": groups,
                "models": self._model_meta()}, fields, sources

    def public_get(self, target):
        result, _fields, _sources = self.get(target)
        return result

    def _build(self, sources):
        groups = []
        fields = {}
        self._generic_groups(sources["ConfigTuning.inc"], "tuning", "tuning", groups, fields)
        self._motor_groups(sources["ConfigHardware.inc"], groups, fields)
        self._pitch_middle(sources, groups, fields)
        self._input_groups(sources["ConfigInput.inc"], groups, fields)
        self._generic_groups(sources["ConfigHardware.inc"], "hardware", "other", groups, fields,
                             excluded={"motor"})
        self._generic_groups(sources["ConfigInput.inc"], "input", "other", groups, fields,
                             excluded={"input"})
        self._generic_groups(sources["ConfigOperation.inc"], "operation", "other", groups, fields)
        self._servo_groups(sources["ConfigHardware.inc"], groups, fields)
        return groups, fields

    def _generic_groups(self, source, prefix, category, groups, fields, excluded=None):
        excluded = excluded or set()
        for node in source["nodes"]:
            if node.key in excluded:
                continue
            public_fields = []
            self._generic_node(source, node, f"{prefix}.{node.key}", public_fields, fields)
            if public_fields:
                groups.append({"id": f"{prefix}.{node.key}", "label": GROUP_LABELS.get(node.key, node.key),
                               "category": category, "fields": public_fields})

    def _generic_node(self, source, node, path, output, fields):
        if node.children:
            for child in node.children:
                child_name = child.key if child.key.startswith("[") else f".{child.key}"
                self._generic_node(source, child, path + child_name, output, fields)
            return
        text = source["text"]
        if node.brace_start is not None:
            values = _array_literals(text, source["mask"], node.brace_start + 1, node.brace_end)
            if values is None:
                public = {"id": path, "label": node.key, "value": text[node.value_start:node.value_end].strip(),
                          "type": "text", "editable": False}
                output.append(public)
                return
            pid_labels = PID_LABELS if path.lower().endswith("pid") and len(values) == 5 else None
            desc = _description(text, node.value_end)
            for flat, (indices, start, end, value, kind) in enumerate(values):
                suffix = "".join(f"[{index}]" for index in indices)
                field_id = path + suffix
                label = pid_labels[flat] if pid_labels else _label(node.key, desc, suffix)
                public = {"id": field_id, "label": label, "value": value,
                          "type": "number", "editable": True}
                if desc: public["description"] = desc
                internal = InternalField(public, source["path"].name, start, end, text[start:end],
                                         scale=0 if kind == "integer" else 1)
                output.append(public); fields[field_id] = internal
            return
        raw = text[node.value_start:node.value_end]
        value, kind = _literal(raw)
        enum_options = _enum_options(path)
        desc = _description(text, node.value_end)
        if kind != "text":
            is_bool = node.key in {"enable", "enabled", "invert", "loop"} and value in {0, 1}
            public = {"id": path, "label": _label(node.key, desc), "value": bool(value) if is_bool else value,
                      "type": "boolean" if is_bool else "number", "editable": True}
            internal = InternalField(public, source["path"].name, node.value_start, node.value_end, raw,
                                     scale=0 if kind == "integer" else 1)
            fields[path] = internal
        elif enum_options and value in {item["value"] for item in enum_options}:
            public = {"id": path, "label": _label(node.key, desc), "value": value, "type": "enum", "editable": True,
                      "options": enum_options}
            fields[path] = InternalField(public, source["path"].name, node.value_start, node.value_end, raw)
        else:
            public = {"id": path, "label": _label(node.key, desc), "value": value, "type": "text", "editable": False}
        if desc: public["description"] = desc
        output.append(public)

    def _motor_groups(self, source, groups, fields):
        motor = next((node for node in source["nodes"] if node.key == "motor"), None)
        if motor is None:
            raise ValueError("ConfigHardware.inc 缺少 .motor 初始化器")
        model_meta = self._model_meta()
        for axis, node, fallback in self._motor_nodes(source, motor):
            model, model_span = self._motor_member(source, node, "model", 0)
            motor_id, id_span = self._motor_member(source, node, "can_id", 1)
            if model not in model_meta:
                raise ValueError(f"电机轴 {axis} 使用了未知型号表达式: {model}")
            try:
                can_id_value, motor_id_kind = _literal(motor_id)
            except ValueError as exc:
                raise ValueError(f"电机轴 {axis} 的 ID 无法读取") from exc
            if motor_id_kind != "integer":
                raise ValueError(f"电机轴 {axis} 的 ID 必须是整数字面量")
            feedback_enable = self._motor_hidden_int(source, node, "feedback_id_enable")
            feedback_override = self._motor_hidden_int(source, node, "feedback_id") if feedback_enable else 0
            master_id = self._motor_hidden_int(source, node, "master_id")
            n6014_feedback_override = model == "MOTOR_MODEL_N6014B" and feedback_enable != 0
            # MotorInst 仍以 can_id == 0 判断实例关闭；仅在实例已启用时，N6014 的反馈覆盖才是实际设备地址。
            motor_id_value = feedback_override if n6014_feedback_override and can_id_value != 0 else can_id_value
            n6014_device_zero_enabled = n6014_feedback_override and can_id_value != 0 and feedback_override == 0
            n6014_framework_disabled_override = n6014_feedback_override and can_id_value == 0
            transport, _ = self._motor_member(source, node, "transport")
            can_bus, _ = self._motor_member(source, node, "can_bus")
            rs_port, _ = self._motor_member(source, node, "rs485_port")
            external, external_span = self._motor_member(source, node, "external_reduction_ratio")
            is_rs = transport == "MOTOR_TRANSPORT_RS485" or (transport in {None, "MOTOR_TRANSPORT_INHERIT"} and model in RS485_MODELS)
            if is_rs:
                port = self._int_or_default(rs_port, 0, source)
                bus = f"RS485_{port + 1}"
            else:
                bus = f"CAN{self._int_or_default(can_bus, fallback, source)}"
            external_value = 1.0
            if external is not None:
                parsed, kind = _literal(external)
                if kind == "text":
                    raise ValueError(f"电机轴 {axis} 的外置减速比必须是数值字面量")
                external_value = float(parsed)
            if is_rs:
                effective_feedback_id = feedback_override if n6014_feedback_override else motor_id_value
                feedback_editable = False
                feedback_description = ("协议覆盖地址保留为该值，但 can_id 为 0，框架当前不会创建有效输出实例。" if
                                        n6014_framework_disabled_override else "RS485 反馈按电机设备 ID 匹配。")
            else:
                auto_feedback_id = model_meta[model]["can_id_base"] + can_id_value
                effective_feedback_id = feedback_override if feedback_enable else master_id or auto_feedback_id
                feedback_editable = True
                feedback_description = ("当前为显式反馈 ID；填写 -1 恢复按型号和电机 ID 自动计算。" if
                                        feedback_enable or master_id else
                                        "当前按型号和电机 ID 自动计算；填写 0..2047 可设显式反馈 ID。")
            group_fields = []
            specs = [
                ("model", "电机型号", model, "enum", [{"value": key, "label": MODEL_LABELS.get(key, key)} for key in model_meta]),
                ("id", "电机设备 ID" if n6014_device_zero_enabled else "电机设备 ID（0 表示关闭）",
                 motor_id_value, "number", None),
                ("bus", "总线", bus, "enum", BUS_OPTIONS),
                ("feedback_id", "实际反馈 ID（-1 自动）" if feedback_editable else "实际反馈设备 ID",
                 effective_feedback_id, "number", None),
                ("built_in_reduction_ratio", "型号内置减速比", model_meta[model]["ratio"], "number", None),
                ("external_reduction_ratio", "外置减速比", external_value, "number", None),
            ]
            for member, label, value, kind, options in specs:
                field_id = f"motor.{axis}.{member}"
                editable = member != "built_in_reduction_ratio" and (member != "feedback_id" or feedback_editable)
                public = {"id": field_id, "label": label, "value": value, "type": kind, "editable": editable}
                if options is not None: public["options"] = options
                if member == "id":
                    maximum = _motor_id_max(model)
                    public.update({"min": 0, "max": maximum, "step": 1})
                    if n6014_device_zero_enabled:
                        public["description"] = ("旧配置通过显式反馈 ID 启用了 N6014B 设备 0；当前确实会输出。"
                                                 "修改本字段后将改用普通设备 ID 规则，此时 0 表示关闭。")
                    elif n6014_framework_disabled_override:
                        public["description"] = (f"当前 can_id 为 0，框架实例已关闭；配置中保留的协议覆盖地址为 "
                                                 f"{feedback_override}。填写非零设备 ID 后会规范化配置并启用。")
                if member == "feedback_id":
                    public.update({"min": -1 if feedback_editable else 0, "max": 0x7FF, "step": 1,
                                   "description": feedback_description})
                if member == "external_reduction_ratio":
                    public.update({"min": 0, "max": 10000, "step": 0.01,
                                   "description": "0 或 1 表示直连；减速传动比应大于 1，且不超过 10000。"})
                if member == "built_in_reduction_ratio":
                    public["description"] = "来自共享电机型号表，只读。"
                group_fields.append(public)
                if editable:
                    span = model_span if member == "model" else id_span if member == "id" else \
                        external_span if member == "external_reduction_ratio" else None
                    fields[field_id] = InternalField(public, source["path"].name,
                                                     span[0] if span else None, span[1] if span else None,
                                                     source["text"][span[0]:span[1]] if span else None,
                                                     motor=axis, member=member, node=node)
            groups.append({"id": f"motor.{axis}", "label": AXIS_LABELS[axis], "category": "motors",
                           "fields": group_fields})

    def _motor_nodes(self, source, motor):
        fallback_map = {"chassis": 1, "friction": 2, "yaw": 1, "yaw_upper": 1, "pitch": 1, "trigger": 1, "arm": 1}
        for group in ("chassis", "friction", "yaw", "yaw_upper", "pitch", "trigger", "arm"):
            parent = _child(motor, group)
            if parent is None or parent.brace_start is None:
                raise ValueError(f"电机表缺少 {group}")
            count = 4 if group in {"chassis", "friction"} else 6 if group == "arm" else 1
            if count == 1:
                yield group, parent, fallback_map[group]
                continue
            items = _top_items(source["text"], source["mask"], parent.brace_start + 1, parent.brace_end)
            if len(items) == 1 and source["text"][items[0][0]:items[0][1]].strip() == "0":
                for index in range(count):
                    yield f"{group}[{index}]", Node(f"virtual:{index}", parent.brace_start + 1,
                                                      parent.brace_end, parent.brace_start,
                                                      parent.brace_end), fallback_map[group]
                continue
            item_nodes = []
            for start, end in items:
                prefix = re.match(r"\s*\[(\d+)\]\s*=\s*", source["mask"][start:end])
                if prefix:
                    index = int(prefix.group(1)); brace = source["mask"].find("{", start + prefix.end(), end)
                else:
                    index = len(item_nodes); brace = source["mask"].find("{", start, end)
                if index >= count or brace < 0:
                    continue
                brace_end = _matching(source["mask"], brace)
                node = Node(str(index), brace + 1, brace_end, brace, brace_end,
                            _nodes(source["text"], source["mask"], brace + 1, brace_end))
                item_nodes.append((index, node))
            if len(item_nodes) != count:
                raise ValueError(f"电机表 {group} 应有 {count} 项")
            for index, node in sorted(item_nodes):
                yield f"{group}[{index}]", node, fallback_map[group]

    @staticmethod
    def _motor_member(source, node, name, positional=None):
        if node.key.startswith("virtual:"):
            if name == "model": return "MOTOR_MODEL_3508", None
            if name == "can_id": return "0u", None
            return None, None
        named = _child(node, name)
        if named is not None:
            return source["text"][named.value_start:named.value_end].strip(), (named.value_start, named.value_end)
        if positional is None:
            return None, None
        items = _top_items(source["text"], source["mask"], node.brace_start + 1, node.brace_end)
        positional_items = [(a, b) for a, b in items if not source["mask"][a:b].lstrip().startswith(".")]
        if positional >= len(positional_items):
            return None, None
        a, b = positional_items[positional]
        return source["text"][a:b].strip(), (a, b)

    @staticmethod
    def _int_or_default(raw, default, source=None):
        if raw is None: return default
        value, kind = _literal(raw)
        if kind == "integer": return int(value)
        if source is not None and re.fullmatch(r"[A-Za-z_]\w*", raw.strip()):
            header = source["path"].with_name("RobotConfig.h")
            if header.is_file():
                match = re.search(rf"^\s*#\s*define\s+{re.escape(raw.strip())}\s+([^\s/]+)",
                                  header.read_text(encoding="utf-8-sig"), re.MULTILINE)
                if match is not None:
                    resolved, resolved_kind = _literal(match.group(1))
                    if resolved_kind == "integer": return int(resolved)
        raise ValueError(f"无法安全解析整数表达式: {raw.strip()}")

    def _model_meta(self):
        source = (self.workspace.root / "shared/application/motors/MotorModelDb.c").read_text(encoding="utf-8")
        result = {}
        starts = list(re.finditer(r"\[(MOTOR_MODEL_[A-Z0-9_]+)\]\s*=", source))
        for index, match in enumerate(starts):
            name = match.group(1)
            end = starts[index + 1].start() if index + 1 < len(starts) else len(source)
            block = source[match.end():end]
            ratio = re.search(r"\.reduction_ratio\s*=\s*([^,}]+)", block)
            if ratio is None: continue
            expr = ratio.group(1).strip()
            parts = [part.strip() for part in expr.split("/")]
            try:
                values = [float(re.sub(r"[fFlLuU]+$", "", part)) for part in parts]
            except ValueError:
                continue
            value = values[0]
            for divisor in values[1:]: value /= divisor
            bits = re.search(r"\.encoder_bits\s*=\s*(\d+)u?", block)
            base = re.search(r"\.can_id_base\s*=\s*(0[xX][0-9a-fA-F]+|\d+)u?", block)
            result[name] = {"ratio": value, "transport": "RS485" if name in RS485_MODELS else "CAN",
                            "encoder_bits": int(bits.group(1)) if bits else 13,
                            "can_id_base": int(base.group(1), 0) if base else 0}
        if not result:
            raise ValueError("无法读取共享电机型号表")
        return result

    def _pitch_middle(self, sources, groups, fields):
        model_meta = self._model_meta()
        for bounded_field, motor_field in (("tuning.gimbal.yaw_middle_ecd", "motor.yaw.model"),
                                           ("tuning.dual_gimbal.yaw_upper_middle_ecd", "motor.yaw_upper.model")):
            item = fields.get(bounded_field)
            model = fields.get(motor_field)
            if item is not None and model is not None:
                bits = model_meta.get(model.public["value"], {}).get("encoder_bits", 16)
                item.public.update({"min": 0, "max": (1 << bits) - 1, "step": 1})
        field_id = "tuning.gimbal.pitch_middle_ecd"
        if field_id in fields:
            model = fields.get("motor.pitch.model")
            bits = model_meta.get(model.public["value"], {}).get("encoder_bits", 16) if model else 16
            fields[field_id].public.update({"min": 0, "max": (1 << bits) - 1, "step": 1})
            return
        tuning = sources["ConfigTuning.inc"]
        gimbal = next((node for node in tuning["nodes"] if node.key == "gimbal"), None)
        if gimbal is None or gimbal.brace_start is None:
            return
        default = 0
        header = tuning["path"].with_name("RobotConfig.h")
        if header.is_file():
            match = re.search(r"^\s*#define\s+GIMBAL_PITCH_MIDDLE_ECD\s+([^\s/]+)",
                              header.read_text(encoding="utf-8"), re.MULTILINE)
            if match:
                value, kind = _literal(match.group(1))
                if kind == "integer": default = value
        model = fields.get("motor.pitch.model")
        bits = self._model_meta().get(model.public["value"], {}).get("encoder_bits", 16) if model else 16
        public = {"id": field_id, "label": "Pitch 中位编码器值", "value": default, "type": "number",
                  "editable": True, "min": 0, "max": (1 << bits) - 1, "step": 1,
                  "description": "首次保存后启用车型级 Pitch 中位；此前沿用 RobotConfig.h 的旧值。"}
        group = next((item for item in groups if item["id"] == "tuning.gimbal"), None)
        if group:
            group["fields"].append(public)
            fields[field_id] = InternalField(public, "ConfigTuning.inc", node=gimbal, virtual="pitch_middle")

    def _servo_groups(self, source, groups, fields):
        root = next((node for node in source["nodes"] if node.key == "servo"), None)
        channels = _child(root, "channels") if root else None
        parsed = {}
        if channels and channels.brace_start is not None:
            items = _top_items(source["text"], source["mask"], channels.brace_start + 1, channels.brace_end)
            for index, (start, end) in enumerate(items[:4]):
                brace = source["mask"].find("{", start, end)
                if brace >= 0:
                    close = _matching(source["mask"], brace)
                    parsed[index] = Node(str(index), brace + 1, close, brace, close,
                                         _nodes(source["text"], source["mask"], brace + 1, close))
        defaults = {"enabled": 0, "port": 0, "inputMode": 0, "inputChannel": 0, "invert": 0,
                    "minUs": 500, "centerUs": 1500, "maxUs": 2500, "stepUs": 10}
        port_labels = ("P11 PA0", "P11 PA2", "P11 PE9", "P11 PE13")
        for index in range(4):
            node = parsed.get(index)
            public_fields = []
            for member, default in defaults.items():
                child = _child(node, member) if node else None
                value = default
                if child:
                    value, kind = _literal(source["text"][child.value_start:child.value_end])
                    if kind != "integer": raise ValueError(f"servo.channels[{index}].{member} 必须是整数字面量")
                field_id = f"servo.channels[{index}].{member}"
                if member in {"enabled", "invert"}:
                    public = {"id": field_id, "label": "启用" if member == "enabled" else "反向",
                              "value": bool(value), "type": "boolean", "editable": True}
                elif member == "port":
                    public = {"id": field_id, "label": "PWM 端口", "value": value + 1, "type": "enum",
                              "editable": True,
                              "options": [{"value": item + 1, "label": label} for item, label in enumerate(port_labels)]}
                elif member == "inputMode":
                    public = {"id": field_id, "label": "控制方式", "value": value, "type": "enum", "editable": True,
                              "options": [{"value": 0, "label": "键盘步进"},
                                          {"value": 1, "label": "统一手动输入逻辑轴"}]}
                elif member == "inputChannel":
                    public = {"id": field_id, "label": "手动输入逻辑轴", "value": value + 1, "type": "enum",
                              "editable": True,
                              "options": [{"value": item, "label": f"逻辑轴 {item}"} for item in range(1, 6)]}
                else:
                    labels = {"minUs": "最小脉宽", "centerUs": "中位脉宽", "maxUs": "最大脉宽", "stepUs": "步进脉宽"}
                    public = {"id": field_id, "label": labels[member], "value": value, "type": "number",
                              "editable": True, "min": 1 if member == "stepUs" else 500,
                              "max": 200 if member == "stepUs" else 2500, "step": 1, "unit": "us"}
                if member == "enabled":
                    public["description"] = "需要在 RobotConfig.toml 启用 SERVO 服务；输出仍受统一安全许可控制。"
                public_fields.append(public)
                fields[field_id] = InternalField(public, source["path"].name,
                                                 child.value_start if child else None,
                                                 child.value_end if child else None,
                                                 source["text"][child.value_start:child.value_end] if child else None,
                                                 node=node or root, member=member, virtual=f"servo:{index}")
            groups.append({"id": f"servo.channels[{index}]", "label": f"PWM 舵机 {index + 1}",
                           "category": "servo", "fields": public_fields})

    def _input_groups(self, source, groups, fields):
        root = next((node for node in source["nodes"] if node.key == "input"), None)
        if root is None: raise ValueError("ConfigInput.inc 缺少 .input 初始化器")
        elrs_fields = []
        for member, count, label in (("ElrsChMap", 5, "RC 轴"), ("ElrsSwMap", 2, "RC 拨杆")):
            node = _child(root, member)
            values = _array_literals(source["text"], source["mask"], node.brace_start + 1, node.brace_end) if node else None
            if values is None or len(values) != count: raise ValueError(f"{member} 必须是 {count} 个整数字面量")
            for index, (_indices, start, end, value, kind) in enumerate(values):
                if kind != "integer": raise ValueError(f"{member} 必须是整数字面量")
                field_id = f"input.elrs.{member}.{index}"
                public = {"id": field_id, "label": f"{label} {index + 1}", "value": value + 1,
                          "type": "enum", "editable": True,
                          "options": [{"value": channel, "label": f"CH{channel}"} for channel in range(1, 17)]}
                elrs_fields.append(public)
                fields[field_id] = InternalField(public, source["path"].name, start, end, source["text"][start:end], scale=-1)
        groups.append({"id": "input.elrs", "label": "ELRS 真实通道", "category": "input", "fields": elrs_fields})
        for member, labels, channel_name in (("axis", INPUT_AXIS_LABELS, "rc_ch"), ("sw", INPUT_SWITCH_LABELS, "rc_sw")):
            parent = _child(root, member)
            public_fields = []
            for node in parent.children if parent else []:
                enum = node.key.strip("[]")
                if enum not in labels: continue
                for sub, suffix, label in ((channel_name, "channel", "逻辑通道"), ("invert", "invert", "反向")):
                    child = _child(node, sub)
                    if child is None: continue
                    value, kind = _literal(source["text"][child.value_start:child.value_end])
                    if kind != "integer": raise ValueError(f"{enum}.{sub} 必须是整数字面量")
                    field_id = f"input.{member}.{enum}.{suffix}"
                    if suffix == "channel":
                        maximum = 5 if member == "axis" else 2
                        public = {"id": field_id, "label": f"{labels[enum]} {label}", "value": value + 1,
                                  "type": "enum", "editable": True,
                                  "options": [{"value": i, "label": f"RC {'CH' if member == 'axis' else 'SW'}{i}"}
                                              for i in range(1, maximum + 1)]}
                        scale = -1
                    else:
                        public = {"id": field_id, "label": f"{labels[enum]} {label}", "value": bool(value),
                                  "type": "boolean", "editable": True}
                        scale = 0
                    public_fields.append(public)
                    fields[field_id] = InternalField(public, source["path"].name, child.value_start, child.value_end,
                                                     source["text"][child.value_start:child.value_end], scale=scale)
            groups.append({"id": f"input.{member}", "label": "逻辑轴映射" if member == "axis" else "逻辑拨杆映射",
                           "category": "input", "fields": public_fields})

    def update(self, target, revision, changes):
        if not isinstance(revision, str) or not revision: raise ValueError("配置修订号不能为空")
        if not isinstance(changes, dict) or not changes: raise ValueError("配置变更必须是非空对象")
        with self.workspace._lock:
            _result, fields, sources = self.get(target)
            if self._revision(sources) != revision:
                raise ValueError("配置已被外部修改，请重新读取后再保存")
            unknown = [key for key in changes if key not in fields]
            if unknown: raise ValueError(f"未知或只读的配置字段: {unknown[0]}")
            edits = {filename: [] for filename in CONFIG_FILES}
            motor_changes = {}
            servo_changes = {}
            pitch_middle = None
            for field_id, value in changes.items():
                item = fields[field_id]
                if item.motor:
                    motor_changes.setdefault(item.motor, {})[item.member] = value
                elif item.virtual == "pitch_middle":
                    pitch_middle = value
                elif item.virtual and item.virtual.startswith("servo:"):
                    servo_changes.setdefault(int(item.virtual.split(":", 1)[1]), {})[item.member] = value
                else:
                    edits[item.filename].append((item.start, item.end, self._format_field(item, value)))
            self._motor_edits(sources["ConfigHardware.inc"], fields, motor_changes, edits["ConfigHardware.inc"])
            if pitch_middle is not None:
                self._pitch_middle_edit(sources["ConfigTuning.inc"], fields, pitch_middle,
                                        edits["ConfigTuning.inc"])
            if servo_changes:
                self._servo_edits(sources["ConfigHardware.inc"], fields, servo_changes,
                                  edits["ConfigHardware.inc"])
            candidates = {}
            for filename, file_edits in edits.items():
                text = sources[filename]["text"]
                self._check_edits(file_edits)
                for start, end, replacement in sorted(file_edits, reverse=True):
                    text = text[:start] + replacement + text[end:]
                # 修改后的文件必须仍可被同一结构解析器完整读取。
                _nodes(text, _mask(text))
                newline = sources[filename]["newline"]
                normalized = text.replace("\r\n", "\n").replace("\r", "\n")
                if newline == "CRLF": normalized = normalized.replace("\n", "\r\n")
                candidates[filename] = (b"\xef\xbb\xbf" if sources[filename]["bom"] else b"") + normalized.encode("utf-8")
            self._validate_motor_candidate(target, candidates, sources)
            current_target, current = self._sources(target)
            del current_target
            if self._revision(current) != revision:
                raise ValueError("配置已被外部修改，请重新读取后再保存")
            self._commit(sources, candidates)
            return self.public_get(target)

    @staticmethod
    def _format_field(item, value):
        kind = item.public["type"]
        if kind == "boolean":
            if not isinstance(value, bool): raise ValueError(f"{item.public['id']} 必须是布尔值")
            value = 1 if value else 0
        if kind == "enum":
            choices = {option["value"] for option in item.public.get("options", [])}
            if value not in choices: raise ValueError(f"{item.public['id']} 的选项无效")
            if isinstance(value, str): return value
        if item.scale == -1:
            value -= 1
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            minimum = item.public.get("min")
            maximum = item.public.get("max")
            if minimum is not None and value < minimum:
                raise ValueError(f"{item.public['id']} 不能小于 {minimum}")
            if maximum is not None and value > maximum:
                raise ValueError(f"{item.public['id']} 不能大于 {maximum}")
        return _format_number(value, item.original, integer=item.scale <= 0)

    def _pitch_middle_edit(self, source, fields, value, output):
        item = fields["tuning.gimbal.pitch_middle_ecd"]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
            raise ValueError("tuning.gimbal.pitch_middle_ecd 必须是整数")
        if value < item.public["min"] or value > item.public["max"]:
            raise ValueError("tuning.gimbal.pitch_middle_ecd 超出当前 Pitch 电机编码器范围")
        self._motor_named_edits(source, item.node,
                                {"pitch_middle_ecd": f"{int(value)}u", "pitch_middle_ecd_enable": "1u"}, output)

    def _servo_edits(self, source, fields, changes, output):
        states = []
        for index in range(4):
            prefix = f"servo.channels[{index}]."
            state = {member: fields[prefix + member].public["value"] for member in
                     ("enabled", "port", "inputMode", "inputChannel", "invert",
                      "minUs", "centerUs", "maxUs", "stepUs")}
            state.update(changes.get(index, {}))
            for member in ("enabled", "invert"):
                if not isinstance(state[member], bool): raise ValueError(f"{prefix}{member} 必须是布尔值")
            for member in ("port", "inputMode", "inputChannel", "minUs", "centerUs", "maxUs", "stepUs"):
                value = state[member]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
                    raise ValueError(f"{prefix}{member} 必须是整数")
                state[member] = int(value)
            if state["port"] not in range(1, 5): raise ValueError(f"{prefix}port 的选项无效")
            if state["inputMode"] not in {0, 1}: raise ValueError(f"{prefix}inputMode 的选项无效")
            if state["inputChannel"] not in range(1, 6): raise ValueError(f"{prefix}inputChannel 的选项无效")
            if not (500 <= state["minUs"] < state["centerUs"] < state["maxUs"] <= 2500):
                raise ValueError(f"{prefix} 必须满足 500 <= minUs < centerUs < maxUs <= 2500")
            if not 1 <= state["stepUs"] <= 200: raise ValueError(f"{prefix}stepUs 必须在 1..200")
            states.append(state)
        used_ports = {}
        for index, state in enumerate(states):
            if state["enabled"] and state["port"] in used_ports:
                raise ValueError(f"舵机端口冲突: 舵机 {used_ports[state['port']] + 1} 与舵机 {index + 1}")
            if state["enabled"]: used_ports[state["port"]] = index
        rows = []
        for state in states:
            rows.append("{" + ", ".join((
                f".enabled = {1 if state['enabled'] else 0}u", f".port = {state['port'] - 1}u",
                f".inputMode = {state['inputMode']}u", f".inputChannel = {state['inputChannel'] - 1}u",
                f".invert = {1 if state['invert'] else 0}u", f".minUs = {state['minUs']}u",
                f".centerUs = {state['centerUs']}u", f".maxUs = {state['maxUs']}u",
                f".stepUs = {state['stepUs']}u")) + "}")
        replacement = "{\n            .configured = 1u,\n            .channels =\n                {\n                    " + \
                      ",\n                    ".join(rows) + ",\n                },\n        }"
        root = next((node for node in source["nodes"] if node.key == "servo"), None)
        if root:
            output.append((root.value_start, root.value_end, replacement))
        else:
            suffix = "" if source["text"].endswith(("\n", "\r")) else "\n"
            output.append((len(source["text"]), len(source["text"]),
                           suffix + "    .servo = " + replacement + ",\n"))

    def _motor_edits(self, source, fields, changes, output):
        states = {}
        virtual_parents = {}
        meta = self._model_meta()
        for axis, values in changes.items():
            prefix = f"motor.{axis}."
            node = fields[prefix + "model"].node
            original_model = fields[prefix + "model"].public["value"]
            feedback_field = fields.get(prefix + "feedback_id")
            current = {name: fields[prefix + name].public["value"] for name in
                       ("model", "id", "bus", "external_reduction_ratio")}
            current["feedback_id"] = feedback_field.public["value"] if feedback_field is not None else current["id"]
            current.update(values)
            model = current["model"]
            if model not in meta: raise ValueError(f"{prefix}model 的选项无效")
            model_changed = model != original_model
            motor_id = current["id"]
            if isinstance(motor_id, bool) or not isinstance(motor_id, (int, float)) or int(motor_id) != motor_id:
                raise ValueError(f"{prefix}id 必须是整数")
            motor_id = int(motor_id)
            max_id = _motor_id_max(model)
            if motor_id < 0 or motor_id > max_id: raise ValueError(f"{prefix}id 超出该型号范围 0..{max_id}")
            bus = current["bus"]
            if bus not in {item["value"] for item in BUS_OPTIONS}: raise ValueError(f"{prefix}bus 的选项无效")
            is_rs = bus.startswith("RS485_")
            if is_rs != (model in RS485_MODELS):
                raise ValueError(f"{MODEL_LABELS.get(model, model)} 只能使用{'RS485' if model in RS485_MODELS else 'CAN'}")
            if "feedback_id" in values and is_rs:
                raise ValueError(f"{prefix}feedback_id 由 RS485 电机设备 ID 决定")
            original_feedback_explicit = (
                self._motor_hidden_int(source, node, "feedback_id_enable") != 0 or
                self._motor_hidden_int(source, node, "master_id") != 0)
            if is_rs:
                feedback_id = motor_id
            else:
                requested_feedback = current["feedback_id"]
                if isinstance(requested_feedback, bool) or not isinstance(requested_feedback, (int, float)) or \
                        int(requested_feedback) != requested_feedback:
                    raise ValueError(f"{prefix}feedback_id 必须是整数")
                requested_feedback = int(requested_feedback)
                if requested_feedback < -1 or requested_feedback > 0x7FF:
                    raise ValueError(f"{prefix}feedback_id 超出 -1..2047")
                if "feedback_id" in values:
                    feedback_id = meta[model]["can_id_base"] + motor_id if requested_feedback < 0 else requested_feedback
                elif model_changed or ("id" in values and not original_feedback_explicit):
                    feedback_id = meta[model]["can_id_base"] + motor_id
                else:
                    feedback_id = requested_feedback
            external = current["external_reduction_ratio"]
            try:
                finite_external = isinstance(external, (int, float)) and not isinstance(external, bool) and \
                    math.isfinite(float(external))
            except (OverflowError, ValueError):
                finite_external = False
            if not finite_external:
                raise ValueError(f"{prefix}external_reduction_ratio 必须是有限数字")
            if external < 0 or external > 10000: raise ValueError(f"{prefix}external_reduction_ratio 超出 0..10000")
            if 0 < external < 1: raise ValueError(f"{prefix}external_reduction_ratio 应为 0、1 或大于 1 的减速比")
            if external not in {0, 1} and model in RM_MODELS and not (
                axis.startswith("chassis[") or axis in {"yaw", "yaw_upper", "pitch"}
            ):
                raise ValueError(f"{prefix}external_reduction_ratio 尚未接入该 RM 业务轴的物理量换算")
            source_can_id_raw, _ = self._motor_member(source, node, "can_id", 1)
            source_can_id = self._int_or_default(source_can_id_raw, 0, source)
            enabled = motor_id != 0 if model_changed or "id" in values else source_can_id != 0
            states[axis] = {"model": model, "id": motor_id, "bus": bus, "node": node,
                            "feedback_id": feedback_id, "enabled": enabled,
                            "external_reduction_ratio": float(external)}
            if states[axis]["node"].key.startswith("virtual:"):
                virtual_parents[states[axis]["node"].brace_start] = states[axis]["node"]
                continue
            if "model" in values:
                item = fields[prefix + "model"]
                output.append((item.start, item.end, model))
            if "id" in values or model_changed:
                item = fields[prefix + "id"]
                output.append((item.start, item.end, _format_number(motor_id, item.original, True)))
            desired = {}
            if model_changed:
                desired.update({"protocol": "MOTOR_PROTOCOL_INHERIT",
                                "control_mode": "MOTOR_CONTROL_MODE_INHERIT",
                                "master_id": "0u", "feedback_id": "0u", "feedback_id_enable": "0u",
                                "baudrate": "0u", "rx_timeout_ms": "0u"})
            if model == "MOTOR_MODEL_N6014B" and "id" in values:
                desired.update({"master_id": "0u", "feedback_id": "0u", "feedback_id_enable": "0u"})
            if "feedback_id" in values and not is_rs:
                requested_feedback = int(values["feedback_id"])
                if requested_feedback < 0:
                    desired.update({"master_id": "0u", "feedback_id": "0u", "feedback_id_enable": "0u"})
                else:
                    desired.update({"master_id": "0u", "feedback_id": f"{requested_feedback}u",
                                    "feedback_id_enable": "1u"})
            if "bus" in values:
                if is_rs:
                    desired.update({"transport": "MOTOR_TRANSPORT_RS485", "rs485_port": f"{int(bus[-1]) - 1}u"})
                else:
                    desired.update({"transport": "MOTOR_TRANSPORT_CAN", "can_bus": f"{int(bus[-1])}u"})
            if "external_reduction_ratio" in values:
                desired["external_reduction_ratio"] = _format_number(external,
                    fields[prefix + "external_reduction_ratio"].original or "1.0f", False)
            self._motor_named_edits(source, states[axis]["node"], desired, output)
        for _brace_start, node in virtual_parents.items():
            rows = []
            for index in range(6):
                axis = f"arm[{index}]"; prefix = f"motor.{axis}."
                state = {name: fields[prefix + name].public["value"] for name in
                         ("model", "id", "bus", "external_reduction_ratio")}
                state.update(changes.get(axis, {}))
                bus = state["bus"]
                members = [f".model = {state['model']}", f".can_id = {int(state['id'])}u"]
                if bus.startswith("RS485_"):
                    members += [".transport = MOTOR_TRANSPORT_RS485", f".rs485_port = {int(bus[-1]) - 1}u"]
                else:
                    members += [".transport = MOTOR_TRANSPORT_CAN", f".can_bus = {int(bus[-1])}u"]
                    feedback = changes.get(axis, {}).get("feedback_id", -1)
                    if feedback >= 0:
                        members += [f".feedback_id = {int(feedback)}u", ".feedback_id_enable = 1u"]
                members.append(".external_reduction_ratio = " +
                               _format_number(state["external_reduction_ratio"], "1.0f", False))
                rows.append("{" + ", ".join(members) + "}")
            output.append((node.brace_start + 1, node.brace_end, "\n                    " +
                           ",\n                    ".join(rows) + ",\n                "))
        self._validate_motor_states(source, states, fields)

    @staticmethod
    def _motor_named_edits(source, node, desired, output):
        missing = []
        for name, replacement in desired.items():
            existing = _child(node, name)
            if existing is None: missing.append(f".{name} = {replacement}")
            else: output.append((existing.value_start, existing.value_end, replacement))
        if missing:
            mask = source["mask"]
            last = node.brace_end - 1
            while last > node.brace_start and mask[last].isspace(): last -= 1
            if last > node.brace_start and mask[last] == ",":
                line_start = source["text"].rfind("\n", 0, node.brace_end) + 1
                field_indent = None
                for child in node.children:
                    child_line = source["text"].rfind("\n", 0, child.value_start) + 1
                    indent = source["text"][child_line:child_line + len(source["text"][child_line:]) - len(source["text"][child_line:].lstrip())]
                    if indent:
                        field_indent = indent
                        break
                field_indent = field_indent or source["text"][line_start:node.brace_end] + "    "
                inserted = "".join(f"{field_indent}{item},\n" for item in missing)
                output.append((line_start, line_start, inserted))
            else:
                output.append((node.brace_end, node.brace_end, ", " + ", ".join(missing)))

    def _validate_motor_states(self, source, changed, fields):
        all_states = {}
        meta = self._model_meta()
        for axis in AXIS_LABELS:
            prefix = f"motor.{axis}."
            if prefix + "model" not in fields: continue
            node = fields[prefix + "model"].node
            state = {name: fields[prefix + name].public["value"] for name in ("model", "id", "bus")}
            state["node"] = node
            feedback = fields.get(prefix + "feedback_id")
            state["feedback_id"] = feedback.public["value"] if feedback is not None else state["id"]
            source_can_id_raw, _ = self._motor_member(source, node, "can_id", 1)
            state["enabled"] = self._int_or_default(source_can_id_raw, 0, source) != 0
            all_states[axis] = state
        for axis, state in changed.items():
            all_states[axis].update({key: value for key, value in state.items()
                                     if key in {"model", "id", "bus", "feedback_id", "enabled"}})
        occupied_cmd = {}
        occupied_feedback = {}
        for axis, state in all_states.items():
            if not state["enabled"]: continue
            model = state["model"]; bus = state["bus"]; node = state["node"]
            if bus.startswith("RS485_"):
                cmd_key = feedback_key = (bus, int(state["id"]))
            else:
                base = meta[model]["can_id_base"]
                cmd_key = (bus, base + int(state["id"]))
                feedback_key = (bus, int(state["feedback_id"]))
            if cmd_key in occupied_cmd:
                raise ValueError(f"电机命令 ID 冲突: {occupied_cmd[cmd_key]} 与 {axis} 共用 {cmd_key[0]} ID {cmd_key[1]:#x}")
            if feedback_key in occupied_feedback:
                raise ValueError(f"电机反馈 ID 冲突: {occupied_feedback[feedback_key]} 与 {axis} 共用 "
                                 f"{feedback_key[0]} ID {feedback_key[1]:#x}")
            occupied_cmd[cmd_key] = axis
            occupied_feedback[feedback_key] = axis

    def _motor_hidden_int(self, source, node, name):
        if node.key.startswith("virtual:"):
            return 0
        raw, _span = self._motor_member(source, node, name)
        if raw is None:
            return 0
        return self._int_or_default(raw, 0, source)

    def _validate_motor_candidate(self, target, candidates, sources):
        # 重新读取候选电机表，确保插入字段也能被完整解析；其余语义已在生成替换前检查。
        text = candidates["ConfigHardware.inc"].decode("utf-8-sig")
        mask = _mask(text)
        nodes = _nodes(text, mask)
        if not any(node.key == "motor" for node in nodes):
            raise ValueError("修改后电机表无效")

    @staticmethod
    def _check_edits(edits):
        previous = -1
        for start, end, _replacement in sorted(edits):
            if start is None or end is None or start < previous or start > end:
                raise ValueError("配置变更范围冲突")
            previous = end

    def _commit(self, sources, candidates):
        changed = [name for name in CONFIG_FILES if candidates[name] != sources[name]["data"]]
        staged = {}
        try:
            for name in changed:
                path = sources[name]["path"]
                with tempfile.NamedTemporaryFile(mode="wb", prefix=".configedit-", dir=path.parent, delete=False) as handle:
                    handle.write(candidates[name])
                    staged[name] = Path(handle.name)
            replaced = []
            try:
                for name in changed:
                    os.replace(staged[name], sources[name]["path"])
                    replaced.append(name)
            except BaseException:
                for name in reversed(replaced):
                    self.workspace._write_atomic(sources[name]["path"], sources[name]["data"])
                raise
        finally:
            for path in staged.values(): path.unlink(missing_ok=True)
