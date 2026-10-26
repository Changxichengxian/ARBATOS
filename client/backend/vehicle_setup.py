"""新车型向导的能力目录和真实装配规则。"""

from __future__ import annotations

import re
import shutil
from pathlib import Path

import tomlkit


CATALOG_SCHEMA = 1

_TEMPLATES = {
    "classic_single": {"label": "普通底盘＋普通云台＋发射", "board": "dm_mc02_h7",
                       "source": "HERO-M", "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"]},
    "classic_dual_yaw": {"label": "普通底盘＋双水平轴云台＋发射", "board": "dm_mc02_h7",
                         "source": "SENTINEL-M", "mechanisms": ["classic_chassis", "dual_yaw_gimbal", "rm_shoot"]},
    "wheelleg": {"label": "轮腿底盘＋可选云台和发射", "board": "dm_mc02_h7",
                 "source": "MINIWHEELEG-M", "mechanisms": ["wheelleg"]},
    "a_classic_single": {"label": "A 板普通底盘＋普通云台＋发射", "board": "dji_a_f427",
                         "source": "INFANTRY-TIAN-A",
                         "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"]},
    "c_classic_single": {"label": "C 板普通底盘＋普通云台＋发射", "board": "dji_c_f407",
                         "source": "BASE-C",
                         "mechanisms": ["classic_chassis", "single_gimbal", "rm_shoot"]},
}

_RM_CURRENT = (("MOTOR_MODEL_3508", "DJI M3508"), ("MOTOR_MODEL_3510", "DJI M3510"),
               ("MOTOR_MODEL_2006", "DJI M2006"))
_RM_GIMBAL = _RM_CURRENT + (("MOTOR_MODEL_6020", "DJI GM6020"),
                            ("MOTOR_MODEL_6623", "DJI GM6623"))
_MIT = (("MOTOR_MODEL_DM_H3510_V10", "达妙 H3510 V1.0"),
        ("MOTOR_MODEL_DM_J4310_2EC_V11", "达妙 J4310-2EC V1.1"),
        ("MOTOR_MODEL_DM_J4310_2EC_V12", "达妙 J4310-2EC V1.2"),
        ("MOTOR_MODEL_DM_J8009_2EC_V10", "达妙 J8009-2EC V1.0"),
        ("MOTOR_MODEL_DM_J8006_2EC_V10", "达妙 J8006-2EC V1.0"),
        ("MOTOR_MODEL_DM_J8006_2EC_V11", "达妙 J8006-2EC V1.1"),
        ("MOTOR_MODEL_DM_6215", "达妙 6215"))


def _models(rows, transport="CAN"):
    return [{"id": ident, "label": label, "transport": transport} for ident, label in rows]


_MECHANISMS = {
    "classic_chassis": {"label": "普通底盘", "kind": "chassis", "controller": ("chassis", "classic"),
                        "requires": ["classic_single", "classic_dual_yaw", "a_classic_single", "c_classic_single"],
                        "groups": [{"id": "chassis", "label": "底盘电机", "capacity": 4,
                                    "transport": ["CAN"], "models": _models(_RM_CURRENT)}]},
    "wheelleg": {"label": "轮腿底盘", "kind": "chassis", "controller": ("wheelleg", "mit"),
                 "requires": ["wheelleg"],
                 "groups": [{"id": "wheelleg", "label": "轮腿六轴", "capacity": 6,
                             "transport": ["CAN"], "models": _models(_MIT)}]},
    "single_gimbal": {"label": "普通云台", "kind": "gimbal", "controller": ("gimbal", "single"),
                      "requires": ["classic_single", "wheelleg", "a_classic_single", "c_classic_single"],
                      "groups": [{"id": "gimbal", "label": "云台电机", "capacity": 2,
                                  "transport": ["CAN"], "models": _models(_RM_GIMBAL)}]},
    "dual_yaw_gimbal": {"label": "双水平轴云台", "kind": "gimbal", "controller": ("gimbal", "dual_yaw"),
                        "requires": ["classic_dual_yaw"],
                        "groups": [{"id": "dual_gimbal", "label": "双水平轴云台三轴", "capacity": 3,
                                    "transport": ["CAN"], "models": _models(_RM_GIMBAL)}]},
    "rm_shoot": {"label": "发射机构", "kind": "shoot", "controller": ("shoot", "rm"),
                 "requires": ["classic_single", "classic_dual_yaw", "wheelleg", "a_classic_single", "c_classic_single"],
                 "groups": [{"id": "shoot", "label": "摩擦轮、摩擦轮、拨弹", "capacity": 3,
                             "transport": ["CAN"], "models": _models(_RM_CURRENT)}]},
    "pwm_servos": {"label": "PWM 舵机", "kind": "accessory", "service": "SERVO",
                   "requires": ["classic_single", "classic_dual_yaw", "wheelleg"],
                   "groups": [{"id": "pwm_servos", "label": "PWM 舵机", "capacity": 4,
                               "transport": ["PWM"], "models": [
                                   {"id": "PWM_SERVO", "label": "标准 PWM 舵机", "transport": "PWM"}]}]},
}

_RADIOS = {
    "dji_sbus": {"label": "大疆遥控器（现有 DBUS 接收链）", "services": ["RC_SBUS"]},
    "elrs_crsf": {"label": "ELRS 遥控器（CRSF）", "services": ["ELRS_LINK"]},
}

_GROUP_AXES = {
    "chassis": ["chassis[0]", "chassis[1]", "chassis[2]", "chassis[3]"],
    "wheelleg": ["arm[0]", "arm[1]", "arm[2]", "arm[3]", "arm[4]", "arm[5]"],
    "gimbal": ["yaw", "pitch"], "dual_gimbal": ["yaw", "yaw_upper", "pitch"],
    "shoot": ["friction[0]", "friction[1]", "trigger"],
}

_GROUP_DEFAULTS = {
    "chassis": [{"model": "MOTOR_MODEL_3508", "id": i, "bus": "CAN1"} for i in range(1, 5)],
    "wheelleg": [
        {"model": "MOTOR_MODEL_DM_H3510_V10", "id": 6, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_DM_J4310_2EC_V11", "id": 1, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_DM_J4310_2EC_V11", "id": 2, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_DM_J4310_2EC_V12", "id": 3, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_DM_J4310_2EC_V12", "id": 4, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_DM_H3510_V10", "id": 5, "bus": "CAN1"}],
    "gimbal": [{"model": "MOTOR_MODEL_6020", "id": 1, "bus": "CAN2"},
               {"model": "MOTOR_MODEL_6020", "id": 2, "bus": "CAN2"}],
    "dual_gimbal": [{"model": "MOTOR_MODEL_6020", "id": i, "bus": "CAN2"} for i in range(1, 4)],
    "shoot": [{"model": "MOTOR_MODEL_3510", "id": 1, "bus": "CAN2"},
              {"model": "MOTOR_MODEL_3510", "id": 2, "bus": "CAN2"},
              {"model": "MOTOR_MODEL_2006", "id": 3, "bus": "CAN2"}],
}

_A_GROUP_DEFAULTS = {
    "chassis": [
        {"model": "MOTOR_MODEL_3508", "id": 3, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_3508", "id": 2, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_3508", "id": 4, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_3508", "id": 1, "bus": "CAN1"}],
    "gimbal": [
        {"model": "MOTOR_MODEL_6020", "id": 1, "bus": "CAN1"},
        {"model": "MOTOR_MODEL_6020", "id": 2, "bus": "CAN2"}],
    "shoot": [
        {"model": "MOTOR_MODEL_3508", "id": 1, "bus": "CAN2"},
        {"model": "MOTOR_MODEL_3508", "id": 2, "bus": "CAN2"},
        {"model": "MOTOR_MODEL_2006", "id": 7, "bus": "CAN2"}],
}

_POWER_ENUM = {"Auto": "ChassisPowerLimitAuto", "TotalCommand": "ChassisPowerLimitTotalCommand",
               "Model": "ChassisPowerLimitModel", "ModelAndTotal": "ChassisPowerLimitModelAndTotal"}
_POWER_ALIASES = {"auto": "Auto", "totalcommand": "TotalCommand", "total_command": "TotalCommand",
                  "model": "Model", "modelandtotal": "ModelAndTotal", "model_and_total": "ModelAndTotal"}


def catalog():
    mechanisms = []
    for ident, item in _MECHANISMS.items():
        motor_groups = []
        for group in item["groups"]:
            group_info = {**group, "direction_status": "pending_validation"}
            group_id = group["id"]
            if group_id == "pwm_servos":
                group_info["defaults"] = []
                group_info["empty_label"] = "不启用"
            else:
                group_info["defaults"] = [dict(row) for row in _GROUP_DEFAULTS[group_id]]
                defaults_by_board = {"dm_mc02_h7": [dict(row) for row in _GROUP_DEFAULTS[group_id]]}
                if group_id in _A_GROUP_DEFAULTS:
                    defaults_by_board["dji_a_f427"] = [dict(row) for row in _A_GROUP_DEFAULTS[group_id]]
                    defaults_by_board["dji_c_f407"] = [dict(row) for row in _GROUP_DEFAULTS[group_id]]
                group_info["defaults_by_board"] = defaults_by_board
                group_info["empty_label"] = "使用默认装配"
            motor_groups.append(group_info)
        mechanisms.append({"id": ident, "label": item["label"], "kind": item["kind"],
                           "requires": list(item["requires"]),
                           "motor_groups": motor_groups})
    return {"schema": CATALOG_SCHEMA,
            "boards": [
                {"id": "dm_mc02_h7", "label": "M 板（DM MC02 H7）", "available": True,
                 "reason": "已有完整机器人运行入口",
                 "templates": [ident for ident, item in _TEMPLATES.items()
                               if item["board"] == "dm_mc02_h7"]},
                {"id": "dji_a_f427", "label": "A 板（DJI A F427）", "available": True,
                 "reason": "经典底盘、普通云台、发射、DBUS/ELRS 和裁判已接入配置；新建组合尚未实板验证",
                 "templates": ["a_classic_single"]},
                {"id": "dji_c_f407", "label": "C 板（DJI C F407）", "available": True,
                 "reason": "经典底盘、普通云台和发射；支持 DBUS/ELRS，装配参数仍需实车确认",
                 "templates": ["c_classic_single"]}],
            "templates": [{"id": ident, **item} for ident, item in _TEMPLATES.items()],
            "mechanisms": mechanisms,
            "radios": [{"id": ident, **item,
                        "requires_board": ["dm_mc02_h7", "dji_a_f427", "dji_c_f407"]}
                       for ident, item in _RADIOS.items()],
            "power": {"limitMode": list(_POWER_ENUM), "note": "只用于经典四轮底盘；轮腿保持 Auto。"},
            "transport_capabilities": [
                {"id": "CAN", "available": True, "label": "CAN / CAN-FD 电机"},
                {"id": "RS485", "available": True, "label": "RS485 电机",
                 "reason": "已有驱动可在装配页使用；当前轮腿控制器只开放 MIT CAN 型号。"},
                {"id": "PWM", "available": True, "label": "PWM 舵机"},
                {"id": "UART_MOTOR", "available": False, "label": "总线舵机",
                 "reason": "具体厂商协议尚未确定，当前不生成虚假的串口驱动。"}],
            "steps": ["create", "select_board", "select_mechanisms", "select_motors_and_radio",
                      "validate_build", "direction_check"]}


def _normalize_power(power, wheelleg):
    if power is None:
        return {"limitMode": "Auto"}
    allowed = {"limitMode", "limit_mode", "power_limit", "no_judge_total_current_limit"}
    if not isinstance(power, dict) or set(power) - allowed:
        raise ValueError("功率策略字段无效")
    raw = power.get("limitMode", power.get("limit_mode", "Auto"))
    if not isinstance(raw, str):
        raise ValueError("功率策略模式无效")
    mode = raw if raw in _POWER_ENUM else _POWER_ALIASES.get(raw.replace("-", "_").lower())
    if mode is None:
        raise ValueError("功率策略模式无效")
    if wheelleg and mode != "Auto":
        raise ValueError("轮腿当前不读取经典四轮功率策略，只能使用 Auto")
    result = {"limitMode": mode}
    for field in ("power_limit", "no_judge_total_current_limit"):
        if field in power:
            value = power[field]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 1_000_000:
                raise ValueError(f"功率策略 {field} 必须是 0 到 1000000 的数字")
            result[field] = value
    return result


def prepare(setup):
    if not isinstance(setup, dict):
        raise ValueError("新车型向导配置必须是对象")
    board = setup.get("board")
    if board not in {"dm_mc02_h7", "dji_a_f427", "dji_c_f407"}:
        raise ValueError("请选择能力目录中的可用开发板")
    template = setup.get("template")
    if template not in _TEMPLATES or _TEMPLATES[template]["board"] != board:
        raise ValueError("车型骨架与开发板不匹配")
    selected = setup.get("mechanisms")
    if not isinstance(selected, list) or len(selected) != len(set(selected)) or any(x not in _MECHANISMS for x in selected):
        raise ValueError("机构选择无效或重复")
    disallowed = [item for item in selected if template not in _MECHANISMS[item]["requires"]]
    if disallowed:
        raise ValueError("当前车型骨架不支持机构: " + ", ".join(disallowed))
    chassis = [item for item in selected if _MECHANISMS[item]["kind"] == "chassis"]
    gimbal = [item for item in selected if _MECHANISMS[item]["kind"] == "gimbal"]
    shoots = [item for item in selected if _MECHANISMS[item]["kind"] == "shoot"]
    expected = "wheelleg" if template == "wheelleg" else "classic_chassis"
    if chassis != [expected]:
        raise ValueError("车型必须保留所选骨架对应的一个底盘")
    if len(gimbal) > 1 or len(shoots) > 1:
        raise ValueError("云台和发射各最多选择一个")
    radio = setup.get("radio")
    if radio not in _RADIOS:
        raise ValueError("请选择大疆 DBUS 或 ELRS CRSF 遥控器")
    groups = {group["id"]: group for ident in selected for group in _MECHANISMS[ident]["groups"]}
    motors = setup.get("motors", {})
    if motors is None:
        motors = {}
    if not isinstance(motors, dict) or set(motors) - set(groups):
        raise ValueError("电机选择只能填写已启用机构的电机组")
    for group_id, rows in motors.items():
        if not isinstance(rows, list) or len(rows) > groups[group_id]["capacity"]:
            raise ValueError(f"{group_id} 电机数量超过该机构容量")
    verification_required = ["电机型号、总线、地址和转向",
                             "底盘或轮腿几何、PID 参数和关节零位",
                             "IMU 安装方向、安装部件和上电姿态基准",
                             "遥控通道映射、安全档和失联行为"]
    if expected == "wheelleg" and gimbal:
        verification_required.append("轮腿车体姿态与云台姿态是否需要独立 IMU；单板 IMU 的物理归属不能由向导推断")
    return {"source": _TEMPLATES[template]["source"], "board": board, "template": template,
            "mechanisms": list(selected), "radio": radio, "motor_groups": list(groups.values()),
            "motors": motors, "power": _normalize_power(setup.get("power"), expected == "wheelleg"),
            "direction_check_required": True, "commissioning_required": True,
            "motion_ready": False, "verification_required": verification_required}


def apply_to_config(source_text, prepared):
    doc = tomlkit.parse(source_text.removeprefix("\ufeff"))
    doc["board"] = prepared["board"]
    controllers = doc.get("controllers") or tomlkit.table()
    doc["controllers"] = controllers
    for key in list(controllers):
        del controllers[key]
    for ident in prepared["mechanisms"]:
        controller = _MECHANISMS[ident].get("controller")
        if controller:
            domain, value = controller
            controllers[domain] = {"type": value}
    services = [item for item in doc.get("services", [])
                if item not in {"RC_SBUS", "ELRS_LINK", "HOST_LINK", "SERVO"}]
    services.extend(_RADIOS[prepared["radio"]]["services"])
    if "pwm_servos" in prepared["mechanisms"]:
        services.append("SERVO")
    services = list(dict.fromkeys(services))
    doc["services"] = services
    doc["profile"] = "infantry" if prepared["board"] in {"dji_a_f427", "dji_c_f407"} else (
        "wheelleg" if "wheelleg" in prepared["mechanisms"] else
        ("sentry" if "dual_yaw_gimbal" in prepared["mechanisms"] else "hero"))
    if prepared["board"] == "dm_mc02_h7":
        ports = tomlkit.table()
        if prepared["radio"] == "dji_sbus":
            ports["rc_sbus"] = "uart5"
        else:
            ports["elrs_crsf"] = {"port": "uart5", "baud": 420000, "protocol": "crsf"}
        if "REFEREE_RX" in services:
            ports["referee"] = {"port": "uart7", "baud": 115200, "protocol": "rm_referee"}
        doc["ports"] = ports
        build = doc["build"]
        build["sources"] = ["RobotConfig.c", "Mc02Compat.c", "DetectTask.c", "ArmMotorTable.c"]
        if any(item in prepared["mechanisms"] for item in ("single_gimbal", "dual_yaw_gimbal")):
            build["sources"].append("PitchCaliBuiltin.c")
        build["shared_sources"] = ["shared/application/comm/host/HostLinkTaskStub.c"]
        for key in ("port_sources", "overlay"):
            if key in build: del build[key]
    elif prepared["board"] in {"dji_a_f427", "dji_c_f407"}:
        ports = tomlkit.table()
        if prepared["board"] == "dji_a_f427":
            if prepared["radio"] == "dji_sbus":
                ports["rc_sbus"] = "usart1"
            else:
                ports["elrs_crsf"] = {"port": "uart8", "baud": 420000, "protocol": "crsf"}
        else:
            if prepared["radio"] == "dji_sbus":
                ports["rc_sbus"] = "usart3"
            else:
                ports["elrs_crsf"] = {"port": "usart1", "baud": 420000, "protocol": "crsf"}
        if "REFEREE_RX" in services:
            ports["referee"] = {"port": "usart6", "baud": 115200, "protocol": "rm_referee"}
        doc["ports"] = ports
    elif "ports" in doc:
        del doc["ports"]
    return tomlkit.dumps(doc)


def _masked(text):
    result = list(text); index = 0; quote = None
    while index < len(text):
        if quote:
            if text[index] == "\\":
                result[index] = " "; index += 1
                if index < len(text): result[index] = " "
            elif text[index] == quote:
                result[index] = " "; quote = None
            else: result[index] = " "
        elif text.startswith("//", index):
            end = text.find("\n", index); end = len(text) if end < 0 else end
            result[index:end] = " " * (end - index); index = end - 1
        elif text.startswith("/*", index):
            end = text.find("*/", index + 2); end = len(text) - 2 if end < 0 else end
            result[index:end + 2] = " " * (end + 2 - index); index = end + 1
        elif text[index] in {'"', "'"}:
            quote = text[index]; result[index] = " "
        index += 1
    return "".join(result)


def _initializer_value_span(text, key):
    mask = _masked(text)
    match = re.search(rf"(?m)^\s*\.{re.escape(key)}\s*=\s*", mask)
    if not match: raise ValueError(f"模板缺少 {key} 配置")
    start = match.end()
    while start < len(mask) and mask[start].isspace(): start += 1
    if start >= len(mask): raise ValueError(f"模板的 {key} 配置不完整")
    if mask[start] != "{":
        end = mask.find(",", start)
        if end < 0: raise ValueError(f"模板的 {key} 配置不完整")
        return start, end
    depth = 0
    for index in range(start, len(mask)):
        if mask[index] == "{": depth += 1
        elif mask[index] == "}":
            depth -= 1
            if depth == 0: return start, index + 1
    raise ValueError(f"模板的 {key} 配置括号不完整")


def _copy_initializer_value(destination_text, source_text, key):
    a, b = _initializer_value_span(destination_text, key)
    c, d = _initializer_value_span(source_text, key)
    return destination_text[:a] + source_text[c:d] + destination_text[b:]


def _motor_row(item, control_mode=None, master=False):
    fields = [f".model = {item['model']}", f".can_id = {item['id']}u", f".can_bus = {int(item['bus'][-1])}u"]
    if control_mode: fields.append(f".control_mode = {control_mode}")
    if master: fields.extend([f".master_id = {item['id']}u", ".transport = MOTOR_TRANSPORT_CAN"])
    if item["model"] in {"MOTOR_MODEL_6020", "MOTOR_MODEL_6623"}:
        fields.append(".command_limit_abs = 16384")
    return "{" + ", ".join(fields) + "}"


def _group_defaults(prepared, group_id):
    if prepared["board"] == "dji_a_f427" and group_id in _A_GROUP_DEFAULTS:
        return _A_GROUP_DEFAULTS[group_id]
    return _GROUP_DEFAULTS[group_id]


def _motor_initializer(prepared):
    selected = set(prepared["mechanisms"])
    off = lambda model, count, bus: [{"model": model, "id": 0, "bus": bus} for _ in range(count)]
    chassis = _group_defaults(prepared, "chassis") if "classic_chassis" in selected else off("MOTOR_MODEL_3508", 4, "CAN1")
    shoot_defaults = _group_defaults(prepared, "shoot")
    friction = (shoot_defaults[:2] if "rm_shoot" in selected else
                off("MOTOR_MODEL_3510", 2, "CAN2")) + off("MOTOR_MODEL_3510", 2, "CAN2")
    if "single_gimbal" in selected:
        yaw, pitch = _group_defaults(prepared, "gimbal")
        yaw_upper = {"model": "MOTOR_MODEL_6020", "id": 0, "bus": "CAN2"}
    elif "dual_yaw_gimbal" in selected:
        yaw, yaw_upper, pitch = _GROUP_DEFAULTS["dual_gimbal"]
    else:
        yaw = yaw_upper = pitch = {"model": "MOTOR_MODEL_6020", "id": 0, "bus": "CAN2"}
    trigger = shoot_defaults[2] if "rm_shoot" in selected else {"model": "MOTOR_MODEL_2006", "id": 0, "bus": "CAN2"}
    arm = _GROUP_DEFAULTS["wheelleg"] if "wheelleg" in selected else off("MOTOR_MODEL_DM_J4310_2EC_V12", 6, "CAN1")
    lines = ["{", "            .chassis =", "                {"]
    lines += [f"                    {_motor_row(item)}," for item in chassis]
    lines += ["                },", "            .friction =", "                {"]
    lines += [f"                    {_motor_row(item)}," for item in friction]
    lines += ["                },", f"            .yaw = {_motor_row(yaw, 'MOTOR_CONTROL_MODE_VOLTAGE')},",
              f"            .yaw_upper = {_motor_row(yaw_upper, 'MOTOR_CONTROL_MODE_VOLTAGE')},",
              f"            .pitch = {_motor_row(pitch, 'MOTOR_CONTROL_MODE_VOLTAGE')},",
              f"            .trigger = {_motor_row(trigger)},", "            .arm =", "                {"]
    lines += [f"                    {_motor_row(item, 'MOTOR_CONTROL_MODE_MIT', True)}," for item in arm]
    lines += ["                },", "        }"]
    return "\n".join(lines)


def _replace_power(text, power):
    start, end = _initializer_value_span(text, "power")
    body = text[start:end]
    body, count = re.subn(r"(\.limitMode\s*=\s*)[^,]+", r"\g<1>" + _POWER_ENUM[power["limitMode"]], body, count=1)
    if count != 1: raise ValueError("模板缺少 limitMode，不能创建向导车型")
    for field in ("power_limit", "no_judge_total_current_limit"):
        if field in power:
            body, count = re.subn(r"(\." + re.escape(field) + r"\s*=\s*)[^,]+",
                                  r"\g<1>" + f"{float(power[field]):.1f}f", body, count=1)
            if count != 1: raise ValueError(f"模板缺少 {field}，不能创建向导车型")
    return text[:start] + body + text[end:]


def apply_files(destination, prepared):
    destination = Path(destination); robot_root = destination.parent; selected = set(prepared["mechanisms"])
    tuning_path = destination / "ConfigTuning.inc"; tuning = tuning_path.read_text(encoding="utf-8")
    if "classic_chassis" in selected and prepared["board"] == "dm_mc02_h7":
        hero = (robot_root / "HERO-M/ConfigTuning.inc").read_text(encoding="utf-8")
        tuning = _copy_initializer_value(tuning, hero, "chassis")
        tuning = _copy_initializer_value(tuning, hero, "power")
    if "wheelleg" in selected:
        wheel = (robot_root / "MINIWHEELEG-M/ConfigTuning.inc").read_text(encoding="utf-8")
        tuning = _copy_initializer_value(tuning, wheel, "WheelLegMit")
    if "single_gimbal" in selected and prepared["board"] == "dm_mc02_h7":
        hero = (robot_root / "HERO-M/ConfigTuning.inc").read_text(encoding="utf-8")
        tuning = _copy_initializer_value(tuning, hero, "gimbal")
    if "dual_yaw_gimbal" in selected:
        sentinel = (robot_root / "SENTINEL-M/ConfigTuning.inc").read_text(encoding="utf-8")
        tuning = _copy_initializer_value(tuning, sentinel, "gimbal")
        tuning = _copy_initializer_value(tuning, sentinel, "dual_gimbal")
    if "rm_shoot" in selected and prepared["board"] == "dm_mc02_h7":
        hero = (robot_root / "HERO-M/ConfigTuning.inc").read_text(encoding="utf-8")
        tuning = _copy_initializer_value(tuning, hero, "shoot")
    tuning_path.write_text(_replace_power(tuning, prepared["power"]), encoding="utf-8")

    input_path = destination / "ConfigInput.inc"; input_text = input_path.read_text(encoding="utf-8")
    source_name = ("BASE-C" if prepared["board"] == "dji_c_f407" else
                  "INFANTRY-TIAN-A" if prepared["board"] == "dji_a_f427" else
                   ("MINIWHEELEG-M" if "wheelleg" in selected else
                    ("SENTINEL-M" if "dual_yaw_gimbal" in selected else "HERO-M")))
    source_input = (robot_root / source_name / "ConfigInput.inc").read_text(encoding="utf-8")
    input_text = _copy_initializer_value(input_text, source_input, "manual_input")
    input_text = _copy_initializer_value(input_text, source_input, "input")
    active = "MANUAL_INPUT_SRC_ELRS" if prepared["radio"] == "elrs_crsf" else "MANUAL_INPUT_SRC_DBUS"
    input_text, count = re.subn(r"(\.active_source\s*=\s*)MANUAL_INPUT_SRC_\w+", r"\g<1>" + active, input_text, count=1)
    if count != 1: raise ValueError("模板缺少 active_source，不能创建向导车型")
    input_path.write_text(input_text, encoding="utf-8")

    hardware_path = destination / "ConfigHardware.inc"; hardware = hardware_path.read_text(encoding="utf-8")
    start, end = _initializer_value_span(hardware, "motor")
    hardware_path.write_text(hardware[:start] + _motor_initializer(prepared) + hardware[end:], encoding="utf-8")
    if prepared["board"] == "dm_mc02_h7" and any(item in selected for item in ("single_gimbal", "dual_yaw_gimbal")):
        calibration = destination / "PitchCaliBuiltin.c"
        if not calibration.exists(): shutil.copy2(robot_root / "HERO-M/PitchCaliBuiltin.c", calibration)

    note_path = destination / "安装说明.md"
    section = [f"# {destination.name} 安装说明", "", "## 向导新车首次启用", "",
               "该车型由桌面向导组合生成，默认保持单电机安全模式，且未选择任何目标电机。",
               f"机构参数分别取自已有车型模板，组合骨架来自 `{prepared['source']}`。这些数值只是调试起点。",
               "编译通过只说明配置和代码能组成固件，不代表已经通过运动测试。首次上车前仍需确认：", ""]
    section.extend(f"- {item}" for item in prepared["verification_required"])
    section.extend(["", "确认完成后，先做断动力检查，再做短时、单变量、低输出的方向测试。", ""])
    note_path.write_text("\n".join(section), encoding="utf-8")


def motor_changes(prepared):
    """转成 configedit 的字段修改，并复用其协议、范围和总线冲突检查。"""
    changes = {}; bindings = prepared.get("motors") or {}
    groups = {group["id"]: group for group in prepared["motor_groups"]}
    for group_id, axes in _GROUP_AXES.items():
        if group_id not in groups: continue
        allowed = {row["id"] for row in groups[group_id]["models"]}
        requested = bindings.get(group_id, []); defaults = _group_defaults(prepared, group_id)
        for index, (axis, default) in enumerate(zip(axes, defaults)):
            item = requested[index] if index < len(requested) else ""; values = dict(default)
            if isinstance(item, str):
                if item: values["model"] = item
            elif isinstance(item, dict):
                if set(item) - {"model", "id", "bus"}: raise ValueError(f"{group_id} 电机只能填写 model、id、bus")
                values.update(item)
            else: raise ValueError(f"{group_id} 电机选择必须是型号字符串或装配对象")
            if values["model"] not in allowed: raise ValueError(f"{group_id} 不支持型号 {values['model']}")
            for field in ("model", "id", "bus"): changes[f"motor.{axis}.{field}"] = values[field]
    if "pwm_servos" in groups:
        requested = bindings.get("pwm_servos", [])
        for index in range(4):
            item = requested[index] if index < len(requested) else ""
            if isinstance(item, dict):
                if set(item) != {"model"}: raise ValueError("PWM 舵机只接受型号选择")
                item = item["model"]
            if not isinstance(item, str) or item not in {"", "PWM_SERVO"}: raise ValueError("PWM 舵机型号无效")
            changes[f"servo.channels[{index}].enabled"] = item == "PWM_SERVO"
            changes[f"servo.channels[{index}].port"] = index + 1
    return changes
