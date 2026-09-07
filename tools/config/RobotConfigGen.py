"""从 Robotconfig 的车型声明生成构建输入；不修改手写源码。

TOML 只负责选能力和装配，任务目录负责描述实现。所有生成文件写入
构建目录；缺项、冲突和未知字段在编译前报错，不能静默使用另一个车型。
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config.SourcePolicy import FORBIDDEN_SOURCE


ROOT = Path(__file__).resolve().parents[2]
CATALOG = "shared/application/robot/RobotTaskCatalog.def"
BOARDS = {
    "dm_mc02_h7": {"directory": "boards/DmMc02H7", "kind": "STM32H7", "cpu_hz": 500000000,
                   "can_buses": 2, "rs485_ports": 2, "fpu": 1, "stack": 1024},
    "dji_a_f427": {"directory": "boards/DjiAF427", "kind": "STM32F427", "cpu_hz": 180000000,
                   "can_buses": 2, "rs485_ports": 0, "fpu": 1, "stack": 256},
    "dji_c_f407": {"directory": "boards/DjiCF407", "kind": "STM32F407", "cpu_hz": 168000000,
                   "can_buses": 2, "rs485_ports": 0, "fpu": 1, "stack": 256},
}
BUILTINS = {
    "chassis": {"classic": "CLASSIC_CHASSIS"},
    "gimbal": {"single": "SINGLE_GIMBAL", "dual_yaw": "DUAL_YAW_GIMBAL"},
    "wheelleg": {"mit": "WHEELLEG_MIT"},
    "arm": {"standard": "ARM"},
    "shoot": {"rm": None},
}
PRIORITIES = {"Low", "Normal", "AboveNormal", "High", "Realtime"}
IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
TARGET_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_-]*\Z")


class ConfigError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise ConfigError(message)


def keys(value, allowed, where):
    require(isinstance(value, dict), f"{where} 必须是表")
    unknown = set(value) - set(allowed)
    require(not unknown, f"{where} 有未知字段: {', '.join(sorted(unknown))}")


def read_toml(path):
    try:
        return tomllib.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise ConfigError(f"{path}: {exc}") from exc


def inside(base, value, suffix=None):
    require(isinstance(value, str) and value, "路径必须是非空字符串")
    path = (base / value).resolve()
    require(path.is_relative_to(base.resolve()), f"路径超出允许目录: {value}")
    require(path.is_file(), f"文件不存在: {path}")
    if suffix:
        require(path.suffix.lower() in suffix, f"文件类型不允许: {path}")
    return path


def string_list(value, where):
    require(isinstance(value, list) and all(isinstance(x, str) for x in value),
            f"{where} 必须是字符串列表")
    require(len(set(value)) == len(value), f"{where} 有重复条目")
    return value


def number(value, where, low, high, integer=False):
    require(type(value) in ((int,) if integer else (int, float)), f"{where} 必须是数值")
    require(math.isfinite(value) and low <= value <= high, f"{where} 必须在 {low} 到 {high} 之间")
    return value


def task_catalog(root):
    result = {}
    used_ids = set()
    for line_no, line in enumerate((root / CATALOG).read_text(encoding="utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line.startswith("ROBOT_TASK("):
            continue
        require(line.endswith(")"), f"任务目录第 {line_no} 行必须完整写在一行")
        row = next(csv.reader([line[len("ROBOT_TASK("):-1]], skipinitialspace=True))
        require(len(row) == 17, f"任务目录第 {line_no} 行需要 17 个字段，实际 {len(row)}")
        symbol, task_id, module, task, kind, period, budget, sm, so, priority, flags, resource, header, entry, order, deps, source = row
        kind = kind.removeprefix("RobotModuleKind")
        priority = priority.removeprefix("RobotModulePriority")
        task_id = int(task_id.rstrip("uU"))
        require(IDENTIFIER.fullmatch(symbol) and symbol not in result, f"任务名无效或重复: {symbol}")
        require(0 < task_id < 128 and task_id not in used_ids, f"任务编号无效或重复: {task_id}")
        require(priority in PRIORITIES, f"任务 {symbol} 优先级无效: {priority}")
        used_ids.add(task_id)
        result[symbol] = dict(symbol=symbol, id=task_id, module=module, task=task, kind=kind,
                              period=period, budget=budget, stack_m=int(sm.rstrip("uU")),
                              stack_other=int(so.rstrip("uU")), priority=priority, flags=flags,
                              resource=resource, header=header, entry=entry, order=int(order.rstrip("uU")), source=source,
                              dependencies=deps.split("|") if deps else [])
    require(result, "任务目录为空")
    for task in result.values():
        require(all(x in result for x in task["dependencies"]), f"{task['symbol']} 有不存在的依赖")
    return result


def targets(root):
    root = Path(root).resolve()
    result = []
    seen = set()
    for path in sorted((root / "Robotconfig").glob("*/RobotConfig.toml")):
        name = path.parent.name
        require(TARGET_NAME.fullmatch(name), f"车型目录名无效: {name}")
        require(name.lower() not in seen, f"车型目录名大小写冲突: {name}")
        seen.add(name.lower())
        config = read_toml(path)
        require(config.get("board") in BOARDS, f"{name}: 未支持的开发板 {config.get('board')}")
        result.append(dict(name=name, preset=name.lower(), board=config["board"],
                           path=path.relative_to(root).as_posix()))
    return result


def plugins(root):
    root = Path(root).resolve()
    result = {}
    symbols = set()
    for path in sorted((root / "shared/controllers").glob("*/Controller.toml")):
        spec = read_toml(path)
        keys(spec, {"schema", "name", "domain", "header", "symbol", "sources", "requires", "parameters", "outputs"}, str(path))
        require(spec.get("schema") == 1, f"{path}: schema 必须为 1")
        name = spec.get("name")
        domain = spec.get("domain")
        require(isinstance(name, str) and TARGET_NAME.fullmatch(name), f"{path}: 控制器名字无效")
        require(name != "none", f"{path}: none 是关闭控制器的保留名字")
        require(domain in {"chassis", "gimbal"}, f"{path}: 控制域必须为 chassis 或 gimbal")
        require((domain, name) not in result and name not in BUILTINS[domain], f"重复控制器: {domain}/{name}")
        require(isinstance(spec.get("symbol"), str) and IDENTIFIER.fullmatch(spec["symbol"]), f"{path}: C 导出符号无效")
        require(spec["symbol"] not in symbols, f"{path}: 重复 C 导出符号 {spec['symbol']}")
        symbols.add(spec["symbol"])
        number(spec.get("outputs"), f"{path}: outputs", 1, 8, integer=True)
        spec["header_path"] = inside(path.parent, spec["header"], {".h"}).relative_to(root).as_posix()
        spec["source_paths"] = [inside(path.parent, x, {".c", ".cpp"}).relative_to(root).as_posix()
                                for x in string_list(spec.get("sources", []), f"{path}: sources")]
        require(spec["source_paths"], f"{path}: sources 不能为空")
        string_list(spec.get("requires", []), f"{path}: requires")
        parameters = spec.setdefault("parameters", {})
        require(isinstance(parameters, dict), f"{path}: parameters 必须是表")
        require(len(parameters) <= 255, f"{path}: 参数数量超过接口容量 255")
        for key, param in parameters.items():
            require(IDENTIFIER.fullmatch(key), f"{path}: 参数名无效 {key}")
            keys(param, {"default", "min", "max", "unit", "description"}, f"{name}.{key}")
            low, high = param.get("min", -1e30), param.get("max", 1e30)
            number(low, f"{key}.min", -1e30, 1e30)
            number(high, f"{key}.max", low, 1e30)
            if "default" in param:
                number(param["default"], f"{key}.default", low, high)
        spec["manifest"] = path.relative_to(root).as_posix()
        result[domain, name] = spec
    return result


def resolve(root, requested):
    # 文件检查会解析真实路径；仓库根目录也须统一，兼容 Windows 短路径和 ..。
    root = Path(root).resolve()
    found = [t for t in targets(root) if t["name"].lower() == requested.lower()]
    require(len(found) == 1, f"找不到车型 {requested}，需要 Robotconfig/<车型>/RobotConfig.toml")
    target = found[0]
    require(target["board"] == "dm_mc02_h7",
            f"{target['board']}: 板级定义保留，但完整机器人运行栈尚未迁移；当前仅支持 dm_mc02_h7")
    path = root / target["path"]
    data = read_toml(path)
    keys(data, {"schema", "board", "profile", "services", "controllers", "build", "features", "tasks"}, str(path))
    require(data.get("schema") == 1, f"{path}: schema 必须为 1")
    profile = data.get("profile", "custom")
    require(profile in {"hero", "infantry", "wheelleg", "sentry", "carrier", "custom"}, "未知 profile")
    catalog = task_catalog(root)
    available = plugins(root)
    selected = []
    reasons = {}
    visiting = set()

    def add(symbol, reason):
        require(symbol in catalog, f"{target['name']}: 未知任务 {symbol}（来源 {reason}）")
        require(symbol not in visiting, f"任务依赖循环: {' -> '.join(sorted(visiting))} -> {symbol}")
        if symbol in selected:
            reasons[symbol].append(reason)
            return
        task = catalog[symbol]
        require(task["entry"] != "None" and task["header"], f"任务 {symbol} 尚无当前平台运行入口")
        visiting.add(symbol)
        for dep in task["dependencies"]:
            add(dep, f"{symbol} 依赖")
        visiting.remove(symbol)
        selected.append(symbol)
        reasons[symbol] = [reason]

    for service in string_list(data.get("services", []), "services"):
        require(service in catalog and catalog[service]["kind"] != "Control", f"{service} 请在 controllers 选择控制器")
        add(service, "services 明确选择")
    controllers = data.get("controllers", {})
    keys(controllers, BUILTINS, "controllers")
    resolved_controls = []
    for domain, selection in controllers.items():
        keys(selection, {"type", "motors", "parameters", "period_ms"}, f"controllers.{domain}")
        name = selection.get("type")
        require(isinstance(name, str), f"controllers.{domain}.type 必须是控制器名字")
        if name == "none":
            require(len(selection) == 1, f"{domain}: none 不能附带参数")
            continue
        if name in BUILTINS[domain]:
            require(set(selection) == {"type"}, f"内置 {domain}/{name} 参数仍在 ConfigTuning.inc 设置")
            module = BUILTINS[domain][name]
            if module:
                add(module, f"controllers.{domain}={name}")
            else:
                add("CAN_COMMAND_TX", "shoot.rm")
                add("CAN_FEEDBACK_RX", "shoot.rm")
            resolved_controls.append(dict(domain=domain, type=name, builtin=True))
            continue
        require((domain, name) in available, f"找不到控制器 {domain}/{name}；检查 shared/controllers/*/Controller.toml")
        plugin = available[domain, name]
        for dep in plugin.get("requires", []):
            add(dep, f"{domain}/{name} 依赖")
        dependencies = set()
        pending = list(plugin.get("requires", []))
        while pending:
            dependency = pending.pop()
            if dependency not in dependencies:
                dependencies.add(dependency)
                pending.extend(catalog[dependency]["dependencies"])
        add("CONTROL_CHASSIS" if domain == "chassis" else "CONTROL_GIMBAL", f"controllers.{domain}={name}")
        motors = string_list(selection.get("motors", []), f"{domain}.motors")
        require(0 < len(motors) <= 8 and all(re.fullmatch(r"[A-Za-z0-9_.-]+", m) for m in motors),
                f"{domain}.motors 需要 1 到 8 个有效设备名")
        if "outputs" in plugin:
            require(len(motors) == plugin["outputs"], f"{name} 需要 {plugin['outputs']} 个电机绑定")
        supplied = selection.get("parameters", {})
        keys(supplied, plugin["parameters"], f"{domain}.parameters")
        values = {}
        for key, param in plugin["parameters"].items():
            require(key in supplied or "default" in param, f"{domain}/{name} 缺少参数 {key}")
            values[key] = number(supplied.get(key, param.get("default")), f"{domain}.{key}",
                                 param.get("min", -1e30), param.get("max", 1e30))
        period = number(selection.get("period_ms", 2), f"{domain}.period_ms", 1, 100, integer=True)
        resolved_controls.append(dict(domain=domain, type=name, builtin=False, motors=motors,
                                      parameters=values, period_ms=period, requires_imu="IMU" in dependencies,
                                      plugin=plugin))

    conflicts = [("CLASSIC_CHASSIS", "CONTROL_CHASSIS", "WHEELLEG_MIT", "WHEELLEG_SERVO"),
                 ("SINGLE_GIMBAL", "DUAL_YAW_GIMBAL", "CONTROL_GIMBAL")]
    for group in conflicts:
        chosen = set(group) & set(selected)
        require(len(chosen) <= 1, f"控制输出冲突: {', '.join(sorted(chosen))}")
    if any(c["domain"] == "shoot" and c["type"] == "rm" for c in resolved_controls):
        require(bool({"SINGLE_GIMBAL", "DUAL_YAW_GIMBAL"} & set(selected)),
                "shoot.rm 当前由 single/dual_yaw 云台任务调用；其他机构请关闭 shoot，不能只选择却没有执行入口")
    used_motors = set()
    for control in resolved_controls:
        for motor in control.get("motors", []):
            require(motor not in used_motors, f"不同控制域重复占用电机 {motor}")
            used_motors.add(motor)
    require(len(selected) <= 16, "任务数量超过现有固定容量 16；请减少服务或调整并验证容量")
    overrides = data.get("tasks", {})
    keys(overrides, selected, "tasks（只能覆盖已启用的任务）")
    board = BOARDS[target["board"]]
    task_rows = []
    for symbol in selected:
        row = dict(catalog[symbol])
        override = overrides.get(symbol, {})
        keys(override, {"stack_words", "priority"}, f"tasks.{symbol}")
        row["stack_words"] = number(override.get("stack_words", row["stack_m"] if board["kind"] == "STM32H7" else row["stack_other"]),
                                     f"{symbol}.stack_words", 128, 4096, integer=True)
        row["priority"] = override.get("priority", row["priority"])
        require(row["priority"] in PRIORITIES, f"{symbol}: 未知优先级")
        task_rows.append(row)
    features = data.get("features", {})
    keys(features, {"subboard_music", "imu_mount"}, "features")
    require(all(type(x) is bool for x in features.values()), "features 只能为 true/false")
    require(not features.get("subboard_music") or target["board"] == "dm_mc02_h7", "副板音乐需要 MC02 H7")
    if features.get("imu_mount"):
        inside(path.parent, "ImuMount.h", {".h"})
    build = data.get("build", {})
    keys(build, {"sources", "shared_sources", "port_sources", "overlay"}, "build")
    target_sources = [inside(path.parent, x, {".c", ".cpp"}).relative_to(root).as_posix()
                      for x in string_list(build.get("sources", []), "build.sources")]
    require(f"Robotconfig/{target['name']}/RobotConfig.c" in target_sources, "build.sources 必须包含 RobotConfig.c")
    for key in ("shared_sources", "port_sources"):
        for value in string_list(build.get(key, []), f"build.{key}"):
            require(value.startswith("shared/"), f"{key} 只能引用 shared 下的实现")
    shared_sources = [inside(root, x, {".c", ".cpp"}).relative_to(root).as_posix()
                      for x in build.get("shared_sources", [])]
    port_sources = [inside(root, x, {".c", ".cpp"}).relative_to(root).as_posix()
                    for x in build.get("port_sources", [])]
    for task in task_rows:
        source = task["source"]
        if not source:
            require(False, f"{task['symbol']} 尚未迁移实现源，不能选择")
        if source.startswith("@target/"):
            require(any(path.endswith("/" + source[8:]) for path in target_sources),
                    f"{task['symbol']} 需要车型实现 {source[8:]}")
            continue
        if source.startswith("@platform/"):
            require(any(path.endswith("/" + source[10:]) for path in port_sources),
                    f"{task['symbol']} 需要平台实现 {source[10:]}")
            continue
        shared_sources.append(inside(root, source, {".c", ".cpp"}).relative_to(root).as_posix())
    for control in resolved_controls:
        if not control["builtin"]:
            shared_sources.extend(control["plugin"]["source_paths"])
    if any(not c["builtin"] for c in resolved_controls):
        shared_sources.extend(["shared/application/robot/ControlRuntime.c", "shared/application/robot/ControlRuntimeTask.c"])
    for source in target_sources + shared_sources + port_sources:
        require(not FORBIDDEN_SOURCE.search(source), f"禁止引入旧平台实现 (forbidden legacy entry): {source}")
    overlay = str(inside(path.parent, build["overlay"])) if build.get("overlay") else ""
    target.update(profile=profile, tasks=task_rows, controllers=resolved_controls, reasons=reasons,
                  features=features, board_info=board, sources=sorted(set(target_sources + shared_sources)),
                  port_sources=port_sources, overlay=overlay, catalog=catalog)
    fingerprint = hashlib.sha256(path.read_bytes() + (root / CATALOG).read_bytes())
    for control in resolved_controls:
        if not control["builtin"]:
            fingerprint.update((root / control["plugin"]["manifest"]).read_bytes())
    target["fingerprint"] = fingerprint.hexdigest()
    return target


def write_changed(path, content):
    content = content.replace("\r\n", "\n")
    encoded = content.encode("utf-8")
    if not path.exists() or path.read_bytes() != encoded:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(encoded)


def c_string(value):
    return json.dumps(str(value), ensure_ascii=True)


def generate(root, target, out):
    board, catalog = target["board_info"], target["catalog"]
    selected = {row["symbol"]: row for row in target["tasks"]}
    header = ["/* 自动生成：修改 Robotconfig 中的 RobotConfig.toml，然后重新构建。 */", "#pragma once",
              f'#define ARBATOS_TARGET_NAME {c_string(target["name"])}',
              f'#define ARBATOS_BOARD_NAME {c_string(Path(board["directory"]).name)}',
              f'#define ROBOT_PROFILE_KIND ROBOT_PROFILE_KIND_{target["profile"].upper()}',
              f'#define ROBOT_BOARD_KIND ROBOT_BOARD_KIND_{board["kind"]}',
              f'#define ROBOT_BOARD_CPU_HZ {board["cpu_hz"]}u',
              f'#define ROBOT_BOARD_CAN_BUS_COUNT {board["can_buses"]}u',
              f'#define ROBOT_BOARD_RS485_PORT_COUNT {board["rs485_ports"]}u',
              f'#define ROBOT_BOARD_HAS_FPU {board["fpu"]}u',
              f'#define ROBOT_RUNTIME_DEFAULT_STACK_WORDS {board["stack"]}u',
              f'#define ROBOT_IMU_MOUNT_CONFIGURED {int(target["features"].get("imu_mount", False))}',
              '#define ROBOT_SUBBOARD_RTC_SERVICE ' + str(int(
                  "shared/zephyr/port/subboard/SubBoardBringupZephyr.c" in target["port_sources"])),
              f'#define ROBOT_CONFIG_FINGERPRINT "{target["fingerprint"]}"',
              "#define ROBOT_TASK_SELECTED_STACK(symbol, m, other) ROBOT_TASK_STACK_##symbol",
              "#define ROBOT_TASK_SELECTED_PRIORITY(symbol, fallback) ROBOT_TASK_PRIORITY_##symbol"]
    for symbol, task in catalog.items():
        stack = task["stack_m"] if board["kind"] == "STM32H7" else task["stack_other"]
        row = selected.get(symbol, task)
        header.extend([f"#define ROBOT_TASK_BUILD_{symbol} {int(symbol in selected)}",
                       f'#define ROBOT_TASK_STACK_{symbol} {row.get("stack_words", stack)}u',
                       f'#define ROBOT_TASK_PRIORITY_{symbol} RobotModulePriority{row["priority"]}'])
    shoot = any(c["domain"] == "shoot" for c in target["controllers"])
    header.append(f"#define ROBOT_TASK_BUILD_SHOOT_RM {int(shoot)}")
    for domain in ("chassis", "gimbal"):
        period = next((c["period_ms"] for c in target["controllers"]
                       if c["domain"] == domain and not c["builtin"]), 2)
        header.append(f"#define ROBOT_CONTROL_{domain.upper()}_PERIOD_MS {period}u")
        require_imu = any(c["domain"] == domain and c.get("requires_imu", False)
                          for c in target["controllers"])
        header.append(f"#define ROBOT_CONTROL_{domain.upper()}_REQUIRE_IMU {int(require_imu)}")
    watch = {"LOCOMOTION_CLASSIC": "CLASSIC_CHASSIS", "LOCOMOTION_WHEELLEG_SERVO": "WHEELLEG_SERVO",
             "LOCOMOTION_WHEELLEG_MIT": "WHEELLEG_MIT", "GIMBAL_DUAL": "DUAL_YAW_GIMBAL",
             "ARM_J0_UNITREE": "ARM", "CONTROL_CHASSIS": "CONTROL_CHASSIS", "CONTROL_GIMBAL": "CONTROL_GIMBAL"}
    for watch_name, module in watch.items():
        header.append(f"#define WATCH_ENABLE_{watch_name} {int(module in selected)}")
    header.extend([f'#define WATCH_ENABLE_GIMBAL_SINGLE {int(bool({"SINGLE_GIMBAL", "DUAL_YAW_GIMBAL"} & selected.keys()))}',
                   f"#define WATCH_ENABLE_SHOOT_RM {int(shoot)}"])
    write_changed(out / "RobotTargetConfig.h", "\n".join(header) + "\n")
    profile = ["/* 此运行任务表与编译开关来自同一份车型声明。 */", "    .profile = {",
               f'        .task_module_count = {len(selected)}u,', "        .task_modules = {"]
    profile += [f"            ROBOT_TASK_MODULE_{symbol}," for symbol in selected]
    profile += ["        },", "    },", ""]
    write_changed(out / "RobotTargetProfile.inc", "\n".join(profile))
    rows = sorted(selected.values(), key=lambda row: (row["order"], row["id"]))
    write_changed(out / "RobotTargetTaskHeaders.h", "#pragma once\n" + "".join(
        f'#include "{header}"\n' for header in dict.fromkeys(row["header"] for row in rows)))
    write_changed(out / "RobotTargetTasks.inc", "/* 供运行入口多次展开，不加 include guard。 */\n" + "".join(
        f'ROBOT_RUNTIME_TASK({r["symbol"]}, {r["entry"]}, robotTask{r["id"]}, osPriority{r["priority"]}, {r["stack_words"]}u)\n'
        for r in rows))
    variables = {"ARBATOS_TARGET_DIR": target["name"], "ARBATOS_BOARD_DIR": board["directory"],
                 "ARBATOS_BOARD": target["board"], "ARBATOS_TARGET_SOURCES": target["sources"],
                 "ARBATOS_TARGET_PORT_SOURCES": target["port_sources"], "ARBATOS_TARGET_OVERLAY": target["overlay"],
                 "ARBATOS_TARGET_GENERATED_DIR": out.as_posix()}
    cmake = ["# 自动生成；路径来自已校验的车型声明。"]
    for name, values in variables.items():
        if not isinstance(values, list):
            values = [values]
        require(all(not any(c in str(v) for c in ';"\n\r$') for v in values), "路径包含不支持的 CMake 字符")
        cmake.append(f'set({name} ' + " ".join('"' + str(v).replace("\\", "/") + '"' for v in values) + ")")
    write_changed(out / "RobotTarget.cmake", "\n".join(cmake) + "\n")
    conf = [f'CONFIG_ARBATOS_ROBOT_NAME="{target["name"]}"', "CONFIG_ARBATOS_LEGACY_SOURCES=y"]
    if target["board"] == "dm_mc02_h7":
        conf.append("CONFIG_CAN_FD_MODE=y")
    conf.append("CONFIG_ARBATOS_SUBBOARD_MUSIC=" + ("y" if target["features"].get("subboard_music") else "n"))
    write_changed(out / "robot.conf", "\n".join(conf) + "\n")
    report = {k: v for k, v in target.items() if k != "catalog"}
    write_changed(out / "robot-config.json", json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    write_control_selection(target, out)
    write_plugin_parameters(root, out)
    return report


def write_control_selection(target, out):
    # 接口定义由 ControlRuntime 提供，内置控制器仍走原有已验证的控制链。
    controls = [c for c in target["controllers"] if not c["builtin"]]
    lines = ["/* 自动生成：选定的可复用控制器及车型装配。 */", "#pragma once"]
    lines.append('#include "ControlRuntime.h"')
    if not controls:
        lines.extend(["#define ROBOT_CONTROL_SELECTION_COUNT 0u",
                      "static const ControlModuleSpec *const g_robot_control_modules = NULL;",
                      "static const uint8_t g_robot_control_module_count = 0u;"])
    else:
        for control in controls:
            lines.append(f'#include "{control["plugin"]["header"]}"')
        for i, control in enumerate(controls):
            lines.append(f"static const char *const robotControlMotors{i}[] = {{" +
                         ", ".join(c_string(m) for m in control["motors"]) + "};")
            if control["parameters"]:
                lines.append(f"static const ControlParam robotControlParams{i}[] = {{")
                for key, value in control["parameters"].items():
                    lines.append(f"    {{{c_string(key)}, {float(value)!r}f}},")
                lines.append("};")
        lines.append("static const ControlModuleSpec g_robot_control_modules[] = {")
        for i, control in enumerate(controls):
            params = f"robotControlParams{i}" if control["parameters"] else "NULL"
            lines.append("    {" + f'&{control["plugin"]["symbol"]}, robotControlMotors{i}, '
                         f'{len(control["motors"])}u, {params}, {len(control["parameters"])}u, '
                         f'{control["period_ms"]}u, {int(control["requires_imu"])}u' + "},")
        lines.extend(["};", f"static const uint8_t g_robot_control_module_count = {len(controls)}u;"])
        lines.append(f"#define ROBOT_CONTROL_SELECTION_COUNT {len(controls)}u")
    write_changed(out / "RobotControlSelection.h", "\n".join(lines) + "\n")


def write_plugin_parameters(root, out):
    for plugin in plugins(root).values():
        symbol = plugin["symbol"]
        lines = ["/* 参数默认值和范围只在 Controller.toml 维护。 */", "#pragma once",
                 '#include "ControlAlgorithm.h"']
        if plugin["parameters"]:
            lines.append("enum {")
            for i, key in enumerate(plugin["parameters"]):
                lines.append(f"    {symbol}Param_{key} = {i},")
            lines.append("};")
            lines.append(f"static const ControlParamDef {symbol}Params[] = {{")
            for key, param in plugin["parameters"].items():
                default = float(param.get("default", 0))
                low, high = float(param.get("min", -1e30)), float(param.get("max", 1e30))
                lines.append(f"    {{{c_string(key)}, {default!r}f, {low!r}f, {high!r}f}},")
            lines.append("};")
        else:
            lines.append(f"static const ControlParamDef *const {symbol}Params = NULL;")
        lines.append(f"#define {symbol}ParamCount {len(plugin['parameters'])}u")
        write_changed(out / f"{symbol}Params.h", "\n".join(lines) + "\n")


def create_robot(root, name, source):
    root = Path(root).resolve()
    require(TARGET_NAME.fullmatch(name) is not None and name.lower() != "all", "新车型名字无效")
    base = resolve(root, source)
    destination = root / "Robotconfig" / name
    require(not destination.exists(), f"目标已存在，不覆盖: {destination}")
    require(not any(t["name"].lower() == name.lower() for t in targets(root)), "车型名字大小写冲突")
    origin = root / "Robotconfig" / base["name"]
    require(not any(p.is_symlink() for p in origin.rglob("*")), "模板含符号链接，请先核对再复制")
    # 新项目先保留模板的装配，输出模式置为“未选中任何电机”。
    # 只改新副本，不改变模板；首次接线和方向验证由使用者显式启用。
    text = (origin / "ConfigOperation.inc").read_bytes().decode("utf-8")
    text, changed = re.subn(r"(\.mode\s*=\s*)ROBOT_RUN_MODE_\w+", r"\g<1>ROBOT_RUN_MODE_SINGLE_MOTOR", text, count=1)
    require(changed == 1, "模板缺少运行模式，请检查 ConfigOperation.inc；未创建新车型")
    text, changed = re.subn(r"(\.target_motor\s*=\s*)[^,]+", r"\g<1>(uint8_t)MotorCount", text, count=1)
    require(changed == 1, "模板缺少目标电机，不能保证新车默认停机；未创建新车型")
    text = re.sub(r"(\.variant\s*=\s*)ROBOT_RUN_VARIANT_\w+", r"\g<1>ROBOT_RUN_VARIANT_NORMAL", text, count=1)
    shutil.copytree(origin, destination)
    operation = destination / "ConfigOperation.inc"
    operation.write_bytes(text.encode("utf-8"))
    result = resolve(root, name)
    return dict(name=name, path=str(destination), based_on=base["name"],
                output="单电机模式且未选择电机；核对装配后在 ConfigOperation.inc 显式启用",
                board=result["board"])


def update_presets(root, requested):
    root = Path(root).resolve()
    path = root / "projects/CMakeUserPresets.json"
    data = json.loads(path.read_text(encoding="utf-8-sig")) if path.exists() else {"version": 3}
    configure = data.setdefault("configurePresets", [])
    builds = data.setdefault("buildPresets", [])
    existing = {p["name"] for p in configure}
    existing_builds = {p["name"] for p in builds}
    names = targets(root)
    if requested.lower() != "all":
        names = [t for t in names if t["name"].lower() == requested.lower()]
        require(names, f"车型不存在: {requested}")
    added = []
    for target in names:
        resolve(root, target["name"])
        name = target["preset"] + "-local"
        if name in existing:
            continue  # 已有的 CLion 本机配置由用户维护，不覆盖。
        preset = dict(name=name, inherits="zephyr-base", displayName=target["name"] + " / RobotConfig",
                      binaryDir="${sourceDir}/../local/build/" + target["preset"],
                      cacheVariables={"ARBATOS_ROBOT": target["name"]})
        venv = root / "local/cache/zephyrproject/.venv/Scripts"
        zephyr = root / "local/cache/zephyrproject/zephyr"
        sdk = root / "local/cache/zephyr-sdk"
        if (venv / "python.exe").exists() and zephyr.exists() and sdk.exists():
            preset["environment"] = {"ZEPHYR_BASE": zephyr.as_posix(), "ZEPHYR_SDK_INSTALL_DIR": sdk.as_posix(),
                                     "PATH": venv.as_posix() + ";$penv{PATH}"}
            preset["cacheVariables"].update(Python3_EXECUTABLE=(venv / "python.exe").as_posix(),
                                            CMAKE_MAKE_PROGRAM=(venv / "ninja.exe").as_posix())
            preset["cacheVariables"]["Zephyr-sdk_DIR"] = (sdk / "cmake").as_posix()
        configure.append(preset)
        if name not in existing_builds:
            builds.append(dict(name=name, configurePreset=name, jobs=2))
        added.append(name)
    if added:
        write_changed(path, json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    return dict(path=str(path), added=added, preserved=len(existing))


def main(argv=None):
    # Windows 重定向输出仍统一 UTF-8，供 PowerShell/CLion 和测试读取。
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["list", "generate", "check", "new", "presets"])
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--target", default="all")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--from", dest="source", default="HERO-M")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    try:
        if args.command == "list":
            result = targets(root)
        elif args.command == "new":
            result = create_robot(root, args.target, args.source)
        elif args.command == "presets":
            result = update_presets(root, args.target)
        elif args.command == "check":
            names = [t["name"] for t in targets(root)] if args.target.lower() == "all" else [args.target]
            result = [{k: v for k, v in resolve(root, name).items() if k != "catalog"} for name in names]
        else:
            require(args.out is not None, "generate 需要 --out 构建输出目录")
            result = generate(root, resolve(root, args.target), args.out.resolve())
        if args.json or args.command in {"list", "new", "presets"}:
            print(json.dumps(result, ensure_ascii=False))
        else:
            print(f"车型配置检查通过: {args.target}")
        return 0
    except (ConfigError, OSError, ValueError) as exc:
        print(f"车型配置错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
