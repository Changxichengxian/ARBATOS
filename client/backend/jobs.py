"""桌面客户端的构建、检查和烧录任务。

外部参数只允许选择配置生成器给出的车型和固定动作。命令行由本模块组装，
不会接收任意命令、脚本路径或构建目录。
"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import queue
import re
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable

import yaml


_ACTIONS = {"check", "build", "rebuild", "flash"}
_DEDICATED_MODES = (
    "CONFIG_ARBATOS_MUSIC_ONLY=y",
    "CONFIG_ARBATOS_PREFLIGHT_ONLY=y",
    "CONFIG_ARBATOS_RECEIVE_ONLY=y",
)
_MACRO = re.compile(r'^#define\s+([A-Z0-9_]+)\s+"(.*)"$', re.MULTILINE)
_OPENOCD_ARGS = ("--cmd-load", "flash write_image erase", "--cmd-verify", "verify_image")


class JobError(ValueError):
    """可直接显示给客户端用户的任务参数或安全校验错误。"""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping", node.start_mark, "映射键必须可比较", key_node.start_mark
            ) from exc
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping", node.start_mark, f"重复键：{key}", key_node.start_mark
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


Runner = Callable[[list[str], Path, Callable[[str], None], threading.Event], int]


class Jobs:
    def __init__(
        self,
        root: str | Path,
        runner: Runner | None = None,
        *,
        max_lines: int = 2000,
        history_limit: int = 20,
    ) -> None:
        self.root = Path(root).resolve()
        self._build_script = self.root / "tools" / "build.ps1"
        if not self._build_script.is_file():
            raise JobError(f"找不到构建脚本：{self._build_script}")
        self._generator = self._load_generator()
        self._runner = runner or self._run_process
        self._max_lines = max(20, int(max_lines))
        self._history_limit = max(1, int(history_limit))
        self._jobs: dict[str, dict[str, Any]] = {}
        self._threads: dict[str, threading.Thread] = {}
        self._cancel_events: dict[str, threading.Event] = {}
        self._lock = threading.RLock()
        self._active_id: str | None = None
        self._closing = False

    def dispatch(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        params = params or {}
        if method == "job.start":
            return self._start(params)
        if method == "job.get":
            return self._get(params)
        if method == "job.cancel":
            return self._cancel(params)
        if method == "job.plan_flash":
            target = self._target(params.get("target"))
            return self._plan_flash(target)
        raise JobError(f"不支持的任务方法：{method}")

    def shutdown(self) -> None:
        with self._lock:
            self._closing = True
            events = list(self._cancel_events.values())
            threads = list(self._threads.values())
        for event in events:
            event.set()
        for thread in threads:
            if thread is not threading.current_thread():
                thread.join(timeout=10.0)
        with self._lock:
            alive = [thread.name for thread in self._threads.values() if thread.is_alive()]
        if alive:
            raise RuntimeError(f"任务未能及时结束：{', '.join(alive)}")

    def is_busy(self) -> bool:
        """后台任务从登记到线程完成期间都视为占用。"""
        with self._lock:
            if self._active_id is None:
                return False
            job = self._jobs.get(self._active_id)
            thread = self._threads.get(self._active_id)
            return bool(job and job["status"] == "running" and thread and thread.is_alive())

    def _load_generator(self) -> Any:
        generator = self.root / "tools" / "config" / "RobotConfigGen.py"
        spec = importlib.util.spec_from_file_location("arbatos_client_robot_config", generator)
        if spec is None or spec.loader is None:
            raise JobError("无法加载车型配置生成器")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _target(self, value: Any) -> str:
        if not isinstance(value, str):
            raise JobError("车型无效，请选择 Robotconfig 下的实际车型")
        try:
            entry = self._target_entry(value)
        except Exception as exc:
            raise JobError(f"车型无效：{exc}") from exc
        return str(entry["name"])

    def _target_entry(self, value: str) -> dict[str, Any]:
        entry = dict(self._generator.target_identity(self.root, value))
        config_root = self.root / "Robotconfig"
        target_dir = config_root / str(entry["name"])
        manifest = target_dir / "RobotConfig.toml"
        for path in (config_root, target_dir, manifest):
            if path.is_symlink() or (hasattr(path, "is_junction") and path.is_junction()):
                raise JobError(f"车型路径不能是链接或联接点：{path}")
        resolved_root = config_root.resolve(strict=True)
        resolved_manifest = manifest.resolve(strict=True)
        if not resolved_manifest.is_relative_to(resolved_root) or resolved_manifest.parent != target_dir.resolve(strict=True):
            raise JobError("车型配置超出 Robotconfig 目录")
        entry["path"] = manifest.relative_to(self.root).as_posix()
        return entry

    def _start(self, params: dict[str, Any]) -> dict[str, Any]:
        target = self._target(params.get("target"))
        action = params.get("action")
        if action not in _ACTIONS:
            raise JobError("动作无效，只能选择 check、build、rebuild 或 flash")

        flash_revision: str | None = None
        if action == "flash":
            confirmation = params.get("confirmation")
            if not isinstance(confirmation, dict):
                raise JobError("烧录前需要确认当前固件")
            plan = self._plan_flash(target)
            if not plan["ready"]:
                raise JobError(str(plan.get("reason") or "当前固件不能烧录"))
            if confirmation.get("target") != target:
                raise JobError("烧录确认的车型与当前选择不一致")
            if confirmation.get("artifactRevision") != plan["artifactRevision"]:
                raise JobError("固件已变化，请重新检查并确认后再烧录")
            flash_revision = str(plan["artifactRevision"])

        with self._lock:
            if self._closing:
                raise JobError("客户端正在关闭，不能启动新任务")
            if self._active_id is not None:
                active = self._jobs.get(self._active_id)
                if active and active["status"] == "running":
                    raise JobError("已有任务正在运行，请等待结束或先取消")
                self._active_id = None

            job_id = uuid.uuid4().hex
            now = time.time()
            job = {
                "id": job_id,
                "target": target,
                "action": action,
                "status": "running",
                "exitCode": None,
                "startedAt": now,
                "finishedAt": None,
                "lines": deque(maxlen=self._max_lines),
                "lastSeq": 0,
            }
            cancel_event = threading.Event()
            thread = threading.Thread(
                target=self._worker,
                args=(job_id, flash_revision),
                name=f"arbatos-job-{job_id[:8]}",
                daemon=False,
            )
            self._jobs[job_id] = job
            self._cancel_events[job_id] = cancel_event
            self._threads[job_id] = thread
            self._active_id = job_id
            thread.start()
            return self._snapshot(job)

    def _get(self, params: dict[str, Any]) -> dict[str, Any]:
        job_id = params.get("id")
        if not isinstance(job_id, str):
            raise JobError("缺少任务编号")
        after = params.get("after", 0)
        if isinstance(after, bool) or not isinstance(after, int) or after < 0:
            raise JobError("after 必须是非负整数")
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None:
                raise JobError("找不到这个任务")
            return self._snapshot(job, after)

    def _cancel(self, params: dict[str, Any]) -> dict[str, Any]:
        job_id = params.get("id")
        if not isinstance(job_id, str):
            raise JobError("缺少任务编号")
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None:
                raise JobError("找不到这个任务")
            event = self._cancel_events.get(job_id)
            if event is not None and job["status"] == "running":
                event.set()
            return self._snapshot(job)

    def _snapshot(self, job: dict[str, Any], after: int = 0) -> dict[str, Any]:
        lines = [dict(item) for item in job["lines"] if item["seq"] > after]
        return {
            "id": job["id"],
            "target": job["target"],
            "action": job["action"],
            "status": job["status"],
            "exitCode": job["exitCode"],
            "startedAt": job["startedAt"],
            "finishedAt": job["finishedAt"],
            "lines": lines,
            "lastSeq": job["lastSeq"],
        }

    def _append_line(self, job_id: str, text: str) -> None:
        text = str(text).rstrip("\r\n")
        if not text:
            return
        with self._lock:
            job = self._jobs[job_id]
            job["lastSeq"] += 1
            job["lines"].append({"seq": job["lastSeq"], "text": text[:8192]})

    def _command(self, target: str, action: str) -> list[str]:
        command = [
            "pwsh", "-NoLogo", "-NoProfile", "-File", str(self._build_script),
            "-Action", "build" if action == "rebuild" else action,
            "-Project", target,
        ]
        if action in {"build", "rebuild"}:
            command.extend(["-Jobs", "2"])
        if action == "rebuild":
            command.append("-Pristine")
        return command

    def _worker(self, job_id: str, flash_revision: str | None) -> None:
        with self._lock:
            job = self._jobs[job_id]
            event = self._cancel_events[job_id]
            target, action = job["target"], job["action"]
        exit_code = 1
        try:
            if action == "flash":
                with self._hold_flash_files(target):
                    current = self._plan_flash(target)
                    if not current["ready"] or current.get("artifactRevision") != flash_revision:
                        raise JobError("固件在任务启动后发生变化，已取消烧录")
                    exit_code = int(self._runner(self._command(target, action), self.root,
                                                 lambda line: self._append_line(job_id, line), event))
            else:
                exit_code = int(self._runner(self._command(target, action), self.root,
                                             lambda line: self._append_line(job_id, line), event))
        except Exception as exc:
            self._append_line(job_id, f"任务失败：{exc}")
            exit_code = 1
        with self._lock:
            if event.is_set():
                job["status"] = "cancelled"
                job["exitCode"] = None
            elif exit_code == 0:
                job["status"] = "succeeded"
                job["exitCode"] = 0
            else:
                job["status"] = "failed"
                job["exitCode"] = exit_code
            job["finishedAt"] = time.time()
            if self._active_id == job_id:
                self._active_id = None
            self._threads.pop(job_id, None)
            self._cancel_events.pop(job_id, None)
            while len(self._jobs) > self._history_limit:
                oldest = next(iter(self._jobs))
                if oldest == self._active_id:
                    break
                self._jobs.pop(oldest, None)

    def _plan_flash(self, target: str) -> dict[str, Any]:
        try:
            entry = self._target_entry(target)
        except Exception as exc:
            raise JobError(f"车型无效：{exc}") from exc
        build_dir = self.root / "local" / "build" / str(entry["preset"])
        zephyr_dir = build_dir / "zephyr"
        image = zephyr_dir / "zephyr.hex"
        required = [
            build_dir / "CMakeCache.txt",
            zephyr_dir / "runners.yaml",
            zephyr_dir / ".config",
            zephyr_dir / "zephyr.elf",
            image,
            zephyr_dir / "zephyr.bin",
            build_dir / "generated" / "build_info_autogen.h",
        ]
        base: dict[str, Any] = {
            "target": target,
            "artifactRevision": None,
            "path": str(image),
            "sha": None,
            "warnings": [],
            "ready": False,
        }
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            base["reason"] = "构建产物不完整，请先构建：" + "；".join(missing)
            return base
        if (build_dir / "domains.yaml").exists():
            base["reason"] = "检测到本项目不使用的 domains.yaml，拒绝按多镜像方式烧录"
            return base
        try:
            allowed_root = (self.root / "local" / "build").resolve()
            for path in [self.root / "local" / "build", build_dir, zephyr_dir, *required]:
                if path.is_symlink() or (hasattr(path, "is_junction") and path.is_junction()):
                    base["reason"] = f"构建产物路径包含链接或联接点，拒绝烧录：{path}"
                    return base
                if not path.resolve().is_relative_to(allowed_root):
                    base["reason"] = f"构建产物不在仓库 local/build 内，拒绝烧录：{path}"
                    return base
            config = (zephyr_dir / ".config").read_text(encoding="utf-8", errors="replace")
            if f'CONFIG_ARBATOS_ROBOT_NAME="{target}"' not in config.splitlines():
                base["reason"] = "构建产物的车型与当前选择不一致"
                return base
            enabled_mode = next((mode[:-2] for mode in _DEDICATED_MODES if mode in config.splitlines()), None)
            if enabled_mode:
                base["reason"] = f"构建产物启用了专用模式 {enabled_mode}，不能作为正常固件烧录"
                return base

            runners = (zephyr_dir / "runners.yaml").read_text(encoding="utf-8", errors="strict")
            runner_error = self._validate_runners(entry, runners)
            if runner_error:
                base["reason"] = runner_error
                return base

            header = (build_dir / "generated" / "build_info_autogen.h").read_text(encoding="ascii")
            values = dict(_MACRO.findall(header))
            built_state = values.get("ARBATOS_SOURCE_STATE")
            if not built_state or built_state == "unknown":
                base["reason"] = "构建产物没有可验证的源码状态"
                return base
            state_bytes = built_state.encode("ascii")
            if not self._file_contains(zephyr_dir / "zephyr.elf", state_bytes):
                base["reason"] = "ELF 内的构建版本与版本头不一致，请重新完整构建"
                return base
            if not self._file_contains(zephyr_dir / "zephyr.bin", state_bytes):
                base["reason"] = "BIN 内的构建版本与版本头不一致，请重新完整构建"
                return base
            if not self._intel_hex_contains(image, state_bytes):
                base["reason"] = "HEX 内的构建版本与版本头不一致，请重新完整构建"
                return base
            build_inputs = [
                build_dir / "generated" / "build_info_autogen.h",
                zephyr_dir / ".config",
                zephyr_dir / "runners.yaml",
            ]
            completed_images = [zephyr_dir / "zephyr.elf", image, zephyr_dir / "zephyr.bin"]
            newest_input = max(path.stat().st_mtime_ns for path in build_inputs)
            if any(path.stat().st_mtime_ns < newest_input for path in completed_images):
                base["reason"] = "构建配置或版本头晚于固件产物，上次构建可能未完成，请重新完整构建"
                return base
            current_state = self._current_source_state()
            if current_state == "unknown":
                base["reason"] = "当前源码状态无法验证"
                return base
            if built_state != current_state:
                base["reason"] = "源码在这份固件构建后已经变化，请重新构建"
                return base

            image_sha = self._sha256(image)
            covered = [
                zephyr_dir / "zephyr.elf",
                zephyr_dir / "zephyr.hex",
                zephyr_dir / "zephyr.bin",
                zephyr_dir / "runners.yaml",
                zephyr_dir / ".config",
                build_dir / "generated" / "build_info_autogen.h",
            ]
            token = hashlib.sha256()
            token.update(target.encode("utf-8") + b"\0" + built_state.encode("utf-8") + b"\0")
            for path in covered:
                token.update(path.name.encode("utf-8") + b"\0")
                token.update(bytes.fromhex(self._sha256(path)))
            base["sha"] = image_sha
            base["artifactRevision"] = token.hexdigest()
            date, clock = values.get("ARBATOS_BUILD_DATE"), values.get("ARBATOS_BUILD_TIME")
            if date and clock and date != "unknown" and clock != "unknown":
                base["buildTime"] = f"{date} {clock}"
            base["ready"] = True
            return base
        except (OSError, UnicodeError, ValueError) as exc:
            base["reason"] = f"检查构建产物失败：{exc}"
            return base

    def _validate_runners(self, entry: dict[str, Any], content: str) -> str | None:
        """只接受 Zephyr 正式构建当前生成的 OpenOCD 烧录关键配置。"""
        try:
            parsed = self._parse_runners(content)
        except ValueError as exc:
            return f"烧录配置格式不安全：{exc}"
        if parsed["flash-runner"] != "openocd" or "openocd" not in parsed["runners"]:
            return "烧录配置的默认下载工具不是正式 OpenOCD 配置"
        for key, expected in (("elf_file", "zephyr.elf"), ("hex_file", "zephyr.hex"),
                              ("bin_file", "zephyr.bin")):
            if parsed["config"].get(key) != expected:
                return f"烧录配置 {key} 已偏离正式产物，拒绝烧录"

        try:
            target_config = self._generator.read_toml(self.root / str(entry["path"]))
            board = str(target_config["board"])
            board_directory = str(self._generator.BOARDS[board]["directory"])
            expected_board_dir = (self.root / board_directory / "zephyr").resolve()
            configured = parsed["config"].get("board_dir")
            if configured is None:
                return "烧录配置缺少开发板目录"
            configured_board_dir = Path(configured).resolve()
        except Exception as exc:
            return f"无法确认烧录配置对应的正式开发板：{exc}"
        if configured_board_dir != expected_board_dir or not expected_board_dir.is_dir():
            return "烧录配置的开发板目录与车型声明不一致"

        sdk = self.root / "local" / "cache" / "zephyr-sdk"
        expected_tools = {
            "gdb": sdk / "gnu" / "arm-zephyr-eabi" / "bin" / "arm-zephyr-eabi-gdb.exe",
            "openocd": sdk / "hosttools" / "openocd" / "bin" / "openocd.exe",
        }
        expected_search = sdk / "hosttools" / "openocd" / "share" / "openocd" / "scripts"
        for key, expected in expected_tools.items():
            configured = parsed["config"].get(key)
            if not isinstance(configured, str) or Path(configured).resolve() != expected.resolve():
                return f"烧录配置的 {key} 工具路径已偏离仓库 SDK"
            if not expected.is_file() or self._has_reparse(expected, sdk):
                return f"仓库 SDK 的 {key} 工具路径无效或包含链接"
        search = parsed["config"].get("openocd_search")
        if not isinstance(search, list) or len(search) != 1 or not isinstance(search[0], str):
            return "烧录配置的 OpenOCD 搜索目录不唯一"
        if Path(search[0]).resolve() != expected_search.resolve() or not expected_search.is_dir():
            return "烧录配置的 OpenOCD 搜索目录已偏离仓库 SDK"
        if self._has_reparse(expected_search, sdk):
            return "仓库 SDK 的 OpenOCD 搜索目录包含链接"

        if tuple(parsed["args"].get("openocd", [])) != _OPENOCD_ARGS:
            return "OpenOCD 烧录参数已偏离正式配置，拒绝烧录"
        return None

    @staticmethod
    def _has_reparse(path: Path, stop: Path) -> bool:
        current = path
        boundary = stop.resolve()
        while True:
            if current.is_symlink() or (hasattr(current, "is_junction") and current.is_junction()):
                return True
            if current.resolve() == boundary:
                return False
            if current.parent == current:
                return True
            current = current.parent

    @staticmethod
    def _parse_runners(content: str) -> dict[str, Any]:
        """按 West 使用的 YAML 语义解析，并拒绝所有层级的重复键。"""
        try:
            result = yaml.load(content, Loader=_UniqueKeyLoader)
        except yaml.YAMLError as exc:
            raise ValueError(str(exc)) from exc
        if not isinstance(result, dict):
            raise ValueError("顶层必须是映射")
        required = {"runners", "flash-runner", "debug-runner", "config", "args"}
        if set(result) != required:
            raise ValueError("顶层字段与正式 runners.yaml 不一致")
        if not isinstance(result["runners"], list) or not all(isinstance(item, str) for item in result["runners"]):
            raise ValueError("runners 必须是字符串列表")
        if len(result["runners"]) != len(set(result["runners"])):
            raise ValueError("runners 有重复项")
        if not isinstance(result["config"], dict) or not isinstance(result["args"], dict):
            raise ValueError("config 和 args 必须是映射")
        allowed_config = {"board_dir", "elf_file", "hex_file", "bin_file", "gdb",
                          "openocd", "openocd_search"}
        if not set(result["config"]).issubset(allowed_config):
            raise ValueError("config 包含非正式字段")
        if not all(isinstance(key, str) and isinstance(value, list)
                   and all(isinstance(item, str) for item in value)
                   for key, value in result["args"].items()):
            raise ValueError("args 必须按下载工具保存字符串列表")
        return result

    def _flash_files(self, target: str) -> list[Path]:
        entry = self._target_entry(target)
        build = self.root / "local" / "build" / str(entry["preset"])
        return [
            build / "CMakeCache.txt",
            build / "zephyr" / "runners.yaml",
            build / "zephyr" / ".config",
            build / "zephyr" / "zephyr.elf",
            build / "zephyr" / "zephyr.hex",
            build / "zephyr" / "zephyr.bin",
            build / "generated" / "build_info_autogen.h",
        ]

    @contextmanager
    def _hold_flash_files(self, target: str):
        """确认后保持只读共享，禁止 West 完成前替换或改写产物。"""
        paths = self._flash_files(target)
        if os.name != "nt":
            streams = [path.open("rb") for path in paths]
            try:
                yield
            finally:
                for stream in reversed(streams):
                    stream.close()
            return

        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        create_file = kernel32.CreateFileW
        create_file.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
                                wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
        create_file.restype = wintypes.HANDLE
        close_handle = kernel32.CloseHandle
        close_handle.argtypes = [wintypes.HANDLE]
        close_handle.restype = wintypes.BOOL
        handles: list[int] = []
        invalid = ctypes.c_void_p(-1).value
        try:
            for path in paths:
                handle = create_file(str(path), 0x80000000, 0x00000001, None, 3, 0x80, None)
                if handle == invalid:
                    raise JobError(f"无法锁定待烧录产物：{path}：{ctypes.WinError(ctypes.get_last_error())}")
                handles.append(handle)
            yield
        finally:
            for handle in reversed(handles):
                close_handle(handle)

    def _current_source_state(self) -> str:
        generator = self.root / "tools" / "build" / "GenBuildInfo.py"
        spec = importlib.util.spec_from_file_location("arbatos_client_build_info", generator)
        if spec is None or spec.loader is None:
            return "unknown"
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
            _sha, _dirty, _submodules, state = module.build_values(self.root)
            return str(state)
        except Exception:
            return "unknown"

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def _file_contains(path: Path, needle: bytes) -> bool:
        overlap = max(0, len(needle) - 1)
        previous = b""
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                data = previous + chunk
                if needle in data:
                    return True
                previous = data[-overlap:] if overlap else b""
        return False

    @staticmethod
    def _intel_hex_contains(path: Path, needle: bytes) -> bool:
        payload = bytearray()
        saw_eof = False
        for number, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
            line = raw.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"HEX 第 {number} 行格式无效")
            try:
                record = bytes.fromhex(line[1:])
            except ValueError as exc:
                raise ValueError(f"HEX 第 {number} 行不是有效十六进制") from exc
            if len(record) < 5 or len(record) != record[0] + 5 or sum(record) & 0xFF:
                raise ValueError(f"HEX 第 {number} 行长度或校验和无效")
            record_type = record[3]
            if record_type == 0:
                payload.extend(record[4:-1])
            elif record_type == 1:
                saw_eof = True
                break
            elif record_type not in {2, 3, 4, 5}:
                raise ValueError(f"HEX 第 {number} 行记录类型不支持")
        if not saw_eof:
            raise ValueError("HEX 缺少结束记录")
        return needle in payload

    @staticmethod
    def _run_process(command: list[str], cwd: Path, emit: Callable[[str], None], cancel: threading.Event) -> int:
        flags = 0
        if os.name == "nt":
            flags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
        environment = os.environ.copy()
        for name in ("ZEPHYR_BASE", "ZEPHYR_SDK_INSTALL_DIR", "Zephyr_sdk_DIR"):
            environment.pop(name, None)
        local_zephyr = cwd / "local" / "cache" / "zephyrproject" / "zephyr"
        local_sdk = cwd / "local" / "cache" / "zephyr-sdk"
        if local_zephyr.is_dir():
            environment["ZEPHYR_BASE"] = str(local_zephyr)
        if local_sdk.is_dir():
            environment["ZEPHYR_SDK_INSTALL_DIR"] = str(local_sdk)
            environment["Zephyr_sdk_DIR"] = str(local_sdk / "cmake")
        process = subprocess.Popen(
            command,
            cwd=str(cwd),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=environment,
            creationflags=flags,
            start_new_session=os.name != "nt",
        )
        output: queue.Queue[str | None] = queue.Queue(maxsize=256)

        def read_output() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                while True:
                    try:
                        output.put(line, timeout=0.1)
                        break
                    except queue.Full:
                        if cancel.is_set() and process.poll() is not None:
                            break
            while True:
                try:
                    output.put(None, timeout=0.1)
                    break
                except queue.Full:
                    continue

        reader = threading.Thread(target=read_output, name="arbatos-job-output", daemon=True)
        reader.start()
        ended = False
        while process.poll() is None or not ended:
            if cancel.is_set() and process.poll() is None:
                Jobs._terminate_tree(process)
            try:
                item = output.get(timeout=0.05)
            except queue.Empty:
                continue
            if item is None:
                ended = True
            else:
                emit(item)
        reader.join(timeout=1.0)
        return int(process.wait())

    @staticmethod
    def _terminate_tree(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        else:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
