"""桌面客户端的本地工作区接口；所有路径都限制在 Robotconfig 内。"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import subprocess
import sys
import tempfile
import threading
import tomllib
from pathlib import Path

import tomlkit


REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("arbatos_robot_config_gen", REPO / "tools/config/RobotConfigGen.py")
GEN = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(GEN)
_config_spec = importlib.util.spec_from_file_location("arbatos_configedit", Path(__file__).with_name("configedit.py"))
CONFIGEDIT = importlib.util.module_from_spec(_config_spec)
assert _config_spec.loader is not None
sys.modules[_config_spec.name] = CONFIGEDIT
_config_spec.loader.exec_module(CONFIGEDIT)

TEXT_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".inc", ".toml", ".md", ".overlay"}
MAX_TEXT_BYTES = 1024 * 1024


class Workspace:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.robot_root = self.root / "Robotconfig"
        if not self.robot_root.is_dir() or self._is_reparse(self.robot_root):
            raise ValueError("工作区缺少 Robotconfig 目录")
        self._robot_root_real = self.robot_root.resolve(strict=True)
        try:
            self._robot_root_real.relative_to(self.root)
        except ValueError as exc:
            raise ValueError("Robotconfig 目录不能指向工作区外") from exc
        self._lock = threading.RLock()
        self._config_editor = CONFIGEDIT.ConfigEditor(self)

    def dispatch(self, method, params):
        if not isinstance(params, dict):
            raise ValueError("请求参数必须是对象")
        handlers = {
            "workspace.summary": self.summary,
            "robot.get": self.robot_get,
            "robot.update": self.robot_update,
            "robot.create": self.robot_create,
            "robot.validate": self.robot_validate,
            "config.get": self.config_get,
            "config.update": self.config_update,
            "ports.get": self.ports_get,
            "file.read": self.file_read,
            "file.write": self.file_write,
        }
        if method not in handlers:
            raise ValueError(f"未知工作区方法: {method}")
        try:
            return handlers[method](**params)
        except GEN.ConfigError as exc:
            raise ValueError(str(exc)) from exc
        except OSError as exc:
            raise ValueError(f"工作区文件操作失败: {exc}") from exc

    @staticmethod
    def _sha(data):
        return hashlib.sha256(data).hexdigest()

    @staticmethod
    def _is_reparse(path):
        return path.is_symlink() or getattr(path, "is_junction", lambda: False)()

    @staticmethod
    def _text_info(data):
        if len(data) > MAX_TEXT_BYTES:
            raise ValueError("文件超过 1MiB，不能由客户端编辑")
        bom = data.startswith(b"\xef\xbb\xbf")
        raw = data[3:] if bom else data
        if b"\0" in raw:
            raise ValueError("文件含有 NUL 字符，不能由客户端编辑")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("文件不是 UTF-8 文本，不能由客户端编辑") from exc
        return text, bom, "CRLF" if b"\r\n" in raw else "LF"

    def _target_entries(self):
        entries = []
        for directory in sorted(self.robot_root.iterdir()):
            if not directory.is_dir() or self._is_reparse(directory):
                continue
            try:
                real = directory.resolve(strict=True)
                real.relative_to(self._robot_root_real)
            except (OSError, ValueError):
                continue
            if not GEN.TARGET_NAME.fullmatch(directory.name):
                continue
            config = directory / "RobotConfig.toml"
            if not config.is_file() or self._is_reparse(config):
                continue
            board = None
            try:
                text, _bom, _newline = self._text_info(config.read_bytes())
                data = tomllib.loads(text)
                board = data.get("board") if isinstance(data.get("board"), str) else None
            except (ValueError, OSError, tomllib.TOMLDecodeError):
                pass
            entries.append({"name": directory.name, "preset": directory.name.lower(), "board": board})
        return entries

    def _target_dir(self, target):
        if not isinstance(target, str) or not target:
            raise ValueError("车型名字不能为空")
        found = [item for item in self._target_entries() if item["name"].lower() == target.lower()]
        if len(found) != 1:
            raise ValueError(f"找不到车型: {target}")
        directory = (self.root / "Robotconfig" / found[0]["name"])
        if self._is_reparse(directory) or not directory.is_dir():
            raise ValueError("车型目录不能是符号链接")
        try:
            directory.resolve(strict=True).relative_to(self._robot_root_real)
        except (OSError, ValueError) as exc:
            raise ValueError("车型目录不能指向 Robotconfig 外") from exc
        return directory, found[0]["name"]

    def _file_path(self, target, name):
        directory, target_name = self._target_dir(target)
        if not isinstance(name, str) or not name:
            raise ValueError("文件名不能为空")
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("文件路径超出车型目录")
        path = directory / relative
        try:
            path.relative_to(directory)
        except ValueError as exc:
            raise ValueError("文件路径超出车型目录") from exc
        parent = path
        while parent != directory:
            if self._is_reparse(parent):
                raise ValueError("文件路径不能经过符号链接")
            parent = parent.parent
        if self._is_reparse(path) or not path.is_file():
            raise ValueError("只能读取车型目录内现有的普通文件")
        try:
            path.resolve(strict=True).relative_to(directory.resolve(strict=True))
        except (OSError, ValueError) as exc:
            raise ValueError("文件路径超出车型目录") from exc
        if path.suffix.lower() not in TEXT_SUFFIXES:
            raise ValueError("该文件类型不允许由客户端编辑")
        return path, target_name

    def _robot_toml(self, target):
        return self._file_path(target, "RobotConfig.toml")[0]

    def _file_detail(self, path, name=None):
        data = path.read_bytes()
        text, _bom, newline = self._text_info(data)
        return {"name": name or path.name, "content": text, "revision": self._sha(data),
                "encoding": "UTF-8-BOM" if data.startswith(b"\xef\xbb\xbf") else "UTF-8", "newline": newline}

    @staticmethod
    def _clean_resolved(result):
        return {key: value for key, value in result.items() if key != "catalog"}

    def _robot_detail(self, target):
        directory, target_name = self._target_dir(target)
        path = directory / "RobotConfig.toml"
        if self._is_reparse(path):
            raise ValueError("RobotConfig.toml 不能是符号链接")
        data = path.read_bytes()
        text, _bom, _newline = self._text_info(data)
        try:
            config = tomllib.loads(text)
            parse_error = None
        except tomllib.TOMLDecodeError as exc:
            config = None
            parse_error = str(exc)
        files = []
        for base, child_dirs, names in os.walk(directory, followlinks=False):
            base_path = Path(base)
            child_dirs[:] = [name for name in child_dirs if not self._is_reparse(base_path / name)]
            for name in names:
                item = base_path / name
                if not self._is_reparse(item) and item.suffix.lower() in TEXT_SUFFIXES:
                    files.append({"name": item.relative_to(directory).as_posix()})
        files.sort(key=lambda item: item["name"])
        try:
            if parse_error:
                raise GEN.ConfigError(f"车型配置无法解析: {parse_error}")
            resolved = self._clean_resolved(GEN.resolve(self.root, target_name))
            validation = {"ok": True, "errors": []}
        except GEN.ConfigError as exc:
            resolved = None
            validation = {"ok": False, "errors": [str(exc)]}
        return {"name": target_name, "config": config, "revision": self._sha(data), "files": files,
                "resolved": resolved, "validation": validation}

    def summary(self):
        catalog = GEN.task_catalog(self.root)
        controllers = []
        for domain, names in GEN.BUILTINS.items():
            for name in names:
                controllers.append({"name": name, "domain": domain, "builtin": True,
                                    "parameters": {}, "requires": []})
        for (domain, name), spec in GEN.plugins(self.root).items():
            parameters = {key: {field: value for field, value in param.items()
                                if field in {"default", "min", "max", "unit", "description"}}
                          for key, param in spec["parameters"].items()}
            row = {"name": name, "domain": domain, "builtin": False, "parameters": parameters,
                   "requires": list(spec.get("requires", []))}
            if "outputs" in spec:
                row["outputs"] = spec["outputs"]
            controllers.append(row)
        services = [{"symbol": item["symbol"], "name": item["task"],
                     "dependencies": list(item["dependencies"]),
                     "available": item["entry"] != "None" and bool(item["header"]) and bool(item["source"])}
                    for item in catalog.values() if item["kind"] != "Control" or item["symbol"] == "SERVO"]
        return {"root": str(self.root), "targets": self._target_entries(),
                "controllers": sorted(controllers, key=lambda row: (row["domain"], row["name"])),
                "services": services, "git": self._git_info()}

    def _git_info(self):
        def read(*args):
            return subprocess.run(["git", *args], cwd=self.root, capture_output=True, text=True,
                                  stdin=subprocess.DEVNULL,
                                  encoding="utf-8", errors="replace", check=True, timeout=3,
                                  creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout.strip()
        try:
            return {"branch": read("rev-parse", "--abbrev-ref", "HEAD"), "sha": read("rev-parse", "HEAD"),
                    "dirty": bool(read("status", "--porcelain"))}
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return {"branch": None, "sha": None, "dirty": None}

    def robot_get(self, target):
        return self._robot_detail(target)

    def config_get(self, target):
        return self._config_editor.public_get(target)

    def config_update(self, target, revision, changes):
        return self._config_editor.update(target, revision, changes)

    def ports_get(self, target, config=None):
        config_text = tomlkit.dumps(config) if isinstance(config, dict) else None
        if config is not None and not isinstance(config, dict):
            raise ValueError("车型配置必须是对象")
        if not hasattr(GEN, "port_options"):
            raise ValueError("当前配置生成器尚未提供端口选项")
        try:
            return GEN.port_options(self.root, target, config_text=config_text)
        except TypeError:
            if config_text is not None:
                raise
            return GEN.port_options(self.root, target)

    @staticmethod
    def _apply_toml(existing, incoming):
        for key in list(existing):
            if key not in incoming:
                del existing[key]
        for key, value in incoming.items():
            current = existing.get(key)
            # 不重建未修改的值，保留多行数组中的逐项说明和排版。
            if key in existing and current == value:
                continue
            if isinstance(value, dict) and current is not None and hasattr(current, "items"):
                Workspace._apply_toml(current, value)
            else:
                existing[key] = value

    def _write_atomic(self, path, data):
        with tempfile.NamedTemporaryFile(mode="wb", prefix=".workspace-", dir=path.parent, delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(data)
        try:
            os.replace(temporary, path)
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise

    def _check_revision(self, path, revision):
        if self._sha(path.read_bytes()) != revision:
            raise ValueError("文件已被外部修改，请重新读取后再保存")

    def robot_update(self, target, revision, config):
        if not isinstance(config, dict):
            raise ValueError("车型配置必须是对象")
        with self._lock:
            path = self._robot_toml(target)
            original = path.read_bytes()
            if revision != self._sha(original):
                raise ValueError("车型配置已被外部修改，请重新读取后再保存")
            text, bom, newline = self._text_info(original)
            try:
                document = tomlkit.parse(text)
            except tomlkit.exceptions.ParseError as exc:
                raise ValueError(f"车型配置无法解析: {exc}") from exc
            self._apply_toml(document, config)
            candidate = tomlkit.dumps(document)
            if newline == "CRLF":
                candidate = candidate.replace("\r\n", "\n").replace("\n", "\r\n")
            candidate_with_bom = ("\ufeff" if bom else "") + candidate
            GEN.resolve(self.root, target, config_text=candidate_with_bom)
            self._check_revision(path, revision)
            self._write_atomic(path, (b"\xef\xbb\xbf" if bom else b"") + candidate.encode("utf-8"))
            return self._robot_detail(target)

    def file_read(self, target, name):
        path, _target_name = self._file_path(target, name)
        return self._file_detail(path, Path(name).as_posix())

    def file_write(self, target, name, content, revision):
        if not isinstance(content, str):
            raise ValueError("文件内容必须是文本")
        if "\0" in content or len(content.encode("utf-8")) > MAX_TEXT_BYTES:
            raise ValueError("文件内容含有 NUL 字符或超过 1MiB")
        with self._lock:
            path, target_name = self._file_path(target, name)
            original = path.read_bytes()
            if revision != self._sha(original):
                raise ValueError("文件已被外部修改，请重新读取后再保存")
            _old, bom, newline = self._text_info(original)
            content = content.removeprefix("\ufeff").replace("\r\n", "\n").replace("\r", "\n")
            if newline == "CRLF":
                content = content.replace("\n", "\r\n")
            data = (b"\xef\xbb\xbf" if bom else b"") + content.encode("utf-8")
            if path.name == "RobotConfig.toml":
                GEN.resolve(self.root, target_name, config_text=("\ufeff" if bom else "") + content)
            self._check_revision(path, revision)
            self._write_atomic(path, data)
            return self._file_detail(path, Path(name).as_posix())

    def robot_create(self, name, source):
        with self._lock:
            result = GEN.create_robot(self.root, name, source)
            result["detail"] = self._robot_detail(name)
            return result

    def robot_validate(self, target, config, revision=None):
        if not isinstance(config, dict):
            raise ValueError("车型配置必须是对象")
        path = self._robot_toml(target)
        if revision is not None and revision != self._sha(path.read_bytes()):
            return {"ok": False, "errors": ["车型配置已被外部修改，请重新读取后再校验"]}
        try:
            document = tomlkit.document()
            self._apply_toml(document, config)
            GEN.resolve(self.root, target, config_text=tomlkit.dumps(document))
            return {"ok": True, "errors": []}
        except (GEN.ConfigError, ValueError, TypeError) as exc:
            return {"ok": False, "errors": [str(exc)]}
