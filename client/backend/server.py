"""本机客户端接口：页面不接触文件系统、串口对象或命令行。

同一后端同时服务桌面窗口和开发浏览器。只监听回环地址；随机会话凭据
通过父进程管道交给窗口，不写入工程配置，不接受网页跨站调用。
"""

from __future__ import annotations

import argparse
import hmac
import json
import mimetypes
import secrets
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

from jobs import Jobs
from serialio import SerialService
from workspace import Workspace


WORKSPACE_METHODS = {
    "workspace.summary", "robot.get", "robot.validate", "robot.update", "robot.create", "file.read", "file.write",
}
JOB_METHODS = {"job.start", "job.get", "job.cancel", "job.plan_flash"}
SERIAL_METHODS = {"serial.ports", "serial.open", "serial.close", "serial.status", "serial.read", "serial.send"}
WRITE_METHODS = {"robot.update", "robot.create", "file.write"}
MAX_BODY = 4 * 1024 * 1024


class Application:
    def __init__(self, root: Path):
        self.workspace = Workspace(root)
        self.jobs = Jobs(root)
        self.serial = SerialService()
        self.operations = threading.RLock()
        self.stopping = False

    def dispatch(self, method, params):
        if not isinstance(method, str) or not isinstance(params, dict):
            raise ValueError("请求需要方法名和参数对象")
        # 建立任务与文件修改共用的顺序，避免 UI 之外的并发请求绕过限制。
        with self.operations:
            if self.stopping:
                raise ValueError("客户端正在关闭")
            if method in WORKSPACE_METHODS:
                if method in WRITE_METHODS and self.jobs.is_busy():
                    raise ValueError("编译或烧录正在运行，结束后再修改工程文件")
                return self.workspace.dispatch(method, params)
            if method in JOB_METHODS:
                if method == "job.start" and params.get("action") == "flash":
                    if self.serial.dispatch("serial.status", {}).get("connected"):
                        raise ValueError("请先断开串口，再烧录固件")
                return self.jobs.dispatch(method, params)
            if method in SERIAL_METHODS:
                if method == "serial.open" and self.jobs.is_busy():
                    raise ValueError("请等待当前编译或烧录结束后再连接串口")
                return self.serial.dispatch(method, params)
        raise ValueError("不支持的客户端操作")

    def shutdown(self):
        with self.operations:
            self.stopping = True
            try:
                self.serial.shutdown()
            finally:
                self.jobs.shutdown()


class LocalServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, application, frontend: Path, port=0, token=None, dev_origin=None):
        self.application = application
        self.frontend = frontend.resolve()
        self.token = token or secrets.token_urlsafe(32)
        super().__init__(("127.0.0.1", port), Handler)
        self.origin = f"http://127.0.0.1:{self.server_port}"
        self.allowed_origins = {self.origin}
        if dev_origin:
            parsed = urlsplit(dev_origin)
            if parsed.scheme != "http" or parsed.hostname != "127.0.0.1" or not parsed.port or parsed.path or parsed.query or parsed.fragment:
                self.server_close()
                raise ValueError("开发页面地址必须是带端口的 http://127.0.0.1 地址")
            self.allowed_origins.add(dev_origin)


class Handler(BaseHTTPRequestHandler):
    server: LocalServer

    def handle(self):
        try:
            super().handle()
        except (ConnectionError, TimeoutError):
            # 刷新或关闭窗口时，对端可能在响应写完前断开。
            self.close_connection = True

    def setup(self):
        super().setup()
        self.connection.settimeout(15)

    def log_message(self, format, *args):
        # 请求 URL 和凭据不进入共享终端日志。
        pass

    def base_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")

    def json_response(self, status, value):
        payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.base_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def failure(self, status, code, message):
        self.json_response(status, {"ok": False, "error": {"code": code, "message": message}})

    def valid_host(self):
        return self.headers.get("Host") == f"127.0.0.1:{self.server.server_port}"

    def do_POST(self):
        if not self.valid_host():
            self.failure(403, "HOST_DENIED", "本机地址不匹配")
            return
        if self.path != "/api/rpc":
            self.failure(404, "NOT_FOUND", "接口不存在")
            return
        origin = self.headers.get("Origin")
        if origin and origin not in self.server.allowed_origins:
            self.failure(403, "ORIGIN_DENIED", "只允许当前客户端访问")
            return
        expected = f"Bearer {self.server.token}"
        if not hmac.compare_digest(self.headers.get("Authorization", "").encode("utf-8"), expected.encode("utf-8")):
            self.failure(401, "SESSION_REQUIRED", "会话已失效，请重新打开客户端")
            return
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
            self.failure(415, "CONTENT_TYPE", "请求必须使用 JSON")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= MAX_BODY:
                self.failure(413, "TOO_LARGE", "请求内容超过允许大小")
                return
            request = json.loads(self.rfile.read(length), parse_constant=lambda _: (_ for _ in ()).throw(ValueError("不能使用非有限数值")))
            if not isinstance(request, dict) or set(request) - {"method", "params"}:
                raise ValueError("请求格式不正确")
            result = self.server.application.dispatch(request.get("method"), request.get("params", {}))
            self.json_response(200, {"ok": True, "result": result})
        except ConnectionError:
            self.close_connection = True
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.failure(400, "OPERATION_REJECTED", str(exc))
        except Exception as exc:
            print(f"客户端接口异常: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
            self.failure(500, "INTERNAL_ERROR", "本机服务执行失败，请查看启动日志")

    def do_GET(self):
        if not self.valid_host():
            self.failure(403, "HOST_DENIED", "本机地址不匹配")
            return
        path = unquote(urlsplit(self.path).path)
        relative = "index.html" if path == "/" else path.lstrip("/")
        candidate = (self.server.frontend / relative).resolve()
        if not candidate.is_relative_to(self.server.frontend) or not candidate.is_file():
            self.failure(404, "NOT_FOUND", "页面不存在，请先构建客户端")
            return
        if candidate.suffix.lower() not in {".html", ".js", ".css", ".svg", ".png", ".ico", ".woff", ".woff2", ".map"}:
            self.failure(404, "NOT_FOUND", "文件类型不可访问")
            return
        content = candidate.read_bytes()
        mime = {".js": "text/javascript", ".css": "text/css", ".svg": "image/svg+xml"}.get(candidate.suffix)
        self.send_response(200)
        self.base_headers()
        self.send_header("Content-Type", mime or mimetypes.guess_type(candidate.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_OPTIONS(self):
        self.failure(403, "ORIGIN_DENIED", "不接受跨站访问")


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="ARBATOS 客户端本机服务")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--frontend", type=Path, default=Path(__file__).resolve().parents[1] / "dist")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--parent-pipe", action="store_true")
    parser.add_argument("--dev-origin", help="仅开发时允许的本机 Vite 页面地址")
    parser.add_argument("--session-file", type=Path, help="开发检查使用；包含临时凭据，不提交")
    args = parser.parse_args()
    app = Application(args.workspace.resolve())
    server = LocalServer(app, args.frontend, args.port, dev_origin=args.dev_origin)
    session = {"url": server.origin, "token": server.token}
    if args.session_file:
        args.session_file.parent.mkdir(parents=True, exist_ok=True)
        args.session_file.write_text(json.dumps(session), encoding="utf-8")
        print(f"本机服务已启动: {server.origin}", flush=True)
    else:
        print(json.dumps(session), flush=True)
    if args.parent_pipe:
        def parent_closed():
            try:
                sys.stdin.buffer.read()
            finally:
                server.shutdown()
        threading.Thread(target=parent_closed, daemon=True).start()
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()
        server.server_close()
        if args.session_file:
            args.session_file.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
