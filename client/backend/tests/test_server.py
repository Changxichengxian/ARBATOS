import http.client
import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import Application, LocalServer


class FakeApplication:
    def __init__(self):
        self.calls = []

    def dispatch(self, method, params):
        self.calls.append((method, params))
        if method == "reject":
            raise ValueError("参数不在允许范围内")
        return {"名称": "测试车型", "params": params}


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.frontend = Path(self.temp.name) / "dist"
        self.frontend.mkdir()
        (self.frontend / "index.html").write_text("<title>客户端</title>", encoding="utf-8")
        (Path(self.temp.name) / "private.txt").write_text("private", encoding="utf-8")
        self.application = FakeApplication()
        self.server = LocalServer(self.application, self.frontend, token="test-session")
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01})
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()
        self.temp.cleanup()

    def request(self, method="POST", path="/api/rpc", headers=None, data=None, send_payload=True):
        defaults = {"Authorization": "Bearer test-session", "Content-Type": "application/json"}
        defaults.update(headers or {})
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        body = json.dumps(data or {"method": "workspace.summary", "params": {}}, ensure_ascii=False).encode("utf-8")
        if not send_payload:
            body = b""
        connection.request(method, path, body=body if method == "POST" else None, headers=defaults)
        response = connection.getresponse()
        payload = response.read()
        result = response.status, payload, dict(response.getheaders())
        connection.close()
        return result

    def test_authorized_rpc_returns_utf8(self):
        status, data, _ = self.request(headers={"Origin": self.server.origin})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(data)["result"]["名称"], "测试车型")
        self.assertEqual(len(self.application.calls), 1)

    def test_invalid_token_host_and_origin_never_dispatch(self):
        for headers in ({"Authorization": ""}, {"Origin": "https://evil.example"}, {"Host": "evil.example"}):
            with self.subTest(headers=headers):
                self.assertIn(self.request(headers=headers, send_payload=False)[0], {401, 403})
        self.assertEqual(self.application.calls, [])

    def test_non_json_and_oversized_requests_never_dispatch(self):
        # 仅发请求头，验证拒绝发生在读取正文之前；避免 WinSock 对未读正文关连接的复位竞态。
        self.assertEqual(self.request(headers={"Content-Type": "text/plain"}, send_payload=False)[0], 415)
        self.assertEqual(self.request(headers={"Content-Length": "999999999"}, send_payload=False)[0], 413)
        self.assertEqual(self.application.calls, [])

    def test_static_pages_have_policy_and_cannot_escape(self):
        status, _, headers = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn("frame-ancestors 'none'", headers["Content-Security-Policy"])
        for path in ("/../private.txt", "/%2e%2e/private.txt", "/api/rpc"):
            self.assertEqual(self.request("GET", path)[0], 404)

    def test_validation_failure_stays_actionable(self):
        status, data, _ = self.request(data={"method": "reject", "params": {}})
        self.assertEqual(status, 400)
        self.assertIn("参数", json.loads(data)["error"]["message"])


class CoordinationTest(unittest.TestCase):
    def setUp(self):
        self.app = Application.__new__(Application)
        self.app.operations = threading.RLock()
        self.app.stopping = False
        self.app.workspace = FakeApplication()
        self.app.jobs = type("Jobs", (), {"is_busy": lambda _: True})()
        self.app.serial = type("Serial", (), {"dispatch": lambda *_: {"connected": True}})()

    def test_busy_job_blocks_config_writes_and_port_open(self):
        for method in ("robot.update", "file.write", "robot.create", "serial.open"):
            with self.subTest(method=method), self.assertRaisesRegex(ValueError, "结束"):
                self.app.dispatch(method, {})
        self.assertEqual(self.app.workspace.calls, [])

    def test_connected_port_blocks_flash(self):
        with self.assertRaisesRegex(ValueError, "断开串口"):
            self.app.dispatch("job.start", {"action": "flash"})

    def test_unknown_method_does_not_expose_objects(self):
        with self.assertRaises(ValueError):
            self.app.dispatch("workspace.__dict__", {})

    def test_shutdown_blocks_new_requests_even_when_serial_cleanup_fails(self):
        stopped = []
        def fail_serial(_):
            raise OSError("模拟串口关闭失败")
        self.app.serial = type("Serial", (), {"shutdown": fail_serial})()
        self.app.jobs = type("Jobs", (), {"shutdown": lambda _: stopped.append(True)})()
        with self.assertRaises(OSError):
            self.app.shutdown()
        self.assertEqual(stopped, [True])
        with self.assertRaisesRegex(ValueError, "正在关闭"):
            self.app.dispatch("workspace.summary", {})


if __name__ == "__main__":
    unittest.main()
