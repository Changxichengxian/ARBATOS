"""受限的 IMU 安装方向读写；不解析或执行任意 C 源码。"""

from __future__ import annotations

import re
import tomllib


_AXIS = {"+X": (0, 1), "-X": (0, -1), "+Y": (1, 1), "-Y": (1, -1),
         "+Z": (2, 1), "-Z": (2, -1)}
_AXIS_BY_VALUE = {value: key for key, value in _AXIS.items()}
_ASSIGNMENT = re.compile(
    r"\bout\s*\[\s*([0-2])\s*\]\s*=\s*(-?)\s*raw\s*\[\s*([0-2])\s*\]\s*;"
)
_FUNCTION = re.compile(
    r"\bstatic\s+inline\s+void\s+ImuMountRotate\s*\(\s*float\s+out\s*\[\s*3\s*\]\s*,"
    r"\s*const\s+float\s+raw\s*\[\s*3\s*\]\s*\)\s*\{(?P<body>.*?)\}", re.DOTALL
)
_FRAME_VERSION = re.compile(r"(^\s*#\s*define\s+ROBOT_IMU_FRAME_VERSION\s+)\d+u\b", re.MULTILINE)
_COMMENTS = re.compile(r"/\*.*?\*/|//[^\r\n]*", re.DOTALL)


def _validate_axes(axes):
    if (not isinstance(axes, list) or len(axes) != 3 or
            any(not isinstance(axis, str) or axis not in _AXIS for axis in axes)):
        raise ValueError("axes 必须是三个 +X/-X/+Y/-Y/+Z/-Z")
    values = [_AXIS[axis] for axis in axes]
    if len({index for index, _sign in values}) != 3:
        raise ValueError("三个输出轴必须分别使用三个不同的原始轴")
    indexes = [index for index, _sign in values]
    inversions = sum(indexes[left] > indexes[right] for left in range(3) for right in range(left + 1, 3))
    determinant = (-1 if inversions % 2 else 1)
    for _index, sign in values:
        determinant *= sign
    if determinant != 1:
        raise ValueError("axes 必须是右手正交坐标映射")
    return values


def _mount_function(text):
    matches = list(_FUNCTION.finditer(text))
    if len(matches) != 1:
        raise ValueError("ImuMount.h 必须只含一个固定签名的 ImuMountRotate 函数")
    return matches[0]


def _parse_axes(text):
    function = _mount_function(text)
    body = function.group("body")
    found = _ASSIGNMENT.findall(body)
    if len(found) != 3:
        raise ValueError("ImuMount.h 必须含有三个固定的 out[i] = +/-raw[j] 赋值")
    rows = {}
    for out_index, minus, raw_index in found:
        index = int(out_index)
        if index in rows:
            raise ValueError("ImuMount.h 的输出轴重复")
        rows[index] = (int(raw_index), -1 if minus else 1)
    if set(rows) != {0, 1, 2}:
        raise ValueError("ImuMount.h 缺少输出轴")
    axes = [_AXIS_BY_VALUE[rows[index]] for index in range(3)]
    _validate_axes(axes)
    # 仅允许三条无条件赋值和注释，拒绝分支、局部变量、函数调用等任意附加 C 逻辑。
    remainder = _ASSIGNMENT.sub("", _COMMENTS.sub("", body))
    if remainder.strip():
        raise ValueError("ImuMountRotate 只能包含三条固定轴赋值和注释")
    return axes


def _configured(workspace, target):
    config = workspace._robot_toml(target)
    try:
        data = tomllib.loads(workspace._text_info(config.read_bytes())[0])
    except tomllib.TOMLDecodeError as exc:
        raise ValueError(f"车型配置无法解析: {exc}") from exc
    features = data.get("features", {})
    return isinstance(features, dict) and features.get("imu_mount") is True


def _path(workspace, target):
    path, target_name = workspace._file_path(target, "ImuMount.h")
    return path, target_name


def _hi14_active(workspace, target):
    resolved = workspace._robot_detail(target).get("resolved")
    if not isinstance(resolved, dict):
        raise ValueError("车型配置未通过检查，不能判断 IMU 姿态源")
    external = resolved["ports"]["assignments"].get("external_imu")
    return external is not None and external.get("protocol") == "hipnuc_hi14"


def get(workspace, target):
    path, target_name = _path(workspace, target)
    if _hi14_active(workspace, target_name):
        return {"target": target_name, "unsupported": True,
                "reason": "当前车型已启用外置 HI14；ImuMount.h 不影响该姿态源。",
                "configured": False, "requiresBuild": False}
    data = path.read_bytes()
    text = workspace._text_info(data)[0]
    return {"target": target_name, "axes": _parse_axes(text), "revision": workspace._sha(data),
            "configured": _configured(workspace, target_name), "requiresBuild": True}


def _replace_mount(text, axes):
    values = _validate_axes(axes)
    _parse_axes(text)
    version_matches = list(_FRAME_VERSION.finditer(text))
    if len(version_matches) != 1:
        raise ValueError("ImuMount.h 必须含有唯一的 ROBOT_IMU_FRAME_VERSION 定义")
    text = _FRAME_VERSION.sub(r"\g<1>3u", text, count=1)
    function = _mount_function(text)
    replacements = {str(index): f"out[{index}] = {'-' if sign < 0 else ''}raw[{raw_index}];"
                    for index, (raw_index, sign) in enumerate(values)}
    body = function.group("body")
    def replace_assignment(match):
        return replacements[match.group(1)]
    new_body, count = _ASSIGNMENT.subn(replace_assignment, body)
    if count != 3:
        raise ValueError("ImuMount.h 的轴赋值数目发生变化，请重新读取")
    return text[:function.start("body")] + new_body + text[function.end("body"):]


def update(workspace, target, axes, revision):
    _validate_axes(axes)
    if not isinstance(revision, str):
        raise ValueError("revision 必须是 ImuMount.h 的 SHA-256")
    with workspace._lock:
        path, target_name = _path(workspace, target)
        if _hi14_active(workspace, target_name):
            raise ValueError("当前车型已启用外置 HI14；ImuMount.h 不影响该姿态源，不能在此修改")
        if not _configured(workspace, target_name):
            raise ValueError("请先在车型配置中启用 features.imu_mount，再重新读取方向")
        original = path.read_bytes()
        if workspace._sha(original) != revision:
            raise ValueError("ImuMount.h 已被外部修改，请重新读取后再保存")
        old_text, bom, _newline = workspace._text_info(original)
        changed = _replace_mount(old_text, axes)
        workspace._write_atomic(path, (b"\xef\xbb\xbf" if bom else b"") + changed.encode("utf-8"))
    return get(workspace, target_name)
