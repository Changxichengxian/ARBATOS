import test from "node:test";
import assert from "node:assert/strict";
import {
    channelPercent, channelUses, fieldValue, functionChannel, rcFresh, semanticSwitch, switchPosition,
} from "../src/receiver.ts";

const hero = () => ({ target: "HERO-M", revision: "test", groups: [{ id: "input", label: "遥控映射", fields: [
    ...[1, 2, 3, 4, 7].map((value, index) => ({ id: `input.elrs.ElrsChMap.${index}`, label: `RC 轴 ${index + 1}`, value, type: "enum" })),
    ...[5, 6].map((value, index) => ({ id: `input.elrs.ElrsSwMap.${index}`, label: `RC 拨杆 ${index + 1}`, value, type: "enum" })),
    { id: "input.axis.INPUT_AXIS_CHASSIS_X.channel", label: "底盘前后 逻辑通道", value: 4, type: "enum" },
    { id: "input.axis.INPUT_AXIS_CHASSIS_Y.channel", label: "底盘左右 逻辑通道", value: 3, type: "enum" },
    { id: "input.axis.INPUT_AXIS_CHASSIS_WZ.channel", label: "底盘旋转 逻辑通道", value: 1, type: "enum" },
    { id: "input.axis.INPUT_AXIS_GIMBAL_YAW.channel", label: "云台 Yaw 逻辑通道", value: 1, type: "enum" },
    { id: "input.axis.INPUT_AXIS_GIMBAL_YAW.invert", label: "云台 Yaw 反向", value: false, type: "boolean" },
    { id: "input.axis.INPUT_AXIS_GIMBAL_PITCH.channel", label: "云台 Pitch 逻辑通道", value: 2, type: "enum" },
    { id: "input.sw.INPUT_SW_GIMBAL_MODE.channel", label: "云台模式 逻辑通道", value: 1, type: "enum" },
    { id: "input.sw.INPUT_SW_CHASSIS_MODE.channel", label: "底盘模式 逻辑通道", value: 1, type: "enum" },
    { id: "input.sw.INPUT_SW_SHOOT_MODE.channel", label: "发射模式 逻辑通道", value: 2, type: "enum" },
] }] });

test("HERO 映射保留重复用途、物理通道为 1 基", () => {
    const uses = channelUses(hero(), {});
    assert.deepEqual(uses[0], ["底盘旋转", "云台 水平"]);
    assert.deepEqual(uses[3], ["底盘前后"]);
    assert.deepEqual(uses[4], ["云台模式", "底盘模式"]);
    assert.deepEqual(uses[5], ["发射模式"]);
    assert.deepEqual(uses[6], ["逻辑轴5"]);
    assert.equal(functionChannel("input.axis.INPUT_AXIS_CHASSIS_X.channel", hero(), {}), 4);
    assert.equal(functionChannel("input.sw.INPUT_SW_SHOOT_MODE.channel", hero(), {}), 6);
});

test("草稿优先，并应用反向标记", () => {
    const data = hero();
    const changes = { "input.axis.INPUT_AXIS_GIMBAL_YAW.invert": true, "input.elrs.ElrsChMap.0": 16 };
    assert.equal(fieldValue(data.groups[0].fields[0], { "input.elrs.ElrsChMap.0": 9 }), 9);
    assert.deepEqual(channelUses(data, changes)[15], ["底盘旋转", "云台 水平·反向"]);
});

test("缺失或无效映射不产生用途", () => {
    assert.deepEqual(channelUses(null, {}), Array.from({ length: 16 }, () => []));
    assert.equal(functionChannel("input.axis.missing.channel", hero(), {}), null);
    assert.equal(functionChannel("other", hero(), {}), null);
});

test("开关和 CRSF 百分比边界", () => {
    assert.equal(switchPosition(582), "DOWN");
    assert.equal(switchPosition(583), "MID");
    assert.equal(switchPosition(1401), "UP");
    assert.equal(switchPosition(1401, true), "DOWN");
    assert.equal(channelPercent(172), 0);
    assert.equal(channelPercent(1811), 100);
    assert.equal(channelPercent(-1), 0);
    assert.equal(channelPercent(9999), 100);
    assert.equal(semanticSwitch("input.sw.INPUT_SW_GIMBAL_MODE.channel"), "INPUT_SW_GIMBAL_MODE");
    assert.equal(semanticSwitch("INPUT_SW_CHASSIS_MODE"), "INPUT_SW_CHASSIS_MODE");
    assert.equal(semanticSwitch("shoot-mode"), "INPUT_SW_SHOOT_MODE");
    assert.equal(semanticSwitch("INPUT_SW_CALIB_L"), null);
});

test("新鲜度同时要求完整有效帧、本机时间和 uint32 设备时间", () => {
    const snapshot = { channels: Array(16).fill(992), rc: { ms: 5, frameMs: 0xfffffff0, valid: true, sequence: 0, receivedAt: 10 } };
    assert.equal(rcFresh(snapshot, true, 10.6), true);
    assert.equal(rcFresh(snapshot, true, 10.60001), false);
    assert.equal(rcFresh({ ...snapshot, rc: { ...snapshot.rc, frameMs: 0xfffffff0 - 101 } }, true, 10.1), false);
    assert.equal(rcFresh({ ...snapshot, rc: { ...snapshot.rc, valid: false } }, true, 10.1), false);
    assert.equal(rcFresh({ ...snapshot, channels: Array(15).fill(0) }, true, 10.1), false);
    assert.equal(rcFresh(snapshot, false, 10.1), false);
});
