import test from "node:test";
import assert from "node:assert/strict";
import { chassisType, chassisWheels, chassisMix, motorFeedback, handTurnReady,
    handTurnStart, handTurnSample, handTurnResult, handTurnAssignment } from "../src/chassis.ts";

function configuration() {
    const motor = (index, id) => [
        { id: `motor.chassis[${index}].model`, value: "MOTOR_MODEL_3508" },
        { id: `motor.chassis[${index}].id`, value: id },
        { id: `motor.chassis[${index}].bus`, value: "CAN1" },
        { id: `motor.chassis[${index}].feedback_id`, value: 512 + id, automatic: true },
    ].map(field => ({ ...field, label: field.id, type: "number", editable: true }));
    return { target: "INFANTRY-TIAN-A", revision: "fixture", groups: [{ id: "test", label: "test", fields: [
        { id: "tuning.chassis.wheel_type", value: "CHASSIS_WHEEL_TYPE_XDRIVE" },
        ...[3,2,4,1].flatMap((id, index) => motor(index,id)),
    ] }], models: { MOTOR_MODEL_3508: { can_id_base: 512, id_max: 8 }, MOTOR_MODEL_6020: { can_id_base: 516, id_max: 4 } } };
}
function snapshot(time = 100) {
    return { generation: 1, available: true, source: "infantry_diag_motor", handTurnCandidateReady: true,
        safety: { outputAllowed: false, receivedAt: time, fresh: true }, motors: [1,2,3,4].map(id => ({
            name: `chassis${id}`, configuredRouteId: id + 10, configuredBus: 1, configuredFeedbackId: 512 + id,
            observedFeedbackId: 512 + id, online: true, feedbackCount: 100, diagnosticFeedbackAgeMs: 1,
            rpm: 0, commandCurrent: 0, receivedAt: time, fresh: true,
        })) };
}
function observationFrame(base, time, moved = [2]) {
    const result = structuredClone(base);
    result.safety.receivedAt = time;
    result.motors.forEach((motor,index) => { motor.receivedAt = time; motor.feedbackCount += (time-100)*1000; motor.rpm = moved.includes(index) ? 80 : 0; });
    return result;
}

test("全向轮沿圆周切线，后轮槽位与麦轮分别对应", () => {
    const x = chassisWheels("xdrive"), mecanum = chassisWheels("mecanum");
    assert.deepEqual(x.map(w=>w.index), [0,1,3,2]);
    assert.deepEqual(mecanum.map(w=>w.index), [0,1,2,3]);
    for (const wheel of x) {
        const a = wheel.angle*Math.PI/180;
        assert.ok(Math.abs((wheel.x-250)*Math.cos(a)+(wheel.y-220)*Math.sin(a)) < 1e-9);
    }
    assert.equal(chassisType(configuration(), { "tuning.chassis.wheel_type": "CHASSIS_WHEEL_TYPE_MECANUM" }), "mecanum");
    assert.equal(chassisType(configuration(), { "tuning.chassis.wheel_type": "CUSTOM" }), null);
});

test("方向预览遵循两套混控，安装反向只作用于对应轮", () => {
    assert.deepEqual(chassisMix("xdrive","forward",[1,1,1,1]), [1,-1,-1,1]);
    assert.deepEqual(chassisMix("xdrive","left",[1,1,1,1]), [1,1,-1,-1]);
    assert.deepEqual(chassisMix("xdrive","ccw",[1,1,1,1]), [1,1,1,1]);
    assert.deepEqual(chassisMix("mecanum","left",[1,1,1,1]), [-1,1,1,-1]);
    assert.deepEqual(chassisMix("mecanum","ccw",[1,1,1,1]), [-1,1,-1,1]);
    assert.deepEqual(chassisMix("xdrive","forward",[-1,1,1,0]), [-1,-1,-1,null]);
});

test("草稿型号与节点自动更新反馈预览，显式地址保留", () => {
    const data = configuration();
    assert.equal(motorFeedback(data, { "motor.chassis[0].id": 5 }, 0), 0x205);
    assert.equal(motorFeedback(data, { "motor.chassis[0].id": 1, "motor.chassis[0].model": "MOTOR_MODEL_6020" }, 0), 0x205);
    const field = data.groups[0].fields.find(f=>f.id==="motor.chassis[0].feedback_id");
    field.automatic = false; field.value = 0x321;
    assert.equal(motorFeedback(data, { "motor.chassis[0].id": 5 }, 0), 0x321);
    assert.equal(motorFeedback(data, { "motor.chassis[0].id": 5, "motor.chassis[0].feedback_id": -1 }, 0), 0x205);
    assert.equal(motorFeedback(data, { "motor.chassis[0].id": 0 }, 0), null);
});

test("手转采集要求新鲜停机反馈，暂停、断连、输出变化均拒绝", () => {
    const base = { connected:true,format:"text",motors:snapshot() };
    assert.equal(handTurnReady(base,false,100.5),true);
    assert.equal(handTurnReady(base,true,100.5),false);
    assert.equal(handTurnReady({...base,connected:false},false,100.5),false);
    assert.equal(handTurnReady(base,false,103),false);
    for (const mutate of [s=>s.safety.outputAllowed=true,s=>s.safety=null,s=>s.motors[0].commandCurrent=30,s=>s.motors[0].diagnosticFeedbackAgeMs=200,s=>s.motors[0].online=false]) {
        const data=structuredClone(base);mutate(data.motors);
        assert.equal(handTurnReady(data,false,100.5),false);
    }
});

test("重复轮询不当新采样，单轮持续响应产生候选，多轮响应拒绝", () => {
    const base=snapshot(), capture=handTurnStart(base,100);
    const one=observationFrame(base,101);
    assert.equal(handTurnSample(capture,one),true);
    assert.equal(handTurnSample(capture,one),true);
    assert.equal(handTurnResult(capture).candidate,null);
    handTurnSample(capture,observationFrame(base,102));
    assert.equal(handTurnResult(capture).candidate.observedFeedbackId,0x203);
    const ambiguous=handTurnStart(base,100);
    handTurnSample(ambiguous,observationFrame(base,101,[0,2]));
    handTurnSample(ambiguous,observationFrame(base,102,[0,2]));
    assert.equal(handTurnResult(ambiguous).candidate,null);
});

test("反馈计数不前进、会话变化或计数回退都不接受旧响应", () => {
    const base=snapshot();
    const capture=handTurnStart(base,100);
    const stale=observationFrame(base,101);stale.motors.forEach(m=>m.feedbackCount=100);
    handTurnSample(capture,stale);
    assert.equal(handTurnResult(capture).candidate,null);
    assert.equal(handTurnSample(capture,{...base,generation:2}),false);
    const reset=observationFrame(base,102);reset.motors[0].feedbackCount=0;
    assert.equal(handTurnSample(capture,reset),false);
});

test("绑定由实际反馈推算节点，不使用固件内部实例编号", () => {
    const data=configuration(), motor=snapshot().motors[2];
    motor.configuredRouteId=97;
    assert.deepEqual(handTurnAssignment(data,{},0,motor),{
        "motor.chassis[0].id":3,"motor.chassis[0].bus":"CAN1","motor.chassis[0].feedback_id":-1,
    });
    assert.equal(handTurnAssignment(data,{"motor.chassis[0].model":"MOTOR_MODEL_6020"},0,motor),null);
    assert.equal(handTurnAssignment(data,{"motor.chassis[0].model":"CUSTOM"},0,motor),null);
});
