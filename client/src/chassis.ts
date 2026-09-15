import type { ConfigField, ConfigValues } from "./components/ConfigValuesPanel.vue";

export type WheelType = "xdrive" | "mecanum";
export type Motion = "forward" | "backward" | "left" | "right" | "ccw" | "cw";
export type MotorObservation = {
    name: string;
    configuredRouteId: number;
    configuredBus: number;
    configuredFeedbackId: number;
    observedFeedbackId: number;
    online: boolean;
    feedbackCount: number;
    diagnosticFeedbackAgeMs: number;
    rpm: number;
    commandCurrent?: number;
    receivedAt: number;
    fresh: boolean;
};
export type MotorSnapshot = {
    generation: number;
    available: boolean;
    source: string | null;
    motors: MotorObservation[];
    handTurnCandidateReady: boolean;
    safety?: { outputAllowed: boolean; receivedAt: number; fresh: boolean } | null;
};
export type MotorSerial = { connected: boolean; format?: string; motors?: MotorSnapshot; port?: string; error?: string };

export function chassisFields(data: ConfigValues | null) {
    return new Map((data?.groups ?? []).flatMap(group => group.fields).map(field => [field.id, field]));
}

export function chassisValue(field: ConfigField | undefined, changes: Record<string, unknown>) {
    return field && (Object.hasOwn(changes, field.id) ? changes[field.id] : field.value);
}

export function chassisType(data: ConfigValues | null, changes: Record<string, unknown>): WheelType | null {
    const value = chassisValue(chassisFields(data).get("tuning.chassis.wheel_type"), changes);
    if (value === "CHASSIS_WHEEL_TYPE_XDRIVE") return "xdrive";
    if (value === "CHASSIS_WHEEL_TYPE_MECANUM") return "mecanum";
    return null;
}

export function chassisWheels(type: WheelType) {
    // 轮位来自现有两种混控的槽位约定，不能把后两轮共用同一套标签。
    const indices = type === "xdrive" ? [0, 1, 3, 2] : [0, 1, 2, 3];
    return ["左前", "右前", "左后", "右后"].map((label, position) => ({
        index: indices[position], label, position,
        x: position % 2 === 0 ? 148 : 352,
        y: position < 2 ? 118 : 322,
        angle: type === "xdrive" ? (position === 0 || position === 3 ? -45 : 45) : 90,
    }));
}

export function chassisMix(type: WheelType, motion: Motion, directions: number[]) {
    const vectors: Record<Motion, number[]> = {
        forward: [1, 0, 0], backward: [-1, 0, 0], left: [0, 1, 0], right: [0, -1, 0], ccw: [0, 0, 1], cw: [0, 0, -1],
    };
    const [x, y, z] = vectors[motion];
    // 只展示混控目标符号；不是 PID 给定，也不向串口发送命令。
    const speeds = type === "xdrive" ? [x+y+z, -x+y+z, -x-y+z, x-y+z] : [x-y-z, x+y+z, x+y-z, x-y+z];
    return speeds.map((speed, i) => directions[i] === 1 || directions[i] === -1 ? Math.sign(speed * directions[i]) : null);
}

export function motorFeedback(data: ConfigValues, changes: Record<string, unknown>, index: number): number | null {
    const fields = chassisFields(data), prefix = `motor.chassis[${index}].`;
    const field = fields.get(prefix + "feedback_id");
    const model = data.models?.[String(chassisValue(fields.get(prefix + "model"), changes))];
    const id = Number(chassisValue(fields.get(prefix + "id"), changes));
    if (!model || !Number.isInteger(id) || id <= 0) return null;
    const value = chassisValue(field, changes);
    const automatic = Object.hasOwn(changes, prefix + "feedback_id") ? value === -1 : field?.automatic === true;
    const feedback = automatic ? model.can_id_base + id : Number(value);
    return Number.isInteger(feedback) && feedback >= 0 && feedback <= 2047 ? feedback : null;
}

export function motorRecent(motor: MotorObservation, now: number) {
    const age = now - motor.receivedAt;
    return motor.fresh && motor.online && age >= 0 && age <= 2.5 && motor.feedbackCount > 0 &&
        motor.diagnosticFeedbackAgeMs <= 100 && Number.isFinite(motor.rpm);
}

export function handTurnReady(serial: MotorSerial, paused: boolean, now: number) {
    const snapshot = serial.motors, safety = snapshot?.safety;
    return serial.connected && serial.format === "text" && !paused && snapshot?.available === true &&
        snapshot.handTurnCandidateReady === true && safety?.outputAllowed === false && safety.fresh &&
        now >= safety.receivedAt && now - safety.receivedAt <= 2.5 && snapshot.motors.length >= 4 &&
        snapshot.motors.every(m => motorRecent(m, now) && m.commandCurrent === 0);
}

export type HandTurnCapture = {
    generation: number; startedAt: number;
    routes: Record<string, { lastCount: number; lastTime: number; samples: number; movement: number; motor: MotorObservation }>;
};
const routeKey = (motor: MotorObservation) => `${motor.configuredBus}:${motor.observedFeedbackId}:${motor.configuredRouteId}`;

export function handTurnStart(snapshot: MotorSnapshot, now: number): HandTurnCapture {
    return { generation: snapshot.generation, startedAt: now, routes: Object.fromEntries(snapshot.motors.map(motor => [
        routeKey(motor), { lastCount: motor.feedbackCount, lastTime: motor.receivedAt, samples: 0, movement: 0, motor },
    ])) };
}

export function handTurnSample(capture: HandTurnCapture, snapshot: MotorSnapshot) {
    if (snapshot.generation !== capture.generation || snapshot.motors.length !== Object.keys(capture.routes).length) return false;
    for (const motor of snapshot.motors) {
        const entry = capture.routes[routeKey(motor)];
        if (!entry) return false;
        if (motor.receivedAt <= entry.lastTime) continue;
        const countAdvance = (motor.feedbackCount - entry.lastCount) >>> 0;
        if (countAdvance >= 0x80000000) return false;
        entry.lastTime = motor.receivedAt;
        entry.lastCount = motor.feedbackCount;
        entry.motor = motor;
        if (countAdvance === 0) continue;
        entry.samples++;
        if (Math.abs(motor.rpm) >= 5) entry.movement++;
    }
    return true;
}

export function handTurnResult(capture: HandTurnCapture): { candidate: MotorObservation | null; reason: string } {
    const routes = Object.values(capture.routes);
    if (routes.some(entry => entry.samples < 2)) return { candidate: null, reason: "反馈更新不足，请保持手转并重新采集。" };
    const moved = routes.filter(entry => entry.movement > 0);
    if (moved.length === 0) return { candidate: null, reason: "没有捕捉到明显转动，请转动整个主轮后重试。" };
    if (moved.length !== 1 || moved[0].movement < 2) return { candidate: null, reason: "响应不唯一或持续时间不足，不能确定轮位，请重试。" };
    return { candidate: moved[0].motor, reason: "已找到持续响应的反馈，请核对后加入草稿。" };
}

export function handTurnAssignment(data: ConfigValues, changes: Record<string, unknown>, index: number, motor: MotorObservation) {
    const fields = chassisFields(data), prefix = `motor.chassis[${index}].`;
    const modelName = String(chassisValue(fields.get(prefix + "model"), changes));
    const model = data.models?.[modelName];
    // 首版仅接收有明确反馈基址的 RM 型号，不把内部实例编号当成节点号。
    if (!model || !["MOTOR_MODEL_3508", "MOTOR_MODEL_3510", "MOTOR_MODEL_2006", "MOTOR_MODEL_6020", "MOTOR_MODEL_6623"].includes(modelName)) return null;
    const node = motor.observedFeedbackId - model.can_id_base;
    if (!Number.isInteger(node) || node <= 0 || node > (model.id_max ?? 0) || ![1,2].includes(motor.configuredBus)) return null;
    const result = { [prefix + "id"]: node, [prefix + "bus"]: `CAN${motor.configuredBus}`, [prefix + "feedback_id"]: -1 };
    if (Object.keys(result).some(id => !fields.has(id) || fields.get(id)?.editable === false)) return null;
    return result;
}
