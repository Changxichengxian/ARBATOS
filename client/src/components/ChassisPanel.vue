<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from "vue";
import type { ConfigField, ConfigValues } from "./ConfigValuesPanel.vue";
import { chassisFields, chassisMix, chassisType, chassisValue, chassisWheels, motorFeedback, motorRecent,
    handTurnAssignment, handTurnReady, handTurnResult, handTurnSample, handTurnStart,
    type HandTurnCapture, type Motion, type MotorObservation, type MotorSerial } from "../chassis";

const props = defineProps<{
    data: ConfigValues | null;
    changes: Record<string, unknown>;
    serial: MotorSerial;
    paused: boolean;
    disabled: boolean;
    ports: { device: string; description: string }[];
    port: string;
}>();
const emit = defineEmits<{
    change: [id: string, value: unknown];
    assign: [changes: Record<string, unknown>];
    port: [value: string]; toggle: []; refresh: []; resume: [];
}>();
const selected = ref(0);
const motion = ref<Motion>("forward");
const confirmed = ref(false);
const capture = ref<HandTurnCapture | null>(null);
const candidate = ref<MotorObservation | null>(null);
const candidateGeneration = ref<number | null>(null);
const candidateCreatedAt = ref(0);
const captureMessage = ref("");
const now = ref(Date.now() / 1000);
const fields = computed(() => chassisFields(props.data));
const type = computed(() => chassisType(props.data, props.changes));
const supported = computed(() => props.data?.chassis?.supported === true && type.value !== null);
const wheels = computed(() => chassisWheels(type.value ?? "xdrive"));
const selectedWheel = computed(() => wheels.value.find(wheel => wheel.index === selected.value)!);
const prefix = computed(() => `motor.chassis[${selected.value}].`);
const wheelTypeField = computed(() => fields.value.get("tuning.chassis.wheel_type"));
const directionField = computed(() => fields.value.get(`tuning.chassis.motor_dir[${selected.value}]`));
const editorFields = computed(() => ["model", "bus", "id", "external_reduction_ratio"].map(member => fields.value.get(prefix.value + member)).filter((field): field is ConfigField => !!field));
const feedbackField = computed(() => fields.value.get(prefix.value + "feedback_id"));
const model = computed(() => props.data?.models?.[String(value(fields.value.get(prefix.value + "model")))]);
const feedback = computed(() => props.data ? motorFeedback(props.data, props.changes, selected.value) : null);
const feedbackAutomatic = computed(() => feedbackField.value && (Object.hasOwn(props.changes, feedbackField.value.id)
    ? props.changes[feedbackField.value.id] === -1 : feedbackField.value.automatic === true));
const signs = computed(() => chassisMix(type.value ?? "xdrive", motion.value,
    [0,1,2,3].map(index => Number(value(fields.value.get(`tuning.chassis.motor_dir[${index}]`))))));
const ready = computed(() => handTurnReady(props.serial, props.paused, Math.max(now.value, Date.now()/1000)));
const stationary = computed(() => props.serial.motors?.motors.every(motor => Math.abs(motor.rpm) < 5));
const selectedChanged = computed(() => Object.keys(props.changes).some(id => id.startsWith(prefix.value) || id === directionField.value?.id));
const assignment = computed(() => candidate.value && props.data ? handTurnAssignment(props.data, props.changes, selected.value, candidate.value) : null);
const candidateValid = computed(() => ready.value && now.value - candidateCreatedAt.value <= 30 && props.serial.motors?.generation === candidateGeneration.value && candidate.value &&
    props.serial.motors.motors.some(m => m.configuredRouteId === candidate.value!.configuredRouteId &&
        m.configuredBus === candidate.value!.configuredBus && m.observedFeedbackId === candidate.value!.observedFeedbackId && motorRecent(m, now.value)));
const serialLabel = computed(() => {
    if (!props.serial.connected) return "电脑串口未连接";
    if (props.paused) return "观测已暂停";
    if (props.serial.format !== "text") return "需要文本格式串口";
    if (!props.serial.motors?.available) return "等待电机诊断数据";
    return ready.value ? "已收到停机状态与新鲜反馈" : "等待停机状态和完整新鲜反馈";
});
const conflicts = computed(() => {
    if (!props.data) return [];
    const addresses = new Map<string, string>();
    const messages: string[] = [];
    for (const wheel of wheels.value) {
        const base = `motor.chassis[${wheel.index}].`;
        const fb = motorFeedback(props.data, props.changes, wheel.index);
        if (fb === null) continue;
        const key = `${value(fields.value.get(base + "bus"))}:${fb}`;
        const other = addresses.get(key);
        if (other) messages.push(`${other}与${wheel.label}使用同一反馈地址，请完成交换后再保存。`);
        addresses.set(key, wheel.label);
    }
    return messages;
});
const motions: { id: Motion; label: string }[] = [
    { id: "forward", label: "前进" }, { id: "backward", label: "后退" }, { id: "left", label: "左移" },
    { id: "right", label: "右移" }, { id: "ccw", label: "左转" }, { id: "cw", label: "右转" },
];
function value(field?: ConfigField) { return chassisValue(field, props.changes); }
function hex(number: number | null | undefined) { return number == null ? "—" : `0x${number.toString(16).toUpperCase().padStart(3, "0")}`; }
function wheelAddress(index: number) {
    const base = `motor.chassis[${index}].`;
    const id = value(fields.value.get(base + "id"));
    return id === 0 ? "未启用" : `${value(fields.value.get(base + "bus")) ?? "—"} · 节点 ${id ?? "—"}`;
}
function fieldLabel(field: ConfigField) {
    return field.id.endsWith(".id") ? "电机节点号（0 为关闭）" : field.label;
}
function fieldOptions(field: ConfigField) {
    return field.id.endsWith(".bus") ? field.options?.filter(option => String(option.value).startsWith(model.value?.transport === "RS485" ? "RS485" : "CAN")) : field.options;
}
function selectField(field: ConfigField, event: Event) {
    const selectedValue = (event.target as HTMLSelectElement).value;
    const option = field.options?.find(option => String(option.value) === selectedValue);
    if (option) emit("change", field.id, option.value);
}
function numberField(field: ConfigField, event: Event) {
    const text = (event.target as HTMLInputElement).value;
    emit("change", field.id, text === "" ? "" : Number(text));
}
function chooseWheel(index: number) {
    if (capture.value) return;
    selected.value = index;
    resetObservation();
}
function resetObservation() {
    capture.value = null;
    candidate.value = null;
    candidateGeneration.value = null;
    candidateCreatedAt.value = 0;
    captureMessage.value = "";
    confirmed.value = false;
}
function beginCapture() {
    if (props.disabled || !supported.value || !confirmed.value || !ready.value || !stationary.value || !props.serial.motors) return;
    candidate.value = null;
    candidateGeneration.value = null;
    capture.value = handTurnStart(props.serial.motors, Date.now()/1000);
    captureMessage.value = `保持停机，只手转${selectedWheel.value.label}主轮，其余轮子保持不动。`;
}
function stopCapture() {
    capture.value = null;
    candidate.value = null;
    captureMessage.value = "本次采集已取消。";
}
function tick() {
    now.value = Date.now()/1000;
    if (candidate.value && (props.disabled || !ready.value || now.value - candidateCreatedAt.value > 30)) {
        resetObservation();
        captureMessage.value = "观测状态改变或候选已超时，请重新采集。";
        return;
    }
    if (!capture.value) return;
    if (props.disabled || !ready.value || !props.serial.motors || !handTurnSample(capture.value, props.serial.motors)) {
        stopCapture();
        confirmed.value = false;
        captureMessage.value = "观测中断、状态改变或反馈过期，本次结果已作废。";
        return;
    }
    if (now.value - capture.value.startedAt >= 8) {
        const result = handTurnResult(capture.value);
        candidate.value = result.candidate;
        candidateGeneration.value = capture.value.generation;
        candidateCreatedAt.value = now.value;
        captureMessage.value = result.reason;
        capture.value = null;
    }
}
function acceptCandidate() {
    if (props.disabled || !candidateValid.value || !assignment.value) return;
    emit("assign", assignment.value);
    resetObservation();
    captureMessage.value = "轮位候选已加入草稿；请核对其余轮位，再统一保存。安装方向仍需单独确认。";
}
watch(() => props.data?.target, resetObservation);
watch(() => props.data?.revision, resetObservation);
watch(() => props.serial.motors?.generation, () => {
    if (capture.value || candidate.value) {
        resetObservation();
        captureMessage.value = "串口会话已改变，请重新采集。";
    }
});
watch(type, resetObservation);
watch(() => ["model", "bus", "id", "feedback_id"].map(member => value(fields.value.get(prefix.value + member))).join("|"), resetObservation);
watch([ready, () => props.disabled], ([isReady, isDisabled]) => {
    if ((!isReady || isDisabled) && (capture.value || candidate.value)) {
        resetObservation();
        captureMessage.value = "观测或停机状态已中断，旧结果已作废，请重新采集。";
    }
});
const timer = window.setInterval(tick, 100);
onBeforeUnmount(() => window.clearInterval(timer));
</script>

<template>
    <div v-if="!data" class="panel muted">正在读取底盘配置…</div>
    <div v-else-if="!supported" class="panel chassis-unavailable">
        <h2>当前车型未启用经典四轮底盘</h2>
        <p>装配图支持麦克纳姆轮和 X 形全向轮。请在“车型与接口”确认底盘控制器；其他机构继续使用“参数与设备”。</p>
    </div>
    <div v-else class="chassis-panel">
        <div class="chassis-toolbar panel">
            <label v-if="wheelTypeField" class="field"><span>底盘轮型</span>
                <select :value="value(wheelTypeField)" :disabled="disabled || !!capture || wheelTypeField.editable === false" @change="selectField(wheelTypeField, $event)">
                    <option v-for="option in wheelTypeField.options" :key="String(option.value)" :value="option.value">{{ option.label }}</option>
                </select>
            </label>
            <span>车头朝上 · 点击车轮编辑对应位置</span>
            <span class="chassis-file-state">{{ Object.keys(changes).length ? '含未保存草稿' : '当前车型文件' }} · 车上版本未核对</span>
        </div>
        <p v-if="Object.hasOwn(changes, 'tuning.chassis.wheel_type')" class="chassis-warning">轮型已改变，请重新核对后轮位置和方向；当前仅修改草稿。</p>
        <div class="chassis-layout">
            <article class="panel chassis-diagram">
                <div class="chassis-heading"><h2>四轮位置</h2><span>{{ type === 'xdrive' ? '切向安装全向轮' : '麦克纳姆轮' }}</span></div>
                <svg viewBox="0 0 500 440" role="img" aria-label="底盘俯视图，点击四个车轮选择轮位" class="wheel-diagram">
                    <title>四轮底盘装配</title>
                    <circle v-if="type === 'xdrive'" cx="250" cy="220" r="132" class="chassis-body" />
                    <rect v-else x="158" y="119" width="184" height="202" rx="18" class="chassis-body" />
                    <path d="M 250 176 V 125 M 237 138 L 250 125 L 263 138" class="front-arrow" />
                    <text x="250" y="105" text-anchor="middle" class="diagram-front">车头</text>
                    <text x="250" y="220" text-anchor="middle" class="diagram-mode">{{ motions.find(item => item.id === motion)?.label }}</text>
                    <text x="250" y="247" text-anchor="middle" class="diagram-caption">方向预览</text>
                    <g v-for="wheel in wheels" :key="wheel.position" class="wheel-hit" :class="{ selected: selected === wheel.index }"
                        role="button" tabindex="0" :aria-label="`${wheel.label}，${wheelAddress(wheel.index)}`" :aria-pressed="selected === wheel.index"
                        @click="chooseWheel(wheel.index)" @keydown.enter.prevent="chooseWheel(wheel.index)" @keydown.space.prevent="chooseWheel(wheel.index)">
                        <rect :x="wheel.position % 2 === 0 ? 35 : 295" :y="wheel.position < 2 ? 16 : 268" width="170" height="165" fill="transparent" />
                        <g :transform="`translate(${wheel.x} ${wheel.y}) rotate(${wheel.angle})`">
                            <rect x="-65" y="-25" width="130" height="50" fill="transparent" />
                            <rect x="-56" y="-12" width="112" height="24" rx="7" class="wheel-tire" />
                            <path v-if="type === 'xdrive'" d="M -40 -9 V 9 M -20 -9 V 9 M 0 -9 V 9 M 20 -9 V 9 M 40 -9 V 9" class="wheel-rollers" />
                            <path v-else :d="wheel.position === 0 || wheel.position === 3 ? 'M -47 -9 L -33 9 M -27 -9 L -13 9 M -7 -9 L 7 9 M 13 -9 L 27 9 M 33 -9 L 47 9' : 'M -47 9 L -33 -9 M -27 9 L -13 -9 M -7 9 L 7 -9 M 13 9 L 27 -9 M 33 9 L 47 -9'" class="wheel-rollers" />
                        </g>
                        <text :x="wheel.position % 2 === 0 ? 95 : 405" :y="wheel.position < 2 ? 38 : 399" text-anchor="middle" class="wheel-label">{{ wheel.label }} <tspan class="wheel-sign">{{ signs[wheel.index] === null ? '?' : signs[wheel.index]! > 0 ? '+' : '−' }}</tspan></text>
                        <text :x="wheel.position % 2 === 0 ? 95 : 405" :y="wheel.position < 2 ? 61 : 422" text-anchor="middle" class="wheel-address">{{ wheelAddress(wheel.index) }}</text>
                    </g>
                </svg>
                <div class="motion-buttons" role="group" aria-label="预览底盘动作">
                    <button v-for="item in motions" :key="item.id" class="button" :class="{ primary: motion === item.id }" :aria-pressed="motion === item.id" @click="motion = item.id">{{ item.label }}</button>
                </div>
                <p class="chassis-hint">+ / − 表示混控后电机目标的符号。预览不驱动电机，实际转向需核对安装方向。</p>
            </article>
            <article class="panel chassis-editor">
                <div class="chassis-heading"><h2>{{ selectedWheel.label }}轮</h2><span>{{ selectedChanged ? '有未保存修改' : '配置值' }}</span></div>
                <div class="chassis-fields">
                    <label v-for="field in editorFields" :key="field.id" class="field" :class="{ changed: Object.hasOwn(changes, field.id) }">
                        <span>{{ fieldLabel(field) }}</span>
                        <select v-if="field.options" :value="value(field)" :disabled="disabled || !!capture || field.editable === false" @change="selectField(field, $event)">
                            <option v-for="option in fieldOptions(field)" :key="String(option.value)" :value="option.value">{{ option.label }}</option>
                        </select>
                        <input v-else type="number" :value="value(field) as number | string" :min="field.min" :max="field.id.endsWith('.id') ? model?.id_max : field.max" :step="field.step ?? 'any'"
                            :disabled="disabled || !!capture || field.editable === false" @input="numberField(field, $event)" />
                    </label>
                    <label v-if="directionField" class="field"><span>安装方向</span>
                        <select :value="value(directionField)" :disabled="disabled || !!capture || directionField.editable === false" @change="emit('change', directionField.id, Number(($event.target as HTMLSelectElement).value))">
                            <option :value="1">保持（+1）</option><option :value="-1">反向（−1）</option>
                            <option v-if="![1,-1].includes(Number(value(directionField)))" :value="value(directionField) as string">当前值需在参数页核对</option>
                        </select>
                    </label>
                    <div class="chassis-derived"><span>预计反馈地址</span><strong>{{ feedback === null ? '未启用 / 待核对' : hex(feedback) }}</strong><small>{{ feedbackAutomatic ? '按型号与节点号计算' : '使用显式反馈地址' }}</small></div>
                </div>
                <div class="chassis-ratio">型号内置减速比 <strong>{{ model?.ratio ?? '—' }}</strong> · 编码器位数 <strong>{{ model?.encoder_bits ?? '—' }}</strong></div>
                <details v-if="feedbackField" class="chassis-advanced"><summary>反馈地址设置</summary>
                    <label class="field"><span>显式反馈 ID（十进制，−1 为自动）</span>
                        <input type="number" :value="feedbackAutomatic ? -1 : value(feedbackField) as number" :min="-1" :max="2047" step="1" :disabled="disabled || !!capture || feedbackField.editable === false" @input="numberField(feedbackField, $event)" />
                    </label>
                </details>
                <p v-for="message in conflicts" :key="message" class="chassis-warning" role="alert">{{ message }}</p>
                <p class="chassis-hint">这里绑定现有电机节点，不会修改电机硬件 ID。保存会统一检查全车电机命令 ID 和反馈 ID 冲突。</p>
            </article>
        </div>
        <article class="panel hand-turn">
            <div class="chassis-heading"><h2>手转识别轮位</h2><span :class="{ 'hand-ready': ready }">{{ serialLabel }}</span></div>
            <div class="hand-connect">
                <label for="chassis-port">电脑串口</label>
                <select id="chassis-port" :value="port" :disabled="serial.connected || disabled" @change="emit('port', ($event.target as HTMLSelectElement).value)">
                    <option value="" disabled>选择串口</option><option v-for="item in ports" :key="item.device" :value="item.device">{{ item.device }} · {{ item.description }}</option>
                </select>
                <button class="button" :disabled="disabled" @click="emit('refresh')">刷新</button>
                <button class="button" :disabled="disabled || (!serial.connected && !port)" @click="emit('toggle')">{{ serial.connected ? '断开串口' : '连接观测串口' }}</button>
                <button v-if="paused" class="button" @click="emit('resume')">继续观测</button>
            </div>
            <p class="chassis-hint">当前支持 INFANTRY-TIAN-A 的 UART6 电机诊断，115200、文本格式。先保持遥控安全上档，确认主轮停机；本页只接收数据。</p>
            <p v-if="serial.error" class="chassis-warning">{{ serial.error }}</p>
            <div class="hand-actions">
                <label><input v-model="confirmed" type="checkbox" :disabled="disabled || !!capture" /> 已核对串口对应当前车辆，主轮已停机，只手转选中轮子</label>
                <button class="button primary" :disabled="disabled || !ready || !stationary || !confirmed || !!capture" @click="beginCapture">识别{{ selectedWheel.label }}轮</button>
                <button v-if="capture" class="button" @click="stopCapture">取消采集</button>
                <span v-if="capture" role="status">采集中 · {{ Math.max(0, Math.ceil(8 - (now - capture.startedAt))) }} 秒</span>
            </div>
            <p v-if="captureMessage" class="hand-message" role="status">{{ captureMessage }}</p>
            <div v-if="candidate" class="hand-candidate">
                <div><strong>响应候选：{{ hex(candidate.observedFeedbackId) }}</strong><span>固件路由 CAN{{ candidate.configuredBus }} · {{ candidate.name }}</span></div>
                <button class="button primary" :disabled="disabled || !candidateValid || !assignment" @click="acceptCandidate">确认绑定到{{ selectedWheel.label }}草稿</button>
                <span v-if="!candidateValid">当前反馈已失效，请重新采集。</span>
                <span v-else-if="!assignment">该反馈无法按所选型号换算节点，请人工核对。</span>
            </div>
            <p class="chassis-hint">依据连续转速反馈给出候选；没有编码器角度时不自动判断安装方向。路由总线来自板上配置，仍需核对实际接线。</p>
        </article>
    </div>
</template>

<style scoped>
.chassis-panel { display: grid; gap: 16px; }
.chassis-toolbar { display: flex; align-items: center; flex-wrap: wrap; gap: 20px; }
.chassis-toolbar > span { font-size: 12px; color: #617079; }
.chassis-toolbar .field { min-width: 180px; }
.chassis-file-state { margin-left: auto; }
.chassis-layout { display: grid; grid-template-columns: minmax(360px, 1.15fr) minmax(315px, 1fr); gap: 16px; align-items: start; }
.chassis-heading { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; }
.chassis-heading h2 { margin: 0; }
.chassis-heading > span { font-size: 12px; color: #728087; }
.wheel-diagram { display: block; width: 100%; max-height: 440px; }
.chassis-body { fill: #f2f5f6; stroke: #c3cdd1; stroke-width: 2; }
.front-arrow { fill: none; stroke: #7c8f98; stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; }
.diagram-front { font-size: 16px; fill: #5c6b74; }
.diagram-mode { font-size: 25px; font-weight: 650; fill: #35464f; }
.diagram-caption { font-size: 14px; fill: #7a8991; }
.wheel-hit { cursor: pointer; outline: none; }
.wheel-tire { fill: #576871; stroke: #576871; stroke-width: 3; }
.wheel-rollers { stroke: #dae2e4; stroke-width: 3; }
.wheel-hit.selected .wheel-tire { fill: #eabb38; stroke: #bf8d0c; }
.wheel-hit.selected .wheel-rollers { stroke: #7e631d; }
.wheel-hit:focus-visible .wheel-tire { stroke: #167193; stroke-width: 5; }
.wheel-label { font-size: 17px; font-weight: 650; fill: #33434c; }
.wheel-sign { font-size: 23px; }
.wheel-address { font-size: 14px; fill: #6c7c84; }
.motion-buttons { display: flex; flex-wrap: wrap; gap: 6px; justify-content: center; }
.motion-buttons .button { min-width: 52px; justify-content: center; }
.chassis-hint { font-size: 12px; line-height: 1.7; color: #6b7a83; margin: 14px 0 0; }
.chassis-fields { display: grid; grid-template-columns: 1fr 1fr; gap: 20px 14px; margin-top: 24px; }
.chassis-fields .field { font-size: 12px; }
.chassis-fields select, .chassis-fields input { width: 100%; height: 36px; }
.chassis-fields .changed > span { color: #8f6503; }
.chassis-derived { display: flex; flex-direction: column; gap: 5px; font-size: 12px; }
.chassis-derived strong { font-size: 21px; color: #35464f; }
.chassis-derived small { color: #6b7a83; }
.chassis-ratio { font-size: 12px; margin-top: 22px; padding: 15px 0; border-top: 1px solid #e3e9eb; color: #667780; }
.chassis-advanced { margin-top: 8px; font-size: 12px; }
.chassis-advanced summary { cursor: pointer; color: #506771; }
.chassis-advanced .field { margin-top: 12px; }
.chassis-warning { padding: 12px; color: #8e5511; background: #fff5df; font-size: 12px; line-height: 1.6; margin: 0; }
.chassis-editor .chassis-warning { margin-top: 14px; }
.hand-connect { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; margin-top: 18px; font-size: 12px; }
.hand-connect select { max-width: 330px; min-width: 130px; height: 34px; border: 1px solid #cdd5d7; padding: 0 8px; background: white; }
.hand-actions { display: flex; align-items: center; flex-wrap: wrap; gap: 14px; margin-top: 18px; font-size: 12px; }
.hand-actions label { display: flex; align-items: center; gap: 7px; }
.hand-message { font-size: 13px; color: #49616d; line-height: 1.6; }
.hand-candidate { display: flex; align-items: center; flex-wrap: wrap; gap: 16px; padding: 14px; background: #f4f7f8; font-size: 12px; }
.hand-candidate > div { display: grid; gap: 6px; }
.hand-candidate > div > span { color: #70818a; }
.chassis-heading .hand-ready { color: #267b61; }
.chassis-unavailable p { color: #687982; line-height: 1.7; font-size: 13px; }
@media (max-width: 1150px) {
    .chassis-layout { grid-template-columns: 1fr; }
    .wheel-diagram { max-height: 400px; }
}
</style>
