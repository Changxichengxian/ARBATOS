<script setup lang="ts">
import { computed, ref, watch } from "vue";

export type VehicleSetupCatalog = {
    schema: number;
    boards: { id: string; label: string; available: boolean; reason?: string; templates: string[] }[];
    templates: { id: string; label: string; board: string; source: string; mechanisms?: string[] }[];
    mechanisms: {
        id: string;
        label: string;
        requires: string[];
        motor_groups: {
            id: string;
            label: string;
            capacity: number;
            transport?: string | string[];
            models: { id: string; label: string; transport: string }[];
            defaults?: ({ model: string; id: number; bus: string } | null)[];
            defaults_by_board?: Record<string, ({ model: string; id: number; bus: string } | null)[]>;
            empty_label?: string;
            direction_status?: string;
        }[];
    }[];
    radios: { id: string; label: string; services: string[]; requires_board: string[] }[];
    transport_capabilities?: { id: string; label?: string; available: boolean; reason?: string }[];
    steps: string[];
};
type Setup = { board: string; template: string; mechanisms: string[]; radio: string; motors?: Record<string, string[]>; power?: { limit_mode: string } };
const props = defineProps<{ catalog: VehicleSetupCatalog | null; busy?: boolean; error?: string }>();
const emit = defineEmits<{ submit: [name: string, setup: Setup]; cancel: [] }>();

const name = ref("");
const board = ref("");
const template = ref("");
const mechanisms = ref<string[]>([]);
const radio = ref("");
const motors = ref<Record<string, string[]>>({});
const powerLimitMode = ref("Auto");
const selectedBoard = computed(() => props.catalog?.boards.find(x => x.id === board.value));
const templates = computed(() => props.catalog?.templates.filter(x => x.board === board.value) ?? []);
const availableMechanisms = computed(() => props.catalog?.mechanisms.filter(x => !x.requires.length || x.requires.includes(template.value)) ?? []);
const selectedMechanisms = computed(() => availableMechanisms.value.filter(x => mechanisms.value.includes(x.id)));
const radios = computed(() => props.catalog?.radios.filter(x => !x.requires_board.length || x.requires_board.includes(board.value)) ?? []);
const motorGroups = computed(() => selectedMechanisms.value.flatMap(mechanism => mechanism.motor_groups.map(group => ({ ...group, mechanism: mechanism.label }))));
const hasShootWithoutGimbal = computed(() => mechanisms.value.includes("rm_shoot") && !mechanisms.value.some(id => /gimbal|yaw/.test(id)));
const hasClassicChassis = computed(() => mechanisms.value.includes("classic_chassis"));
const ready = computed(() => !!name.value.trim() && !!board.value && !!template.value && !!radio.value && mechanisms.value.length > 0);
const step = computed(() => !board.value ? 1 : !template.value || !mechanisms.value.length ? 2 : !radio.value ? 3 : 4);
function boardCanStart(id: string) {
    return !!props.catalog?.templates.some(item => item.board === id) && !!props.catalog?.radios.some(item => !item.requires_board.length || item.requires_board.includes(id));
}
function boardReason(item: VehicleSetupCatalog["boards"][number]) {
    if (!boardCanStart(item.id)) return [item.reason, "当前目录没有可用遥控链路，不能通过向导创建。"].filter(Boolean).join("；");
    return item.reason || "";
}

function chooseBoard(id: string) {
    if (id === board.value) return;
    board.value = id;
    template.value = "";
    mechanisms.value = [];
    motors.value = {};
    radio.value = "";
}
function chooseTemplate(id: string) {
    template.value = id;
    const preset = props.catalog?.templates.find(x => x.id === id);
    const allowed = new Set((props.catalog?.mechanisms ?? []).filter(item => !item.requires.length || item.requires.includes(id)).map(item => item.id));
    mechanisms.value = (preset?.mechanisms ?? []).filter(item => allowed.has(item));
    motors.value = {};
}
function toggleMechanism(id: string, checked: boolean) {
    mechanisms.value = checked ? [...new Set([...mechanisms.value, id])] : mechanisms.value.filter(x => x !== id);
    const allowed = new Set(selectedMechanisms.value.flatMap(x => x.motor_groups.map(group => group.id)));
    motors.value = Object.fromEntries(Object.entries(motors.value).filter(([key]) => allowed.has(key)));
}
function motorValue(group: { id: string; capacity: number }, index: number) { return motors.value[group.id]?.[index] ?? ""; }
function setMotor(group: { id: string; capacity: number }, index: number, value: string) {
    const next = [...(motors.value[group.id] ?? Array(group.capacity).fill(""))];
    next[index] = value;
    motors.value = { ...motors.value, [group.id]: next };
}
function transportText(value?: string | string[]) {
    const ids = Array.isArray(value) ? value : value ? [value] : [];
    if (!ids.length) return "后端未声明";
    const known: Record<string, string> = { CAN: "CAN", CAN_FD: "CAN-FD", RS485: "RS485", PWM: "PWM", SERVO: "PWM 舵机", UART_MOTOR: "总线舵机（尚未接入具体协议）" };
    return ids.map(id => known[id] || id).join("、");
}
function transportReason(value?: string | string[]) {
    const ids = Array.isArray(value) ? value : value ? [value] : [];
    const unavailable = props.catalog?.transport_capabilities?.find(item => ids.includes(item.id) && !item.available);
    return unavailable?.reason || (ids.includes("UART_MOTOR") ? "尚未接入具体协议，不能作为可运行电机型号选择。" : "");
}
type MotorDefault = { model: string; id: number; bus: string } | null;
function defaultMotor(group: { defaults?: MotorDefault[]; defaults_by_board?: Record<string, MotorDefault[]> }, index: number) { return (group.defaults_by_board?.[board.value] ?? group.defaults)?.[index]; }
function defaultMotorText(group: { defaults?: MotorDefault[]; defaults_by_board?: Record<string, MotorDefault[]>; empty_label?: string; models: { id: string; label: string }[]; transport?: string | string[] }, index: number) {
    const value = defaultMotor(group, index);
    if (value) return `默认：${group.models.find(model => model.id === value.model)?.label || value.model} · ${value.bus} · ID ${value.id}`;
    return group.empty_label || ((Array.isArray(group.transport) ? group.transport : [group.transport]).includes("PWM") ? "不启用" : "沿用车型骨架默认电机");
}
function create() {
    if (!ready.value) return;
    const compactMotors = Object.fromEntries(Object.entries(motors.value).filter(([, values]) => values.some(Boolean)));
    emit("submit", name.value.trim(), { board: board.value, template: template.value, mechanisms: mechanisms.value, radio: radio.value, motors: compactMotors, power: { limit_mode: hasClassicChassis.value ? powerLimitMode.value : "Auto" } });
}
watch(() => props.catalog, catalog => { if (catalog && !board.value) { const first = catalog.boards.find(x => x.available); if (first) chooseBoard(first.id); } }, { immediate: true });
</script>

<template>
    <section class="vehicle-setup">
        <header><p class="eyebrow">快速新建车型</p><h2>选板子、勾机构、核对后创建</h2><p>创建只生成离线车型文件。随后按引导完成装配、接收机设置和编译；不会连接、烧录或驱动硬件。</p></header>
        <p v-if="!catalog" class="muted">正在读取当前工程可创建的板级、机构和协议目录…</p>
        <template v-else>
            <ol class="setup-steps"><li :class="{ active: step === 1, done: board }">1 选板子</li><li :class="{ active: step === 2, done: template && mechanisms.length }">2 勾机构</li><li :class="{ active: step === 3, done: radio }">3 电机与遥控</li><li :class="{ active: step === 4 }">4 检查创建</li></ol>
            <label class="setup-name">车型名称<input v-model="name" :disabled="busy" placeholder="例如 WHEELEG-M-01" /></label>
            <div class="setup-section"><h3>控制板</h3><div class="choice-grid"><button v-for="item in catalog.boards" :key="item.id" type="button" :disabled="busy || !item.available || !boardCanStart(item.id)" :class="{ selected: board === item.id, unavailable: !item.available || !boardCanStart(item.id) }" @click="chooseBoard(item.id)"><b>{{ item.label }}</b><small>{{ item.id }}</small><em v-if="boardReason(item)">{{ boardReason(item) }}</em></button></div></div>
            <div v-if="board" class="setup-section"><h3>车型骨架</h3><p class="muted">选项来自当前板子的车型模板；更换板子会清掉不兼容的机构、电机选择和遥控器。</p><div class="choice-grid"><button v-for="item in templates" :key="item.id" type="button" :disabled="busy" :class="{ selected: template === item.id }" @click="chooseTemplate(item.id)"><b>{{ item.label }}</b><small>基础来源：{{ item.source }}</small></button></div><p v-if="!templates.length" class="field-error">该板级当前没有可创建的整车骨架。</p></div>
            <div v-if="template" class="setup-section"><h3>机构组</h3><div class="check-grid"><label v-for="item in availableMechanisms" :key="item.id"><input type="checkbox" :disabled="busy" :checked="mechanisms.includes(item.id)" @change="toggleMechanism(item.id, ($event.target as HTMLInputElement).checked)" /><span><b>{{ item.label }}</b><small>{{ item.motor_groups.map(x => `${x.label} × ${x.capacity}`).join('；') || '无电机组' }}</small></span></label></div></div>
            <div v-if="selectedMechanisms.length" class="setup-section"><h3>电机表与传输方式</h3><p class="muted">默认值来自车型骨架；这里选择型号会替换默认型号。总线地址和安装方向仍在创建后进入装配页逐项核对；方向必须实车检查。</p><div class="motor-group" v-for="group in motorGroups" :key="group.id"><div><b>{{ group.mechanism }} · {{ group.label }}</b><small>预留 {{ group.capacity }} 位 · 传输：{{ transportText(group.transport) }} · {{ group.direction_status === 'pending_validation' ? '方向待验证' : group.direction_status || '状态由后端提供' }}</small><em v-if="transportReason(group.transport)">{{ transportReason(group.transport) }}</em></div><label v-for="index in group.capacity" :key="index">{{ group.label }} {{ index }}<select :disabled="busy" :value="motorValue(group, index - 1)" @change="setMotor(group, index - 1, ($event.target as HTMLSelectElement).value)"><option value="">{{ defaultMotorText(group, index - 1) }}</option><option v-for="model in group.models" :key="model.id" :value="model.id">{{ model.label }} · {{ transportText(model.transport) }}</option></select><small>{{ defaultMotorText(group, index - 1) }}</small></label></div></div>
            <p v-if="hasShootWithoutGimbal" class="setup-note">发射机构可以独立运行，并保留遥控安全档和故障保护。若这台车需要由云台完成瞄准，请同时勾选普通云台或双水平轴云台。</p>
            <div v-if="template" class="setup-section"><h3>遥控器</h3><div class="choice-grid"><button v-for="item in radios" :key="item.id" type="button" :disabled="busy" :class="{ selected: radio === item.id }" @click="radio = item.id"><b>{{ item.label }}</b><small>{{ item.services.join('、') }}</small></button></div><p v-if="!radios.length" class="field-error">此板级当前没有后端声明的可用遥控链路。</p></div>
            <div v-if="template" class="setup-section"><template v-if="hasClassicChassis"><h3>功率限制策略</h3><select v-model="powerLimitMode" :disabled="busy"><option value="Auto">自动</option><option value="TotalCommand">总指令限制</option><option value="Model">模型限制</option><option value="ModelAndTotal">模型与总指令</option></select><p class="muted">仅经典四轮底盘读取这项离线默认策略；功率表和实车边界仍要在“参数与设备”中核对。</p></template><template v-else><h3>功率限制策略</h3><p class="muted">当前机构不读取经典四轮底盘的功率策略。创建时会保留 Auto，不在这里提供选择。</p></template></div>
            <div class="setup-preview"><h3>创建预览</h3><p>{{ selectedBoard?.label || '未选板子' }} · {{ templates.find(x => x.id === template)?.label || '未选车型骨架' }} · {{ selectedMechanisms.map(x => x.label).join('、') || '未选机构' }} · {{ radios.find(x => x.id === radio)?.label || '未选遥控器' }}</p><ol><li>创建离线车型文件</li><li>进入底盘装配，核对电机总线、地址与方向</li><li>进入接收机页，核对通道和拨杆</li><li>在编译页检查并构建；烧录需单独操作</li></ol></div>
            <p v-if="error" class="field-error">{{ error }}</p><footer><button class="button" :disabled="busy" @click="emit('cancel')">取消</button><button class="button primary" :disabled="busy || !ready" @click="create">创建车型</button></footer>
        </template>
    </section>
</template>

<style scoped>
:global(.dialog.vehicle-setup-dialog) { width: min(980px, 94vw); max-height: calc(100vh - 36px); overflow: auto; } :global(.advanced-copy) { margin-top: 18px; padding-top: 12px; border-top: 1px solid #dce2e4; } :global(.advanced-copy summary) { cursor: pointer; font-size: 12px; font-weight: 650; } :global(.advanced-copy > .button) { margin-top: 4px; }
.vehicle-setup { max-width: 920px; display: grid; gap: 18px; } .vehicle-setup h2,.vehicle-setup h3 { margin: 0; } .vehicle-setup header p { margin: 7px 0 0; color: #657177; } .setup-steps { list-style: none; padding: 0; margin: 0; display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; } .setup-steps li { padding: 9px; border-radius: 8px; background: #f3f6f7; color: #657177; font-size: 13px; } .setup-steps .active { color: #27313a; outline: 1px solid #dca712; } .setup-steps .done { color: #286849; } .setup-name { display: grid; gap: 6px; font-weight: 700; max-width: 420px; } .setup-name input,.setup-section select { height: 32px; border: 1px solid #cdd5d7; background: #fff; padding: 0 8px; font: inherit; color: #26333a; } .setup-section { display: grid; gap: 10px; padding-top: 2px; } .choice-grid,.check-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; } .choice-grid button,.check-grid label { text-align: left; display: grid; gap: 4px; padding: 11px; border: 1px solid #dce2e4; background: #fff; color: #27313a; border-radius: 8px; cursor: pointer; } .choice-grid button.selected { border-color: #dca712; box-shadow: inset 0 0 0 1px #dca712; } .choice-grid button.unavailable { opacity: .62; cursor: not-allowed; } .choice-grid small,.check-grid small,.motor-group small { color: #657177; } .choice-grid em,.motor-group em { color: #a87812; font-style: normal; font-size: 12px; } .check-grid label { grid-template-columns: auto 1fr; align-items: start; } .check-grid label span { display: grid; gap: 3px; } .motor-group { display: grid; grid-template-columns: minmax(190px, 1fr) repeat(auto-fit, minmax(135px, 1fr)); gap: 10px; align-items: end; padding: 12px; border: 1px solid #dce2e4; border-radius: 8px; } .motor-group > div { display: grid; gap: 4px; } .motor-group label { display: grid; gap: 4px; font-size: 13px; } .setup-note { margin: -2px 0 0; padding: 9px 11px; background: #fff8e7; border-left: 3px solid #dca712; color: #6f5922; font-size: 12px; line-height: 1.5; } .setup-preview { padding: 14px; border-radius: 8px; background: #f3f6f7; } .setup-preview p { margin: 8px 0; } .setup-preview ol { margin: 0; padding-left: 20px; display: grid; gap: 3px; color: #657177; } footer { display: flex; justify-content: flex-end; gap: 8px; } @media (max-width: 650px) { .setup-steps { grid-template-columns: repeat(2, 1fr); } .motor-group { grid-template-columns: 1fr; } }
</style>
