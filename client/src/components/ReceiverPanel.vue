<script setup lang="ts">
import { computed, onBeforeUnmount, ref } from "vue";
import type { ConfigField, ConfigValues } from "./ConfigValuesPanel.vue";
import { channelPercent, channelUses, fieldValue, functionChannel, rcFresh,
    semanticSwitch, switchPosition, type ReceiverStatus } from "../receiver";

const props = defineProps<{
    data: ConfigValues | null;
    changes: Record<string, unknown>;
    serial: ReceiverStatus & { port?: string; baud?: number; error?: string };
    ports: { device: string; description: string }[];
    port: string;
    paused: boolean;
    disabled: boolean;
}>();
const emit = defineEmits<{
    change: [id: string, value: unknown];
    port: [value: string];
    toggle: [];
    refresh: [];
    resume: [];
}>();
const tab = ref("channels");
const now = ref(Date.now() / 1000);
const timer = window.setInterval(() => { now.value = Date.now() / 1000; }, 100);
onBeforeUnmount(() => window.clearInterval(timer));
function currentTime() {
    // 定时值负责触发断流刷新；新包到达时必须即时取时，避免缓存时钟落后于收包时间。
    return Math.max(now.value, Date.now() / 1000);
}
const snapshot = computed(() => props.serial.receiver);
const live = computed(() => !props.paused && props.serial.format === "text" &&
    rcFresh(snapshot.value, props.serial.connected, currentTime()));
const fields = computed(() => props.data?.groups.flatMap(group => group.fields) ?? []);
const physical = computed(() => fields.value.filter(field => field.id.startsWith("input.elrs.")));
const axes = computed(() => fields.value.filter(field => field.id.startsWith("input.axis.") && field.id.endsWith(".channel")));
const switches = computed(() => fields.value.filter(field => field.id.startsWith("input.sw.") && field.id.endsWith(".channel")));
const semantics = computed(() => fields.value.filter(field => field.id.startsWith("input.manual_input.semantics.") && field.id.endsWith("Pos") && semanticSwitch(field.id)));
const uses = computed(() => channelUses(props.data, props.changes));
const stateText = computed(() => {
    if (!props.serial.connected) return "电脑串口未连接";
    if (props.paused) return "显示已暂停";
    if (props.serial.format !== "text") return "请使用文本格式连接";
    if (!snapshot.value?.channels) return "等待 16 路通道数据";
    return live.value ? "正在接收" : "通道已失效";
});
function recent(receivedAt?: number) {
    const age = receivedAt === undefined ? Infinity : currentTime() - receivedAt;
    return props.serial.connected && !props.paused && receivedAt !== undefined &&
        age >= 0 && age <= 2.5;
}
const quality = computed(() => {
    const link = snapshot.value?.link;
    // 固件在断链时保留诊断计数，只有等待通道或在线状态的质量值仍可用。
    return link && recent(link.receivedAt) && link.portActive && (link.state === 1 || link.state === 2)
        ? `${link.lq}%` : "—";
});
const source = computed(() => {
    const input = snapshot.value?.input;
    if (!input || !recent(input.receivedAt)) return "等待状态";
    if (!input.online) return "输入离线";
    return ({ 0: "无", 1: "SBUS / DBUS", 2: "ELRS", 3: "图传遥控", 4: "USB" } as Record<number, string>)[input.source] ?? `来源 ${input.source}`;
});
function value(field: ConfigField) { return fieldValue(field, props.changes); }
function selectValue(field: ConfigField, event: Event) {
    const text = (event.target as HTMLSelectElement).value;
    const option = field.options?.find(item => String(item.value) === text);
    if (option) emit("change", field.id, option.value);
}
function title(field: ConfigField) {
    return field.label.replace(" 逻辑通道", "").replace("Yaw", "水平").replace("Pitch", "俯仰");
}
function invertField(field: ConfigField) {
    return fields.value.find(item => item.id === field.id.replace(/\.channel$/, ".invert"));
}
function inverted(field: ConfigField) {
    const inverse = invertField(field);
    return inverse ? Boolean(value(inverse)) : false;
}
function changeInvert(field: ConfigField, event: Event) {
    const inverse = invertField(field);
    if (inverse) emit("change", inverse.id, (event.target as HTMLInputElement).checked);
}
function physicalLabel(field: ConfigField) {
    const ch = functionChannel(field.id, props.data, props.changes);
    return ch ? `CH${ch}` : "未映射";
}
function logicalOption(field: ConfigField, logical: number | string) {
    const kind = field.id.startsWith("input.axis.") ? "ElrsChMap" : "ElrsSwMap";
    const physicalField = fields.value.find(item => item.id === `input.elrs.${kind}.${Number(logical) - 1}`);
    return `${kind === "ElrsChMap" ? "轴" : "拨杆"} ${logical} · ${physicalField ? `CH${value(physicalField)}` : "未映射"}`;
}
function modeInfo(field: ConfigField) {
    const sw = semanticSwitch(field.id);
    const map = fields.value.find(item => item.id === `input.sw.${sw}.channel`);
    if (!map) return { channel: "未映射", position: "—", matched: false };
    const ch = functionChannel(map.id, props.data, props.changes);
    const raw = ch ? snapshot.value?.channels?.[ch - 1] : undefined;
    const pos = raw === undefined || !live.value ? null : switchPosition(raw, inverted(map));
    return { channel: ch ? `CH${ch}${inverted(map) ? " · 反向" : ""}` : "未映射",
        position: pos ? ({ UP: "上档", MID: "中档", DOWN: "下档" })[pos] : "—",
        matched: pos !== null && value(field) === `MANUAL_INPUT_SWITCH_POS_${pos}` };
}
</script>

<template>
    <div class="receiver-panel">
        <div class="rx-connect">
            <label for="receiver-port">电脑串口</label>
            <select id="receiver-port" :value="port" :disabled="serial.connected || disabled" @change="emit('port', ($event.target as HTMLSelectElement).value)">
                <option value="" disabled>选择串口</option>
                <option v-for="item in ports" :key="item.device" :value="item.device">{{ item.device }} · {{ item.description }}</option>
            </select>
            <button class="button" :disabled="disabled" @click="emit('refresh')">刷新</button>
            <button class="button primary" :disabled="disabled || (!serial.connected && !port)" @click="emit('toggle')">{{ serial.connected ? '断开' : '连接接收机监视' }}</button>
            <span>MC02 UART8 · 115200 · 文本</span>
        </div>
        <div v-if="paused" class="rx-note">串口显示已暂停。<button class="button" @click="emit('resume')">继续接收</button></div>
        <div v-if="serial.error" class="rx-warning">{{ serial.error }}</div>
        <div class="rx-stats">
            <div class="rx-state" :class="{ live }"><small>通道状态</small><strong><i />{{ stateText }}</strong></div>
            <div><small>链路质量 LQ</small><strong>{{ quality }}</strong></div>
            <div><small>固件当前输入来源</small><strong>{{ source }}</strong></div>
            <div><small>UART 错误 · 开机累计</small><strong>{{ snapshot?.uart?.errors ?? '—' }}</strong></div>
        </div>
        <div class="rx-tabs" role="tablist" aria-label="接收机视图">
            <button v-for="item in [{ id: 'channels', label: '实时通道' }, { id: 'mapping', label: '功能分配' }, { id: 'modes', label: '拨杆模式' }]" :key="item.id"
                role="tab" :aria-selected="tab === item.id" :class="{ selected: tab === item.id }" @click="tab = item.id">{{ item.label }}</button>
        </div>
        <p class="rx-context">用途与档位按 {{ data?.target ?? '所选车型' }} 的{{ Object.keys(changes).length ? '未保存草稿' : '配置文件' }}展示；保存后需重新编译、烧录。通道值来自当前串口设备，尚未核对设备内配置版本。</p>

        <div v-if="tab === 'channels'" class="rx-card">
            <div class="rx-card-heading"><div><h2>16 路实时通道</h2><p>移动遥控器摇杆或拨杆，观察对应通道。数值为 CRSF 原始值。</p></div><span class="rx-badge">{{ live ? '实时' : '等待 / 失效' }}</span></div>
            <div v-if="!snapshot?.channels" class="rx-note">{{ snapshot?.rc ? '已收到板端观测数据，尚未收到接收机通道帧。请检查接收机供电、接线和对频状态。' : '需要带接收机观测输出的固件。串口连接成功后，收到 @RC 数据才会显示通道。' }}</div>
            <div class="rx-channel-head"><span>通道</span><span>数值</span><div><span>172</span><span>992 · 中位</span><span>1811</span></div><span>当前配置用途</span></div>
            <div v-for="n in 16" :key="n" class="rx-channel" :class="{ stale: !live, unused: !uses[n - 1]?.length }" :data-channel="n">
                <strong>CH{{ n }}</strong><output>{{ snapshot?.channels?.[n - 1] ?? '—' }}</output>
                <div class="rx-track" :aria-label="`CH${n} 原始通道值`"><i class="rx-mid" /><div v-if="snapshot?.channels" class="rx-fill" :style="{ width: `${channelPercent(snapshot.channels[n - 1])}%` }" /><i v-if="snapshot?.channels" class="rx-marker" :style="{ left: `${channelPercent(snapshot.channels[n - 1])}%` }" /></div>
                <div class="rx-uses"><span v-for="use in uses[n - 1]" :key="use">{{ use }}</span><em v-if="!uses[n - 1]?.length">未映射</em></div>
            </div>
            <p class="rx-footnote">{{ live ? '当前通道有效。' : '失效时保留最后数值，仅供检查。' }}标尺为常用范围 172～1811；超出标尺的原始值仍如实显示。一个通道可以被多个功能共用。</p>
        </div>

        <div v-else-if="tab === 'mapping'" class="rx-mapping">
            <div class="rx-card">
                <div class="rx-card-heading"><div><h2>接收机 → 逻辑通道</h2><p>把遥控器的 CH 通道接到机器人现有的 5 个轴、2 个拨杆。</p></div></div>
                <div v-for="field in physical" :key="field.id" class="rx-binding" :class="{ modified: Object.hasOwn(changes, field.id) }">
                    <label :for="field.id">{{ field.label.replace('RC', '逻辑') }}</label>
                    <select :id="field.id" :value="value(field)" :disabled="disabled || !field.editable" @change="selectValue(field, $event)"><option v-for="option in field.options" :key="option.value" :value="option.value">{{ option.label }}</option></select>
                </div>
            </div>
            <div class="rx-card">
                <div class="rx-card-heading"><div><h2>逻辑通道 → 机器人功能</h2><p>可复用同一通道；方向对不上时调整该功能的反向选项。</p></div></div>
                <div v-for="field in [...axes, ...switches]" :key="field.id" class="rx-function" :class="{ modified: Object.hasOwn(changes, field.id) || (invertField(field) && Object.hasOwn(changes, invertField(field)!.id)) }">
                    <label :for="field.id">{{ title(field) }}<small>{{ physicalLabel(field) }}</small></label>
                    <select :id="field.id" :value="value(field)" :disabled="disabled || !field.editable" @change="selectValue(field, $event)"><option v-for="option in field.options" :key="option.value" :value="option.value">{{ logicalOption(field, option.value) }}</option></select>
                    <label class="rx-invert"><input type="checkbox" :aria-label="`${title(field)}反向`" :checked="inverted(field)" :disabled="disabled || !invertField(field)?.editable" @change="changeInvert(field, $event)" />反向</label>
                </div>
            </div>
            <p v-if="!physical.length" class="rx-note">当前车型没有可编辑的 ELRS 映射。</p>
        </div>

        <div v-else class="rx-card">
            <div class="rx-card-heading"><div><h2>三档拨杆与模式</h2><p>档位匹配按当前配置预览，不代表机器人已执行该动作。</p></div></div>
            <div class="rx-ranges"><span>下档 · ≤ 582</span><span>中档 · 583～1400</span><span>上档 · ≥ 1401</span></div>
            <p class="rx-footnote">以上为未反向的原始值范围；启用反向后，上、下档互换。</p>
            <div v-for="field in semantics" :key="field.id" class="rx-mode" :class="{ matched: modeInfo(field).matched, modified: Object.hasOwn(changes, field.id) }">
                <label :for="field.id">{{ field.label }}</label><span>{{ modeInfo(field).channel }}</span>
                <select :id="field.id" :value="value(field)" :disabled="disabled || !field.editable" @change="selectValue(field, $event)"><option v-for="option in field.options" :key="option.value" :value="option.value">{{ option.label }}档</option></select>
                <span class="rx-match">{{ modeInfo(field).matched ? '档位匹配' : `当前 ${modeInfo(field).position}` }}</span>
            </div>
            <p v-if="!semantics.length" class="rx-note">当前车型没有可显示的模式档位。</p>
        </div>
        <details class="rx-diagnostics"><summary>串口与帧诊断</summary><div>
            <span>接收机串口：{{ snapshot?.uart?.irqBytes ?? '—' }} 字节</span><span>有效通道帧：{{ snapshot?.link?.rcFrames ?? '—' }}</span>
            <span>溢出错误：{{ snapshot?.uart?.overrun ?? '—' }}</span><span>帧错误：{{ snapshot?.uart?.framing ?? '—' }}</span>
            <span>噪声错误：{{ snapshot?.uart?.noise ?? '—' }}</span><span>CRC 校验错误：{{ snapshot?.link?.crcErrors ?? '—' }}</span>
            <span>接收重启：{{ snapshot?.link?.restarts ?? '—' }}</span><span>统计帧超时：{{ snapshot?.link?.statsTimeouts ?? '—' }}</span>
        </div><p>计数包含开机和接收机上电阶段。比较数值是否持续增加，再判断当前链路是否仍有错误；未收到新数据时显示的是最后一次记录。</p></details>
    </div>
</template>

<style scoped>
.receiver-panel { --rx-accent: #d69b05; }
.rx-connect { display: flex; align-items: center; gap: 9px; flex-wrap: wrap; background: #fff; padding: 13px 16px; border: 1px solid #d8dfe3; border-radius: 5px; font-size: 12px; }
.rx-connect > span { color: #677780; margin-left: auto; }
.rx-connect select { max-width: 300px; }
select { background: #fff; border: 1px solid #b8c5cb; border-radius: 3px; padding: 7px 9px; font: inherit; color: #27313a; min-width: 90px; }
.rx-stats { display: grid; grid-template-columns: 1.5fr 1fr 1.2fr 1.2fr; gap: 12px; margin: 14px 0 20px; }
.rx-stats > div { background: #fff; border: 1px solid #d8dfe3; border-radius: 5px; padding: 14px 16px; }
.rx-stats small { display: block; font-size: 11px; color: #6c7b84; margin-bottom: 9px; }
.rx-stats strong { font-size: 19px; font-variant-numeric: tabular-nums; }
.rx-state strong { font-size: 16px; }
.rx-state i { display: inline-block; width: 8px; height: 8px; background: #8b969a; border-radius: 50%; margin-right: 8px; }
.rx-state.live i { background: #36a276; box-shadow: 0 0 0 3px #36a27620; }
.rx-state.live strong { color: #227d5a; }
.rx-tabs { display: flex; border-bottom: 1px solid #cbd4d9; gap: 8px; }
.rx-tabs button { background: transparent; border: 0; border-bottom: 3px solid transparent; padding: 10px 20px; cursor: pointer; font: inherit; font-size: 13px; color: #697983; }
.rx-tabs button.selected { border-bottom-color: #e7ab13; color: #28333a; font-weight: 700; }
.rx-context { color: #687780; font-size: 12px; line-height: 1.7; margin: 12px 0 16px; }
.rx-card { background: #fff; border: 1px solid #d8dfe3; border-radius: 5px; overflow: hidden; }
.rx-card-heading { padding: 16px 20px; border-bottom: 1px solid #e6ebee; display: flex; justify-content: space-between; align-items: center; gap: 12px; }
h2 { margin: 0 0 6px; font-size: 15px; }
.rx-card-heading p, .rx-footnote { font-size: 12px; color: #71818b; margin: 0; line-height: 1.65; }
.rx-badge { white-space: nowrap; font-size: 11px; color: #776329; background: #fff5d8; padding: 5px 9px; border-radius: 3px; }
.rx-channel, .rx-channel-head { display: grid; grid-template-columns: 48px 48px minmax(130px, 1fr) minmax(245px, 1.25fr); align-items: center; gap: 14px; margin: 0 20px; padding: 9px 0; border-bottom: 1px solid #edf0f2; font-size: 12px; }
.rx-channel-head { color: #87949c; font-size: 10px; padding-top: 14px; }
.rx-channel-head > div { display: flex; justify-content: space-between; }
.rx-channel output { font-variant-numeric: tabular-nums; color: #5b6d78; text-align: right; }
.rx-track { position: relative; height: 14px; background: #edf0f2; border-radius: 3px; }
.rx-fill { height: 100%; border-radius: 3px 0 0 3px; background: #efc04c; }
.rx-mid { position: absolute; height: 22px; top: -4px; left: 50.03%; border-left: 1px dashed #aab6bd; z-index: 1; }
.rx-marker { position: absolute; width: 3px; height: 18px; background: #82630e; top: -2px; transform: translateX(-2px); border-radius: 2px; }
.rx-uses { display: flex; flex-wrap: wrap; gap: 4px; }
.rx-uses span { background: #f1f5f7; padding: 2px 6px; border-radius: 3px; color: #546873; font-size: 11px; }
.rx-uses em { font-style: normal; color: #a1acb3; font-size: 11px; }
.rx-channel.unused strong { color: #89969e; }
.rx-channel.stale .rx-fill { background: #cbd3d8; }
.rx-channel.stale .rx-marker { background: #86969f; }
.rx-footnote { padding: 12px 20px; }
.rx-note, .rx-warning { background: #fff8e4; color: #806718; padding: 12px 16px; font-size: 12px; line-height: 1.7; }
.rx-warning { background: #fff0ed; color: #a04435; }
.rx-mapping { display: grid; grid-template-columns: minmax(240px, .8fr) minmax(400px, 1.6fr); gap: 16px; align-items: start; }
.rx-binding, .rx-function, .rx-mode { display: grid; align-items: center; gap: 12px; padding: 11px 20px; border-bottom: 1px solid #edf0f2; font-size: 12px; }
.rx-binding { grid-template-columns: 1fr 110px; }
.rx-function { grid-template-columns: 1fr 126px 59px; }
.rx-function small { display: block; margin-top: 4px; color: #84929b; font-size: 10px; }
.rx-invert { display: flex; align-items: center; gap: 4px; white-space: nowrap; }
input[type=checkbox] { accent-color: #c99616; }
.modified { background: #fffae9; }
.rx-mode { grid-template-columns: 1.2fr 1fr 1fr 1fr; }
.rx-mode.matched { background: #f1f8f5; }
.rx-mode.matched .rx-match { color: #2b8362; font-weight: 700; }
.rx-match { color: #88949c; font-size: 11px; }
.rx-ranges { display: flex; margin: 18px 20px 0; background: #f3f5f6; border: 1px solid #d8dfe3; border-radius: 3px; }
.rx-ranges span { width: 33.33%; padding: 12px; text-align: center; font-size: 12px; }
.rx-ranges span + span { border-left: 1px solid #d8dfe3; }
.rx-diagnostics { margin-top: 18px; font-size: 12px; color: #71808a; }
.rx-diagnostics summary { cursor: pointer; padding: 10px 0; }
.rx-diagnostics > div { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; background: #e1e7eb; padding: 16px; border-radius: 4px; }
.rx-diagnostics p { line-height: 1.7; }
@media (max-width: 1150px) { .rx-mapping { grid-template-columns: 1fr; } .rx-connect > span { margin-left: 0; } .rx-channel, .rx-channel-head { grid-template-columns: 40px 40px minmax(125px, 1fr) 240px; gap: 10px; } }
</style>
