<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from "vue";
import { rpc } from "../api";

type ImuLast = Record<string, number | string> & {
  target: string; state: number; id: number; seq: number; flags: number;
  temp_mC: number; target_mC: number; roll_mdeg: number; pitch_mdeg: number; yaw_mdeg: number;
  gx_mdps: number; gy_mdps: number; gz_mdps: number; ax_mmps2: number; ay_mmps2: number; az_mmps2: number;
  mapX: number; mapY: number; mapZ: number;
};
type ImuSnapshot = { last: ImuLast | null; fresh: boolean; healthy: boolean; safeToStart: boolean; accepted: number; dropped: number; invalid: number; restarts: number; observation: { seconds: number; angleDriftDegPerMin: Record<string, number>; gyroMeanDegPerSec: Record<string, number> } | null };
type Mount = { axes?: string[]; revision?: string; configured: boolean; requiresBuild: boolean; unsupported?: boolean; reason?: string };
const props = defineProps<{ target: string; connected: boolean; imu?: ImuSnapshot }>();
const emit = defineEmits<{ status: [unknown]; error: [unknown] }>();
const mount = ref<Mount | null>(null);
const axes = ref(["+X", "+Y", "+Z"]);
const saving = ref(false);
const commanding = ref(false);
const pendingCalibration = ref<{ target: string; calibrateId: number; cancelId?: number; seq: number; restarts: number } | null>(null);
let requestId = crypto.getRandomValues(new Uint32Array(1))[0] || 1;
let mountRequest = 0;
let renewTimer: number | undefined;
let alive = true;
let connectionEpoch = 0;
let renewing = false;
const last = computed(() => props.imu?.fresh ? props.imu.last : null);
const targetMatches = computed(() => last.value?.target === props.target);
const calibrationActive = computed(() => !!last.value && [1, 2, 3].includes(last.value.state));
const fmt = (value: number | undefined, digits = 2) => value === undefined ? "—" : value.toFixed(digits);
function nextRequestId() { let id = requestId >>> 0; do { id = id === 0xffffffff ? 1 : id + 1; } while (id === 0 || id === last.value?.id); requestId = id; return id; }
async function loadMount() { const target = props.target; const request = ++mountRequest; mount.value = null; if (!target) return; try { const result = await rpc<Mount>("imu.mount", { target }); if (request !== mountRequest || target !== props.target) return; mount.value = result; axes.value = result.axes ? [...result.axes] : ["+X", "+Y", "+Z"]; } catch (e) { if (request === mountRequest && target === props.target) emit("error", e); } }
function seqAdvanced(before: number, after: number) { const delta = (after - before) >>> 0; return delta > 0 && delta < 0x80000000; }
const calibrationOutcome = computed(() => {
  const pending = pendingCalibration.value;
  if (!pending || !last.value || !props.imu || pending.target !== props.target || last.value.target !== pending.target || props.imu.restarts !== pending.restarts) return "";
  const held = (last.value.flags & 4) !== 0;
  if (last.value.state === 4 && last.value.id === pending.calibrateId && seqAdvanced(pending.seq, last.value.seq)) return "请求已保存：收到匹配请求编号且序号前进的 Flash 回读结果。";
  if (last.value.state === 5 && last.value.id === pending.calibrateId) return held ? "本次校准失败。设备仍保持输出锁定，需重启后解除。" : "本次校准未开始，Flash 未写入。";
  if (last.value.state === 6 && last.value.id === pending.cancelId) return held ? "本次校准已取消。设备仍保持输出锁定，需重启后解除。" : "取消请求已处理，Flash 未写入。";
  return "";
});
async function command(action: "watch" | "stop" | "calibrate" | "cancel" | "reset") {
  const id = nextRequestId();
  const target = props.target;
  const epoch = connectionEpoch;
  const before = props.imu;
  const pending = pendingCalibration.value;
  const manual = action === "calibrate" || action === "cancel";
  if (manual) commanding.value = true;
  try {
    const response = await rpc<{ status: { imu?: ImuSnapshot } }>("serial.imu", { action, id, target });
    if (!alive || epoch !== connectionEpoch || target !== props.target) return;
    if (action === "calibrate" && before?.last) {
      pendingCalibration.value = { target, calibrateId: id, seq: before.last.seq, restarts: before.restarts };
    }
    if (action === "cancel" && pending?.target === target) pendingCalibration.value = { ...pending, cancelId: id };
    emit("status", response);
  } catch (e) {
    if (alive && epoch === connectionEpoch && target === props.target) emit("error", e);
  } finally {
    if (manual) commanding.value = false;
  }
}
async function saveMount() { const target = props.target; const current = mount.value; if (!current?.revision || current.unsupported) return; saving.value = true; try { const result = await rpc<Mount>("imu.mount.update", { target, axes: axes.value, revision: current.revision }); if (target === props.target && current === mount.value) mount.value = result; } catch (e) { if (target === props.target) emit("error", e); } finally { if (target === props.target) saving.value = false; } }
async function renew() {
  if (!props.connected || renewing) return;
  renewing = true;
  try { await command("watch"); } finally { renewing = false; }
}
watch(() => props.connected, (connected) => { connectionEpoch++; if (renewTimer) window.clearInterval(renewTimer); if (connected) { renew(); renewTimer = window.setInterval(renew, 1000); } }, { immediate: true });
watch(() => props.target, () => { pendingCalibration.value = null; saving.value = false; loadMount(); }, { immediate: true });
watch(() => props.connected, (connected) => { if (!connected) pendingCalibration.value = null; });
onBeforeUnmount(() => { alive = false; if (renewTimer) window.clearInterval(renewTimer); if (props.connected) command("stop"); });
</script>

<template>
  <section class="imu-panel">
    <div class="title-row"><div><p class="eyebrow">IMU</p><h1>姿态与零偏</h1><p>观测只显示串口实际收到的遥测。手动校准会保持输出锁定，直到设备重启。</p></div><span :class="['imu-state', { good: imu?.healthy && imu?.safeToStart }]">{{ !connected ? '串口未连接' : !imu?.fresh ? '等待新遥测' : imu.healthy ? (imu.safeToStart ? '健康且可开始' : '健康，未进入安全模式') : '遥测异常' }}</span></div>
    <p v-if="connected && !imu?.fresh" class="imu-unavailable">尚未收到 @IMU2。当前固件可能未提供此接口；外置 HI14 姿态源不支持板载零偏写入。</p>
    <p v-if="connected && last && !targetMatches" class="imu-unavailable">当前设备为 {{ last.target }}，与所选车型 {{ target }} 不同。切换到对应车型后再校准。</p>
    <div class="imu-grid"><article><small>温度 / 设备</small><strong>{{ last ? `${fmt(last.temp_mC / 1000)} °C` : '—' }}</strong><span>{{ last?.target ?? '—' }} · 目标 {{ last ? `${fmt(last.target_mC / 1000)} °C` : '—' }}</span></article><article><small>横滚 / 俯仰 / 航向</small><strong>{{ last ? `${fmt(last.roll_mdeg / 1000)} / ${fmt(last.pitch_mdeg / 1000)} / ${fmt(last.yaw_mdeg / 1000)} °` : '—' }}</strong><span>{{ last ? `角速度 X ${fmt(last.gx_mdps/1000)} / Y ${fmt(last.gy_mdps/1000)} / Z ${fmt(last.gz_mdps/1000)} °/s` : '—' }}</span></article><article><small>加速度</small><strong>{{ last ? `${fmt(last.ax_mmps2/1000)} / ${fmt(last.ay_mmps2/1000)} / ${fmt(last.az_mmps2/1000)} m/s²` : '—' }}</strong><span>map {{ last ? `${last.mapX} / ${last.mapY} / ${last.mapZ}` : '—' }}</span></article><article><small>校准状态</small><strong>{{ last ? ['空闲','预热','采样','写入 Flash','已保存','失败','已取消'][last.state] : '—' }}</strong><span>请求 {{ last?.id ?? '—' }} · 序号 {{ last?.seq ?? '—' }}</span></article><article><small>帧统计</small><strong>{{ imu?.accepted ?? 0 }}</strong><span>间隙 {{ imu?.dropped ?? 0 }} · 异常 {{ imu?.invalid ?? 0 }} · 重启 {{ imu?.restarts ?? 0 }}</span></article></div>
    <div class="imu-card"><h2>方向配置</h2><p v-if="mount?.unsupported">{{ mount.reason }}</p><p v-else>这里保存的是车型文件，保存后需重新编译并烧录才会在设备上生效。实时回报的 mapX/mapY/mapZ 可用来与文件对照。姿态数值只供观察；保存前仍需手动前后翻转、左右翻转和绕轴转动，核对横滚、俯仰、航向的方向是否符合实车。</p><div v-if="!mount?.unsupported" class="axis-row" v-for="(_, index) in axes" :key="index"><label>INS {{ ['X','Y','Z'][index] }}</label><select v-model="axes[index]"><option v-for="axis in ['+X','-X','+Y','-Y','+Z','-Z']" :key="axis">{{ axis }}</option></select><span v-if="last">当前 {{ [last.mapX, last.mapY, last.mapZ][index] }}</span></div><button v-if="!mount?.unsupported" class="button primary" :disabled="saving || !mount?.revision" @click="saveMount">保存方向到车型文件</button></div>
    <div class="imu-card"><h2>零漂观测</h2><p>先把设备静止放稳，再看漂移速度；方向是否正确仍需按上面的翻转和绕轴动作人工核对。</p><p v-if="!imu?.observation">至少需要两帧有效遥测；离线或数据过期时不显示推测数值。</p><template v-else><p>当前连续观测窗口 {{ fmt(imu.observation.seconds, 1) }} 秒。</p><div class="metrics"><span>横滚 {{ fmt(imu.observation.angleDriftDegPerMin.roll, 3) }} °/min</span><span>俯仰 {{ fmt(imu.observation.angleDriftDegPerMin.pitch, 3) }} °/min</span><span>航向 {{ fmt(imu.observation.angleDriftDegPerMin.yaw, 3) }} °/min</span><span v-for="axis in ['x','y','z']" :key="axis">角速度 {{ axis.toUpperCase() }} {{ fmt(imu.observation.gyroMeanDegPerSec[axis], 3) }} °/s</span></div></template><button class="button" @click="command('reset')">复位观测统计</button></div>
    <div class="imu-card danger">
      <h2>手动校准零偏</h2>
      <p>仅在“健康且可开始”时可发送校准命令。过程中请保持设备静止；校准会保持输出锁定，必须重启后才会恢复正常输出。校准失败后可在保持停机的状态下重新开始。</p>
      <p v-if="calibrationOutcome" class="saved">{{ calibrationOutcome }}</p>
      <button class="button primary" :disabled="commanding || !connected || !targetMatches || !imu?.healthy || !imu?.safeToStart" @click="command('calibrate')">开始校准并写 Flash</button>
      <button class="button" :disabled="commanding || !connected || !targetMatches || !calibrationActive" @click="command('cancel')">取消校准</button>
    </div>
  </section>
</template>

<style scoped>
.imu-panel { display: grid; gap: 16px; }.imu-state { align-self: center; background: #fff0ed; color: #964333; padding: 7px 10px; border-radius: 4px; font-size: 12px; }.imu-state.good,.saved { color: #207253; background: #edf8f1; }.imu-unavailable { margin:0; padding:12px; background:#fff8e4; color:#806718; font-size:12px; }.imu-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }.imu-grid article,.imu-card { background:#fff; border:1px solid #d8dfe3; border-radius:5px; padding:16px; }.imu-grid small,.imu-grid span { display:block; color:#71808a; font-size:12px; margin-top:6px; }.imu-grid strong { font-size:20px; }.imu-card h2 { margin:0 0 6px; font-size:16px; }.imu-card p { color:#687780; font-size:12px; line-height:1.7; }.saved { padding:8px; }.axis-row { display:flex; gap:12px; align-items:center; margin:8px 0; }.axis-row label { width:60px; }.axis-row span { color:#687780; font-size:12px; }.metrics { display:flex; gap:14px; flex-wrap:wrap; margin:12px 0; font-variant-numeric:tabular-nums; }.danger { border-color:#e5bdad; }.button + .button { margin-left:8px; } @media(max-width:800px){ .imu-grid { grid-template-columns:1fr; } }
</style>
