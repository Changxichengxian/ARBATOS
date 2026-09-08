<script setup lang="ts">
import { computed, onMounted, ref } from "vue";
import { rpc } from "../api";
import TelemetryChart from "./TelemetryChart.vue";

type Download = { name: string; mime: string; data: string; bytes: number };
type Capabilities = { maxInputBytes: number; audio: { output: string; wav: boolean; ffmpeg: boolean; ffmpegSuffixes: string[] }; sdlog: { extensions: string[]; maxCurvePoints: number } };
type SdlogResult = Download & { format: string; inputBytes: number; blocks: number; compressedBlocks: number; records: number; series: { key: string; name: string; count: number; fields: string[] }[]; unknown: { tag: number; tag_name: string; count: number }[]; unknownKinds: number; unknownRecords: number; curves: { key: string; name: string; field: string; points: [number, number][] }[] };
type AudioResult = Download & { source: string; format: string; sampleRateHz: number; durationSeconds: number };

const props = defineProps<{ disabled?: boolean }>();
const emit = defineEmits<{ error: [message: string]; busy: [active: boolean] }>();
const capabilities = ref<Capabilities | null>(null);
const sdlog = ref<SdlogResult | null>(null);
const audio = ref<AudioResult | null>(null);
const working = ref(false);
const selectedCurve = ref(0);
const curve = computed(() => sdlog.value?.curves[selectedCurve.value]);
const curveSamples = computed(() => (curve.value?.points ?? []).map(([time, value], index) => ({
    seq: index + 1, time: time / 1000, values: [value],
})));

const maxText = computed(() => capabilities.value ? `${Math.floor(capabilities.value.maxInputBytes / 1024 / 1024)} MiB` : "24 MiB");

function failure(error: unknown) {
    emit("error", error instanceof Error ? error.message : "本地工具执行失败");
}
function base64(bytes: ArrayBuffer) {
    const data = new Uint8Array(bytes);
    const size = 0x8000;
    let text = "";
    for (let start = 0; start < data.length; start += size)
        text += String.fromCharCode(...data.subarray(start, start + size));
    return btoa(text);
}
function download(item: Download) {
    const text = atob(item.data);
    const bytes = Uint8Array.from(text, (ch) => ch.charCodeAt(0));
    const url = URL.createObjectURL(new Blob([bytes], { type: item.mime }));
    const link = document.createElement("a");
    link.href = url;
    link.download = item.name;
    document.body.appendChild(link);
    link.click();
    link.remove();
    window.setTimeout(() => URL.revokeObjectURL(url), 0);
}
async function select(event: Event, method: "tools.sdlog" | "tools.audio") {
    const input = event.target as HTMLInputElement;
    const file = input.files?.[0];
    input.value = "";
    if (!file || working.value || props.disabled) return;
    if (capabilities.value && file.size > capabilities.value.maxInputBytes) {
        emit("error", `文件超过 ${maxText.value} 限制`);
        return;
    }
    working.value = true;
    if (method === "tools.sdlog") sdlog.value = null;
    else audio.value = null;
    emit("busy", true);
    try {
        const params = { name: file.name, data: base64(await file.arrayBuffer()) };
        if (method === "tools.sdlog") {
            sdlog.value = await rpc<SdlogResult>(method, params);
            selectedCurve.value = 0;
        }
        else audio.value = await rpc<AudioResult>(method, params);
    } catch (error) {
        failure(error);
    } finally {
        working.value = false;
        emit("busy", false);
    }
}
onMounted(async () => {
    try { capabilities.value = await rpc<Capabilities>("tools.capabilities"); }
    catch (error) { failure(error); }
});
</script>

<template>
    <section class="panel tools-panel">
        <div class="panel-title"><div><span>本地工具</span><small>上传内容只在本机服务内解析，单个文件最多 {{ maxText }}</small></div></div>
        <div class="tool-actions">
            <label class="button upload" :class="{ disabled: disabled || working }">解析 SD 日志<input type="file" accept=".bin,.lz4" :disabled="disabled || working" @change="select($event, 'tools.sdlog')" /></label>
            <label class="button upload" :class="{ disabled: disabled || working }">歌曲转 U8<input type="file" accept=".wav,.wave,.mp3,.aac,.m4a,.flac,.ogg,.opus" :disabled="disabled || working" @change="select($event, 'tools.audio')" /></label>
            <span v-if="working" class="muted">正在本机处理…</span>
        </div>
        <p v-if="capabilities" class="muted">歌曲输出：{{ capabilities.audio.output }}。支持 PCM WAV{{ capabilities.audio.ffmpegSuffixes.length ? '、' + capabilities.audio.ffmpegSuffixes.map(item => item.slice(1).toUpperCase()).join('、') : '' }}。</p>
        <div v-if="audio" class="tool-result"><b>已转换 {{ audio.name }}</b><span>{{ audio.format }}，{{ audio.durationSeconds }} 秒，{{ (audio.bytes / 1024).toFixed(1) }} KiB（{{ audio.source }}）</span><button class="button" @click="download(audio)">下载 .u8</button></div>
        <div v-if="sdlog" class="tool-result"><b>日志已解析：{{ sdlog.blocks }} 块、{{ sdlog.records }} 条记录，压缩块 {{ sdlog.compressedBlocks }}</b><span>{{ sdlog.series.length }} 组数据；导出 {{ (sdlog.bytes / 1024).toFixed(1) }} KiB；曲线预览每组最多 {{ capabilities?.sdlog.maxCurvePoints ?? 1200 }} 点。</span><button class="button" @click="download(sdlog)">{{ sdlog.mime === 'application/zip' ? '下载各组 CSV（ZIP）' : '下载 CSV' }}</button><ul><li v-for="series in sdlog.series.slice(0, 8)" :key="series.key">{{ series.name }}：{{ series.count }} 条，{{ series.fields.join('、') }}</li></ul><p v-if="sdlog.unknownRecords" class="muted">未识别记录 {{ sdlog.unknownRecords }} 条、{{ sdlog.unknownKinds }} 种：{{ sdlog.unknown.map(item => `${item.tag_name} × ${item.count}`).join('；') }}<template v-if="sdlog.unknownKinds > sdlog.unknown.length">；其余未展开</template></p></div>
        <div v-if="sdlog?.curves.length" class="log-curve">
            <label class="field"><span>日志曲线预览（离线抽样）</span>
                <select v-model.number="selectedCurve" aria-label="日志曲线">
                    <option v-for="(item, index) in sdlog.curves" :key="`${item.key}.${item.field}`" :value="index">{{ item.name }} · {{ item.field }}</option>
                </select>
            </label>
            <small class="muted">预览前 {{ sdlog.curves.length }} 个数值字段；导出文件包含全部已解析数据。</small>
            <TelemetryChart :key="`${sdlog.name}.${selectedCurve}`" :samples="curveSamples" :channels="1" offline offline-label="SD 日志离线抽样" />
        </div>
    </section>
</template>

<style scoped>
.tools-panel { padding: 16px; }.panel-title div { display: flex; flex-direction: column; gap: 4px; }.tool-actions { display: flex; align-items: center; flex-wrap: wrap; gap: 9px; margin: 13px 0; }.upload.disabled { pointer-events: none; opacity: .55; }.muted { color: #778389; font-size: 12px; }.tool-result { display: flex; align-items: center; flex-wrap: wrap; gap: 9px; margin-top: 12px; padding: 11px; border: 1px solid #e4e9ea; background: #fbfcfc; font-size: 12px; }.tool-result span { color: #59666b; }.tool-result ul { width: 100%; max-height: 120px; margin: 0; overflow: auto; padding-left: 20px; color: #59666b; }.tool-result p { width: 100%; margin: 0; }
</style>
