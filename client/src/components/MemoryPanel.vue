<script setup lang="ts">
import { computed, ref, watch } from "vue";
import { rpc } from "../api";

type Region = {
    name: string; capacity: number; used: number; free: number; used_percent: number;
    is_alias: boolean; unattributed_or_alignment: number;
    categories?: { name: string; bytes: number }[];
};
type Module = { name: string; category: string; flash: number; ram: number; symbols: string[] };
type Allocation = { name: string; bytes: number; address: number; region: string; section: string; kind: string };
type Report = {
    available: boolean; target: string; reason?: string; regions: Region[]; modules: Module[]; allocations?: Allocation[];
    artifacts?: { elf: { path: string; modified_at: string; sha256: string } };
    limitations: string[];
};
const props = defineProps<{ target: string }>();
const report = ref<Report | null>(null);
const loading = ref(false);
const error = ref("");
const query = ref("");
const showAll = ref(false);
const allocationQuery = ref("");
const allocationKind = ref("data");
const allocationCount = ref(20);
const allocations = computed(() => (report.value?.allocations ?? []).filter(item =>
    (!allocationKind.value || item.kind === allocationKind.value) &&
    `${item.name} ${item.region} ${item.section}`.toLowerCase().includes(allocationQuery.value.toLowerCase())));
watch([allocationQuery, allocationKind], () => { allocationCount.value = 20; });
let request = 0;
const regions = computed(() => report.value?.regions.filter((r) => !r.is_alias && r.capacity > 0 && /^(FLASH|RAM|ITCM|DTCM|SRAM\d*)$/.test(r.name)) ?? []);
const otherRegions = computed(() => report.value?.regions.filter((r) => !r.is_alias && !regions.value.includes(r)) ?? []);
const modules = computed(() => (report.value?.modules ?? []).filter((m) =>
    `${m.name} ${m.category} ${m.symbols.join(" ")}`.toLowerCase().includes(query.value.toLowerCase())));
const visibleModules = computed(() => showAll.value ? modules.value : modules.value.slice(0, 15));
const buildTime = computed(() => report.value?.artifacts
    ? new Date(report.value.artifacts.elf.modified_at).toLocaleString() : "");
function size(bytes: number) {
    if (bytes >= 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
    return bytes >= 1024 ? `${(bytes / 1024).toFixed(1)} KiB` : `${bytes} B`;
}
function color(name: string) {
    const palette = ["#246a94", "#c38618", "#418573", "#8565a0", "#b45d58", "#528b9c", "#63704a"];
    let hash = 0;
    for (const c of name) hash = (hash * 31 + c.charCodeAt(0)) >>> 0;
    return palette[hash % palette.length];
}
function shortName(name: string) { return name.match(/\(([^)]+)\)$/)?.[1] ?? name; }
async function refresh() {
    const current = ++request;
    report.value = null;
    error.value = "";
    if (!props.target) { loading.value = false; return; }
    loading.value = true;
    try {
        const value = await rpc<Report>("firmware.memory", { target: props.target });
        if (current === request) report.value = value;
    } catch (e) {
        if (current === request) error.value = e instanceof Error ? e.message : String(e);
    } finally { if (current === request) loading.value = false; }
}
watch(() => props.target, () => { query.value = ""; showAll.value = false; void refresh(); }, { immediate: true });
</script>

<template>
    <div class="title-row">
        <div><p class="eyebrow">固件资源</p><h1>内存与 Flash</h1>
            <p>{{ target }} · 查看本地最近一次构建的实际分配。</p></div>
        <button class="button" :disabled="loading || !target" @click="refresh">{{ loading ? "读取中…" : "刷新构建结果" }}</button>
    </div>
    <p v-if="error" role="alert" class="memory-error">{{ error }}</p>
    <section v-if="report && !report.available" class="panel"><h2>尚无可用报告</h2><p>{{ report.reason }}</p><p class="muted">到“编译与烧录”完成构建后再刷新。</p></section>
    <template v-if="report?.available">
        <section class="panel memory-context">
            <strong>构建时间 {{ buildTime }}</strong>
            <p>这是编译产物的空间分配，尚未核对车上固件。修改配置后需要重新构建；任务栈和缓冲区按预留大小计入，剩余空间不等于运行时栈余量。</p>
            <details><summary>固件文件与校验值</summary><code>{{ report.artifacts?.elf.path }}</code><code>SHA-256 {{ report.artifacts?.elf.sha256 }}</code></details>
        </section>
        <div class="memory-regions">
            <section v-for="region in regions" :key="region.name" class="panel memory-region">
                <div class="memory-heading"><h2>{{ region.name }}</h2><strong>{{ region.used_percent }}%</strong></div>
                <p><b>{{ size(region.used) }}</b> 已分配 / {{ size(region.capacity) }} 可用区域容量</p>
                <div class="memory-bar" role="img" :aria-label="`${region.name} 已分配 ${size(region.used)}，剩余 ${size(region.free)}`">
                    <span v-for="group in region.categories ?? []" :key="group.name"
                        :style="{ width: `${100 * group.bytes / region.capacity}%`, background: color(group.name) }"
                        :title="`${group.name}：${size(group.bytes)}`" />
                    <span v-if="!region.categories?.length" :style="{ width: `${region.used_percent}%`, background: '#246a94' }" />
                </div>
                <div class="memory-legend"><span v-for="group in region.categories ?? []" :key="group.name"><i :style="{ background: color(group.name) }" />{{ group.name }} {{ size(group.bytes) }}</span></div>
                <p class="memory-free">剩余 {{ size(region.free) }} <small>（{{ region.free.toLocaleString() }} 字节）</small></p>
            </section>
        </div>
        <section class="panel">
            <div class="memory-heading"><h2>主要占用对象</h2><input v-model="query" aria-label="搜索内存模块" placeholder="搜索模块、文件或变量" /></div>
            <p class="muted">按链接后的对象文件统计；初始化数据会分别占用 Flash 和 RAM。保留最大的 80 个对象，未归属内容仍计入区域总量。</p>
            <div class="memory-table-wrap"><table class="memory-table"><thead><tr><th>对象 / 用途</th><th>Flash</th><th>RAM</th></tr></thead>
                <tbody><tr v-for="module in visibleModules" :key="module.name"><td><strong :title="module.name">{{ shortName(module.name) }}</strong><small>{{ module.category }}</small><small v-if="module.symbols.length" class="memory-symbols">{{ module.symbols.slice(0, 3).join(' · ') }}</small></td><td>{{ size(module.flash) }}</td><td>{{ size(module.ram) }}</td></tr></tbody>
            </table></div>
            <p v-if="!modules.length" class="muted">没有匹配的对象。</p>
            <button v-if="modules.length > 15" class="button" @click="showAll = !showAll">{{ showAll ? "收起" : `展开 ${modules.length} 个对象` }}</button>
        </section>
        <section class="panel">
            <div class="memory-heading"><h2>具体变量、任务栈与函数</h2><input v-model="allocationQuery" aria-label="搜索具体内存占用" placeholder="例如 stack、LowCmd、SdLog" /></div>
            <p class="muted">逐项显示真实符号尺寸。任务栈、堆和缓冲区显示的是预留容量；这里不能判断运行时用了多少栈。</p>
            <label>查看范围 <select v-model="allocationKind" aria-label="内存明细类型"><option value="data">变量与预留空间</option><option value="code">函数代码</option><option value="">全部</option></select></label>
            <div class="memory-table-wrap"><table class="memory-table"><thead><tr><th>名称 / 所属段</th><th>区域</th><th>字节</th><th>运行地址</th></tr></thead>
                <tbody><tr v-for="item in allocations.slice(0, allocationCount)" :key="`${item.address}-${item.section}-${item.name}`"><td><strong>{{ item.name }}</strong><small>{{ item.section }}</small></td><td>{{ item.region }}</td><td>{{ item.bytes.toLocaleString() }} B</td><td><code>0x{{ item.address.toString(16).padStart(8, '0') }}</code></td></tr></tbody>
            </table></div>
            <p v-if="!allocations.length" class="muted">没有匹配的符号。</p>
            <button v-if="allocations.length > allocationCount" class="button" @click="allocationCount += 50">继续显示（共 {{ allocations.length }} 项）</button>
        </section>
        <details class="panel"><summary>统计范围与其他地址窗口</summary><p v-for="note in report.limitations" :key="note" class="muted">{{ note }}</p><p class="muted">同一物理区域的别名只展示一次。容量采用当前链接配置；保留给标定或引导程序的空间不一定属于应用可用区域。</p>
            <p v-for="region in otherRegions" :key="region.name" class="muted">{{ region.name }}：链接器声明 {{ size(region.capacity) }}，分配 {{ size(region.used) }}。这些可能是外部地址窗口或链接辅助区，不能据此认定板上装有对应容量的内存。</p>
        </details>
    </template>
</template>

<style scoped>
.memory-regions { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 18px; }
.memory-heading { display: flex; justify-content: space-between; align-items: center; gap: 16px; }
.memory-heading h2 { margin: 0; }
.memory-heading input { max-width: 300px; padding: 9px 12px; border: 1px solid #c7d0d5; border-radius: 5px; }
.memory-bar { display: flex; height: 20px; border-radius: 6px; overflow: hidden; background: #e5eaed; margin: 18px 0 12px; }
.memory-bar span { flex-shrink: 0; }
.memory-legend { display: flex; flex-wrap: wrap; gap: 8px 16px; font-size: 12px; color: #586975; }
.memory-legend i { display: inline-block; width: 9px; height: 9px; border-radius: 2px; margin-right: 5px; }
.memory-free { color: #356958; margin-bottom: 0; }
.memory-context code { display: block; overflow-wrap: anywhere; margin-top: 10px; font-size: 12px; }
.memory-context p { line-height: 1.7; }
.memory-table-wrap { overflow-x: auto; }
.memory-table { width: 100%; border-collapse: collapse; margin: 14px 0; }
.memory-table th, .memory-table td { padding: 12px 8px; border-bottom: 1px solid #e5eaed; text-align: right; white-space: nowrap; }
.memory-table th:first-child, .memory-table td:first-child { text-align: left; white-space: normal; }
.memory-table small { display: block; color: #6b7881; margin-top: 4px; }
.memory-symbols { max-width: 500px; overflow-wrap: anywhere; }
.memory-error { color: #a33535; }
</style>
