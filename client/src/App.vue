<script setup lang="ts">
import {
    computed,
    nextTick,
    onBeforeUnmount,
    onMounted,
    ref,
    watch,
} from "vue";
import { rpc } from "./api";
import RobotConfigPanel from "./components/RobotConfigPanel.vue";
import TelemetryChart from "./components/TelemetryChart.vue";
import ConfigValuesPanel from "./components/ConfigValuesPanel.vue";
import ToolsPanel from "./components/ToolsPanel.vue";
import PortConfigPanel from "./components/PortConfigPanel.vue";
import ReceiverPanel from "./components/ReceiverPanel.vue";
import ChassisPanel from "./components/ChassisPanel.vue";
import MemoryPanel from "./components/MemoryPanel.vue";
import TaskPlanPanel from "./components/TaskPlanPanel.vue";
import VehicleSetupPanel, { type VehicleSetupCatalog } from "./components/VehicleSetupPanel.vue";
import ImuPanel from "./components/ImuPanel.vue";
import { useSerial } from "./useSerial";
import { useBuild } from "./useBuild";
import { useConfigEditor } from "./useConfigEditor";
import type { Job, Robot } from "./types";

type Tab = "overview" | "robot" | "chassis" | "receiver" | "imu" | "parameters" | "files" | "serial" | "tools" | "build" | "memory";
type Summary = {
    root: string;
    targets: { name: string; board: string; preset: string }[];
    controllers: Controller[];
    services: Service[];
    git: { branch: string; sha: string; dirty: boolean | null };
};
type Controller = {
    name: string;
    domain: string;
    builtin: boolean;
    parameters: Record<
        string,
        {
            default?: number;
            min?: number;
            max?: number;
            unit?: string;
            description?: string;
        }
    >;
    outputs?: number;
    requires?: string[];
};
type Service = {
    symbol: string;
    name: string;
    dependencies: string[];
    available: boolean;
};
type FileData = {
    name: string;
    content: string;
    revision: string;
    encoding: string;
    newline: string;
};
const icon = (name: string) =>
    ({
        overview:
            '<path d="M3 13h7V3H3v10Zm0 8h7v-5H3v5Zm11 0h7V11h-7v10Zm0-18v5h7V3h-7Z"/>',
        robot: '<path d="M12 2v3m-6 4V7h12v2m-3 0v4m-6-4v4m-5 0h16v7H4v-7Zm4 3v1m8-1v1M9 22h6"/>',
        chassis: '<circle cx="12" cy="12" r="6"/><path d="m3 8 5-5m8 0 5 5M3 16l5 5m8 0 5-5"/>',
        files: '<path d="M4 3h10l6 6v12H4V3Zm10 0v6h6M8 14h8M8 18h5"/>',
        serial: '<path d="M6 3v4m12-4v4M8 7h8v5a4 4 0 0 1-8 0V7Zm4 9v5m-3 0h6"/>',
        build: '<path d="m14 4 6 6-9 9H5v-6l9-9Zm-2 2 6 6M4 21h16"/>',
        memory: '<path d="M5 5h14v14H5V5Zm3 3h8v8H8V8ZM8 2v3m8-3v3M8 19v3m8-3v3M2 8h3m-3 8h3m14-8h3m-3 8h3"/>',
        parameters: '<path d="M5 3v18M12 3v18M19 3v18M2 8h6m1 8h6m1-10h6"/>',
        receiver: '<path d="M4 9h16v12H4V9Zm3-6 3 6m7-6-3 6M7 15h4m-2-2v4m6-2h2"/>',
        imu: '<path d="M12 3v3m0 12v3M3 12h3m12 0h3M6 6l2 2m8 8 2 2m0-12-2 2m-8 8-2 2"/><circle cx="12" cy="12" r="5"/>',
        tools: '<path d="M4 4h16v16H4V4Zm4 3v6m4-4v8m4-5v5"/>',
        refresh: '<path d="M20 11a8 8 0 1 0 2 5M20 4v7h-7"/>',
        search: '<circle cx="11" cy="11" r="6"/><path d="m16 16 4 4"/>',
        save: '<path d="M5 3h12l3 3v15H4V3h1Zm2 0v6h8V3m-8 18h10v-8H7v8Z"/>',
        play: '<path d="m8 5 11 7-11 7V5Z"/>',
        pause: '<path d="M7 5h4v14H7V5Zm6 0h4v14h-4V5Z"/>',
        close: '<path d="m6 6 12 12M18 6 6 18"/>',
        terminal: '<path d="m5 7 4 4-4 4m6 2h8"/>',
    })[name] ?? "";

const activeTab = ref<Tab>("overview");
const summary = ref<Summary | null>(null);
const target = ref("");
const robot = ref<Robot | null>(null);
const busy = ref(false);
const toast = ref("");
const error = ref("");
const pendingTab = ref<Tab | null>(null);
const pendingFile = ref("");
const selectedFile = ref("");
const file = ref<FileData | null>(null);
const fileText = ref("");
const dirtyFile = ref(false);
const toolsBusy = ref(false);
const disposed = ref(false);
const configState = useConfigEditor({
    target, activeTab, robot, busy, disposed,
    isInterfaceBusy: () => interfaceBusy.value,
    fail, clearError: () => { error.value = ""; }, showToast,
    refreshSummary: async () => refreshSummary(),
});
const {
    configDraft, dirtyConfig, portOptions, validationPreview, configValues,
    valuesLoading, valueChanges, dirtyValues, changeConfig, loadPortOptions,
    loadValues, changeValue, saveValues, saveConfig, validateConfig, resetDraft,
} = configState;
const anyDirty = computed(() => dirtyConfig.value || dirtyFile.value || dirtyValues.value);
const interfaceBusy = computed(() => busy.value || toolsBusy.value || valuesLoading.value);
const query = ref("");
const editor = ref<HTMLTextAreaElement | null>(null);
const lineNumbers = ref<HTMLPreElement | null>(null);
const cursorLine = ref(1);
const createDialog = ref(false);
const setupCatalog = ref<VehicleSetupCatalog | null>(null);
const setupLoading = ref(false);
const setupError = ref("");
const newTargetName = ref("");
const newTargetSource = ref("");
const serialState = useSerial({ fail, showToast, disposed });
const {
    serial, ports, port, baud, serialFormat, channels, serialCursor, serialLines,
    samples, serialInput, paused, offlineReplay, refreshPorts, refreshStatus, pollSerial,
    serialToggle, observationToggle, sendSerial, clearSerial, importCsv,
} = serialState;
const buildState = useBuild({ target, busy, anyDirty, disposed, fail, showToast });
const { job, confirmFlash, startJob, planFlash, doFlash, cancelJob } = buildState;
let requestId = 0;
const targetSelect = ref<HTMLSelectElement | null>(null);

const nav: { key: Tab; label: string; hint: string }[] = [
    { key: "overview", label: "工程概览", hint: "工作区与车型" },
    { key: "robot", label: "车型与接口", hint: "控制器、服务、串口分配" },
    { key: "chassis", label: "底盘装配", hint: "四轮位置、ID、方向与手转识别" },
    { key: "receiver", label: "接收机", hint: "ELRS 通道、功能与拨杆模式" },
    { key: "imu", label: "IMU", hint: "方向、零漂与手动零偏校准" },
    { key: "parameters", label: "参数与设备", hint: "PID、中位、电机、遥控器" },
    { key: "files", label: "文件编辑", hint: "车型现有文件" },
    { key: "serial", label: "串口与曲线", hint: "真实串口数据" },
    { key: "tools", label: "日志与音频", hint: "日志解析、歌曲转 u8" },
    { key: "build", label: "编译与烧录", hint: "检查、构建、下载" },
    { key: "memory", label: "内存与 Flash", hint: "构建占用、模块分布与剩余空间" },
];
const currentTarget = computed(() =>
    summary.value?.targets.find((x) => x.name === target.value),
);
const filteredFiles = computed(() =>
    (robot.value?.files ?? []).filter((x) =>
        x.name.toLowerCase().includes(query.value.toLowerCase()),
    ),
);
const visibleLines = computed(() => fileText.value.split(/\r?\n/).length);

function showToast(message: string) {
    toast.value = message;
    window.setTimeout(() => {
        if (toast.value === message) toast.value = "";
    }, 3600);
}
function fail(e: unknown) {
    error.value = e instanceof Error ? e.message : String(e);
    showToast(`操作失败：${error.value}`);
}
function svg(name: string) {
    return icon(name);
}
function formatTime(value?: number | string) {
    if (value === undefined || value === "") return "—";
    const numeric = Number(value);
    return Number.isFinite(numeric) && numeric > 1_000_000_000
        ? new Date(numeric * 1000).toLocaleString("zh-CN", { hour12: false })
        : String(value);
}
function jobState(state?: Job["status"]) {
    const states: Record<Job["status"], string> = {
        running: "运行中",
        succeeded: "已完成",
        failed: "失败",
        cancelled: "已取消",
    };
    return state ? states[state] : "等待";
}
async function refreshSummary() {
    try {
        summary.value = await rpc<Summary>("workspace.summary");
        if (
            !target.value ||
            !summary.value.targets.some((x) => x.name === target.value)
        )
            target.value = summary.value.targets[0]?.name ?? "";
    } catch (e) {
        fail(e);
    }
}
async function loadRobot() {
    if (!target.value) return;
    const id = ++requestId;
    busy.value = true;
    try {
        const result = await rpc<Robot>("robot.get", { target: target.value });
        if (id !== requestId || result.name !== target.value) return;
        robot.value = result;
        configDraft.value = JSON.parse(JSON.stringify(result.config ?? {}));
        portOptions.value = null;
        loadPortOptions();
        validationPreview.value = null;
        dirtyConfig.value = false;
        selectedFile.value = "";
        file.value = null;
        fileText.value = "";
        dirtyFile.value = false;
        configValues.value = null;
        valueChanges.value = {};
        if (["parameters", "receiver", "chassis"].includes(activeTab.value)) await loadValues();
    } catch (e) {
        if (id === requestId) fail(e);
    } finally {
        if (id === requestId) busy.value = false;
    }
}
async function chooseTarget(name: string) {
    if (interfaceBusy.value) return;
    if (name === target.value) return;
    if (anyDirty.value) {
        pendingTarget.value = name;
        await nextTick();
        if (targetSelect.value) targetSelect.value.value = target.value;
        return;
    }
    target.value = name;
    await loadRobot();
}
const pendingTarget = ref("");
function cancelUnsaved() {
    pendingTarget.value = "";
    pendingTab.value = null;
    pendingFile.value = "";
    nextTick(() => {
        if (targetSelect.value) targetSelect.value.value = target.value;
    });
}
function leaveUnsaved(save: boolean) {
    if (interfaceBusy.value) return;
    const next = pendingTarget.value;
    const tab = pendingTab.value;
    const name = pendingFile.value;
    if (save) {
        const operation = dirtyValues.value ? saveValues() : dirtyFile.value ? saveFile() : saveConfig();
        operation.then((ok) => {
            if (ok) {
                pendingTarget.value = "";
                pendingTab.value = null;
                pendingFile.value = "";
                proceed(next, tab, name);
            }
        });
        return;
    }
    if (dirtyFile.value && file.value) fileText.value = file.value.content;
    dirtyFile.value = false;
    resetDraft();
    loadPortOptions();
    error.value = "";
    pendingTarget.value = "";
    pendingTab.value = null;
    pendingFile.value = "";
    proceed(next, tab, name);
}
function proceed(next: string, tab: Tab | null, name = "") {
    if (next) {
        target.value = next;
        loadRobot();
    }
    if (tab) activeTab.value = tab;
    if (name) openFile(name);
}
function switchTab(tab: Tab) {
    if (interfaceBusy.value) return;
    if (tab === activeTab.value) return;
    if (anyDirty.value) {
        pendingTab.value = tab;
        return;
    }
    activeTab.value = tab;
}
async function createTarget() {
    const name = newTargetName.value.trim();
    if (!name || !newTargetSource.value) {
        showToast("请输入新车型名称并选择一个模板车型。");
        return;
    }
    try {
        const result = await rpc<{ name: string }>("robot.create", {
            name,
            source: newTargetSource.value,
        });
        createDialog.value = false;
        await refreshSummary();
        await chooseTarget(result.name);
        showToast(`已创建车型 ${result.name}。`);
    } catch (e) {
        fail(e);
    }
}
async function openFile(name: string) {
    if (busy.value) return;
    if (name === selectedFile.value && file.value) return;
    if (dirtyFile.value && name !== selectedFile.value) {
        pendingFile.value = name;
        return;
    }
    const id = ++requestId;
    try {
        const result = await rpc<FileData>("file.read", {
            target: target.value,
            name,
        });
        if (id !== requestId) return;
        file.value = result;
        selectedFile.value = name;
        fileText.value = result.content;
        dirtyFile.value = false;
        await nextTick();
        cursorLine.value = 1;
        if (lineNumbers.value) lineNumbers.value.scrollTop = 0;
        editor.value?.focus({ preventScroll: true });
        editor.value?.setSelectionRange(0, 0);
        if (editor.value) editor.value.scrollTop = 0;
    } catch (e) {
        fail(e);
    }
}
async function saveFile(): Promise<boolean> {
    if (!file.value) return false;
    busy.value = true;
    try {
        const result = await rpc<FileData>("file.write", {
            target: target.value,
            name: file.value.name,
            content: fileText.value,
            revision: file.value.revision,
        });
        file.value = result;
        fileText.value = result.content;
        dirtyFile.value = false;
        error.value = "";
        // 文件和表单共用同一份配置；保存后同步修订号与解析结果。
        const updated = await rpc<Robot>("robot.get", { target: target.value });
        robot.value = updated;
        configDraft.value = JSON.parse(JSON.stringify(updated.config ?? {}));
        loadPortOptions();
        validationPreview.value = null;
        showToast("文件已保存。");
        return true;
    } catch (e) {
        fail(e);
        return false;
    } finally {
        busy.value = false;
    }
}
async function openCreateDialog() {
    createDialog.value = true;
    setupError.value = "";
    if (setupCatalog.value || setupLoading.value) return;
    setupLoading.value = true;
    try {
        setupCatalog.value = await rpc<VehicleSetupCatalog>("vehicle_setup.catalog");
    } catch (e) {
        setupError.value = e instanceof Error ? e.message : String(e);
    } finally {
        setupLoading.value = false;
    }
}
async function createVehicleFromSetup(name: string, setup: { board: string; template: string; mechanisms: string[]; radio: string; motors?: Record<string, string[]>; power?: { limit_mode: string } }) {
    setupError.value = "";
    busy.value = true;
    try {
        const result = await rpc<{ name: string; validation?: { ok: boolean; errors: string[] } }>("robot.create", { name, setup });
        await refreshSummary();
        createDialog.value = false;
        busy.value = false;
        if (anyDirty.value) {
            pendingTarget.value = result.name;
            pendingTab.value = "chassis";
        } else {
            target.value = result.name;
            activeTab.value = "chassis";
            await loadRobot();
        }
        showToast(result.validation?.ok === false ? `已创建 ${result.name}，仍有配置问题需要处理。` : `已创建 ${result.name}。下一步核对电机总线、地址和方向。`);
    } catch (e) {
        setupError.value = e instanceof Error ? e.message : String(e);
    } finally {
        busy.value = false;
    }
}
function updateCursor() {
    const el = editor.value;
    if (!el) return;
    cursorLine.value = fileText.value
        .slice(0, el.selectionStart)
        .split(/\r?\n/).length;
}
function syncEditorScroll() {
    if (lineNumbers.value && editor.value)
        lineNumbers.value.scrollTop = editor.value.scrollTop;
}
function updateSerialStatus(value: unknown) {
    serial.value = (value as { status: typeof serial.value }).status;
}
watch(target, () => {
    if (target.value && !robot.value) loadRobot();
});
watch(activeTab, (tab) => {
    window.scrollTo(0, 0);
    if (tab === "robot") loadPortOptions();
    if (["parameters", "receiver", "chassis"].includes(tab)) loadValues();
    if (["serial", "receiver", "chassis", "imu"].includes(tab)) {
        refreshPorts();
        refreshStatus();
    }
});
function preventUnload(event: BeforeUnloadEvent) {
    if (
        anyDirty.value ||
        toolsBusy.value ||
        job.value?.status === "running"
    ) {
        event.preventDefault();
        event.returnValue = "";
    }
}
onMounted(async () => {
    disposed.value = false;
    window.addEventListener("beforeunload", preventUnload);
    await Promise.all([refreshSummary(), refreshPorts()]);
    await loadRobot();
});
onBeforeUnmount(() => {
    disposed.value = true;
    window.removeEventListener("beforeunload", preventUnload);
});
</script>

<template>
    <div class="shell">
        <header class="topbar">
            <div class="brand">
                <span class="brand-mark">A</span><strong>ARBATOS</strong
                ><span class="tag">工程台</span>
            </div>
            <div class="connection">
                <span
                    class="status-dot"
                    :class="{ on: serial.connected }"
                ></span
                ><span class="conn-state">{{
                    serial.connected ? "串口已连接" : "未连接设备"
                }}</span>
                <select
                    v-model="port"
                    aria-label="串口"
                    :disabled="serial.connected"
                >
                    <option value="">选择串口</option>
                    <option
                        v-for="p in ports"
                        :key="p.device"
                        :value="p.device"
                    >
                        {{ p.device }} · {{ p.description }}
                    </option>
                </select>
                <select
                    v-model.number="baud"
                    aria-label="波特率"
                    :disabled="serial.connected"
                >
                    <option :value="115200">115200</option>
                    <option :value="921600">921600</option>
                    <option :value="460800">460800</option>
                </select>
                <button
                    class="button primary"
                    :disabled="!serial.connected && !port"
                    @click="serialToggle"
                >
                    {{ serial.connected ? "断开" : "连接" }}
                </button>
            </div>
        </header>
        <aside class="sidebar">
            <div class="target-select">
                <label>当前车型</label
                ><select
                    ref="targetSelect"
                    :disabled="interfaceBusy"
                    :value="target"
                    @change="
                        chooseTarget(($event.target as HTMLSelectElement).value)
                    "
                >
                    <option
                        v-for="t in summary?.targets ?? []"
                        :key="t.name"
                        :value="t.name"
                    >
                        {{ t.name }}
                    </option></select
                ><small>{{ currentTarget?.board ?? "等待工作区" }}</small>
            </div>
            <nav>
                <button
                    v-for="item in nav"
                    :key="item.key"
                    :disabled="interfaceBusy"
                    class="nav-item"
                    :class="{ active: activeTab === item.key }"
                    @click="switchTab(item.key)"
                >
                    <svg viewBox="0 0 24 24" v-html="svg(item.key)" /><span>{{
                        item.label
                    }}</span
                    ><small>{{ item.hint }}</small>
                </button>
            </nav>
            <div class="sidebar-foot">
                <span :class="['git-dot', { dirty: summary?.git.dirty }]" />
                {{ summary?.git.branch ?? "读取工程中" }}<br /><small>{{
                    summary?.git.sha?.slice(0, 8)
                }}</small>
            </div>
        </aside>
        <main class="content">
            <div v-if="error" class="inline-error">
                <span>{{ error }}</span
                ><button @click="error = ''">关闭</button>
            </div>
            <section v-if="activeTab === 'overview'" class="view overview">
                <div class="title-row">
                    <div>
                        <p class="eyebrow">工作区</p>
                        <h1>{{ target || "未发现车型" }}</h1>
                        <p>{{ summary?.root ?? "正在读取工程目录…" }}</p>
                    </div>
                    <button
                        class="button icon-button"
                        title="刷新"
                        @click="refreshSummary"
                    >
                        <svg viewBox="0 0 24 24" v-html="svg('refresh')" />
                    </button>
                </div>
                <div class="stats">
                    <article>
                        <span>板级定义</span
                        ><b>{{ currentTarget?.board ?? "—" }}</b
                        ><small>预设 {{ currentTarget?.preset ?? "—" }}</small>
                    </article>
                    <article>
                        <span>控制器</span
                        ><b>{{ summary?.controllers.length ?? 0 }}</b
                        ><small>工作区已识别</small>
                    </article>
                    <article>
                        <span>依赖服务</span
                        ><b
                            >{{
                                summary?.services.filter((x) => x.available)
                                    .length ?? 0
                            }}<i>/ {{ summary?.services.length ?? 0 }}</i></b
                        ><small>当前可用</small>
                    </article>
                    <article>
                        <span>配置校验</span
                        ><b
                            :class="{
                                good: robot?.validation.ok === true,
                                bad: robot && !robot.validation.ok,
                            }"
                            >{{
                                robot
                                    ? robot.validation.ok
                                        ? "通过"
                                        : "待处理"
                                    : "—"
                            }}</b
                        ><small>{{
                            robot?.validation.errors[0] ?? "读取车型后显示"
                        }}</small>
                    </article>
                </div>
                <div class="two-col">
                    <article class="panel">
                        <h2>已解析的运行框架</h2>
                        <div class="summary-list">
                            <div>
                                <span>任务</span
                                ><b>{{
                                    robot?.resolved?.tasks?.length ?? 0
                                }}</b>
                            </div>
                            <div>
                                <span>控制器</span
                                ><b>{{
                                    robot?.resolved?.controllers?.length ?? 0
                                }}</b>
                            </div>
                            <div>
                                <span>文件</span
                                ><b>{{ robot?.files.length ?? 0 }}</b>
                            </div>
                        </div>
                        <p class="muted">
                            这里显示工程解析结果；串口收发已接入，控制输出和在线参数保存通路尚未接入。
                        </p>
                    </article>
                    <article class="panel">
                        <h2>源码与版本</h2>
                        <dl>
                            <dt>分支</dt>
                            <dd>{{ summary?.git.branch ?? "—" }}</dd>
                            <dt>提交</dt>
                            <dd class="mono">{{ summary?.git.sha ?? "—" }}</dd>
                            <dt>工作区</dt>
                            <dd>
                                {{
                                    summary?.git.dirty === true
                                        ? "存在未提交修改"
                                        : summary?.git.dirty === false
                                          ? "干净"
                                          : "状态未知"
                                }}
                            </dd>
                        </dl>
                    </article>
                </div>
            </section>
            <section v-else-if="activeTab === 'robot'" class="view">
                <div class="title-row">
                    <div>
                        <p class="eyebrow">车型配置</p>
                        <h1>{{ target }}</h1>
                        <p>修改前会检查版本；外部编辑导致冲突时不会覆盖。</p>
                    </div>
                    <div class="title-actions">
                        <button
                            class="button"
                            :disabled="busy"
                            @click="openCreateDialog"
                        >
                            新建车型</button
                        ><button
                            class="button primary"
                            :disabled="!dirtyConfig || busy"
                            @click="saveConfig"
                        >
                            <svg
                                viewBox="0 0 24 24"
                                v-html="svg('save')"
                            />保存配置
                        </button>
                    </div>
                </div>
                <div v-if="robot && !robot.validation.ok" class="validation">
                    <b>已保存配置需要处理</b
                    ><span v-for="e in robot.validation.errors" :key="e">{{
                        e
                    }}</span>
                </div>
                <div
                    v-if="validationPreview"
                    class="validation"
                    :class="{ 'validation-ok': validationPreview.ok }"
                >
                    <b>{{
                        validationPreview.ok
                            ? "修改后的配置检查通过"
                            : "修改后的配置需要处理"
                    }}</b
                    ><span v-for="e in validationPreview.errors" :key="e">{{
                        e
                    }}</span>
                </div>
                <div class="config-actions">
                    <button
                        class="button"
                        :disabled="busy"
                        @click="validateConfig"
                    >
                        检查修改</button
                    ><span v-if="dirtyConfig" class="dirty-label">未保存</span>
                </div>
                <RobotConfigPanel
                    :model-value="configDraft"
                    :controllers="summary?.controllers ?? []"
                    :services="summary?.services ?? []"
                    :disabled="busy"
                    @update:model-value="changeConfig"
                />
                <PortConfigPanel :model-value="configDraft" :data="portOptions" :disabled="busy"
                    @update:model-value="changeConfig" />
                <TaskPlanPanel :resolved="validationPreview?.resolved ?? robot?.resolved ?? null"
                    :draft="!!validationPreview?.resolved" :pending="dirtyConfig && !validationPreview?.resolved" />
                <p class="muted tuning-note">
                    调参只写入离线配置；可先用“检查”验证修改。串口已实现，在线参数保存和控制输出通路尚未接入。
                </p>
            </section>
            <section v-else-if="activeTab === 'memory'" class="view"><MemoryPanel :target="target" /></section>
            <section v-else-if="activeTab === 'imu'" class="view">
                <ImuPanel :target="target" :connected="serial.connected" :imu="serial.imu"
                    @status="updateSerialStatus" @error="fail" />
            </section>
            <section v-else-if="activeTab === 'chassis'" class="view">
                <div class="title-row">
                    <div><p class="eyebrow">新车装配</p><h1>底盘装配</h1><p>{{ target }} · 轮位、总线和安装方向统一配置。保存后重新编译并烧录到车上。</p></div>
                    <div class="title-actions">
                        <button class="button" :disabled="interfaceBusy || dirtyValues" @click="loadValues">重新读取配置</button>
                        <button class="button primary" :disabled="interfaceBusy || !dirtyValues || job?.status === 'running'" @click="saveValues">{{ dirtyValues ? `保存 ${Object.keys(valueChanges).length} 项修改` : '配置已保存' }}</button>
                    </div>
                </div>
                <ChassisPanel :key="target" :data="configValues" :changes="valueChanges" :serial="serial" :paused="paused"
                    :ports="ports" :port="port" :disabled="interfaceBusy || job?.status === 'running'" @change="changeValue"
                    @assign="changes => Object.entries(changes).forEach(([id, value]) => changeValue(id, value))"
                    @port="port = $event" @toggle="observationToggle" @refresh="refreshPorts" @resume="paused = false" />
            </section>
            <section v-else-if="activeTab === 'receiver'" class="view">
                <div class="title-row">
                    <div><p class="eyebrow">遥控输入</p><h1>接收机</h1><p>查看每个通道的实时数值，设置它在 {{ target }} 中的用途。</p></div>
                    <div class="title-actions">
                        <button class="button" :disabled="interfaceBusy || dirtyValues" @click="loadValues">重新读取配置</button>
                        <button class="button primary" :disabled="interfaceBusy || !dirtyValues || job?.status === 'running'" @click="saveValues">{{ dirtyValues ? `保存 ${Object.keys(valueChanges).length} 项修改` : '配置已保存' }}</button>
                    </div>
                </div>
                <ReceiverPanel :data="configValues" :changes="valueChanges" :serial="serial" :ports="ports" :port="port" :paused="paused"
                    :disabled="interfaceBusy || job?.status === 'running'" @change="changeValue" @port="port = $event"
                    @toggle="observationToggle" @refresh="refreshPorts" @resume="paused = false" />
            </section>
            <section v-else-if="activeTab === 'parameters'" class="view">
                <div class="title-row">
                    <div>
                        <p class="eyebrow">参数与设备</p>
                        <h1>{{ target }}</h1>
                        <p>直接编辑本车的参数文件。保存后重新编译，参数才会进入固件。</p>
                    </div>
                    <div class="title-actions">
                        <button class="button" :disabled="interfaceBusy || dirtyValues" @click="loadValues">重新读取</button>
                        <button class="button primary" :disabled="interfaceBusy || !dirtyValues || job?.status === 'running'" @click="saveValues">
                            {{ Object.keys(valueChanges).length ? `保存 ${Object.keys(valueChanges).length} 项修改` : '暂无修改' }}
                        </button>
                    </div>
                </div>
                <ConfigValuesPanel :data="configValues" :changes="valueChanges"
                    :disabled="interfaceBusy || job?.status === 'running'" @change="changeValue" />
            </section>
            <section v-else-if="activeTab === 'tools'" class="view">
                <div class="title-row">
                    <div><p class="eyebrow">本机工具</p><h1>日志与音频</h1><p>解析机器人日志，或把歌曲转换为副板可播放的 u8 文件。</p></div>
                </div>
                <ToolsPanel :disabled="busy || job?.status === 'running'" @error="fail" @busy="toolsBusy = $event; if ($event) error = ''" />
            </section>
            <section v-else-if="activeTab === 'files'" class="view files-view">
                <div class="title-row">
                    <div>
                        <p class="eyebrow">文件编辑</p>
                        <h1>{{ target }}</h1>
                        <p>只列出这个车型后端允许读取的现有文件。</p>
                    </div>
                    <button
                        class="button primary"
                        :disabled="!dirtyFile || busy"
                        @click="saveFile"
                    >
                        <svg viewBox="0 0 24 24" v-html="svg('save')" />保存文件
                    </button>
                </div>
                <div class="file-layout">
                    <aside class="file-list">
                        <div class="search">
                            <svg
                                viewBox="0 0 24 24"
                                v-html="svg('search')"
                            /><input v-model="query" placeholder="筛选文件" />
                        </div>
                        <button
                            v-for="item in filteredFiles"
                            :key="item.name"
                            :disabled="busy"
                            :class="{ selected: selectedFile === item.name }"
                            @click="openFile(item.name)"
                        >
                            {{ item.name }}
                        </button>
                        <p v-if="!filteredFiles.length" class="muted">
                            没有匹配文件
                        </p>
                    </aside>
                    <article class="editor-panel">
                        <div class="editor-head">
                            <span>{{ file?.name ?? "选择文件开始编辑" }}</span
                            ><span v-if="file">第 {{ cursorLine }} 行</span
                            ><small v-if="file"
                                >{{ file.encoding }} · {{ file.newline }}</small
                            ><span v-if="dirtyFile" class="dirty-label"
                                >未保存</span
                            >
                        </div>
                        <div class="code-area">
                            <pre ref="lineNumbers" aria-hidden="true">{{
                                Array.from(
                                    { length: visibleLines },
                                    (_, i) => i + 1,
                                ).join("\n")
                            }}</pre>
                            <textarea
                                ref="editor"
                                v-model="fileText"
                                :disabled="!file || busy"
                                aria-label="文件内容"
                                spellcheck="false"
                                wrap="off"
                                @scroll="syncEditorScroll"
                                @input="dirtyFile = true"
                                @keyup="updateCursor"
                                @click="updateCursor"
                            />
                        </div>
                    </article>
                </div>
            </section>
            <section
                v-else-if="activeTab === 'serial'"
                class="view serial-view"
            >
                <div class="title-row">
                    <div>
                        <p class="eyebrow">串口与曲线</p>
                        <h1>
                            {{ serial.connected ? serial.port : "未连接设备" }}
                        </h1>
                        <p>
                            {{
                                offlineReplay
                                    ? "正在显示本机 CSV 离线回放数据，不代表串口在线。"
                                    : serial.connected
                                      ? `接收 ${serial.rxBytes} B，发送 ${serial.txBytes} B`
                                      : "连接后才会显示设备实时数据。"
                            }}
                        </p>
                    </div>
                    <div class="title-actions">
                        <label class="upload button"
                            >导入 CSV 回放<input
                                type="file"
                                accept=".csv,text/csv"
                                @change="importCsv" /></label
                        ><button class="button" @click="clearSerial">
                            清空
                        </button>
                    </div>
                </div>
                <div class="serial-controls panel">
                    <label
                        >格式<select
                            v-model="serialFormat"
                            :disabled="serial.connected"
                        >
                            <option value="text">text</option>
                            <option value="csv">csv</option>
                            <option value="justfloat">justfloat</option>
                        </select></label
                    ><label
                        >通道<select
                            v-model.number="channels"
                            :disabled="serial.connected"
                        >
                            <option v-for="n in 16" :key="n" :value="n">
                                {{ n }}
                            </option>
                        </select></label
                    ><button
                        class="button"
                        :disabled="!serial.connected && !port"
                        @click="serialToggle"
                    >
                        {{ serial.connected ? "断开串口" : "连接串口" }}</button
                    ><button
                        class="button"
                        :disabled="!serial.connected || offlineReplay"
                        @click="paused = !paused"
                    >
                        <svg
                            viewBox="0 0 24 24"
                            v-html="svg(paused ? 'play' : 'pause')"
                        />{{ paused ? "继续" : "暂停" }}</button
                    ><span v-if="serial.error" class="field-error">{{
                        serial.error
                    }}</span>
                </div>
                <TelemetryChart
                    :samples="samples"
                    :channels="channels"
                    :offline="offlineReplay"
                />
                <div class="serial-bottom">
                    <article class="panel console">
                        <div class="panel-head">
                            <h2>接收文本</h2>
                            <small>{{ serialLines.length }} 行</small>
                        </div>
                        <pre>{{
                            serialLines
                                .map((x) => `[${x.time}] ${x.text}`)
                                .join("\n") || "等待串口文本…"
                        }}</pre>
                    </article>
                    <article class="panel send">
                        <h2>手动发送</h2>
                        <textarea
                            v-model="serialInput"
                            :disabled="!serial.connected || offlineReplay"
                            placeholder="输入要发送的文本…"
                            @keydown.ctrl.enter.prevent="sendSerial"
                        /><button
                            class="button primary"
                            :disabled="
                                !serial.connected ||
                                !serialInput.trim() ||
                                offlineReplay
                            "
                            @click="sendSerial"
                        >
                            <svg
                                viewBox="0 0 24 24"
                                v-html="svg('terminal')"
                            />发送 LF
                        </button>
                        <p class="muted">
                            仅手动发送文本；离线回放时发送会禁用。
                        </p>
                    </article>
                </div>
            </section>
            <section v-else class="view build-view">
                <div class="title-row">
                    <div>
                        <p class="eyebrow">编译与烧录</p>
                        <h1>{{ target }}</h1>
                        <p>
                            构建和下载由本机后端执行；烧录前会展示产物与哈希。
                        </p>
                    </div>
                </div>
                <div class="build-actions">
                    <button
                        class="action-card"
                        :disabled="busy || job?.status === 'running'"
                        @click="startJob('check')"
                    >
                        <b>检查</b><span>验证车型配置和依赖</span></button
                    ><button
                        class="action-card"
                        :disabled="busy || job?.status === 'running'"
                        @click="startJob('build')"
                    >
                        <b>增量编译</b><span>按当前工程状态构建</span></button
                    ><button
                        class="action-card"
                        :disabled="busy || job?.status === 'running'"
                        @click="startJob('rebuild')"
                    >
                        <b>重新编译</b
                        ><span>清理后完整构建当前车型</span></button
                    ><button
                        class="action-card warning"
                        :disabled="busy || job?.status === 'running'"
                        @click="planFlash"
                    >
                        <b>准备烧录</b><span>先核对固件产物和哈希</span>
                    </button>
                </div>
                <article class="panel build-log">
                    <div class="panel-head">
                        <div>
                            <h2>任务日志</h2>
                            <p class="muted">
                                {{
                                    job
                                        ? `${job.action} · ${jobState(job.status)} · ${job.target}`
                                        : "尚未启动任务"
                                }}
                            </p>
                        </div>
                        <button
                            v-if="job?.status === 'running'"
                            class="button danger"
                            @click="cancelJob"
                        >
                            取消
                        </button>
                    </div>
                    <pre>{{
                        job?.lines.map((x) => x.text).join("\n") ||
                        "等待编译任务…"
                    }}</pre>
                    <footer v-if="job">
                        <span>开始：{{ formatTime(job.startedAt) }}</span
                        ><span v-if="job.finishedAt"
                            >结束：{{ formatTime(job.finishedAt) }}</span
                        ><span v-if="job.exitCode !== undefined"
                            >退出码：{{ job.exitCode }}</span
                        >
                    </footer>
                </article>
            </section>
        </main>
        <div v-if="toast" class="toast">{{ toast }}</div>
        <div
            v-if="pendingTarget || pendingTab || pendingFile"
            class="dialog-backdrop"
        >
            <section class="dialog">
                <h2>有未保存的修改</h2>
                <p>切换前需要决定如何处理当前修改。</p>
                <div>
                    <button class="button" @click="cancelUnsaved">取消</button
                    ><button class="button" @click="leaveUnsaved(false)">
                        放弃修改</button
                    ><button class="button primary" @click="leaveUnsaved(true)">
                        保存后继续
                    </button>
                </div>
            </section>
        </div>
        <div v-if="createDialog" class="dialog-backdrop">
            <section class="dialog vehicle-setup-dialog">
                <VehicleSetupPanel :catalog="setupCatalog" :busy="busy || setupLoading" :error="setupError"
                    @cancel="createDialog = false" @submit="createVehicleFromSetup" />
                <details class="advanced-copy"><summary>高级选项：按已有车型复制</summary><p>用于保留某个现有车型文件的完整副本；常规新车请使用上面的向导。</p><label class="dialog-field">名称<input v-model="newTargetName" placeholder="例如 HERO-M-TEST" /></label><label class="dialog-field">复制来源<select v-model="newTargetSource"><option value="" disabled>选择模板车型</option><option v-for="t in summary?.targets ?? []" :key="t.name" :value="t.name">{{ t.name }}</option></select></label><button class="button" :disabled="busy" @click="createTarget">按副本创建</button></details>
            </section>
        </div>
        <div v-if="confirmFlash" class="dialog-backdrop">
            <section class="dialog flash-dialog">
                <h2>确认烧录固件</h2>
                <dl>
                    <dt>车型</dt>
                    <dd>{{ confirmFlash.target }}</dd>
                    <dt>产物</dt>
                    <dd class="mono">{{ confirmFlash.path }}</dd>
                    <dt>哈希</dt>
                    <dd class="mono">{{ confirmFlash.sha }}</dd>
                    <dt>构建时间</dt>
                    <dd>{{ confirmFlash.buildTime ?? "未提供" }}</dd>
                </dl>
                <p v-if="confirmFlash.reason" class="field-error">
                    {{ confirmFlash.reason }}
                </p>
                <ul v-if="confirmFlash.warnings.length">
                    <li v-for="w in confirmFlash.warnings" :key="w">{{ w }}</li>
                </ul>
                <div>
                    <button class="button" @click="confirmFlash = null">
                        返回</button
                    ><button
                        class="button danger"
                        :disabled="!confirmFlash.ready"
                        @click="doFlash"
                    >
                        确认烧录
                    </button>
                </div>
            </section>
        </div>
    </div>
</template>
