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
import ConfigValuesPanel, { type ConfigValues } from "./components/ConfigValuesPanel.vue";
import ToolsPanel from "./components/ToolsPanel.vue";
import PortConfigPanel, { type PortOptions } from "./components/PortConfigPanel.vue";

type Tab = "overview" | "robot" | "parameters" | "files" | "serial" | "tools" | "build";
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
type Robot = {
    name: string;
    config: Record<string, unknown> | null;
    revision: string;
    files: { name: string }[];
    resolved: {
        tasks?: unknown[];
        controllers?: unknown[];
        [key: string]: unknown;
    } | null;
    validation: { ok: boolean; errors: string[] };
};
type FileData = {
    name: string;
    content: string;
    revision: string;
    encoding: string;
    newline: string;
};
type Job = {
    id: string;
    target: string;
    action: "check" | "build" | "rebuild" | "flash";
    status: "running" | "succeeded" | "failed" | "cancelled";
    exitCode?: number;
    startedAt: number | string;
    finishedAt?: number | string;
    lines: { seq: number; text: string }[];
    lastSeq: number;
};
type SerialStatus = {
    connected: boolean;
    port?: string;
    baud?: number;
    format?: string;
    rxBytes: number;
    txBytes: number;
    error?: string;
};

const icon = (name: string) =>
    ({
        overview:
            '<path d="M3 13h7V3H3v10Zm0 8h7v-5H3v5Zm11 0h7V11h-7v10Zm0-18v5h7V3h-7Z"/>',
        robot: '<path d="M12 2v3m-6 4V7h12v2m-3 0v4m-6-4v4m-5 0h16v7H4v-7Zm4 3v1m8-1v1M9 22h6"/>',
        files: '<path d="M4 3h10l6 6v12H4V3Zm10 0v6h6M8 14h8M8 18h5"/>',
        serial: '<path d="M6 3v4m12-4v4M8 7h8v5a4 4 0 0 1-8 0V7Zm4 9v5m-3 0h6"/>',
        build: '<path d="m14 4 6 6-9 9H5v-6l9-9Zm-2 2 6 6M4 21h16"/>',
        parameters: '<path d="M5 3v18M12 3v18M19 3v18M2 8h6m1 8h6m1-10h6"/>',
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
const dirtyConfig = ref(false);
const configDraft = ref<Record<string, unknown>>({});
const portOptions = ref<PortOptions | null>(null);
let portPreviewTimer: number | undefined;
let portPreviewSeq = 0;
const validationPreview = ref<{ ok: boolean; errors: string[] } | null>(null);
const selectedFile = ref("");
const file = ref<FileData | null>(null);
const fileText = ref("");
const dirtyFile = ref(false);
const configValues = ref<ConfigValues | null>(null);
const valuesLoading = ref(false);
let valuesRequestSeq = 0;
const valueChanges = ref<Record<string, unknown>>({});
const dirtyValues = computed(() => Object.keys(valueChanges.value).length > 0);
const toolsBusy = ref(false);
const anyDirty = computed(() => dirtyConfig.value || dirtyFile.value || dirtyValues.value);
const interfaceBusy = computed(() => busy.value || toolsBusy.value || valuesLoading.value);
const query = ref("");
const editor = ref<HTMLTextAreaElement | null>(null);
const lineNumbers = ref<HTMLPreElement | null>(null);
const cursorLine = ref(1);
const job = ref<Job | null>(null);
const serial = ref<SerialStatus>({ connected: false, rxBytes: 0, txBytes: 0 });
const ports = ref<{ device: string; description: string; hwid: string }[]>([]);
const port = ref("");
const baud = ref(115200);
const serialFormat = ref<"text" | "csv" | "justfloat">("text");
const channels = ref(1);
const serialCursor = ref(0);
const serialLines = ref<{ seq: number; time: number | string; text: string }[]>(
    [],
);
const samples = ref<{ seq: number; time: number | string; values: number[] }[]>(
    [],
);
const serialInput = ref("");
const paused = ref(false);
const offlineReplay = ref(false);
const clearAt = ref(0);
const createDialog = ref(false);
const newTargetName = ref("");
const newTargetSource = ref("");
const confirmFlash = ref<{
    target: string;
    artifactRevision: string;
    path: string;
    sha: string;
    buildTime?: string;
    warnings: string[];
    ready: boolean;
    reason?: string;
} | null>(null);
let jobTimer: number | undefined;
let serialTimer: number | undefined;
let serialPollingSession: number | null = null;
let serialSession = 0;
let disposed = false;
let requestId = 0;
const targetSelect = ref<HTMLSelectElement | null>(null);

const nav: { key: Tab; label: string; hint: string }[] = [
    { key: "overview", label: "工程概览", hint: "工作区与车型" },
    { key: "robot", label: "车型与接口", hint: "控制器、服务、串口分配" },
    { key: "parameters", label: "参数与设备", hint: "PID、中位、电机、遥控器" },
    { key: "files", label: "文件编辑", hint: "车型现有文件" },
    { key: "serial", label: "串口与曲线", hint: "真实串口数据" },
    { key: "tools", label: "日志与音频", hint: "日志解析、歌曲转 u8" },
    { key: "build", label: "编译与烧录", hint: "检查、构建、下载" },
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
        if (activeTab.value === "parameters") await loadValues();
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
    if (dirtyConfig.value && robot.value)
        configDraft.value = JSON.parse(
            JSON.stringify(robot.value.config ?? {}),
        );
    dirtyFile.value = false;
    dirtyConfig.value = false;
    valueChanges.value = {};
    loadPortOptions();
    validationPreview.value = null;
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
function changeConfig(value: Record<string, unknown>) {
    configDraft.value = value;
    dirtyConfig.value = true;
    validationPreview.value = null;
    error.value = "";
    if (portPreviewTimer) window.clearTimeout(portPreviewTimer);
    portPreviewTimer = window.setTimeout(loadPortOptions, 300);
}
async function loadPortOptions() {
    const seq = ++portPreviewSeq;
    const expected = target.value;
    if (!expected) return;
    try {
        const result = await rpc<PortOptions>("ports.get", { target: expected, config: configDraft.value });
        if (seq === portPreviewSeq && expected === target.value) portOptions.value = result;
    } catch (e) {
        if (!portOptions.value && seq === portPreviewSeq) fail(e);
    }
}
async function loadValues() {
    const expectedTarget = target.value;
    if (!expectedTarget || dirtyValues.value) return;
    const seq = ++valuesRequestSeq;
    valuesLoading.value = true;
    try {
        const result = await rpc<ConfigValues>("config.get", { target: expectedTarget });
        if (target.value !== expectedTarget || seq !== valuesRequestSeq || disposed || dirtyValues.value) return;
        configValues.value = result;
        valueChanges.value = {};
    } catch (e) { if (seq === valuesRequestSeq && !disposed) fail(e); }
    finally { if (seq === valuesRequestSeq) valuesLoading.value = false; }
}
function changeValue(id: string, value: unknown) {
    if (interfaceBusy.value) return;
    const original = configValues.value?.groups.flatMap(group => group.fields).find(field => field.id === id);
    if (!original || original.editable === false) return;
    const next = { ...valueChanges.value };
    if (Object.is(value, original.value)) delete next[id];
    else next[id] = value;
    valueChanges.value = next;
    error.value = "";
}
async function saveValues(): Promise<boolean> {
    if (!configValues.value || !dirtyValues.value) return false;
    busy.value = true;
    try {
        configValues.value = await rpc<ConfigValues>("config.update", {
            target: target.value, revision: configValues.value.revision, changes: valueChanges.value,
        });
        valueChanges.value = {};
        error.value = "";
        await refreshSummary();
        showToast("参数已保存到车型文件，重新编译后生效。");
        return true;
    } catch (e) { fail(e); return false; }
    finally { busy.value = false; }
}
async function saveConfig(): Promise<boolean> {
    if (!robot.value) return false;
    busy.value = true;
    try {
        const updated = await rpc<Robot>("robot.update", {
            target: target.value,
            revision: robot.value.revision,
            config: configDraft.value,
        });
        if (updated.name === target.value) {
            robot.value = updated;
            configDraft.value = JSON.parse(
                JSON.stringify(updated.config ?? {}),
            );
            dirtyConfig.value = false;
            loadPortOptions();
            validationPreview.value = null;
            error.value = "";
            showToast("车型配置已保存。");
            return true;
        }
        return false;
    } catch (e) {
        fail(e);
        return false;
    } finally {
        busy.value = false;
    }
}
async function validateConfig() {
    if (!robot.value || busy.value) return;
    busy.value = true;
    error.value = "";
    try {
        validationPreview.value = await rpc<{ ok: boolean; errors: string[] }>(
            "robot.validate",
            {
                target: target.value,
                revision: robot.value.revision,
                config: configDraft.value,
            },
        );
        showToast(
            validationPreview.value.ok
                ? "修改后的配置检查通过。"
                : "配置检查发现问题。",
        );
    } catch (e) {
        fail(e);
    } finally {
        busy.value = false;
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
async function startJob(action: Job["action"]) {
    if (!target.value) return;
    if (anyDirty.value) {
        showToast("请先保存或放弃当前修改，再编译，避免构建旧文件内容。");
        return;
    }
    busy.value = true;
    try {
        job.value = await rpc<Job>("job.start", {
            target: target.value,
            action,
        });
        pollJob();
    } catch (e) {
        fail(e);
    } finally {
        busy.value = false;
    }
}
async function planFlash() {
    if (busy.value) return;
    if (anyDirty.value) {
        showToast("请先保存或放弃当前修改，再检查烧录产物。");
        return;
    }
    busy.value = true;
    try {
        confirmFlash.value = await rpc<typeof confirmFlash.value>(
            "job.plan_flash",
            { target: target.value },
        );
    } catch (e) {
        fail(e);
    } finally {
        busy.value = false;
    }
}
async function doFlash() {
    if (!confirmFlash.value || busy.value) return;
    busy.value = true;
    try {
        job.value = await rpc<Job>("job.start", {
            target: target.value,
            action: "flash",
            confirmation: {
                target: confirmFlash.value.target,
                artifactRevision: confirmFlash.value.artifactRevision,
            },
        });
        confirmFlash.value = null;
        pollJob();
    } catch (e) {
        fail(e);
    } finally {
        busy.value = false;
    }
}
async function pollJob() {
    if (disposed || !job.value || job.value.status !== "running") return;
    try {
        const newer = await rpc<Job>("job.get", {
            id: job.value.id,
            after: job.value.lastSeq,
        });
        if (disposed) return;
        job.value = {
            ...newer,
            lines: newer.lines?.length
                ? [...(job.value?.lines ?? []), ...newer.lines].slice(-1500)
                : (job.value?.lines ?? []),
        };
        if (job.value.status === "running")
            jobTimer = window.setTimeout(pollJob, 700);
    } catch (e) {
        fail(e);
    }
}
async function cancelJob() {
    if (!job.value) return;
    try {
        job.value = await rpc<Job>("job.cancel", { id: job.value.id });
    } catch (e) {
        fail(e);
    }
}
async function refreshPorts() {
    try {
        ports.value = await rpc<typeof ports.value>("serial.ports");
        if (!port.value) port.value = ports.value[0]?.device ?? "";
    } catch (e) {
        fail(e);
    }
}
async function serialToggle() {
    const session = ++serialSession;
    try {
        if (serial.value.connected) {
            if (serialTimer) window.clearTimeout(serialTimer);
            serial.value = await rpc<SerialStatus>("serial.close");
        } else {
            offlineReplay.value = false;
            paused.value = false;
            serial.value = await rpc<SerialStatus>("serial.open", {
                port: port.value,
                baud: baud.value,
                format: serialFormat.value,
                channels: channels.value,
            });
            serialCursor.value = 0;
            clearAt.value = 0;
            serialLines.value = [];
            samples.value = [];
            if (session === serialSession) pollSerial(session);
        }
    } catch (e) {
        fail(e);
    }
}
async function pollSerial(session = serialSession) {
    if (disposed || session !== serialSession) return;
    if (
        serialPollingSession === session ||
        !serial.value.connected ||
        paused.value
    )
        return;
    serialPollingSession = session;
    try {
        const data = await rpc<{
            status: SerialStatus;
            lines: { seq: number; time: number | string; text: string }[];
            samples: { seq: number; time: number | string; values: number[] }[];
            cursor: number;
            dropped: number;
        }>("serial.read", { after: serialCursor.value });
        if (disposed || session !== serialSession) return;
        serial.value = data.status;
        serialCursor.value = data.cursor;
        serialLines.value = [
            ...serialLines.value,
            ...data.lines.filter((x) => x.seq > clearAt.value),
        ].slice(-300);
        samples.value = [
            ...samples.value,
            ...data.samples.filter((x) => x.seq > clearAt.value),
        ].slice(-600);
    } catch (e) {
        fail(e);
    } finally {
        if (serialPollingSession === session) serialPollingSession = null;
        if (
            !disposed &&
            session === serialSession &&
            serial.value.connected &&
            !paused.value
        )
            serialTimer = window.setTimeout(() => pollSerial(session), 160);
    }
}
async function sendSerial() {
    if (!serialInput.value.trim()) return;
    try {
        serial.value = await rpc<SerialStatus>("serial.send", {
            text: serialInput.value,
            lineEnding: "LF",
        });
        serialInput.value = "";
    } catch (e) {
        fail(e);
    }
}
function clearSerial() {
    clearAt.value = serialCursor.value;
    serialLines.value = [];
    samples.value = [];
}
async function importCsv(event: Event) {
    const input = event.target as HTMLInputElement;
    const csv = input.files?.[0];
    if (!csv) return;
    try {
        if (serial.value.connected)
            throw new Error("请先断开真实串口，再导入离线 CSV 回放。");
        if (csv.size > 5 * 1024 * 1024) throw new Error("CSV 不能超过 5 MB");
        const rows = (await csv.text())
            .split(/\r?\n/)
            .map((row) => row.trim())
            .filter(Boolean);
        if (rows.length > 100001) throw new Error("CSV 最多 100000 行");
        if (!rows.length) throw new Error("CSV 是空文件，无法回放。");
        const first = rows[0].split(",").map((x) => x.trim());
        const firstName = first[0].toLowerCase();
        const timeColumn = [
            "time",
            "time_s",
            "timestamp",
            "tick_ms",
            "time_ms",
        ].includes(firstName);
        const isNumber = (value: string) =>
            value !== "" && Number.isFinite(Number(value));
        const hasHeader = timeColumn || first.some((cell) => !isNumber(cell));
        const start = hasHeader ? 1 : 0;
        const columns = first.length - (timeColumn ? 1 : 0);
        if (columns < 1 || columns > 16)
            throw new Error("CSV 必须有 1 至 16 个数值通道");
        const parsed: {
            seq: number;
            time: number | string;
            values: number[];
        }[] = [];
        for (const [i, row] of rows.slice(start).entries()) {
            const cells = row.split(",").map((x) => x.trim());
            if (cells.length !== columns + (timeColumn ? 1 : 0))
                throw new Error(`第 ${i + start + 1} 行列数不一致`);
            if (cells.some((cell) => cell === ""))
                throw new Error(`第 ${i + start + 1} 行存在空单元格`);
            const values = cells.slice(timeColumn ? 1 : 0).map(Number);
            if (values.some((n) => !Number.isFinite(n)))
                throw new Error(`第 ${i + start + 1} 行包含非数值通道`);
            let time: number | string = `#${i + 1}`;
            if (timeColumn) {
                const rawTime = Number(cells[0]);
                if (!Number.isFinite(rawTime))
                    throw new Error(`第 ${i + start + 1} 行时间无效`);
                time =
                    firstName === "tick_ms" || firstName === "time_ms"
                        ? rawTime / 1000
                        : rawTime;
                if (i > 0 && Number(time) < Number(parsed[i - 1].time))
                    throw new Error(`第 ${i + start + 1} 行时间倒退`);
            }
            parsed.push({ seq: i + 1, time, values });
        }
        if (!parsed.length) throw new Error("CSV 没有数据行");
        serialSession++;
        if (serialTimer) window.clearTimeout(serialTimer);
        samples.value = parsed.slice(-100000);
        serialLines.value = [];
        channels.value = columns;
        offlineReplay.value = true;
        paused.value = true;
        showToast(`已加载 ${parsed.length} 行离线 CSV 数据。`);
    } catch (e) {
        fail(e);
    } finally {
        input.value = "";
    }
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
watch(target, () => {
    if (target.value && !robot.value) loadRobot();
});
watch(activeTab, (tab) => {
    window.scrollTo(0, 0);
    if (tab === "parameters") loadValues();
    if (tab === "serial") {
        refreshPorts();
        rpc<SerialStatus>("serial.status")
            .then((x) => (serial.value = x))
            .catch(fail);
    }
});
watch(paused, (value) => {
    if (!value && serial.value.connected) pollSerial(serialSession);
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
    disposed = false;
    window.addEventListener("beforeunload", preventUnload);
    await Promise.all([refreshSummary(), refreshPorts()]);
    await loadRobot();
});
onBeforeUnmount(() => {
    disposed = true;
    serialSession++;
    window.removeEventListener("beforeunload", preventUnload);
    if (jobTimer) window.clearTimeout(jobTimer);
    if (serialTimer) window.clearTimeout(serialTimer);
    if (portPreviewTimer) window.clearTimeout(portPreviewTimer);
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
                            @click="
                                newTargetSource = target;
                                newTargetName = '';
                                createDialog = true;
                            "
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
                <p class="muted tuning-note">
                    调参只写入离线配置；可先用“检查”验证修改。串口已实现，在线参数保存和控制输出通路尚未接入。
                </p>
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
            <section class="dialog">
                <h2>新建车型</h2>
                <label class="dialog-field"
                    >名称<input
                        v-model="newTargetName"
                        placeholder="例如 HERO-M-TEST" /></label
                ><label class="dialog-field"
                    >复制来源<select v-model="newTargetSource">
                        <option value="" disabled>选择模板车型</option>
                        <option
                            v-for="t in summary?.targets ?? []"
                            :key="t.name"
                            :value="t.name"
                        >
                            {{ t.name }}
                        </option>
                    </select></label
                >
                <div>
                    <button class="button" @click="createDialog = false">
                        取消</button
                    ><button class="button primary" @click="createTarget">
                        创建
                    </button>
                </div>
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
