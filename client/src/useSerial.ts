import { onBeforeUnmount, ref, watch, type Ref } from "vue";
import { rpc } from "./api";
import { parseSerialCsv } from "./serialCsv";
import type { SerialLine, SerialSample, SerialStatus } from "./types";

export function useSerial(options: {
    fail: (error: unknown) => void;
    showToast: (message: string) => void;
    disposed: Ref<boolean>;
}) {
    const serial = ref<SerialStatus>({ connected: false, rxBytes: 0, txBytes: 0 });
    const ports = ref<{ device: string; description: string; hwid: string }[]>([]);
    const port = ref("");
    const baud = ref(115200);
    const serialFormat = ref<"text" | "csv" | "justfloat">("text");
    const channels = ref(1);
    const serialCursor = ref(0);
    const serialLines = ref<SerialLine[]>([]);
    const samples = ref<SerialSample[]>([]);
    const serialInput = ref("");
    const paused = ref(false);
    const offlineReplay = ref(false);
    const clearAt = ref(0);
    let serialTimer: number | undefined;
    let serialPollingSession: number | null = null;
    let serialSession = 0;

    async function refreshPorts() {
        try {
            ports.value = await rpc<typeof ports.value>("serial.ports");
            if (!port.value) port.value = ports.value[0]?.device ?? "";
        } catch (e) {
            options.fail(e);
        }
    }

    async function refreshStatus() {
        if (options.disposed.value) return;
        try {
            const status = await rpc<SerialStatus>("serial.status");
            if (options.disposed.value) return;
            serial.value = status;
            if (status.connected) pollSerial();
        } catch (e) {
            if (!options.disposed.value) options.fail(e);
        }
    }

    async function pollSerial(session = serialSession) {
        if (options.disposed.value || session !== serialSession) return;
        if (serialPollingSession === session || !serial.value.connected || paused.value) return;
        serialPollingSession = session;
        try {
            const data = await rpc<{
                status: SerialStatus;
                lines: SerialLine[];
                samples: SerialSample[];
                cursor: number;
                dropped: number;
            }>("serial.read", { after: serialCursor.value });
            if (options.disposed.value || session !== serialSession) return;
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
            options.fail(e);
        } finally {
            if (serialPollingSession === session) serialPollingSession = null;
            if (!options.disposed.value && session === serialSession && serial.value.connected && !paused.value)
                serialTimer = window.setTimeout(() => pollSerial(session), 160);
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
                    port: port.value, baud: baud.value, format: serialFormat.value, channels: channels.value,
                });
                serialCursor.value = 0;
                clearAt.value = 0;
                serialLines.value = [];
                samples.value = [];
                if (session === serialSession) pollSerial(session);
            }
        } catch (e) {
            options.fail(e);
        }
    }

    async function observationToggle() {
        if (!serial.value.connected) {
            serialFormat.value = "text";
            baud.value = 115200;
        }
        await serialToggle();
    }

    async function sendSerial() {
        if (!serialInput.value.trim()) return;
        try {
            serial.value = await rpc<SerialStatus>("serial.send", { text: serialInput.value, lineEnding: "LF" });
            serialInput.value = "";
        } catch (e) {
            options.fail(e);
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
            if (serial.value.connected) throw new Error("请先断开真实串口，再导入离线 CSV 回放。");
            if (csv.size > 5 * 1024 * 1024) throw new Error("CSV 不能超过 5 MB");
            const { samples: parsed, channels: columns } = parseSerialCsv(await csv.text());
            serialSession++;
            if (serialTimer) window.clearTimeout(serialTimer);
            samples.value = parsed.slice(-100000);
            serialLines.value = [];
            channels.value = columns;
            offlineReplay.value = true;
            paused.value = true;
            options.showToast(`已加载 ${parsed.length} 行离线 CSV 数据。`);
        } catch (e) {
            options.fail(e);
        } finally {
            input.value = "";
        }
    }

    watch(paused, (value) => {
        if (!value && serial.value.connected) pollSerial(serialSession);
    });
    onBeforeUnmount(() => {
        serialSession++;
        if (serialTimer) window.clearTimeout(serialTimer);
    });

    return {
        serial, ports, port, baud, serialFormat, channels, serialCursor, serialLines,
        samples, serialInput, paused, offlineReplay, clearAt, refreshPorts, refreshStatus, pollSerial,
        serialToggle, observationToggle, sendSerial, clearSerial, importCsv,
    };
}
