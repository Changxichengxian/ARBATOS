import type { ConfigField, ConfigValues } from "./components/ConfigValuesPanel.vue";

export type ReceiverSnapshot = {
    channels: number[] | null;
    rc: { ms: number; frameMs: number; valid: boolean; sequence: number; receivedAt: number } | null;
    link: {
        ms: number; portActive: boolean; state: number; lq: number; rcFrames: number;
        statsFrames: number; published: number; crcErrors: number; lengthErrors: number;
        overflows: number; restarts: number; statsTimeouts: number; receivedAt: number;
    } | null;
    uart: {
        ms: number; irqBytes: number; errors: number; overrun: number; framing: number;
        parity: number; noise: number; lastError: number; receivedAt: number;
    } | null;
    input: {
        ms: number; online: boolean; source: number; activeMask: number; channels: number[];
        switches: number[]; receivedAt: number;
    } | null;
};

export type ReceiverStatus = {
    connected: boolean;
    format?: string;
    receiver?: ReceiverSnapshot | null;
};

type Changes = Record<string, unknown>;

function fields(data: ConfigValues | null | undefined): ConfigField[] {
    return data?.groups.flatMap(group => group.fields) ?? [];
}

function field(data: ConfigValues | null | undefined, id: string): ConfigField | undefined {
    return fields(data).find(item => item.id === id);
}

export function fieldValue(field: ConfigField | undefined, changes: Changes): unknown {
    if (!field) return undefined;
    return Object.prototype.hasOwnProperty.call(changes, field.id) ? changes[field.id] : field.value;
}

export function rcFresh(snapshot: ReceiverSnapshot | null | undefined, connected: boolean, nowSeconds: number): boolean {
    if (!connected || !snapshot?.rc?.valid || snapshot.channels?.length !== 16) return false;
    if (!Number.isFinite(nowSeconds) || !Number.isFinite(snapshot.rc.receivedAt)) return false;
    const localAge = nowSeconds - snapshot.rc.receivedAt;
    if (localAge < 0 || localAge > 0.6) return false;
    const deviceAge = (snapshot.rc.ms - snapshot.rc.frameMs) >>> 0;
    return deviceAge <= 100;
}

export function switchPosition(raw: number, invert = false): "UP" | "MID" | "DOWN" {
    let position: "UP" | "MID" | "DOWN" = raw <= 582 ? "DOWN" : raw >= 1401 ? "UP" : "MID";
    if (invert && position !== "MID") position = position === "UP" ? "DOWN" : "UP";
    return position;
}

export function channelPercent(raw: number): number {
    return Math.max(0, Math.min(100, ((raw - 172) / (1811 - 172)) * 100));
}

export function semanticSwitch(id: string): "INPUT_SW_GIMBAL_MODE" | "INPUT_SW_CHASSIS_MODE" | "INPUT_SW_SHOOT_MODE" | null {
    const upper = id.toUpperCase();
    if (upper.includes("GIMBAL")) return "INPUT_SW_GIMBAL_MODE";
    if (upper.includes("CHASSIS")) return "INPUT_SW_CHASSIS_MODE";
    if (upper.includes("SHOOT")) return "INPUT_SW_SHOOT_MODE";
    return null;
}

function positiveInteger(value: unknown, maximum: number): number | null {
    return typeof value === "number" && Number.isInteger(value) && value >= 1 && value <= maximum ? value : null;
}

function mapPhysical(data: ConfigValues | null | undefined, changes: Changes, kind: "axis" | "sw", logical: number): number | null {
    const member = kind === "axis" ? "ElrsChMap" : "ElrsSwMap";
    return positiveInteger(fieldValue(field(data, `input.elrs.${member}.${logical - 1}`), changes), 16);
}

export function functionChannel(fieldId: string, data: ConfigValues | null | undefined, changes: Changes): number | null {
    const match = /^input\.(axis|sw)\.[^.]+\.channel$/.exec(fieldId);
    if (!match) return null;
    const kind = match[1] as "axis" | "sw";
    const logical = positiveInteger(fieldValue(field(data, fieldId), changes), kind === "axis" ? 5 : 2);
    return logical === null ? null : mapPhysical(data, changes, kind, logical);
}

function useLabel(item: ConfigField, data: ConfigValues | null | undefined, changes: Changes): string {
    let label = item.label.replace(/\s*逻辑通道\s*$/, "").replace(/Yaw/g, "水平").replace(/Pitch/g, "俯仰");
    const invert = fieldValue(field(data, item.id.replace(/\.channel$/, ".invert")), changes);
    if (invert === true || invert === 1) label += "·反向";
    return label;
}

export function channelUses(data: ConfigValues | null | undefined, changes: Changes): string[][] {
    const result = Array.from({ length: 16 }, () => [] as string[]);
    const usedLogical = { axis: new Set<number>(), sw: new Set<number>() };
    for (const item of fields(data)) {
        const match = /^input\.(axis|sw)\.[^.]+\.channel$/.exec(item.id);
        if (!match) continue;
        const kind = match[1] as "axis" | "sw";
        const logical = positiveInteger(fieldValue(item, changes), kind === "axis" ? 5 : 2);
        const physical = logical === null ? null : mapPhysical(data, changes, kind, logical);
        if (logical !== null) usedLogical[kind].add(logical);
        if (physical !== null) result[physical - 1].push(useLabel(item, data, changes));
    }
    for (const kind of ["axis", "sw"] as const) {
        const count = kind === "axis" ? 5 : 2;
        for (let logical = 1; logical <= count; logical += 1) {
            if (usedLogical[kind].has(logical)) continue;
            const physical = mapPhysical(data, changes, kind, logical);
            if (physical !== null) result[physical - 1].push(`${kind === "axis" ? "逻辑轴" : "拨杆"}${logical}`);
        }
    }
    return result;
}
