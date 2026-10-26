import type { MotorSnapshot } from "./chassis";
import type { ReceiverSnapshot } from "./receiver";

export type Robot = {
    name: string;
    config: Record<string, unknown> | null;
    revision: string;
    files: { name: string }[];
    resolved: { tasks?: unknown[]; controllers?: unknown[]; [key: string]: unknown } | null;
    validation: { ok: boolean; errors: string[] };
};

export type Job = {
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

export type ImuLast = Record<string, number | string> & {
    target: string; state: number; id: number; seq: number; flags: number;
    temp_mC: number; target_mC: number; roll_mdeg: number; pitch_mdeg: number; yaw_mdeg: number;
    gx_mdps: number; gy_mdps: number; gz_mdps: number; ax_mmps2: number; ay_mmps2: number; az_mmps2: number;
    mapX: number; mapY: number; mapZ: number;
};

export type SerialStatus = {
    connected: boolean;
    port?: string;
    baud?: number;
    format?: string;
    rxBytes: number;
    txBytes: number;
    error?: string;
    receiver?: ReceiverSnapshot;
    motors?: MotorSnapshot;
    imu?: { last: ImuLast | null; fresh: boolean; healthy: boolean; safeToStart: boolean; accepted: number; dropped: number; invalid: number; restarts: number; observation: { seconds: number; angleDriftDegPerMin: Record<string, number>; gyroMeanDegPerSec: Record<string, number> } | null };
};

export type SerialLine = { seq: number; time: number | string; text: string };
export type SerialSample = { seq: number; time: number | string; values: number[] };

export type FlashConfirmation = {
    target: string;
    artifactRevision: string;
    path: string;
    sha: string;
    buildTime?: string;
    warnings: string[];
    ready: boolean;
    reason?: string;
};
