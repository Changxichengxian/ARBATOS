import type { SerialSample } from "./types";

export function parseSerialCsv(content: string): { samples: SerialSample[]; channels: number } {
    const rows = content.split(/\r?\n/).map((row) => row.trim()).filter(Boolean);
    if (rows.length > 100001) throw new Error("CSV 最多 100000 行");
    if (!rows.length) throw new Error("CSV 是空文件，无法回放。");
    const first = rows[0].split(",").map((x) => x.trim());
    const firstName = first[0].toLowerCase();
    const timeColumn = ["time", "time_s", "timestamp", "tick_ms", "time_ms"].includes(firstName);
    const isNumber = (value: string) => value !== "" && Number.isFinite(Number(value));
    const hasHeader = timeColumn || first.some((cell) => !isNumber(cell));
    const start = hasHeader ? 1 : 0;
    const channels = first.length - (timeColumn ? 1 : 0);
    if (channels < 1 || channels > 16) throw new Error("CSV 必须有 1 至 16 个数值通道");
    const samples: SerialSample[] = [];
    for (const [i, row] of rows.slice(start).entries()) {
        const cells = row.split(",").map((x) => x.trim());
        if (cells.length !== channels + (timeColumn ? 1 : 0)) throw new Error(`第 ${i + start + 1} 行列数不一致`);
        if (cells.some((cell) => cell === "")) throw new Error(`第 ${i + start + 1} 行存在空单元格`);
        const values = cells.slice(timeColumn ? 1 : 0).map(Number);
        if (values.some((value) => !Number.isFinite(value))) throw new Error(`第 ${i + start + 1} 行包含非数值通道`);
        let time: number | string = `#${i + 1}`;
        if (timeColumn) {
            const rawTime = Number(cells[0]);
            if (!Number.isFinite(rawTime)) throw new Error(`第 ${i + start + 1} 行时间无效`);
            time = firstName === "tick_ms" || firstName === "time_ms" ? rawTime / 1000 : rawTime;
            if (i > 0 && Number(time) < Number(samples[i - 1].time)) throw new Error(`第 ${i + start + 1} 行时间倒退`);
        }
        samples.push({ seq: i + 1, time, values });
    }
    if (!samples.length) throw new Error("CSV 没有数据行");
    return { samples, channels };
}
