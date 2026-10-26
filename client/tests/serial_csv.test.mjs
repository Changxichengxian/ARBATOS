import test from "node:test";
import assert from "node:assert/strict";
import { parseSerialCsv } from "../src/serialCsv.ts";

test("CSV 时间列换算为秒并保留有序采样", () => {
    const parsed = parseSerialCsv("time_ms,imu_x,imu_y\r\n100,1,2\r\n250,3,4");
    assert.equal(parsed.channels, 2);
    assert.deepEqual(parsed.samples, [
        { seq: 1, time: 0.1, values: [1, 2] },
        { seq: 2, time: 0.25, values: [3, 4] },
    ]);
});

test("无时间列时用行号标记，CSV 错误维持用户可读原因", () => {
    assert.deepEqual(parseSerialCsv("1,2\n3,4"), {
        channels: 2,
        samples: [
            { seq: 1, time: "#1", values: [1, 2] },
            { seq: 2, time: "#2", values: [3, 4] },
        ],
    });
    assert.throws(() => parseSerialCsv("time_ms,x\n200,1\n100,2"), /时间倒退/);
    assert.throws(() => parseSerialCsv("time,x\n1,2,3"), /列数不一致/);
    assert.throws(() => parseSerialCsv("x\nnope"), /非数值通道/);
});

test("无表头边界保留总行数，显示窗口由串口模块截取", () => {
    const parsed = parseSerialCsv(Array.from({ length: 100001 }, (_, i) => String(i)).join("\n"));
    assert.equal(parsed.samples.length, 100001);
    assert.equal(parsed.samples.slice(-100000)[0].seq, 2);
});
