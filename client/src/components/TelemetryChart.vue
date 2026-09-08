<script setup lang="ts">
import { computed, ref, watch } from "vue";

type Sample = { seq: number; time: number | string; values: number[] };
type TimedSample = {
    sample: Sample;
    time: number;
    hasNumericTime: boolean;
    index: number;
};

const props = defineProps<{
    samples: Sample[];
    channels: number;
    offline?: boolean;
    offlineLabel?: string;
}>();

const colors = [
    "#f6bf27",
    "#3785d7",
    "#e05b5b",
    "#49a887",
    "#8b67cc",
    "#dd7f35",
    "#318b9d",
    "#b0587f",
];
const enabled = ref<boolean[]>(Array.from({ length: 16 }, () => true));
const windowSize = ref(1000);
const windowStart = ref(0);
const windowChoices = [120, 500, 1000, 5000, 10000, 50000];
const maxDrawPoints = 720;

function numericSampleTime(sample: Sample) {
    const time =
        typeof sample.time === "number" ? sample.time : Number(sample.time);
    return Number.isFinite(time) ? time : null;
}

const displaySize = computed(() => {
    if (!props.offline) return Math.min(120, props.samples.length);
    return windowSize.value === 0
        ? props.samples.length
        : Math.min(windowSize.value, props.samples.length);
});
const maxStart = computed(() =>
    Math.max(0, props.samples.length - displaySize.value),
);
const startIndex = computed(() =>
    props.offline
        ? Math.min(windowStart.value, maxStart.value)
        : maxStart.value,
);
const windowSamples = computed<TimedSample[]>(() => {
    const start = startIndex.value;
    const end = start + displaySize.value;
    return props.samples.slice(start, end).map((sample, offset) => {
        const time = numericSampleTime(sample);
        return {
            sample,
            time: time ?? start + offset,
            hasNumericTime: time !== null,
            index: start + offset,
        };
    });
});
const channelCount = computed(() => {
    let widestSample = 0;
    for (const item of windowSamples.value) {
        widestSample = Math.max(widestSample, item.sample.values.length);
    }
    return Math.min(16, props.channels, widestSample);
});
const visibleChannels = computed(() =>
    Array.from({ length: channelCount.value }, (_, channel) => channel).filter(
        (channel) => enabled.value[channel],
    ),
);

const timeRange = computed(() => {
    if (!windowSamples.value.length) return { low: 0, high: 1, equal: true };
    let low = windowSamples.value[0].time;
    let high = low;
    for (const item of windowSamples.value) {
        low = Math.min(low, item.time);
        high = Math.max(high, item.time);
    }
    return { low, high, equal: low === high };
});
const useSampleSequence = computed(
    () =>
        timeRange.value.equal ||
        windowSamples.value.some((item) => !item.hasNumericTime),
);
const valueRange = computed(() => {
    let low = Infinity;
    let high = -Infinity;
    for (const item of windowSamples.value) {
        for (const channel of visibleChannels.value) {
            const value = item.sample.values[channel];
            if (!Number.isFinite(value)) continue;
            low = Math.min(low, value);
            high = Math.max(high, value);
        }
    }
    if (!Number.isFinite(low) || !Number.isFinite(high))
        return { low: 0, high: 1 };
    if (low === high) {
        const padding = Math.max(Math.abs(low) * 0.05, 1);
        return { low: low - padding, high: high + padding };
    }
    return { low, high };
});

function decimate(items: TimedSample[]) {
    if (items.length <= maxDrawPoints) return items;
    const channels = visibleChannels.value;
    const perBucket = Math.max(2, channels.length * 2 + 2);
    const bucketCount = Math.max(1, Math.floor(maxDrawPoints / perBucket));
    const bucketSize = Math.ceil(items.length / bucketCount);
    const chosen = new Set<number>();

    for (let start = 0; start < items.length; start += bucketSize) {
        const end = Math.min(items.length, start + bucketSize);
        chosen.add(start);
        chosen.add(end - 1);
        for (const channel of channels) {
            let minIndex = start;
            let maxIndex = start;
            for (let index = start + 1; index < end; index += 1) {
                if (
                    items[index].sample.values[channel] <
                    items[minIndex].sample.values[channel]
                )
                    minIndex = index;
                if (
                    items[index].sample.values[channel] >
                    items[maxIndex].sample.values[channel]
                )
                    maxIndex = index;
            }
            chosen.add(minIndex);
            chosen.add(maxIndex);
        }
    }
    return [...chosen]
        .sort((left, right) => left - right)
        .map((index) => items[index]);
}

const drawSamples = computed(() => decimate(windowSamples.value));
function xPosition(item: TimedSample) {
    if (useSampleSequence.value) {
        return (
            ((item.index - startIndex.value) /
                Math.max(1, windowSamples.value.length - 1)) *
            100
        );
    }
    return (
        ((item.time - timeRange.value.low) /
            (timeRange.value.high - timeRange.value.low)) *
        100
    );
}
const paths = computed(() =>
    Array.from({ length: channelCount.value }, (_, channel) => {
        if (!enabled.value[channel]) return "";
        let path = "";
        let needsMove = true;
        drawSamples.value.forEach((item) => {
            const value = item.sample.values[channel];
            if (!Number.isFinite(value)) {
                needsMove = true;
                return;
            }
            const x = xPosition(item);
            const y =
                58 -
                ((value - valueRange.value.low) /
                    (valueRange.value.high - valueRange.value.low)) *
                    50;
            path += `${needsMove ? "M" : "L"} ${x.toFixed(3)} ${y.toFixed(3)} `;
            needsMove = false;
        });
        return path;
    }),
);

function formatNumber(value: number) {
    if (value === 0) return "0";
    const magnitude = Math.abs(value);
    if (magnitude >= 100000 || magnitude < 0.001) return value.toExponential(3);
    return Number(value.toPrecision(5)).toString();
}
function formatTime(value: number) {
    if (value === 0) return "0 s";
    if (Math.abs(value) >= 946684800) {
        const date = new Date(
            Math.abs(value) > 100000000000 ? value : value * 1000,
        );
        if (!Number.isNaN(date.getTime()))
            return date.toLocaleString("zh-CN", { hour12: false });
    }
    return `${formatNumber(value)} s`;
}
const xLabels = computed(() =>
    [0, 0.5, 1].map((ratio) => {
        if (useSampleSequence.value) {
            const index = Math.round((windowSamples.value.length - 1) * ratio);
            const item = windowSamples.value[Math.max(0, index)];
            return { x: ratio * 100, label: item ? `#${item.sample.seq}` : "" };
        }
        return {
            x: ratio * 100,
            label: formatTime(
                timeRange.value.low +
                    (timeRange.value.high - timeRange.value.low) * ratio,
            ),
        };
    }),
);
const windowDescription = computed(() => {
    if (!props.offline) return `最近 ${windowSamples.value.length} 个点`;
    if (!windowSamples.value.length) return "没有可绘制的点";
    return `第 ${startIndex.value + 1}–${startIndex.value + windowSamples.value.length} 点，共 ${props.samples.length} 点`;
});

function resetWindowStart() {
    windowStart.value = 0;
}
watch(
    () => props.samples.length,
    () => {
        windowStart.value = Math.min(windowStart.value, maxStart.value);
    },
);
watch(
    () => props.offline,
    () => resetWindowStart(),
);
watch(windowSize, () => resetWindowStart());
</script>

<template>
    <article class="chart panel">
        <div class="panel-head chart-head">
            <div>
                <h2>采样曲线</h2>
                <p class="muted">
                    {{ offline ? offlineLabel ?? "本机 CSV 离线回放" : "实时串口采样" }} ·
                    {{ windowDescription }}
                </p>
            </div>
            <div class="legend" aria-label="通道显示开关">
                <label v-for="(_, index) in paths" :key="index">
                    <input v-model="enabled[index]" type="checkbox" />
                    <i
                        :style="{ background: colors[index % colors.length] }"
                    />通道 {{ index + 1 }}
                </label>
            </div>
        </div>
        <div v-if="offline && samples.length" class="chart-tools">
            <label>
                浏览范围
                <select v-model.number="windowSize" @change="resetWindowStart">
                    <option :value="0">整段数据</option>
                    <option
                        v-for="size in windowChoices"
                        :key="size"
                        :value="size"
                    >
                        最近 {{ size }} 点
                    </option>
                </select>
            </label>
            <label class="window-slider">
                位置
                <input
                    v-model.number="windowStart"
                    type="range"
                    min="0"
                    :max="maxStart"
                    step="1"
                />
            </label>
        </div>
        <div class="chart-box">
            <div class="y-axis">
                <span>{{ formatNumber(valueRange.high) }}</span>
                <span>{{
                    formatNumber((valueRange.high + valueRange.low) / 2)
                }}</span>
                <span>{{ formatNumber(valueRange.low) }}</span>
            </div>
            <svg
                viewBox="0 0 100 60"
                preserveAspectRatio="none"
                aria-label="采样曲线图"
            >
                <path
                    v-for="(path, index) in paths"
                    :key="index"
                    :d="path"
                    fill="none"
                    :stroke="colors[index % colors.length]"
                    stroke-width=".7"
                    vector-effect="non-scaling-stroke"
                />
            </svg>
            <div class="x-axis">
                <span
                    v-for="label in xLabels"
                    :key="label.x"
                    :style="{ left: `${label.x}%` }"
                    >{{ label.label }}</span
                >
            </div>
            <p v-if="!windowSamples.length">
                {{
                    offline
                        ? "文件中没有可绘制的数值行。"
                        : "尚未收到数值采样。"
                }}
            </p>
            <small v-else-if="!visibleChannels.length" class="chart-note"
                >请选择至少一个通道以缩放和显示曲线。</small
            >
            <small v-else-if="useSampleSequence" class="chart-note"
                >横轴：采样序号</small
            >
        </div>
    </article>
</template>

<style scoped>
.chart {
    height: auto;
    min-height: 340px;
}
.chart-tools {
    display: flex;
    align-items: center;
    gap: 16px;
    margin: 10px 0 8px;
    padding: 8px 10px;
    color: #566166;
    background: #fffaf0;
    border: 1px solid #f0d98b;
    font-size: 12px;
}
.chart-tools label {
    display: flex;
    align-items: center;
    gap: 7px;
}
.chart-tools select {
    min-width: 104px;
    border: 1px solid #d8c77f;
    border-radius: 3px;
    background: #fff;
    color: #39454a;
    padding: 3px 5px;
}
.window-slider {
    flex: 1;
}
.window-slider input {
    width: min(360px, 100%);
    accent-color: #e7b52a;
}
.chart-note {
    position: absolute;
    right: 8px;
    top: 8px;
    color: #8b6b10;
    background: rgba(255, 250, 240, 0.9);
    padding: 3px 5px;
}
@media (max-width: 720px) {
    .chart-head,
    .chart-tools {
        align-items: stretch;
        flex-direction: column;
        gap: 8px;
    }
    .window-slider input {
        width: 100%;
    }
}
</style>
