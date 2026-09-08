<script setup lang="ts">
import { computed, ref, watch } from "vue";

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
const props = defineProps<{
    modelValue: Record<string, unknown>;
    controllers: Controller[];
    services: Service[];
    disabled?: boolean;
}>();
const emit = defineEmits<{
    "update:modelValue": [value: Record<string, unknown>];
}>();
const local = ref<Record<string, any>>({});
// 仅翻译显示名称，控制器参数和校验规则仍取自后端声明。
const domainNames: Record<string, string> = {
    chassis: "底盘",
    gimbal: "云台",
    wheelleg: "轮腿",
    arm: "机械臂",
    shoot: "发射机构",
};
const controllerNames: Record<string, string> = {
    classic: "经典底盘",
    differential: "双轮差速底盘",
    single: "单云台",
    dual_yaw: "双水平轴云台",
    speed_gimbal: "双轴转速云台",
    mit: "MIT 轮腿",
    standard: "标准机械臂",
    rm: "常规发射机构",
};
const serviceNames: Record<string, string> = {
    SERVO: "PWM 舵机",
    RC_SBUS: "SBUS 遥控接收",
    HEALTH_MONITOR: "运行健康检查",
    SDLOG: "SD 卡日志",
    CAN_COMMAND_TX: "CAN 命令发送",
    CAN_FEEDBACK_RX: "CAN 反馈接收",
    IMU: "姿态传感器",
    HOST_LINK: "上位机通信",
    ELRS_LINK: "ELRS 遥控链路",
    REFEREE_RX: "裁判系统接收",
    BATTERY_MONITOR: "电池监测",
    CALIBRATION: "传感器校准",
    STATUS_LED: "状态指示灯",
    STARTUP_SERVICE: "启动服务",
};
const featureLabels: Record<string, string> = {
    subboard_music: "副板音乐",
    imu_mount: "使用 IMU 安装变换",
};

function clone(value: Record<string, unknown>) {
    return JSON.parse(JSON.stringify(value));
}
function reset(value: Record<string, unknown>) {
    local.value = clone(value);
}
watch(() => props.modelValue, reset, { immediate: true, deep: true });
const domains = computed(() =>
    Object.keys(domainNames).filter((d) =>
        props.controllers.some((c) => c.domain === d),
    ),
);
function choices(domain: string) {
    return props.controllers.filter((x) => x.domain === domain);
}
function selected(domain: string) {
    const raw = local.value.controllers?.[domain];
    return raw && typeof raw === "object" && raw.type !== "none"
        ? String(raw.type ?? "")
        : "";
}
function selectedSpec(domain: string) {
    return choices(domain).find((x) => x.name === selected(domain));
}
function ensureDomain(domain: string) {
    local.value.controllers ??= {};
    return (local.value.controllers[domain] ??= {});
}
function updateType(domain: string, type: string) {
    if (!type) {
        delete local.value.controllers?.[domain];
        commit();
        return;
    }
    const spec = choices(domain).find((c) => c.name === type);
    if (!spec) return;
    const previous = ensureDomain(domain);
    const current: Record<string, any> = { type };
    if (!spec.builtin) {
        current.motors = Array.from(
            { length: spec.outputs || 0 },
            (_, i) => previous.motors?.[i] || "",
        );
        current.period_ms = 2;
        current.parameters = {};
        for (const [key, p] of Object.entries(spec.parameters))
            if (p.default !== undefined) current.parameters[key] = p.default;
    }
    local.value.controllers[domain] = current;
    commit();
}
function updateValue(domain: string, key: string, event: Event) {
    const input = event.target as HTMLInputElement;
    const parameters = (ensureDomain(domain).parameters ??= {});
    if (input.value.trim() === "") delete parameters[key];
    else parameters[key] = Number(input.value);
    commit();
}
function updateMotor(domain: string, index: number, value: string) {
    ensureDomain(domain).motors[index] = value.trim();
    commit();
}
function setService(symbol: string, checked: boolean) {
    const list = Array.isArray(local.value.services)
        ? [...local.value.services]
        : [];
    local.value.services = checked
        ? [...new Set([...list, symbol])]
        : list.filter((x) => x !== symbol);
    commit();
}
function setFeature(name: string, checked: boolean) {
    local.value.features ??= {};
    local.value.features[name] = checked;
    commit();
}
function commit() {
    emit("update:modelValue", clone(local.value));
}
const featureNames = computed(() =>
    Object.keys(local.value.features ?? {}).filter(
        (key) => typeof local.value.features[key] === "boolean",
    ),
);
</script>

<template>
    <div class="robot-form">
        <section class="panel">
            <h2>控制器</h2>
            <p class="muted">
                选择机构类型并绑定本车电机，保存前会检查依赖和参数。
            </p>
            <div
                class="controller-form"
                v-for="domain in domains"
                :key="domain"
            >
                <label class="field"
                    ><span>{{ domainNames[domain] }}</span
                    ><select
                        :disabled="disabled"
                        :value="selected(domain)"
                        @change="
                            updateType(
                                domain,
                                ($event.target as HTMLSelectElement).value,
                            )
                        "
                    >
                        <option value="">未启用</option>
                        <option
                            v-for="item in choices(domain)"
                            :key="item.name"
                            :value="item.name"
                        >
                            {{ controllerNames[item.name] || item.name }} ·
                            {{ item.name }}
                        </option>
                    </select></label
                >
                <p v-if="selectedSpec(domain)?.builtin" class="muted wide">
                    使用现有控制链。PID、中位和限幅可在“参数与设备”页修改。
                </p>
                <template v-else-if="selectedSpec(domain)">
                    <label
                        v-for="(_, index) in Array(
                            selectedSpec(domain)?.outputs || 0,
                        )"
                        :key="index"
                        class="field"
                        ><span>输出 {{ index + 1 }} · 电机名</span
                        ><input
                            :aria-label="`${domainNames[domain]}输出 ${index + 1} 电机名`"
                            :disabled="disabled"
                            :value="ensureDomain(domain).motors?.[index] || ''"
                            placeholder="例如 motor.chassis0"
                            @input="
                                updateMotor(
                                    domain,
                                    index,
                                    ($event.target as HTMLInputElement).value,
                                )
                            "
                    /></label>
                    <label class="field"
                        ><span>运行周期（毫秒）</span
                        ><input
                            type="number"
                            min="1"
                            max="100"
                            step="1"
                            :disabled="disabled"
                            :value="ensureDomain(domain).period_ms ?? 2"
                            @input="
                                ensureDomain(domain).period_ms = Number(
                                    ($event.target as HTMLInputElement).value,
                                );
                                commit();
                            "
                    /></label>
                    <label
                        v-for="(parameter, key) in selectedSpec(domain)
                            ?.parameters"
                        :key="key"
                        class="field"
                        ><span>{{ parameter.description || key }}</span
                        ><input
                            type="number"
                            :aria-label="String(key)"
                            :min="parameter.min"
                            :max="parameter.max"
                            step="any"
                            :disabled="disabled"
                            :value="
                                ensureDomain(domain).parameters?.[key] ??
                                parameter.default ??
                                ''
                            "
                            @input="updateValue(domain, String(key), $event)"
                        /><small
                            >{{ key
                            }}<template v-if="parameter.unit">
                                · {{ parameter.unit }}</template
                            ></small
                        ><small
                            >范围 {{ parameter.min ?? "不限" }} 至
                            {{ parameter.max ?? "不限" }} · 默认
                            {{ parameter.default ?? "需填写" }}</small
                        ></label
                    >
                    <p class="muted wide">
                        电机名对应 ConfigHardware.inc
                        中的设备，输出顺序按控制器说明填写。
                    </p>
                </template>
            </div>
        </section>
        <div class="form-two-col">
            <section class="panel">
                <h2>主动启用的服务</h2>
                <p class="muted">控制器所需依赖会自动加入，无需重复勾选。</p>
                <label
                    v-for="service in services"
                    :key="service.symbol"
                    class="check-row"
                    :class="{ unavailable: !service.available }"
                    ><input
                        type="checkbox"
                        :disabled="
                            disabled ||
                            (!service.available &&
                                !(local.services ?? []).includes(
                                    service.symbol,
                                ))
                        "
                        :checked="
                            (local.services ?? []).includes(service.symbol)
                        "
                        @change="
                            setService(
                                service.symbol,
                                ($event.target as HTMLInputElement).checked,
                            )
                        "
                    /><span
                        ><b>{{
                            serviceNames[service.symbol] || service.name
                        }}</b
                        ><small>{{ service.symbol }}</small></span
                    ><em v-if="!service.available">尚未接入</em></label
                >
            </section>
            <section class="panel">
                <h2>功能开关</h2>
                <p v-if="!featureNames.length" class="muted">
                    当前车型没有额外功能开关。
                </p>
                <label
                    v-for="name in featureNames"
                    :key="name"
                    class="check-row"
                    ><input
                        type="checkbox"
                        :disabled="disabled"
                        :checked="local.features[name]"
                        @change="
                            setFeature(
                                name,
                                ($event.target as HTMLInputElement).checked,
                            )
                        "
                    /><span
                        ><b>{{ featureLabels[name] || name }}</b
                        ><small>{{ name }}</small></span
                    ></label
                >
                <p class="muted">
                    接线、电机和遥控映射可在“参数与设备”页修改；原文件仍可直接编辑。
                </p>
            </section>
        </div>
    </div>
</template>
