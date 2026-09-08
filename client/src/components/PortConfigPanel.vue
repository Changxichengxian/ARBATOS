<script setup lang="ts">
import { computed } from "vue";

type PortChoice = { port: string; label: string; pins: string[]; available: boolean; reasons: string[] };
type PortRole = {
    id: string; label: string; active: boolean; supported: boolean;
    selected: string | { port: string; baud?: number } | null;
    default_baud: number; baud_editable: boolean; protocols: string[];
    note?: string; options: PortChoice[];
};
export type PortOptions = {
    schema: number; target: string; board: string; explicit: boolean;
    assignments: Record<string, { port: string; baud?: number; implicit?: boolean }>;
    roles: PortRole[]; warnings?: string[];
};
const props = defineProps<{
    modelValue: Record<string, unknown>; data: PortOptions | null; disabled?: boolean;
}>();
const emit = defineEmits<{ "update:modelValue": [value: Record<string, unknown>]; }>();
const serviceByRole: Record<string, string> = {
    rc_sbus: "RC_SBUS", elrs_crsf: "ELRS_LINK", tuning_aux: "HOST_LINK", referee: "REFEREE_RX",
};
const draftPorts = computed(() => (props.modelValue.ports ?? {}) as Record<string, any>);
function assigned(role: PortRole): { port: string; baud?: number } {
    const raw = draftPorts.value[role.id] ?? props.data?.assignments?.[role.id] ?? role.selected;
    if (typeof raw === "string") return { port: raw };
    return raw && typeof raw === "object" ? raw : { port: "" };
}
function enabled(role: PortRole) {
    const service = serviceByRole[role.id];
    if (service) return Array.isArray(props.modelValue.services) && props.modelValue.services.includes(service);
    return role.active;
}
function editableDraft() {
    const next = JSON.parse(JSON.stringify(props.modelValue));
    if (!next.ports) {
        // 第一次选口时把现有接线一并写明，避免只新增 ELRS 就丢掉 SBUS、裁判和 RS485。
        next.ports = {};
        for (const role of props.data?.roles ?? []) {
            const previous = assigned(role);
            if (role.supported && previous.port)
                next.ports[role.id] = { port: previous.port, baud: role.baud_editable ? previous.baud ?? role.default_baud : role.default_baud };
        }
    }
    return next;
}
function updatePort(role: PortRole, port: string) {
    const next = editableDraft();
    if (port) next.ports[role.id] = { port, baud: role.baud_editable ? assigned(role).baud ?? role.default_baud : role.default_baud };
    else delete next.ports[role.id];
    if (port && role.id === "external_imu") next.services = [...new Set([...(next.services ?? []), "IMU"])];
    emit("update:modelValue", next);
}
function updateBaud(role: PortRole, event: Event) {
    const next = editableDraft();
    const raw = (event.target as HTMLInputElement).value;
    next.ports ??= {};
    next.ports[role.id] = { port: assigned(role).port, baud: raw === "" ? "" : Number(raw) };
    emit("update:modelValue", next);
}
function toggleRole(role: PortRole, value: boolean) {
    const service = serviceByRole[role.id];
    if (!service) return;
    const next = editableDraft();
    const services: string[] = next.services ?? [];
    next.services = value ? [...new Set([...services, service])] : services.filter(item => item !== service);
    if (value && assigned(role).port) {
        next.ports ??= {};
        next.ports[role.id] = { port: assigned(role).port, baud: role.default_baud };
    }
    emit("update:modelValue", next);
}
</script>

<template>
    <article class="panel port-panel">
        <h2>板载接口分配</h2>
        <p class="muted">这里只列出当前板子已经定义引脚的接口。启用服务并选择接口后，和车型配置一起保存。</p>
        <p v-if="!data" class="muted">正在读取板级接口…</p>
        <div v-for="role in data?.roles ?? []" :key="role.id" class="port-row">
            <div class="port-purpose">
                <strong>{{ role.label }}</strong>
                <label v-if="serviceByRole[role.id]" class="port-enable">
                    <input type="checkbox" :checked="enabled(role)" :disabled="disabled || !role.supported"
                        @change="toggleRole(role, ($event.target as HTMLInputElement).checked)" />启用服务
                </label>
                <small v-else>{{ !role.supported ? '当前驱动未实现' : role.id === 'external_imu' ? 'HI14 UART 姿态源' : '总线接口' }}</small>
            </div>
            <label class="field">
                <span>物理接口</span>
                <select :aria-label="role.label + '接口'" :value="assigned(role).port"
                    :disabled="disabled || !role.supported" @change="updatePort(role, ($event.target as HTMLSelectElement).value)">
                    <option value="">{{ !role.supported ? '需要先接入设备协议' : role.id === 'external_imu' ? '使用板载陀螺仪' : '使用默认 / 未分配' }}</option>
                    <option v-for="option in role.options" :key="option.port" :value="option.port"
                        :disabled="!option.available && assigned(role).port !== option.port">
                        {{ option.label }}{{ !option.available && option.reasons.length ? ` · ${option.reasons.join('；')}` : '' }}
                    </option>
                </select>
            </label>
            <label class="field baud-field">
                <span>波特率</span>
                <input type="number" :value="assigned(role).baud ?? role.default_baud" min="1200" max="3000000" step="1"
                    :disabled="disabled || !role.supported || !role.baud_editable || !assigned(role).port" @input="updateBaud(role, $event)" />
            </label>
            <p v-if="role.note" class="port-note">{{ role.note }}</p>
        </div>
        <div v-if="data?.warnings?.length" class="validation">
            <span v-for="warning in data.warnings" :key="warning">{{ warning }}</span>
        </div>
        <p class="muted">SPI / SX1281 的源码继续保留；外置 ELRS 使用接收机的 CRSF 串口输出。</p>
    </article>
</template>

<style scoped>
.port-row { display: grid; grid-template-columns: minmax(150px, 1fr) minmax(250px, 2fr) 125px; gap: 12px; padding: 15px 0; border-top: 1px solid #e6e9eb; }
.port-purpose { display: flex; flex-direction: column; gap: 8px; font-size: 12px; }
.port-purpose small { color: #88949a; }
.port-enable { font-size: 11px; display: flex; gap: 6px; align-items: center; color: #637079; }
.port-enable input { accent-color: #d4a014; }
.port-note { grid-column: 2 / -1; margin: -4px 0 0; font-size: 11px; line-height: 1.5; color: #718087; }
.field select, .field input { width: 100%; min-width: 0; }
</style>
