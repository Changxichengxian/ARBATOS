<script setup lang="ts">
import { computed, ref } from "vue";

export type ConfigField = {
    id: string;
    label: string;
    value: number | string | boolean;
    type: string;
    options?: { value: number | string; label: string }[];
    editable?: boolean;
    min?: number;
    max?: number;
    step?: number | string;
    unit?: string;
    description?: string;
};
export type ConfigGroup = {
    id: string;
    label: string;
    category?: string;
    fields: ConfigField[];
    description?: string;
};
export type ConfigValues = {
    target: string;
    revision: string;
    groups: ConfigGroup[];
    warnings?: string[];
    models?: Record<string, { ratio: number; encoder_bits: number; transport: string; can_id_base: number }>;
};
const props = defineProps<{
    data: ConfigValues | null;
    changes: Record<string, unknown>;
    disabled?: boolean;
}>();
const emit = defineEmits<{
    change: [id: string, value: unknown];
}>();
const query = ref("");
const category = ref("tuning");
const categories = [
    { id: "tuning", label: "PID 与中位" },
    { id: "motors", label: "电机与传动" },
    { id: "input", label: "遥控映射" },
    { id: "servo", label: "舵机" },
    { id: "all", label: "全部参数" },
];
function groupCategory(group: ConfigGroup) {
    if (group.category) return group.category;
    if (/motor/i.test(group.id)) return "motors";
    if (/servo/i.test(group.id)) return "servo";
    if (/input/i.test(group.id)) return "input";
    return "tuning";
}
const groups = computed(() => {
    const search = query.value.trim().toLowerCase();
    return (props.data?.groups ?? []).map(group => {
        const sameCategory = category.value === "all" || groupCategory(group) === category.value;
        const fields = group.fields.filter(field => {
            if (!sameCategory) return false;
            if (category.value === "tuning" && !/pid|middle|soft_limit|init_(pitch|yaw)_set/i.test(field.id)) return false;
            return !search || `${group.label} ${field.label} ${field.id} ${field.description ?? ""}`.toLowerCase().includes(search);
        });
        return { ...group, fields };
    }).filter(group => group.fields.length > 0).flatMap(group => {
        const pidGroups = new Map<string, ConfigGroup>();
        const remaining: ConfigField[] = [];
        for (const field of group.fields) {
            const match = field.id.match(/^(.*\.([A-Za-z0-9_]*pid))\[\d\]$/);
            if (!match) { remaining.push(field); continue; }
            const names: Record<string, string> = {
                yaw_speed_pid: "水平轴速度环", pitch_speed_pid: "俯仰轴速度环",
                yaw_encode_angle_pid: "水平轴编码器角度环", pitch_encode_angle_pid: "俯仰轴编码器角度环",
                yaw_gyro_angle_pid: "水平轴姿态角度环", pitch_gyro_angle_pid: "俯仰轴姿态角度环",
                motor_speed_pid: "车轮速度环", follow_gimbal_pid: "跟随云台角度环",
                fric_speed_pid: "摩擦轮速度环", trigger_speed_pid: "拨盘速度环", trigger_angle_pid: "拨盘角度环",
            };
            if (!pidGroups.has(match[1])) pidGroups.set(match[1], { ...group, id: match[1], label: `${group.label} · ${names[match[2]] ?? match[2]}`, fields: [] });
            const labels: Record<string, string> = { kp: "比例 Kp", ki: "积分 Ki", kd: "微分 Kd", max_out: "输出上限", max_iout: "积分上限" };
            pidGroups.get(match[1])!.fields.push({ ...field, label: labels[field.label] ?? field.label,
                description: field.description?.replace(/(?:\[\d+\])?(?:kp|ki|kd|max_out|max_iout)\s*/g, "").replace(/^\s*\|\s*/, "") });
        }
        return [...pidGroups.values(), ...(remaining.length ? [{ ...group, fields: remaining }] : [])];
    });
});
const count = computed(() => groups.value.reduce((total, group) => total + group.fields.length, 0));
const modifiedCount = computed(() => Object.keys(props.changes).length);
function valueOf(field: ConfigField) {
    if (field.id.endsWith(".built_in_reduction_ratio")) {
        const modelId = field.id.replace(/built_in_reduction_ratio$/, "model");
        const model = props.data?.models?.[String(props.changes[modelId] ?? "")];
        if (model) return model.ratio;
    }
    return Object.hasOwn(props.changes, field.id) ? props.changes[field.id] : field.value;
}
function inputValue(field: ConfigField, event: Event) {
    const input = event.target as HTMLInputElement;
    const value = input.value === "" ? "" : Number(input.value);
    emit("change", field.id, value);
}
function selectedValue(field: ConfigField, event: Event) {
    const value = (event.target as HTMLSelectElement).value;
    const option = field.options?.find(option => String(option.value) === value);
    if (option) emit("change", field.id, option.value);
}
</script>

<template>
    <div class="config-values">
        <div class="parameter-tabs" aria-label="参数分类">
            <button v-for="item in categories" :key="item.id" class="button"
                :class="{ primary: category === item.id }" @click="category = item.id">
                {{ item.label }}
            </button>
        </div>
        <div class="parameter-filter">
            <input v-model="query" type="search" placeholder="搜索参数、逻辑轴、通道或原字段名" aria-label="搜索参数" />
            <span>{{ count }} 项 · {{ modifiedCount }} 项未保存</span>
        </div>
        <div v-if="data?.warnings?.length" class="validation">
            <span v-for="warning in data.warnings" :key="warning">{{ warning }}</span>
        </div>
        <p v-if="!data" class="muted">正在读取车型参数…</p>
        <p v-else-if="!groups.length" class="muted">当前分类没有匹配项，可切换“全部参数”或清空搜索。</p>
        <article v-for="group in groups" :key="group.id" class="panel parameter-group" :class="{ motors: group.category === 'motors', compact: group.category === 'motors' || /pid$/.test(group.id) }">
            <div class="parameter-heading">
                <h2>{{ group.label }}</h2><small>{{ group.fields.length }} 项</small>
            </div>
            <p v-if="group.description" class="muted">{{ group.description }}</p>
            <div class="parameter-fields">
                <label v-for="field in group.fields" :key="field.id" class="field parameter-field"
                    :class="{ changed: Object.hasOwn(changes, field.id), readonly: field.editable === false }">
                    <span>{{ field.label }} <em v-if="field.unit">{{ field.unit }}</em></span>
                    <select v-if="field.options?.length" :value="valueOf(field)"
                        :disabled="disabled || field.editable === false" @change="selectedValue(field, $event)">
                        <option v-for="option in field.options" :key="String(option.value)" :value="option.value">{{ option.label }}</option>
                    </select>
                    <select v-else-if="field.type === 'boolean'" :value="String(valueOf(field))"
                        :disabled="disabled || field.editable === false" @change="emit('change', field.id, ($event.target as HTMLSelectElement).value === 'true')">
                        <option value="false">关闭</option><option value="true">开启</option>
                    </select>
                    <input v-else-if="field.type === 'number' || field.type === 'integer' || field.type === 'float'"
                        type="number" :value="valueOf(field) as number | string" :min="field.min" :max="field.max" :step="field.step ?? 'any'"
                        :disabled="disabled || field.editable === false" @input="inputValue(field, $event)" />
                    <input v-else :value="String(valueOf(field))" :disabled="disabled || field.editable === false"
                        :readonly="field.type === 'text'" @input="emit('change', field.id, ($event.target as HTMLInputElement).value)" />
                    <small v-if="field.description">{{ field.description }}</small>
                    <code>{{ field.id }}</code>
                    <small v-if="field.editable === false && field.type === 'text'">保留原表达式；请在文件编辑页修改。</small>
                </label>
            </div>
        </article>
    </div>
</template>

<style scoped>
.config-values { display: grid; gap: 16px; }
.parameter-tabs { display: flex; flex-wrap: wrap; gap: 8px; }
.parameter-filter { display: flex; align-items: center; justify-content: space-between; gap: 16px; font-size: 12px; color: #647079; }
.parameter-filter input { flex: 1; max-width: 570px; border: 1px solid #cdd5d7; padding: 10px 12px; font: inherit; }
.parameter-heading { display: flex; gap: 12px; align-items: baseline; }
.parameter-heading h2 { margin: 0; }
.parameter-heading small { color: #78858b; }
.parameter-fields { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 16px; margin-top: 16px; }
.parameter-field { border-top: 2px solid #e7ebec; padding: 10px 0; min-width: 0; }
.parameter-field.changed { border-color: #edb921; }
.parameter-field em { font-style: normal; font-weight: normal; color: #7e8990; }
.parameter-field code { font-size: 10px; color: #839096; overflow-wrap: anywhere; }
.parameter-field input, .parameter-field select { width: 100%; height: 35px; }
.parameter-field.readonly { opacity: .75; }
.parameter-group.compact .parameter-fields { grid-template-columns: repeat(5, minmax(0, 1fr)); gap: 12px; }
.parameter-group.compact.motors .parameter-fields { grid-template-columns: repeat(6, minmax(0, 1fr)); }
.parameter-group.compact .parameter-field code { font-size: 9px; }
.parameter-group.compact .parameter-field:first-child { min-width: 120px; }
@media (max-width: 1200px) {
    .parameter-fields { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .parameter-group.compact.motors .parameter-fields { grid-template-columns: repeat(3, minmax(0, 1fr)); }
}
</style>
