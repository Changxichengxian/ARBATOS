<script setup lang="ts">
import { computed } from "vue";
const props = defineProps<{ resolved: Record<string, unknown> | null; draft: boolean; pending: boolean }>();
type Task = { symbol: string; task: string; stack_words: number; priority: string; dependencies: string[] };
const tasks = computed(() => (props.resolved?.tasks ?? []) as Task[]);
const reasons = computed(() => (props.resolved?.reasons ?? {}) as Record<string, string[]>);
const stackBytes = computed(() => tasks.value.reduce((sum, task) => sum + task.stack_words * 4, 0));
</script>

<template>
    <section class="panel" v-if="resolved">
        <h2>生成的任务清单</h2>
        <p class="muted">{{ draft ? '当前草稿检查结果' : '已保存配置' }} · {{ tasks.length }} 个任务 · 任务栈声明合计 {{ (stackBytes / 1024).toFixed(1) }} KiB。系统线程、保护区和缓冲区另计，实际占用见“内存与 Flash”。</p>
        <p v-if="pending" class="plan-pending">草稿已修改；点击“检查修改”更新这份清单。</p>
        <div class="plan-list"><div v-for="task in tasks" :key="task.symbol" class="plan-task">
            <b>{{ task.symbol }}</b><span>{{ task.stack_words * 4 }} B 栈</span>
            <small>{{ (reasons[task.symbol] ?? []).join('；') || '配置依赖' }}</small>
        </div></div>
        <p class="muted">取消控制器会重新计算依赖；其他机构仍需要的通信、接收和姿态任务会保留。关闭任务后，共用的设备表等基础数据仍可能占空间。</p>
    </section>
</template>

<style scoped>
.plan-list { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
.plan-task { display: grid; grid-template-columns: 1fr auto; gap: 6px; padding: 12px; background: #f3f6f8; border-radius: 5px; font-size: 13px; }
.plan-task small { grid-column: 1 / -1; color: #63727c; overflow-wrap: anywhere; }
.plan-pending { color: #966719; }
</style>
