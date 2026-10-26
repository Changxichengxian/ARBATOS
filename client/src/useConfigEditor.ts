import { computed, onBeforeUnmount, ref, type Ref } from "vue";
import { rpc } from "./api";
import type { ConfigValues } from "./components/ConfigValuesPanel.vue";
import type { PortOptions } from "./components/PortConfigPanel.vue";
import type { Robot } from "./types";

export function useConfigEditor(options: {
    target: Ref<string>;
    activeTab: Ref<string>;
    robot: Ref<Robot | null>;
    busy: Ref<boolean>;
    disposed: Ref<boolean>;
    isInterfaceBusy: () => boolean;
    fail: (error: unknown) => void;
    clearError: () => void;
    showToast: (message: string) => void;
    refreshSummary: () => Promise<void>;
}) {
    const configDraft = ref<Record<string, unknown>>({});
    const dirtyConfig = ref(false);
    const portOptions = ref<PortOptions | null>(null);
    const validationPreview = ref<{ ok: boolean; errors: string[]; resolved?: Record<string, unknown> } | null>(null);
    const configValues = ref<ConfigValues | null>(null);
    const valuesLoading = ref(false);
    const valueChanges = ref<Record<string, unknown>>({});
    const dirtyValues = computed(() => Object.keys(valueChanges.value).length > 0);
    let portPreviewTimer: number | undefined;
    let portPreviewSeq = 0;
    let valuesRequestSeq = 0;

    onBeforeUnmount(() => {
        if (portPreviewTimer) window.clearTimeout(portPreviewTimer);
    });

    function loadPortOptions() {
        const seq = ++portPreviewSeq;
        const expected = options.target.value;
        if (!expected || options.activeTab.value !== "robot") return;
        rpc<PortOptions>("ports.get", { target: expected, config: configDraft.value })
            .then((result) => {
                if (seq === portPreviewSeq && expected === options.target.value) portOptions.value = result;
            })
            .catch((e) => {
                if (!portOptions.value && seq === portPreviewSeq) options.fail(e);
            });
    }

    function changeConfig(value: Record<string, unknown>) {
        configDraft.value = value;
        dirtyConfig.value = true;
        validationPreview.value = null;
        options.clearError();
        if (portPreviewTimer) window.clearTimeout(portPreviewTimer);
        portPreviewTimer = window.setTimeout(loadPortOptions, 300);
    }

    async function loadValues() {
        const expectedTarget = options.target.value;
        if (!expectedTarget || dirtyValues.value) return;
        const seq = ++valuesRequestSeq;
        valuesLoading.value = true;
        try {
            const result = await rpc<ConfigValues>("config.get", { target: expectedTarget });
            if (options.target.value !== expectedTarget || seq !== valuesRequestSeq || options.disposed.value || dirtyValues.value) return;
            configValues.value = result;
            valueChanges.value = {};
        } catch (e) {
            if (seq === valuesRequestSeq && !options.disposed.value) options.fail(e);
        } finally {
            if (seq === valuesRequestSeq) valuesLoading.value = false;
        }
    }

    function changeValue(id: string, value: unknown) {
        if (options.isInterfaceBusy()) return;
        const original = configValues.value?.groups.flatMap((group) => group.fields).find((field) => field.id === id);
        if (!original || original.editable === false) return;
        const next = { ...valueChanges.value };
        const unchanged = original.automatic === true && id.endsWith(".feedback_id")
            ? value === -1 : Object.is(value, original.value);
        if (unchanged) delete next[id];
        else next[id] = value;
        valueChanges.value = next;
        options.clearError();
    }

    async function saveValues(): Promise<boolean> {
        if (!configValues.value || !dirtyValues.value) return false;
        options.busy.value = true;
        try {
            configValues.value = await rpc<ConfigValues>("config.update", {
                target: options.target.value,
                revision: configValues.value.revision,
                changes: valueChanges.value,
            });
            valueChanges.value = {};
            options.clearError();
            await options.refreshSummary();
            options.showToast("参数已保存到车型文件，重新编译并烧录后生效。");
            return true;
        } catch (e) {
            options.fail(e);
            return false;
        } finally {
            options.busy.value = false;
        }
    }

    async function saveConfig(): Promise<boolean> {
        if (!options.robot.value) return false;
        options.busy.value = true;
        try {
            const updated = await rpc<Robot>("robot.update", {
                target: options.target.value,
                revision: options.robot.value.revision,
                config: configDraft.value,
            });
            if (updated.name === options.target.value) {
                options.robot.value = updated;
                configDraft.value = JSON.parse(JSON.stringify(updated.config ?? {}));
                dirtyConfig.value = false;
                loadPortOptions();
                validationPreview.value = null;
                options.clearError();
                options.showToast("车型配置已保存。");
                return true;
            }
            return false;
        } catch (e) {
            options.fail(e);
            return false;
        } finally {
            options.busy.value = false;
        }
    }

    async function validateConfig() {
        if (!options.robot.value || options.busy.value) return;
        options.busy.value = true;
        options.clearError();
        try {
            validationPreview.value = await rpc<{ ok: boolean; errors: string[]; resolved?: Record<string, unknown> }>(
                "robot.validate",
                { target: options.target.value, revision: options.robot.value.revision, config: configDraft.value },
            );
            options.showToast(validationPreview.value.ok ? "修改后的配置检查通过。" : "配置检查发现问题。");
        } catch (e) {
            options.fail(e);
        } finally {
            options.busy.value = false;
        }
    }

    function resetDraft() {
        if (options.robot.value) configDraft.value = JSON.parse(JSON.stringify(options.robot.value.config ?? {}));
        dirtyConfig.value = false;
        valueChanges.value = {};
        validationPreview.value = null;
    }

    return {
        configDraft, dirtyConfig, portOptions, validationPreview, configValues, valuesLoading,
        valueChanges, dirtyValues, changeConfig, loadPortOptions, loadValues, changeValue,
        saveValues, saveConfig, validateConfig, resetDraft,
    };
}
