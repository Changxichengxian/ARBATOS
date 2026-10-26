import { onBeforeUnmount, ref, type Ref } from "vue";
import { rpc } from "./api";
import type { FlashConfirmation, Job } from "./types";

export function useBuild(options: {
    target: Ref<string>;
    busy: Ref<boolean>;
    anyDirty: Ref<boolean>;
    disposed: Ref<boolean>;
    fail: (error: unknown) => void;
    showToast: (message: string) => void;
}) {
    const job = ref<Job | null>(null);
    const confirmFlash = ref<FlashConfirmation | null>(null);
    let jobTimer: number | undefined;

    async function startJob(action: Job["action"]) {
        if (!options.target.value) return;
        if (options.anyDirty.value) {
            options.showToast("请先保存或放弃当前修改，再编译，避免构建旧文件内容。");
            return;
        }
        options.busy.value = true;
        try {
            job.value = await rpc<Job>("job.start", { target: options.target.value, action });
            pollJob();
        } catch (e) {
            options.fail(e);
        } finally {
            options.busy.value = false;
        }
    }

    async function planFlash() {
        if (options.busy.value) return;
        if (options.anyDirty.value) {
            options.showToast("请先保存或放弃当前修改，再检查烧录产物。");
            return;
        }
        options.busy.value = true;
        try {
            confirmFlash.value = await rpc<FlashConfirmation>("job.plan_flash", { target: options.target.value });
        } catch (e) {
            options.fail(e);
        } finally {
            options.busy.value = false;
        }
    }

    async function doFlash() {
        if (!confirmFlash.value || options.busy.value) return;
        options.busy.value = true;
        try {
            job.value = await rpc<Job>("job.start", {
                target: options.target.value,
                action: "flash",
                confirmation: {
                    target: confirmFlash.value.target,
                    artifactRevision: confirmFlash.value.artifactRevision,
                },
            });
            confirmFlash.value = null;
            pollJob();
        } catch (e) {
            options.fail(e);
        } finally {
            options.busy.value = false;
        }
    }

    async function pollJob() {
        if (options.disposed.value || !job.value || job.value.status !== "running") return;
        try {
            const newer = await rpc<Job>("job.get", { id: job.value.id, after: job.value.lastSeq });
            if (options.disposed.value) return;
            job.value = {
                ...newer,
                lines: newer.lines?.length
                    ? [...(job.value?.lines ?? []), ...newer.lines].slice(-1500)
                    : (job.value?.lines ?? []),
            };
            if (job.value.status === "running") jobTimer = window.setTimeout(pollJob, 700);
        } catch (e) {
            options.fail(e);
        }
    }

    async function cancelJob() {
        if (!job.value) return;
        try {
            job.value = await rpc<Job>("job.cancel", { id: job.value.id });
        } catch (e) {
            options.fail(e);
        }
    }

    onBeforeUnmount(() => {
        if (jobTimer) window.clearTimeout(jobTimer);
    });

    return { job, confirmFlash, startJob, planFlash, doFlash, pollJob, cancelJob };
}
