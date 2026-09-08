// 页面只通过这个接口访问后端；更换桌面容器时不修改业务组件。
let sessionToken =
    new URLSearchParams(location.hash.slice(1)).get("token") ||
    sessionStorage.getItem("arbatos-session") ||
    "";
if (sessionToken) {
    sessionStorage.setItem("arbatos-session", sessionToken);
    history.replaceState(null, "", location.pathname + location.search);
}

export class ApiError extends Error {
    constructor(
        public code: string,
        message: string,
    ) {
        super(message);
        this.name = "ApiError";
    }
}

export async function rpc<T = any>(
    method: string,
    params: Record<string, unknown> = {},
): Promise<T> {
    if (!sessionToken)
        throw new ApiError(
            "SESSION_REQUIRED",
            "请通过根目录“打开客户端”启动，以连接本机服务。",
        );
    const controller = new AbortController();
    const timeout = window.setTimeout(() => controller.abort(), 60000);
    try {
        const response = await fetch("/api/rpc", {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${sessionToken}`,
            },
            body: JSON.stringify({ method, params }),
            signal: controller.signal,
        });
        const data = await response.json();
        if (!response.ok || !data.ok)
            throw new ApiError(
                data.error?.code || "REQUEST_FAILED",
                data.error?.message || `服务返回 ${response.status}`,
            );
        return data.result as T;
    } catch (error) {
        if (error instanceof ApiError) throw error;
        throw new ApiError(
            "CONNECTION_LOST",
            "本机服务连接中断，请检查客户端是否仍在运行。",
        );
    } finally {
        window.clearTimeout(timeout);
    }
}
