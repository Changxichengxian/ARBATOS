import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

export default defineConfig({
    plugins: [vue()],
    server: {
        host: "127.0.0.1",
        port: 8766,
        strictPort: true,
        proxy: {
            "/api": {
                target: process.env.ARBATOS_API_URL || "http://127.0.0.1:8765",
                changeOrigin: true,
            },
        },
    },
    build: { sourcemap: true },
});
