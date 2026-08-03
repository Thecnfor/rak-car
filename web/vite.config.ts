import { defineConfig } from "vite";
import { resolve } from "path";

// 多页构建：根跳转页 + monitor（监控台）+ teach（示教器）。
// 产物直接落到 ../runtime/static_web/（提交进仓库），Jetson 侧由 FastAPI
// StaticFiles 挂到 /console/ 提供 —— 车端不需要 node。
export default defineConfig({
  base: "./",
  build: {
    outDir: resolve(__dirname, "../runtime/static_web"),
    emptyOutDir: true,
    rollupOptions: {
      input: {
        main: resolve(__dirname, "index.html"),
        monitor: resolve(__dirname, "monitor/index.html"),
        teach: resolve(__dirname, "teach/index.html"),
        chassis: resolve(__dirname, "chassis/index.html"),
      },
    },
  },
  server: {
    // 开发机 vite dev 时把 runtime API 代理到车端/本机 runtime
    // (默认 http://127.0.0.1:5050；连真车改成 http://192.168.6.231:5050)
    proxy: {
      "/v1": "http://127.0.0.1:5050",
      "/api": "http://127.0.0.1:5050",
      "/video_feed": "http://127.0.0.1:5050",
      "/stream": "http://127.0.0.1:5050",
      "/keypress": "http://127.0.0.1:5050",
    },
  },
});
