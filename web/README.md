# rak-car web console

工程化前端控制台（取代 `camera_stream_service.render_page` 内嵌页），两个页面：

- `monitor/` — 监控台：cam1/cam2 MJPEG + lane/task overlay + 状态栏（原 /stream/ 页功能迁移）
- `teach/` — **示教器**：键盘点动机械臂（WASD/QE，速度模式，松手即停）、位姿示教记录（localStorage + JSON 导入导出）、一键前往已存位姿、软急停

## 构建（开发机，需要 node）

```bash
cd web
npm install
npm run build        # 产物写入 ../runtime/static_web/（提交进仓库）
```

车端（Jetson）**不需要 node**：FastAPI 启动时把 `runtime/static_web/` 挂到 `/console/`。
访问 `http://<车>:5050/console/`（monitor 监控台，teach 示教器）。

## 本地开发（热更新）

```bash
npm run dev          # http://localhost:5173/console/
```

vite dev server 已把 `/v1`、`/video_feed` 等代理到 `http://127.0.0.1:5050`；
连真车时改 `vite.config.ts` proxy 目标为 `http://192.168.6.231:5050`。

## 约束

- 纯 vanilla TS + Vite 多页，不引框架（控制面板，非内容应用）。
- 示教器点动走 `/v1/realtime/arm-velocity`（速度模式、绕开 arm_queue）：
  **x 轴无软限位**，松手/失焦自动发 0；舵机轴（arm/hand）用内部目标角增量下发。
- 位姿库存浏览器 localStorage（key `rakcar.poses`），用"导出"备份成 JSON。
- 旧内嵌页 `/stream/` 保留不动（生产回退），页内链接指向 `/console/`。
