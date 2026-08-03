/* 监控台 —— 逻辑从 camera_stream_service.render_page 内嵌脚本迁移，行为不变 */
import { api, fmt } from "../lib/api";
import "../lib/base.css";
import "./monitor.css";

const laneDot = document.getElementById("laneDot")!;
const laneText = document.getElementById("laneText")!;
const laneEy = document.getElementById("laneEy")!;
const laneEa = document.getElementById("laneEa")!;
const lastKey = document.getElementById("lastKey")!;
let lastKeyText = "";
let keyTimer: number | null = null;

// ---- lane 状态轮询（1Hz；MJPEG 自带 20Hz）----
async function pollLane() {
  try {
    const d = (await api.laneState()) as Record<string, any>;
    const age = d.updated_at ? Date.now() / 1000 - d.updated_at : 999;
    if (!d.updated_at || age > 2) laneDot.className = "dot err";
    else if (d.active) laneDot.className = "dot ok";
    else laneDot.className = "dot warn";
    laneText.textContent = "lane: " + (d.active ? d.mode || "on" : "idle");
    laneEy.textContent = fmt(d.error_y);
    laneEa.textContent = fmt(d.error_angle);
  } catch {
    laneDot.className = "dot err";
    laneText.textContent = "lane: err";
  }
}
pollLane();
window.setInterval(pollLane, 1000);

// ---- cam1 车道 overlay 切换（MJPEG ↔ 10Hz 单帧 preview.jpg）----
const cam1Img = document.getElementById("cam1Img") as HTMLImageElement;
const laneOverlay = document.getElementById("laneOverlay") as HTMLInputElement;
let overlayTimer: number | null = null;
laneOverlay.addEventListener("change", () => {
  if (laneOverlay.checked) {
    const refresh = () => { cam1Img.src = api.lanePreviewUrl("cam1"); };
    refresh();
    overlayTimer = window.setInterval(refresh, 100);
  } else {
    if (overlayTimer) { window.clearInterval(overlayTimer); overlayTimer = null; }
    cam1Img.src = api.videoFeedUrl("cam1");
  }
});

// ---- task 状态轮询 ----
const taskDot = document.getElementById("taskDot")!;
const taskText = document.getElementById("taskText")!;
const taskCount = document.getElementById("taskCount")!;
const taskLabel = document.getElementById("taskLabel")!;

function pickTopLabel(state: Record<string, any>): string {
  const dets = state.detections || [];
  if (!dets.length) return "";
  let top = dets[0];
  for (let i = 1; i < dets.length; i++) {
    if ((dets[i].score || 0) > (top.score || 0)) top = dets[i];
  }
  const name = top.label || "cls_" + (top.cls_id ?? "?");
  return name + " " + (top.score || 0).toFixed(2);
}

async function pollTask() {
  try {
    const d = (await api.taskState()) as Record<string, any>;
    const st = d.task_state || {};
    const age = st.updated_at ? Date.now() / 1000 - st.updated_at : 999;
    if (!st.updated_at || age > 2) taskDot.className = "dot err";
    else if (st.active) taskDot.className = "dot ok";
    else taskDot.className = "dot warn";
    taskText.textContent = "task: " + (st.active ? st.mode || "on" : "idle");
    const n = st.count || 0;
    taskCount.textContent = String(n);
    taskCount.className = "task-count" + (n === 0 ? " zero" : "");
    taskLabel.textContent = pickTopLabel(st);
    taskLabel.title = taskLabel.textContent;
  } catch {
    taskDot.className = "dot err";
    taskText.textContent = "task: err";
  }
}
pollTask();
window.setInterval(pollTask, 1000);

// ---- cam2 task overlay 切换 ----
const cam2Img = document.getElementById("cam2Img") as HTMLImageElement;
const taskOverlay = document.getElementById("taskOverlay") as HTMLInputElement;
let taskOverlayTimer: number | null = null;
taskOverlay.addEventListener("change", () => {
  if (taskOverlay.checked) {
    const refresh = () => { cam2Img.src = api.taskPreviewUrl("cam2"); };
    refresh();
    taskOverlayTimer = window.setInterval(refresh, 100);
  } else {
    if (taskOverlayTimer) { window.clearInterval(taskOverlayTimer); taskOverlayTimer = null; }
    cam2Img.src = api.videoFeedUrl("cam2");
  }
});

// ---- 键盘按键转发（保留原逻辑：点页面后生效，转发到 /keypress）----
let pageActive = false;
document.body.addEventListener("click", () => { pageActive = true; });
document.addEventListener("keydown", (ev) => {
  if (!pageActive) return;
  api.keypress(ev.key)
    .then((d: any) => {
      if (lastKeyText) lastKey.textContent = "";
      lastKeyText = d.received || "";
      lastKey.textContent = lastKeyText;
      lastKey.classList.remove("active");
      void lastKey.offsetWidth;
      lastKey.classList.add("active");
      if (keyTimer) window.clearTimeout(keyTimer);
      keyTimer = window.setTimeout(() => { lastKey.textContent = ""; lastKeyText = ""; }, 1500);
    })
    .catch((err) => console.error("keypress err:", err));
  if (["F5", "F12"].includes(ev.key)) ev.preventDefault();
});
