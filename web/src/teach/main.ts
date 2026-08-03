// 示教器 —— 工业 teach-pendant 工作流的 web 版：
//   点动(jog)到位 → 示教记录 → 存位姿库 → 一键前往回放。
// 点动走 /v1/realtime/arm-velocity（速度模式、绕开 arm_queue、µs gate）；
// 位姿库存浏览器 localStorage（导出/导入 JSON 备份）。
//
// 注意：
//  - x 轴无软限位（arm_base 只有 y 磁感门），点动时人必须盯画面，松手/失焦自动发 0；
//  - arm_feed 只回 x/y 实测位置，大臂/手爪角度前端维护"指令值"（舵机是位置控制，
//    指令即位姿），示教/前往以指令值为准。
import { api, fmt } from "../lib/api";
import "../lib/base.css";
import "./teach.css";

// ---------- 元素 ----------
const $ = (id: string) => document.getElementById(id)!;
const connDot = $("connDot"), connText = $("connText");
const posX = $("posX"), posY = $("posY"), armAng = $("armAng"), handAng = $("handAng");
const btnEstop = $("btnEstop") as HTMLButtonElement;
const btnEstopClear = $("btnEstopClear") as HTMLButtonElement;
const cam2Img = $("cam2Img") as HTMLImageElement;
const taskOverlay = $("taskOverlay") as HTMLInputElement;
const jogSpeed = $("jogSpeed") as HTMLInputElement;
const jogSpeedLabel = $("jogSpeedLabel")!;
const btnGoHome = $("btnGoHome") as HTMLButtonElement;
const btnZeroAll = $("btnZeroAll") as HTMLButtonElement;
const poseName = $("poseName") as HTMLInputElement;
const btnTeach = $("btnTeach") as HTMLButtonElement;
const poseTableBody = document.querySelector("#poseTable tbody")!;
const btnExport = $("btnExport") as HTMLButtonElement;
const btnImport = $("btnImport") as HTMLButtonElement;
const importFile = $("importFile") as HTMLInputElement;
const poseMsg = $("poseMsg")!;

// ---------- 状态 ----------
interface Pose { name: string; x_mm: number; y_mm: number; arm: number; hand: number; ts: number }
const POSES_KEY = "rakcar.poses";
let poses: Pose[] = loadPoses();

// 舵机指令角（前端维护；init/reset 后默认复位位）
let armCmd = 90;    // 大臂 +90 = 复位位
let handCmd = -90;  // 手爪 -90 = UP
let estopped = false;

function loadPoses(): Pose[] {
  try { return JSON.parse(localStorage.getItem(POSES_KEY) || "[]") as Pose[]; }
  catch { return []; }
}
function savePoses() {
  localStorage.setItem(POSES_KEY, JSON.stringify(poses));
}

// ---------- 连接状态 + 实测位置（arm_feed 缓存，10Hz） ----------
async function pollState() {
  try {
    const d = (await api.armState()) as Record<string, any>;
    const st = d.arm_state || {};
    const fresh = st.updated_at && Date.now() / 1000 - st.updated_at < 2;
    connDot.className = "dot " + (fresh ? "ok" : "warn");
    connText.textContent = fresh ? "连接: ok" : "arm_feed 不新鲜";
    posX.textContent = fmt(st.x_mm, 1);
    posY.textContent = fmt(st.y_mm, 1);
    armAng.textContent = armCmd.toFixed(0);
    handAng.textContent = handCmd.toFixed(0);
  } catch {
    connDot.className = "dot err";
    connText.textContent = "连接: err";
  }
}
pollState();
window.setInterval(pollState, 100);

// ---------- cam2 overlay ----------
let overlayTimer: number | null = null;
taskOverlay.addEventListener("change", () => {
  if (taskOverlay.checked) {
    const refresh = () => { cam2Img.src = api.taskPreviewUrl("cam2"); };
    refresh();
    overlayTimer = window.setInterval(refresh, 100);
  } else {
    if (overlayTimer) { window.clearInterval(overlayTimer); overlayTimer = null; }
    cam2Img.src = api.videoFeedUrl("cam2");
  }
});

// ---------- 点动：WASD 速度模式 + QE/RF 舵机步进 ----------
// axes: 当前按下的方向集合。每 100ms 把合成速度发给 arm-velocity（心跳保活 + 抗丢包）；
// 松开全部时立刻发一次 0。
const axes = new Set<string>();
let jogTimer: number | null = null;

function jogVelocity(): number {
  // slider 1..8 → 0.01..0.08 m/s
  return Number(jogSpeed.value) / 100;
}
jogSpeed.addEventListener("input", () => {
  jogSpeedLabel.textContent = jogVelocity().toFixed(2) + " m/s";
});
jogSpeedLabel.textContent = jogVelocity().toFixed(2) + " m/s";

async function sendJog() {
  if (estopped) return;  // 急停后不发速度（runtime 也会 409 拒）
  const v = jogVelocity();
  let xVel = 0, yVel = 0;
  if (axes.has("a")) xVel -= v;
  if (axes.has("d")) xVel += v;
  if (axes.has("w")) yVel -= v;  // y 负 = 向上
  if (axes.has("s")) yVel += v;
  try {
    await api.armVelocity({ x_vel: xVel, y_vel: yVel });
  } catch { /* 连接抖动：下个 tick 重试；急停后 409 属预期 */ }
}

function startJog() {
  if (jogTimer) return;
  jogTimer = window.setInterval(sendJog, 100);
  sendJog();
}
function stopJog(hard = false) {
  axes.clear();
  if (jogTimer) { window.clearInterval(jogTimer); jogTimer = null; }
  if (hard) return;
  api.armVelocity({ x_vel: 0, y_vel: 0 }).catch(() => undefined);
}

function servoStep(axis: "arm" | "hand", delta: number) {
  if (estopped) return;
  if (axis === "arm") {
    armCmd = Math.max(-150, Math.min(90, armCmd + delta));
    api.armVelocity({ arm_angle: armCmd }).catch(() => undefined);
  } else {
    handCmd = Math.max(-90, Math.min(0, handCmd + delta));
    api.armVelocity({ hand_angle: handCmd }).catch(() => undefined);
  }
}
const KEY_AXIS: Record<string, string> = { w: "w", s: "s", a: "a", d: "d" };
const keycaps = new Map<string, HTMLElement>();
document.querySelectorAll<HTMLElement>(".keycap").forEach((el) => {
  keycaps.set(el.textContent!.trim().toLowerCase(), el);
});

document.addEventListener("keydown", (ev) => {
  if (ev.target instanceof HTMLInputElement) return;  // 输入位姿名时不劫持
  const k = ev.key.toLowerCase();
  if (ev.repeat) return;
  if (KEY_AXIS[k]) {
    axes.add(k);
    keycaps.get(k)?.classList.add("pressed");
    startJog();
    ev.preventDefault();
  } else if (k === "q") { servoStep("arm", -5); keyPress(k); ev.preventDefault(); }
  else if (k === "e") { servoStep("arm", +5); keyPress(k); ev.preventDefault(); }
  else if (k === "r") { servoStep("hand", -5); keyPress(k); ev.preventDefault(); }
  else if (k === "f") { servoStep("hand", +5); keyPress(k); ev.preventDefault(); }
});
document.addEventListener("keyup", (ev) => {
  const k = ev.key.toLowerCase();
  if (KEY_AXIS[k]) {
    axes.delete(k);
    keycaps.get(k)?.classList.remove("pressed");
    if (axes.size === 0) stopJog();
  }
});
function keyPress(k: string) {
  const el = keycaps.get(k);
  if (!el) return;
  el.classList.add("pressed");
  window.setTimeout(() => el.classList.remove("pressed"), 180);
}
// 失焦/切页 = 安全第一，立即全停
window.addEventListener("blur", () => stopJog());
document.addEventListener("visibilitychange", () => { if (document.hidden) stopJog(); });

btnZeroAll.addEventListener("click", () => {
  stopJog();
  api.armVelocity({ x_vel: 0, y_vel: 0 }).catch(() => undefined);
});

btnGoHome.addEventListener("click", async () => {
  btnGoHome.disabled = true;
  try {
    await api.executeArm("composite_go_home", {});
    armCmd = 90; handCmd = -90;  // go_home 回复位位
  } catch (e) {
    alert("go_home 失败: " + (e as Error).message);
  } finally {
    btnGoHome.disabled = false;
  }
});

// ---------- 急停 ----------
btnEstop.addEventListener("click", async () => {
  stopJog(true);
  try {
    await api.estop();
    estopped = true;
    btnEstopClear.disabled = false;
    btnEstop.textContent = "已急停";
    btnEstop.disabled = true;
  } catch (e) {
    alert("急停失败: " + (e as Error).message);
  }
});
btnEstopClear.addEventListener("click", async () => {
  try {
    await api.estopClear();
    estopped = false;
    btnEstopClear.disabled = true;
    btnEstop.textContent = "软急停";
    btnEstop.disabled = false;
  } catch (e) {
    alert("解除急停失败: " + (e as Error).message);
  }
});

// ---------- 位姿库 ----------
function renderPoses() {
  poseTableBody.innerHTML = "";
  for (const p of poses) {
    const tr = document.createElement("tr");
    const cells = [
      [p.name, ""],
      [fmt(p.x_mm, 1), "num"],
      [fmt(p.y_mm, 1), "num"],
      [p.arm.toFixed(0), "num"],
      [p.hand.toFixed(0), "num"],
    ];
    for (const [text, cls] of cells) {
      const td = document.createElement("td");
      td.textContent = text;
      if (cls) td.className = cls;
      tr.appendChild(td);
    }
    const tdAct = document.createElement("td");
    const go = document.createElement("button");
    go.textContent = "前往";
    go.className = "row-btn";
    go.addEventListener("click", () => gotoPose(p, go));
    const del = document.createElement("button");
    del.textContent = "删除";
    del.className = "row-btn danger";
    del.addEventListener("click", () => {
      poses = poses.filter((q) => q.name !== p.name);
      savePoses();
      renderPoses();
    });
    tdAct.appendChild(go);
    tdAct.appendChild(del);
    tr.appendChild(tdAct);
    poseTableBody.appendChild(tr);
  }
}

async function gotoPose(p: Pose, btn: HTMLButtonElement) {
  if (!confirm(`前往位姿「${p.name}」?\ncomposite_run: x=${fmt(p.x_mm, 1)}mm y=${fmt(p.y_mm, 1)}mm 臂=${p.arm}° 爪=${p.hand}°`)) return;
  btn.disabled = true;
  const old = btn.textContent;
  btn.textContent = "运动中…";
  try {
    // composite_run: x/y 单位米（SDK 口径），arm/hand 单位度
    await api.executeArm("composite_run", {
      x: p.x_mm / 1000,
      y: p.y_mm / 1000,
      arm: p.arm,
      hand: p.hand,
    });
    armCmd = p.arm;
    handCmd = p.hand;
  } catch (e) {
    alert("前往位姿失败: " + (e as Error).message);
  } finally {
    btn.disabled = false;
    btn.textContent = old;
  }
}

btnTeach.addEventListener("click", async () => {
  // x/y 取 arm_feed 实测值，角度取前端指令值
  let xMm: number | null = null, yMm: number | null = null;
  try {
    const d = (await api.armState()) as Record<string, any>;
    const st = d.arm_state || {};
    xMm = typeof st.x_mm === "number" ? st.x_mm : null;
    yMm = typeof st.y_mm === "number" ? st.y_mm : null;
  } catch { /* ignore */ }
  if (xMm === null || yMm === null) {
    poseMsg.textContent = "arm_feed 无实测位置，示教失败";
    return;
  }
  const name = (poseName.value.trim() || `pose_${poses.length + 1}`);
  poses = poses.filter((p) => p.name !== name);  // 同名覆盖
  poses.push({ name, x_mm: xMm, y_mm: yMm, arm: armCmd, hand: handCmd, ts: Date.now() });
  savePoses();
  renderPoses();
  poseName.value = "";
  poseMsg.textContent = `已记录「${name}」`;
  window.setTimeout(() => { poseMsg.textContent = ""; }, 2000);
});

btnExport.addEventListener("click", () => {
  const blob = new Blob([JSON.stringify(poses, null, 2)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "rak-car-poses.json";
  a.click();
  URL.revokeObjectURL(a.href);
});
btnImport.addEventListener("click", () => importFile.click());
importFile.addEventListener("change", async () => {
  const f = importFile.files?.[0];
  importFile.value = "";
  if (!f) return;
  try {
    const arr = JSON.parse(await f.text()) as Pose[];
    if (!Array.isArray(arr)) throw new Error("不是数组");
    for (const p of arr) {
      if (typeof p.name !== "string" || typeof p.x_mm !== "number") throw new Error("字段缺失");
    }
    poses = arr;
    savePoses();
    renderPoses();
    poseMsg.textContent = `导入 ${arr.length} 个位姿`;
  } catch (e) {
    poseMsg.textContent = "导入失败: " + (e as Error).message;
  }
});

renderPoses();
void estopped;  // 预留：后续可把 health 里的 stop_flag 同步到 estopped
