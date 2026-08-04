// 底盘调试页 —— 里程计轨迹 + IR + 轮编码器 + lane 状态 + 底盘点动。
// 点动走 /v1/realtime/chassis-velocity（直发 vx/vy/wz，绕开里程计耦合与 job_queue）；
// 松手/失焦自动发 0。轨迹画 odom x/y（车坐标系，x 前 y 左），θ 用箭头朝向。
import { api, fmt } from "../lib/api";
import "../lib/base.css";
import "./chassis.css";

const $ = (id: string) => document.getElementById(id)!;
const connDot = $("connDot"), connText = $("connText");
const odomX = $("odomX"), odomY = $("odomY"), odomT = $("odomT"), odomD = $("odomD");
const irL = $("irL"), irR = $("irR");
const btnEstop = $("btnEstop") as HTMLButtonElement;
const btnEstopClear = $("btnEstopClear") as HTMLButtonElement;
const cam1Img = $("cam1Img") as HTMLImageElement;
const laneOverlay = $("laneOverlay") as HTMLInputElement;
const jogLin = $("jogLin") as HTMLInputElement, jogLinLabel = $("jogLinLabel")!;
const jogAng = $("jogAng") as HTMLInputElement, jogAngLabel = $("jogAngLabel")!;
const btnStop = $("btnStop") as HTMLButtonElement;
const btnClearTrace = $("btnClearTrace") as HTMLButtonElement;
const btnZeroOdom = $("btnZeroOdom") as HTMLButtonElement;
const trajInfo = $("trajInfo")!;
const laneMode = $("laneMode"), laneActive = $("laneActive");
const laneEy = $("laneEy"), laneEa = $("laneEa");
const laneDist = $("laneDist");
const cmdVx = $("cmdVx"), cmdVy = $("cmdVy"), cmdWz = $("cmdWz");
const encEls = [$("enc0"), $("enc1"), $("enc2"), $("enc3")];

let estopped = false;

const canvas = $("trajCanvas") as HTMLCanvasElement;
const ctx = canvas.getContext("2d")!;
interface Pt { x: number; y: number; t: number }
let trace: Pt[] = [];
const MAX_TRACE = 4000;

function pushTrace(x: number, y: number, theta: number) {
  const last = trace[trace.length - 1];
  if (last && Math.hypot(x - last.x, y - last.y) < 0.004) return;  // 静止不堆点
  trace.push({ x, y, t: theta });
  if (trace.length > MAX_TRACE) trace.splice(0, trace.length - MAX_TRACE);
}

function drawTrace() {
  const W = canvas.width, H = canvas.height;
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, W, H);
  // 网格 0.5m
  const scale = 140;  // px/m
  const cx = W / 2, cy = H / 2;
  ctx.strokeStyle = "rgba(255,255,255,0.07)";
  ctx.lineWidth = 1;
  for (let gx = cx % (scale / 2); gx < W; gx += scale / 2) {
    ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, H); ctx.stroke();
  }
  for (let gy = cy % (scale / 2); gy < H; gy += scale / 2) {
    ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(W, gy); ctx.stroke();
  }
  // 原点十字
  ctx.strokeStyle = "rgba(255,255,255,0.25)";
  ctx.beginPath(); ctx.moveTo(cx - 8, cy); ctx.lineTo(cx + 8, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, cy - 8); ctx.lineTo(cx, cy + 8); ctx.stroke();

  if (!trace.length) {
    trajInfo.textContent = "无轨迹（车未动 / 未清零后移动）";
    return;
  }
  // 车坐标 → 画布：x 前=上，y 左=左（俯视图，车头朝上）
  const px = (p: Pt) => cx - p.y * scale;
  const py = (p: Pt) => cy - p.x * scale;
  ctx.strokeStyle = "rgba(0,212,255,0.85)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(px(trace[0]), py(trace[0]));
  for (let i = 1; i < trace.length; i++) ctx.lineTo(px(trace[i]), py(trace[i]));
  ctx.stroke();
  // 当前位姿箭头（θ 逆时针为正；车坐标系 x 前 y 左）
  const cur = trace[trace.length - 1];
  const hx = px(cur), hy = py(cur);
  const dx = Math.cos(cur.t), dy = Math.sin(cur.t);  // 车头方向（车坐标）
  const ax = hx - dy * 16, ay = hy - dx * 16;        // 映射到画布
  ctx.strokeStyle = "#00e676";
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(hx, hy); ctx.lineTo(ax, ay); ctx.stroke();
  ctx.fillStyle = "#00e676";
  ctx.beginPath(); ctx.arc(hx, hy, 4, 0, Math.PI * 2); ctx.fill();
  trajInfo.textContent = trace.length + " pts";
}

btnClearTrace.addEventListener("click", () => { trace = []; drawTrace(); });
btnZeroOdom.addEventListener("click", async () => {
  if (!confirm("里程计清零 (car reset_position)？车会认为自己在原点。")) return;
  try {
    await api.executeCar("reset_position", {});
    trace = [];
    drawTrace();
  } catch (e) {
    alert("清零失败: " + (e as Error).message);
  }
});

// ---------- 轮询：odom / ir / encoders / lane（10Hz 状态，2Hz 编码器） ----------
async function pollOdom() {
  try {
    const d = (await api.odomState()) as Record<string, any>;
    const st = d.odom_state || {};
    const fresh = st.updated_at && Date.now() / 1000 - st.updated_at < 2;
    connDot.className = "dot " + (fresh ? "ok" : "warn");
    connText.textContent = fresh ? "连接: ok" : "odom_feed 不新鲜";
    odomX.textContent = fmt(st.x, 3);
    odomY.textContent = fmt(st.y, 3);
    odomT.textContent = fmt(st.theta, 3);
    odomD.textContent = fmt(st.distance, 2);
    if (typeof st.x === "number" && typeof st.y === "number") {
      pushTrace(st.x, st.y, typeof st.theta === "number" ? st.theta : 0);
      drawTrace();
    }
  } catch {
    connDot.className = "dot err";
    connText.textContent = "连接: err";
  }
}
window.setInterval(pollOdom, 100);
pollOdom();

async function pollIr() {
  try {
    const d = (await api.irState()) as Record<string, any>;
    const st = d.ir_state || {};
    irL.textContent = st.left != null ? fmt(st.left, 3) + "m" : "--";
    irR.textContent = st.right != null ? fmt(st.right, 3) + "m" : "--";
  } catch { /* 静默 */ }
}
window.setInterval(pollIr, 200);
pollIr();

async function pollEnc() {
  try {
    const d = (await api.encoders()) as Record<string, any>;
    const enc = d.encoders || [];
    for (let i = 0; i < 4; i++) encEls[i].textContent = typeof enc[i] === "number" ? enc[i].toFixed(2) : "--";
  } catch { /* 静默 */ }
}
window.setInterval(pollEnc, 500);
pollEnc();

async function pollLane() {
  try {
    const d = (await api.laneState()) as Record<string, any>;
    laneMode.textContent = String(d.mode ?? "--");
    laneActive.textContent = String(d.active ?? "--");
    laneEy.textContent = fmt(d.error_y, 4);
    laneEa.textContent = fmt(d.error_angle, 4);
    laneDist.textContent = fmt(d.distance, 2);
  } catch { /* 静默 */ }
  // 三速显示走"最后底盘指令"缓存（lane_feed 的 forward/lateral/angular
  // 字段从未被填 —— 外环在客户端跑，车端只收轮速）
  try {
    const c = (await api.chassisCommand()) as Record<string, any>;
    const cmd = c.chassis_command || {};
    const fresh = cmd.updated_at && Date.now() / 1000 - cmd.updated_at < 2;
    cmdVx.textContent = fresh ? fmt(cmd.vx, 3) : "--";
    cmdVy.textContent = fresh ? fmt(cmd.vy, 3) : "--";
    cmdWz.textContent = fresh ? fmt(cmd.wz, 3) : "--";
  } catch { /* 静默 */ }
}
window.setInterval(pollLane, 200);
pollLane();

// ---------- cam1 lane overlay ----------
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

jogLinLabel.textContent = linV().toFixed(2) + " m/s";
jogAngLabel.textContent = angV().toFixed(1) + " rad/s";
jogLin.addEventListener("input", () => { jogLinLabel.textContent = linV().toFixed(2) + " m/s"; });
jogAng.addEventListener("input", () => { jogAngLabel.textContent = angV().toFixed(1) + " rad/s"; });

// ---------- 底盘点动 ----------
const axes = new Set<string>();
let jogTimer: number | null = null;

function linV(): number { return Number(jogLin.value) / 100; }   // 0.05..0.50
function angV(): number { return Number(jogAng.value) / 10; }     // 0.2..1.5
// 轴向符号标定（2026-08-04 里程计实测）：本车 +vx 物理=后退、+vy 物理=右移、
// +wz=逆时针。前端按"W=前进/A=左移/Q=逆时针"的直觉映射，用符号常量翻转。
// 现场若换车/换接线方向反了，用面板上的翻转勾选修正（localStorage 持久化）。
const SIGNS_KEY = "rakcar.chassis.signs";
const DEFAULT_SIGNS = { vx: -1, vy: -1, wz: +1 };
let signs: { vx: number; vy: number; wz: number } = loadSigns();
function loadSigns() {
  try {
    const s = JSON.parse(localStorage.getItem(SIGNS_KEY) || "null");
    if (s && typeof s.vx === "number" && typeof s.vy === "number" && typeof s.wz === "number") return s;
  } catch { /* ignore */ }
  return { ...DEFAULT_SIGNS };
}
function saveSigns() {
  localStorage.setItem(SIGNS_KEY, JSON.stringify(signs));
}

// 翻转勾选 ↔ signs（默认值已含实测标定；勾选 = 在默认基础上再翻一次）
const flipVx = $("flipVx") as HTMLInputElement;
const flipVy = $("flipVy") as HTMLInputElement;
const flipWz = $("flipWz") as HTMLInputElement;
function applySigns() {
  signs = {
    vx: DEFAULT_SIGNS.vx * (flipVx.checked ? -1 : 1),
    vy: DEFAULT_SIGNS.vy * (flipVy.checked ? -1 : 1),
    wz: DEFAULT_SIGNS.wz * (flipWz.checked ? -1 : 1),
  };
  saveSigns();
}
try {
  const saved = JSON.parse(localStorage.getItem(SIGNS_KEY) || "null");
  if (saved) {
    flipVx.checked = saved.vx !== DEFAULT_SIGNS.vx;
    flipVy.checked = saved.vy !== DEFAULT_SIGNS.vy;
    flipWz.checked = saved.wz !== DEFAULT_SIGNS.wz;
  }
} catch { /* ignore */ }
applySigns();
flipVx.addEventListener("change", applySigns);
flipVy.addEventListener("change", applySigns);
flipWz.addEventListener("change", applySigns);

async function sendJog() {
  if (estopped) return;
  let vx = 0, vy = 0, wz = 0;
  if (axes.has("w")) vx += signs.vx * linV();   // W = 前进
  if (axes.has("s")) vx -= signs.vx * linV();   // S = 后退
  if (axes.has("a")) vy += signs.vy * linV();   // A = 左移
  if (axes.has("d")) vy -= signs.vy * linV();   // D = 右移
  if (axes.has("q")) wz += signs.wz * angV();   // Q = 逆时针
  if (axes.has("e")) wz -= signs.wz * angV();   // E = 顺时针
  try {
    await api.chassisVelocity({ vx, vy, wz });
  } catch { /* 抖动重试；急停 409 预期 */ }
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
  api.chassisVelocity({ vx: 0, vy: 0, wz: 0 }).catch(() => undefined);
}

const KEY_AXIS: Record<string, true> = { w: true, s: true, a: true, d: true, q: true, e: true };
const keycaps = new Map<string, HTMLElement>();
document.querySelectorAll<HTMLElement>(".keycap").forEach((el) => {
  keycaps.set(el.textContent!.trim().toLowerCase(), el);
});
document.addEventListener("keydown", (ev) => {
  if (ev.target instanceof HTMLInputElement) return;
  const k = ev.key.toLowerCase();
  if (ev.repeat) return;
  if (KEY_AXIS[k]) {
    axes.add(k);
    keycaps.get(k)?.classList.add("pressed");
    startJog();
    ev.preventDefault();
  }
});
document.addEventListener("keyup", (ev) => {
  const k = ev.key.toLowerCase();
  if (KEY_AXIS[k]) {
    axes.delete(k);
    keycaps.get(k)?.classList.remove("pressed");
    if (axes.size === 0) stopJog();
  }
});
window.addEventListener("blur", () => stopJog());
document.addEventListener("visibilitychange", () => { if (document.hidden) stopJog(); });

btnStop.addEventListener("click", () => stopJog());

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

drawTrace();
