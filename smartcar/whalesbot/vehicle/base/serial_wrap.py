#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import heapq as _heapq
import itertools as _itertools
import os
import select as _select
from serial.tools import list_ports
import threading
from threading import Lock, Thread
from typing import List
import serial
import time
import sys
# print(time.time())
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# 导入自定义log模块
from ...tools import logger
# logger.info("start time:{}".format(time.time()))


class ControllerTransportError(RuntimeError):
    pass


class ControllerNotReadyError(ControllerTransportError):
    pass


class ControllerNoResponseError(ControllerTransportError):
    pass

# ============================================================
# SerialEngine（2026-08-03 串口 I/O 引擎）
# ------------------------------------------------------------
# 旧模型：调用方持 SerialWrap.lock 阻塞 write+read（一问一答），
#   50Hz 轮速下发 / odom 读 / arm PID 互相在同一把锁上排队，
#   每次 round-trip 5-20ms，50Hz 预算被吃光。
# 新模型：串口 fd 由唯一 io 线程（SerialEngine）持有；
#   所有设备调用 = submit(帧) → 等自己的 Event。引擎提供：
#     1) 写合并：同一 coalesce_key 在队列里只保留最新一帧
#        （轮速 50Hz 突发下发不再堆积物理帧）；
#     2) 读共享：同一 share_key 的并发请求只打一个物理读帧，
#        结果广播（encoder4 / 模拟量不再各自打一遍）；
#     3) 优先级：URGENT（急停/零速）插队，NORMAL 次之，READ 最低；
#     4) 帧超时：引擎内部按 time_out 控制，调用方只等 event。
# 兼容：SerialWrap.get_anwser 默认走引擎；引擎不可用（未连接 /
#   RAK_CAR_SERIAL_ENGINE=0 / 帧类型不受支持）时自动降级为旧的
#   lock + 同步 round-trip。recovery（sync_with_probe）先 pause 引擎，
#   连上后 attach 新 fd，字节层语义不变。
# ============================================================
import heapq as _heapq
import itertools as _itertools
import select as _select

ENGINE_ENABLED = os.environ.get("RAK_CAR_SERIAL_ENGINE", "1") not in {"0", "false", "False"}

# 优先级（数字越小越先出队）
PRIORITY_URGENT = 0
PRIORITY_NORMAL = 1
PRIORITY_READ = 2

# io 线程空闲时的唤醒间隔（队列空且无共享读在飞）
_ENGINE_IDLE_SLEEP = 0.0005


class SerialEngineJob:
    """一次串口 round-trip 的提交凭证。

    调用方 `submit()` 后 `event.wait(timeout)`，成功后从 `result` 取应答 payload。
    失败语义与旧同步路径一一对应（见 SerialWrap.get_anwser）：
      - result=None + error=None       → 控制器无响应（旧 ControllerNoResponseError）
      - result=None + error="transport" → 串口通信异常（旧 ControllerTransportError）
      - result=None + error="not_ready" → 控制器未连接（旧 ControllerNotReadyError）
    """
    __slots__ = (
        "kind", "payload", "time_out", "priority",
        "coalesce_key", "share_key",
        "event", "result", "error", "submitted_at",
    )

    def __init__(self, kind, payload, time_out, priority=PRIORITY_NORMAL,
                 coalesce_key=None, share_key=None):
        self.kind = kind            # "mc602" / "mc602_wireness" / "call"
        self.payload = payload      # bytes（帧载荷）；kind="call" 时是 fn(serial, dev)
        self.time_out = float(time_out)
        self.priority = priority
        self.coalesce_key = coalesce_key  # 同 key 写帧队列内只留最新
        self.share_key = share_key        # 同 key 读请求合并为一次物理读
        self.event = threading.Event()
        self.result = None
        self.error = None
        self.submitted_at = time.time()

    def done(self, result=None, error=None):
        self.result = result
        self.error = error
        self.event.set()


class SerialEngine:
    """单 io 线程串口调度器。线程安全，供 SerialWrap 挂载/卸载。"""

    def __init__(self):
        self._wakeup_r, self._wakeup_w = os.pipe()
        try:
            os.set_blocking(self._wakeup_r, False)
            os.set_blocking(self._wakeup_w, False)
        except Exception:
            pass
        self._seq = _itertools.count()
        self._pending = []          # heapq: (priority, seq, job)
        self._coalesce_groups = {}  # coalesce_key -> [job, ...]（写合并：整组等最新帧应答）
        self._inflight = {}         # share_key -> [job, ...]（读共享：在飞物理帧）
        self._pending_lock = threading.Lock()
        # 运行态由 SerialWrap 在 self.lock 内写入、io 线程快照读取
        self._attached = False
        self._dev_kind = None       # "mc602" / "mc602_wireness" / None
        self._thread = None
        self._stop_event = threading.Event()
        self._idle = threading.Event()  # io 线程无活可干时置位（quiesce 用）
        self._idle.set()
        self.stats = {
            "frames": 0, "coalesced": 0, "shared": 0,
            "timeouts": 0, "errors": 0,
        }

    # ---------- 生命周期 ----------

    def start(self):
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = Thread(target=self._io_loop, name="serial-engine", daemon=True)
        self._thread.start()

    def shutdown(self):
        self._stop_event.set()
        self._wake()

    def attach(self, serial_obj, dev):
        """连接建立后由 SerialWrap 在 self.lock 内调用。"""
        kind = None
        if dev is not None:
            name = str(getattr(dev, "name", "")).lower()
            if "wireness" in name:
                kind = "mc602_wireness"
            elif "mc602" in name:
                kind = "mc602"
        with self._pending_lock:
            self._attached = kind is not None
            self._dev_kind = kind
            self._serial = serial_obj
            self._dev = dev
        self._wake()

    def detach(self):
        """recovery / close 前由 SerialWrap 在 self.lock 内调用。
        顺序：① 置 _attached=False 挡住新提交；② quiesce 等 io 线程跑完
        当前帧（fd 随后可能被 close,不能让 io 线程还在读写）；③ 清空队列，
        所有未完成 job 以 transport 错误结束，避免调用方挂死。"""
        with self._pending_lock:
            self._attached = False
        self.quiesce(timeout=1.5)
        with self._pending_lock:
            self._dev_kind = None
            self._serial = None
            self._dev = None
            pending = [job for _, _, job in self._pending]
            self._pending = []
            self._coalesce_groups.clear()
            inflight = []
            for jobs in self._inflight.values():
                inflight.extend(jobs)
            self._inflight.clear()
        for job in pending + inflight:
            job.done(error="transport")

    def quiesce(self, timeout=1.5):
        """等 io 线程把手头帧跑完（空闲）。recovery 关 fd 前必须调。"""
        return self._idle.wait(timeout)

    def call_on_io_thread(self, fn, time_out, priority=PRIORITY_URGENT):
        """把任意 fn(serial_obj, dev) 丢给 io 线程原子执行（ping / bootloader 握手用）。

        返回 (ok: bool, result)。引擎未 attach / 提交失败 → (False, None)，
        调用方自行降级。结果通过 job.result 携带；执行抛异常 → error="transport"。
        """
        job = SerialEngineJob(kind="call", payload=fn, time_out=time_out, priority=priority)
        if not self.submit(job):
            return False, None
        finished = job.event.wait(time_out + 2.0)
        if not finished or job.error is not None:
            return False, None
        return True, job.result

    def is_attached(self, kind=None):
        if kind is None:
            return self._attached
        return self._attached and self._dev_kind == kind

    # ---------- 提交 ----------

    def supports(self, kind):
        return self.is_attached(kind)

    def submit(self, job):
        if self._stop_event.is_set() or self._thread is None:
            return False
        with self._pending_lock:
            if not self._attached:
                return False
            sk = job.share_key
            if sk is not None:
                group = self._inflight.get(sk)
                if group is not None:
                    # 已有同 key 读帧在飞：直接并组,共享其应答,不进队列
                    group.append(job)
                    self.stats["shared"] += 1
                    job.time_out = max(job.time_out, group[0].time_out)
                    return True
            ck = job.coalesce_key
            if ck is not None:
                # 写合并：job 进组；堆里留一条条目，出队时只认组内最新一条，
                # 过期条目被跳过，整组 job 共享最新帧的应答（调用方都能拿到结果）。
                self._coalesce_groups.setdefault(ck, []).append(job)
            _heapq.heappush(self._pending, (job.priority, next(self._seq), job))
        self._wake()
        return True

    def _wake(self):
        try:
            os.write(self._wakeup_w, b"x")
        except Exception:
            pass

    # ---------- io 线程主循环 ----------

    def _io_loop(self):
        wake_r = self._wakeup_r
        while not self._stop_event.is_set():
            job, members = self._pop_next()
            if job is None:
                self._idle.set()
                # 清掉唤醒字节，然后 select 等待
                try:
                    os.read(wake_r, 64)
                except Exception:
                    pass
                try:
                    _select.select([wake_r], [], [], _ENGINE_IDLE_SLEEP)
                except Exception:
                    time.sleep(_ENGINE_IDLE_SLEEP)
                continue
            self._idle.clear()
            try:
                self._execute(job, members)
            except Exception as exc:
                self.stats["errors"] += 1
                for member in members:
                    member.done(error="transport")
                logger.debug("serial-engine execute err: %s", exc)
        # 收尾：未完成的 job 全部失败返回
        with self._pending_lock:
            leftovers = [job for _, _, job in self._pending]
            self._pending = []
            self._coalesce_groups.clear()
            for jobs in self._inflight.values():
                leftovers.extend(jobs)
            self._inflight.clear()
        for job in leftovers:
            job.done(error="transport")

    def _pop_next(self):
        """出队下一个有效 job。写合并组只认最新一条；过期条目直接丢弃。
        返回 (job, members)：members 是共享同一物理帧应答的全部 job。"""
        with self._pending_lock:
            while self._pending:
                priority, seq, job = _heapq.heappop(self._pending)
                ck = job.coalesce_key
                if ck is None:
                    return job, [job]
                group = self._coalesce_groups.get(ck)
                if not group or group[-1] is not job:
                    continue  # 过期条目：已被更新的同 key 帧顶掉
                # 幸存者：取走整组（后续同 key 提交会开新组）
                self._coalesce_groups.pop(ck, None)
                if len(group) > 1:
                    self.stats["coalesced"] += len(group) - 1
                return job, group
        return None, None

    def _execute(self, job, members):
        # 快照运行态；detach 后统一按 transport 失败
        serial_obj = self._serial
        dev = self._dev
        kind = self._dev_kind
        if not self._attached or serial_obj is None or dev is None:
            for member in members:
                member.done(error="transport")
            return

        # ---- callable job：整段在 io 线程原子执行（ping_rx 等多帧序列） ----
        if job.kind == "call":
            try:
                result = job.payload(serial_obj, dev)
            except Exception as exc:
                logger.debug("serial-engine call job err: %s", exc)
                for member in members:
                    member.done(error="transport")
                return
            for member in members:
                member.done(result=result)
            return

        # ---- 读共享：在飞期间后来的同 key 读已在 submit() 并组 ----
        sk = job.share_key
        if sk is not None:
            with self._pending_lock:
                self._inflight[sk] = list(members)
            group = members
            payload = None
            err = None
            try:
                payload = self._round_trip(serial_obj, dev, kind, job)
            except Exception as exc:
                self.stats["errors"] += 1
                err = "transport"
                logger.debug("serial-engine share round_trip err: %s", exc)
            finally:
                with self._pending_lock:
                    group = self._inflight.pop(sk, None) or members
            for member in group:
                member.done(result=payload, error=err)
            return

        payload = self._round_trip(serial_obj, dev, kind, job)
        for member in members:
            member.done(result=payload)

    def _round_trip(self, serial_obj, dev, kind, job):
        """一次物理 write+read。异常向上抛（调用方统一记 transport 失败）。"""
        self.stats["frames"] += 1
        serial_obj.reset_buffer()
        dev.send_cmd(serial_obj, job.payload)
        payload = dev.get_anwser(serial_obj, job.time_out)
        if payload is None:
            self.stats["timeouts"] += 1
        return payload



# ============================================================
# SerialEngine（串口 I/O 引擎）
# ------------------------------------------------------------
# 旧模型：调用方持 SerialWrap.lock 阻塞 write+read（一问一答），
#   50Hz 轮速下发 / odom 读 / arm PID 互相在同一把锁上排队，
#   每次 round-trip 5-20ms，50Hz 预算被吃光。
# 新模型：串口 fd 由唯一 io 线程（SerialEngine）持有；
#   所有设备调用 = submit(帧) → 等自己的 Event。引擎提供：
#     1) 写合并：同一 coalesce_key 在队列里只保留最新一帧
#        （轮速 50Hz 突发下发不再堆积物理帧）；
#     2) 读共享：同一 share_key 的并发请求只打一个物理读帧，
#        结果广播（encoder 查询不再各自打一遍）；
#     3) 优先级：URGENT（急停/零速）插队，NORMAL 次之，READ 最低；
#     4) 帧超时：引擎内部按 time_out 控制，调用方只等 event。
# 兼容：SerialWrap.get_anwser 默认走引擎；引擎不可用（未连接 /
#   RAK_CAR_SERIAL_ENGINE=0）时自动降级为旧的 lock + 同步 round-trip。
# ============================================================

import threading as _threading

ENGINE_ENABLED = os.environ.get("RAK_CAR_SERIAL_ENGINE", "1") not in {"0", "false", "False"}

# 优先级（数字越小越先出队）
PRIORITY_URGENT = 0
PRIORITY_NORMAL = 1
PRIORITY_READ   = 2

# io 线程空闲时的唤醒间隔（队列空且无共享读在飞）
_ENGINE_IDLE_SLEEP = 0.0005

# 急停/零速帧前缀集合（命令前2字节），命中的走 PRIORITY_URGENT
# MC602 停止帧：速度全零（0x02 0x03 轮速指令 + 后续全0）
# 这里只放最高优先级的前缀；不认识的默认 PRIORITY_NORMAL
_URGENT_CMD_PREFIXES: frozenset = frozenset()  # 可由上层 set_urgent_prefixes() 扩展


def set_urgent_prefixes(prefixes):
    """运行时注册急停帧前缀（bytes 对象集合）。由 mc602_ctl2.py 调用。"""
    global _URGENT_CMD_PREFIXES
    _URGENT_CMD_PREFIXES = frozenset(prefixes)


# 模块级单例引擎（SerialWrap.__init__ 时 attach，recovery 时 detach+attach）
_serial_engine: SerialEngine = SerialEngine()
_serial_engine.start()


def _notify_runtime_session(method_name, detail=None):
    try:
        from runtime.hardware.controller_session import get_controller_session

        session = get_controller_session()
        method = getattr(session, method_name, None)
        if method is None:
            return
        if detail is None:
            method()
        else:
            method(detail)
    except Exception:
        pass

class CotrollerInfo:
    def __init__(self, baudrate, timeout=0.1, mode="USB") -> None:
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_mode = mode
        self.name:str = None

    def send_cmd(self, cmd):
        pass

    def get_anwser(self, cmd):
        pass
    
    def ping_rx(self):
        pass
    
    def download_bin(self, obj):
        pass

    def __str__(self) -> str:
        return "baudrate:{},timeout:{},mode:{}".format(self.baudrate, self.timeout, self.connect_mode)

class SerialWrap(serial.Serial):
    def __init__(self):
        super(SerialWrap, self).__init__(port=None, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, \
                                         stopbits=serial.STOPBITS_ONE, timeout=0.03, xonxoff=False, rtscts=False, \
                                         dsrdtr=False)
        mc601 = MC601()
        mc602_usb = MC602()
        mc602_wireness = MC602Wireness()
        self.dev_list:List[CotrollerInfo] = [mc601, mc602_usb, mc602_wireness]
        self.dev = None
        self.connect_flag = False

        self.lock = Lock()
        self.generation = 0
        self.last_ok_at = 0.0
        self.last_error = None
        self.timeout = 0.01
        # 2026-08-03：串口 I/O 引擎（单 io 线程 + 帧队列 + 写合并/读共享）。
        # get_anwser 优先走引擎；RAK_CAR_SERIAL_ENGINE=0 可整体退回旧同步路径。
        self.engine = SerialEngine()
        if ENGINE_ENABLED:
            self.engine.start()
        # note_io_success 通知节流：引擎路径上每帧都回调会让 controller_session
        # 的 RLock 被 200+ 帧/s 抢爆。改成 0.5s 间隔推送状态（心跳另有 1Hz ping）。
        self._last_session_notify_at = 0.0
        self.auto_connect_on_import = os.environ.get("RAK_CAR_SERIAL_AUTO_CONNECT", "1") not in {
            "0",
            "false",
            "False",
        }
        if self.auto_connect_on_import:
            self.connect_until_ready(timeout=None)
        self.timeout = 0.1

    def _close_locked(self):
        # 先让引擎解除绑定，排空在飞帧，再关物理 fd
        if ENGINE_ENABLED and _serial_engine is not None:
            try:
                _serial_engine.detach()
            except Exception:
                pass
        try:
            self.engine.detach()
        except Exception:
            pass
        try:
            if self.is_open:
                super(SerialWrap, self).close()
        except Exception:
            pass
        self.connect_flag = False

    def _get_dev_candidates(self, controller_name=None):
        if not controller_name:
            return list(self.dev_list)
        candidates = []
        for ctl_dev in self.dev_list:
            if controller_name.lower() in ctl_dev.name.lower():
                candidates.append(ctl_dev)
        for ctl_dev in self.dev_list:
            if ctl_dev not in candidates:
                candidates.append(ctl_dev)
        return candidates

    def _connect_candidate_locked(self, port_name, controller_name=None):
        self.set_port(port_name)
        time.sleep(0.01)
        if not self.open():
            return None
        for ctl_dev in self._get_dev_candidates(controller_name):
            self.set_ctl_serial(ctl_dev)
            try:
                if ctl_dev.ping_rx(self):
                    return ctl_dev
            except Exception:
                pass
        self._close_locked()
        return None

    def _finish_connect_locked(self, ctl_dev):
        self.dev = ctl_dev
        self.generation += 1
        self.last_ok_at = time.time()
        self.last_error = None
        logger.info("port is {}, controller is {}, mode {}".format(self.port, self.dev.name, self.dev.connect_mode))
        _notify_runtime_session("note_io_success")
        # 兜底:MCU 闲置 2s 内核会 autosuspend ttyUSB → 下次串口操作返 device not ready
        # → 触发 note_io_failure → 误判 controller 掉线 → 写 RUNCODE 重启下位机。
        # 写 power/control=on,power/autosuspend=-1 永久禁用;失败不抛(udev 才是正经路子)。
        # 2026-07-16: 配合 controller_session 调阈值,根治 PM2 反复 rebuild 循环。
        try:
            from ...tools.camera import disable_usb_autosuspend_for_tty
            if self.port:
                disable_usb_autosuspend_for_tty(self.port)
        except Exception as exc:
            logger.debug("disable_usb_autosuspend_for_tty({}) 兜底失败: {}".format(self.port, exc))
        # 引擎接管 fd（mc602 / mc602_wireness）；mc601 走旧同步路径
        try:
            self.engine.attach(self, ctl_dev)
        except Exception as exc:
            logger.warning("serial engine attach 失败,走同步路径: {}".format(exc))
        return ctl_dev

    def sync_with_probe(self, probe_result=None):
        with self.lock:
            self._close_locked()
            ctl_dev = None
            if probe_result is not None and getattr(probe_result, "port", None):
                ctl_dev = self._connect_candidate_locked(
                    probe_result.port,
                    getattr(probe_result, "controller", None),
                )
            if ctl_dev is None and probe_result is None:
                ctl_dev = self.ping_port()
            if ctl_dev is None:
                self.last_error = "控制器未进入 program 模式"
                _notify_runtime_session("mark_offline", self.last_error)
                raise ControllerNotReadyError(self.last_error)
            return self._finish_connect_locked(ctl_dev)

    def reconnect(self, timeout=3.0, probe_result=None):
        deadline = None if timeout is None else (time.time() + float(timeout))
        last_error = "控制器未就绪"
        while True:
            try:
                return self.sync_with_probe(probe_result=probe_result)
            except Exception as exc:
                last_error = str(exc)
                if deadline is not None and time.time() >= deadline:
                    raise ControllerNotReadyError(last_error)
                logger.critical("控制器重连失败: {}".format(last_error))
                time.sleep(0.5 if deadline is not None else 1.0)
                probe_result = None

    def connect_until_ready(self, timeout=None):
        return self.reconnect(timeout=timeout, probe_result=None)

    def _engine_kind_for(self, dev):
        name = str(getattr(dev, "name", "")).lower()
        if "wireness" in name:
            return "mc602_wireness"
        if "mc602" in name:
            return "mc602"
        return None

    def _notify_io_success_light(self):
        """引擎路径的成功回调：只刷时间戳 + 0.5s 间隔推一次 session 状态。

        旧实现每帧 1 次 import 查找 + 2 次 RLock（note_io_success），50Hz×4 通道
        下是纯 GIL 开销。controller_session 的 PROGRAM_READY 新鲜度由心跳
        （1Hz ping_current → note_io_success）兜底，0.5s 间隔足够。
        """
        self.last_ok_at = time.time()
        self.last_error = None
        now = self.last_ok_at
        if now - self._last_session_notify_at >= 0.5:
            self._last_session_notify_at = now
            _notify_runtime_session("note_io_success")

    def get_anwser(self, cmd:bytes, time_out=0.1, priority=None,
                   coalesce_key=None, share_key=None)->bytes:
        """一问一答。默认走 SerialEngine（单 io 线程 + 写合并 + 读共享）；
        引擎不可用（未连接 / 禁用 / mc601 / 未启动）时降级为旧 lock 同步路径。

        新增可选参数（全部向后兼容,不传 = 旧行为）：
          priority     PRIORITY_URGENT/NORMAL/READ（默认 NORMAL）
          coalesce_key 同 key 写帧在队列内只保留最新（轮速下发用）
          share_key    同 key 并发读合并成一次物理读（编码器/模拟量用）
        """
        dev = self.dev
        kind = self._engine_kind_for(dev) if dev is not None else None
        if (
            ENGINE_ENABLED
            and kind is not None
            and self.connect_flag
            and self.engine.supports(kind)
        ):
            job = SerialEngineJob(
                kind=kind,
                payload=cmd,
                time_out=time_out,
                priority=PRIORITY_NORMAL if priority is None else priority,
                coalesce_key=coalesce_key,
                share_key=share_key,
            )
            if self.engine.submit(job):
                # 引擎内按 time_out 收帧；额外的宽限覆盖排队等待（旧模型同样要排队,
                # 只是在 lock 上排；这里给 2s 上限,超时按通信异常处理）
                finished = job.event.wait(job.time_out + 2.0)
                if finished and job.error is None and job.result is not None:
                    self._notify_io_success_light()
                    return job.result
                if not finished:
                    self.last_error = "串口引擎响应超时(排队过深)"
                    _notify_runtime_session("note_io_failure", self.last_error)
                    raise ControllerTransportError(self.last_error)
                if job.error == "not_ready":
                    self.last_error = "控制器未连接"
                    _notify_runtime_session("mark_offline", self.last_error)
                    raise ControllerNotReadyError(self.last_error)
                if job.error == "transport":
                    self.last_error = "串口通信异常(引擎)"
                    _notify_runtime_session("note_io_failure", self.last_error)
                    raise ControllerTransportError(self.last_error)
                # result=None 且无 error → 控制器无响应
                self.last_error = "控制器无响应，可能已脱离 program 模式"
                _notify_runtime_session("note_io_failure", self.last_error)
                raise ControllerNoResponseError(self.last_error)
        # ---- 同步降级路径（旧语义,逐字节保留） ----
        else:
            with self.lock:
                if self.dev is None or not self.connect_flag or not self.is_open:
                    self.last_error = "控制器未连接"
                    _notify_runtime_session("mark_offline", self.last_error)
                    raise ControllerNotReadyError(self.last_error)
                try:
                    self.reset_buffer()
                    self.dev.send_cmd(self, cmd)
                    res = self.dev.get_anwser(self)
                except serial.SerialException as exc:
                    self.last_error = "串口通信异常: {}".format(exc)
                    _notify_runtime_session("note_io_failure", self.last_error)
                    raise ControllerTransportError(self.last_error)
                except Exception as exc:
                    self.last_error = "控制器通信异常: {}".format(exc)
                    _notify_runtime_session("note_io_failure", self.last_error)
                    raise ControllerTransportError(self.last_error)
                if res is None:
                    self.last_error = "控制器无响应，可能已脱离 program 模式"
                    _notify_runtime_session("note_io_failure", self.last_error)
                    raise ControllerNoResponseError(self.last_error)
                self.last_ok_at = time.time()
                self.last_error = None
                _notify_runtime_session("note_io_success")
                return res

    def ping_current(self, timeout=0.05):
        with self.lock:
            if self.dev is None or not self.connect_flag or not self.is_open:
                return False
            dev = self.dev
            kind = self._engine_kind_for(dev)
            # 引擎接管 fd 时,ping 必须走 io 线程原子执行 —— 否则 ping 帧会和
            # 正在飞的业务帧在同一 fd 上交叠读字节,两边都拿到串扰数据。
            if kind is not None and self.engine.supports(kind):
                ok, res = self.engine.call_on_io_thread(
                    lambda s, d: bool(d.ping_rx(s, time_out=timeout)),
                    time_out=timeout + 0.05,
                    priority=PRIORITY_READ,
                )
                if not ok:
                    return False
                ping_ok = bool(res)
            else:
                try:
                    self.reset_buffer()
                    ping_ok = bool(dev.ping_rx(self, time_out=timeout))
                except Exception:
                    return False
            if ping_ok:
                self.last_ok_at = time.time()
                self.last_error = None
            return ping_ok

    def set_bps(self, bps):
        self.baudrate = bps

    def set_port(self, port):
        if self.connect_flag:
            self.close()
            self.connect_flag = False
        self.port = port
        
    def open(self):
        try:
            if self.port is None:
                return False
            self.connect_flag = True
            super(SerialWrap, self).open()
            return True
        except Exception as e:
            self.connect_flag = False
            return False

    def get_serial_list(self):
        port_list = list_ports.comports()
        # for port in port_list:
        #     print('端口号：' + port[0] + '   端口名：' + port[1])
        port_list = [port for port in port_list if "CH340" in port[1] or "USB" in port[1]]
        port_list.sort(key=lambda x: "CH340" not in x[1])
        return port_list
    
    def set_ctl_serial(self, ctl_dev:CotrollerInfo):
        self.baudrate = ctl_dev.baudrate

    def ping_port(self):
        serial_list = self.get_serial_list()
        if len(serial_list) == 0:
            logger.error("未找到串口,查看是否插入了串口,或者查看下位机是否开机")
            return None
        for serial in serial_list:
            try:
                logger.info("try:{}".format(serial))
                # _connect_candidate_locked 内部已完整做了 open + ping_rx 全流程
                ctl_dev = self._connect_candidate_locked(serial[0])
                if ctl_dev is not None:
                    return ctl_dev
                # ping 未通 → 尝试下载 bin 启动程序
                for ctl_dev in self.dev_list:
                    # logger.info("try downlaod bin:{}".format(ctl_dev.name))
                    self.set_ctl_serial(ctl_dev)
                    if ctl_dev.download_bin(self):
                        return ctl_dev
                self._close_locked()
            except Exception as e:
                logger.error(e)
        logger.error("未找到支持的设备")
        return None
    
    def reset_buffer(self):
        if not self.is_open:
            return
        self.reset_input_buffer()
        self.reset_output_buffer()

    def assert_dev(self, name_test:str):
        # 转成小写对比
        name_dev = self.dev.name.lower()
        name_test = name_test.lower()
        if name_test in name_dev or name_dev in name_test:
            return True
        else:
            logger.error(f"dev is not {name_test}")
            while True:
                time.sleep(1)

class MC601(CotrollerInfo):
    def __init__(self, baudrate=380400, timeout=0.1, mode="USB") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc601"
        self.header = bytes.fromhex('77 68')
        self.tail = bytes.fromhex('0A')

    def send_cmd(self, serial_obj:SerialWrap, cmd:bytes):
        # cmd_len = len(cmd).to_bytes(1, 'big')
        # # 加入头尾数据帧
        # cmd_all = self.header + cmd_len + cmd + self.tail
        # serial_obj.write(cmd_all)
        serial_obj.write(cmd)

    def get_anwser(self, serial_obj:SerialWrap, time_out=0.05):
        time_start = time.time()
        dst_len = 0
        res = serial_obj.read(3)
        if len(res) != 3:
            return None
        # 总帧长
        dst_len = res[2] + 7
        # 获取剩余数据
        res = res + serial_obj.read(dst_len-3)
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        while True:
            if time.time() - time_start > time_out:
                return None
            # data = res[3:-1]
            
            if len(res) == dst_len:
                if res[0] == self.header[0] and res[-1] == self.tail[0]:
                    # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
                    return res
                else:
                    return None
            res = res + serial_obj.read(dst_len - len(res))
    
    def ping_rx(self, serial_obj:SerialWrap, time_out=0.05):
        time_start = time.time()
        while time.time() - time_start < time_out:
            serial_obj.reset_buffer()
            self.send_cmd(serial_obj, bytes.fromhex('77 68 04 00 01 CA 01 0A'))
            res = self.get_anwser(serial_obj, 0.03)
            if res is not None:
                # 关闭mc601省电模式
                self.send_cmd(serial_obj, bytes.fromhex('77 68 03 00 02 67 0A'))
                return True
        
class MC602(CotrollerInfo):
    def __init__(self, baudrate=1000000, timeout=0.1, mode="USB") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc602"
        self.header = bytes.fromhex('77 68')
        self.tail = bytes.fromhex('0A')

    def send_cmd(self, serial_obj:SerialWrap, cmd:bytes):
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        # 加入头尾数据帧
        cmd_all = self.header + cmd_len + cmd + self.tail
        serial_obj.write(cmd_all)
        # logger.info("send cmd:\'{}\'".format(cmd_all.hex(' ')))

    def get_anwser(self, serial_obj:SerialWrap, time_out=0.2):
        # time.sleep(0.1)
        # res = serial_obj.read(2)
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        time_start = time.time()
        dst_len = 0
        res = serial_obj.read(3)
        if len(res) != 3:
            return None
        # 总帧长
        dst_len = res[2]
        # 获取剩余数据
        res = res + serial_obj.read(dst_len-3)
        while True:
            if time.time() - time_start > time_out:
                return None
            # data = res[3:-1]
            # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
            if len(res) == dst_len:
                if res[0] == self.header[0] and res[-1] == self.tail[0]:
                    return res[3:-1]
                else:
                    return None
            res = res + serial_obj.read(dst_len - len(res))

    
    def ping_rx(self, serial_obj:SerialWrap, time_out=0.05):
        time_start = time.time()

        while time.time() - time_start < time_out:
            serial_obj.reset_buffer()
            self.send_cmd(serial_obj, bytes.fromhex('02 01 10'))
            res = self.get_anwser(serial_obj, 0.02)
            if res is not None:
                return True
        return False

    def download_bin(self, serial_obj:SerialWrap):
        is_mc602 = False
        serial_obj.write(bytes.fromhex('55 AA 00 01 08 00 00 F7'))
        time.sleep(0.01)
        ret = serial_obj.read(10)
        # print(ret.hex())
        if ret == bytes.fromhex('66 BB 01 01 0A 00 5A 02 00 76'):
            is_mc602 = True
            logger.info("is mc602")
            logger.info("load program")
            # 启动控制器加载程序
            start_time = time.time()
            while time.time() - start_time < 1:
                serial_obj.reset_buffer()
                serial_obj.write(bytes.fromhex('55 AA 00 40 0B 00 00 D0 00 08 DD'))
                time.sleep(0.01)
                ret = serial_obj.read(11)
                if ret == bytes.fromhex("66 BB 01 41 0B 00 00 D0 00 08 B9"):
                    break
            if self.ping_rx(serial_obj, 2):
                return True

        if is_mc602:
            try:
                from runtime.core import settings as runtime_settings

                allow_download = runtime_settings.get_auto_download_on_bootloader()
            except Exception:
                allow_download = False
            if not allow_download:
                logger.info("skip auto download, waiting existing program to start")
                return False
            # 下载程序并进入program程序
            logger.info("downloading program")
            serial_obj.close()
            from runtime.hardware.controller_download import download_mc602_program

            result, _msg = download_mc602_program("RunA", isrun=True)
            if result:
                deadline = time.time() + 6.0
                while time.time() < deadline:
                    try:
                        if not serial_obj.is_open:
                            serial_obj.open()
                    except Exception:
                        time.sleep(0.2)
                        continue
                    if self.ping_rx(serial_obj, time_out=0.5):
                        return True
                    try:
                        serial_obj.close()
                    except Exception:
                        pass
                    time.sleep(0.2)
        return False
    
class MC602Wireness(CotrollerInfo):
    def __init__(self, baudrate=115200, timeout=0.2, mode="Wireness") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc602_wireness"
        self.header = bytes.fromhex('FE')
        self.header_escape = bytes.fromhex('FE FC')
        self.tail = bytes.fromhex('FF')
        self.tail_escape = bytes.fromhex('FE FD')
        self.port_src = bytes.fromhex('90')
        self.port_dst = bytes.fromhex('91')
        self.target_id = bytes.fromhex('5D 3D')

    def set_target_id(self, target_id:bytes):
        self.target_id = target_id

    def send_cmd(self, serial_obj:SerialWrap, cmd:bytes):
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        # 端口地址数据组合
        cmd_data = self.port_src + self.port_dst + self.target_id + cmd
        # 转义处理
        cmd_data_escape = cmd_data.replace(self.header, self.header_escape).replace(self.tail, self.tail_escape)
        # 加入头尾数据帧
        cmd_all = self.header + cmd_len + cmd_data_escape + self.tail
        serial_obj.write(cmd_all)
        # logger.info("send cmd:\'{}\'".format(cmd_all.hex(' ')))

    def get_anwser(self, serial_obj:SerialWrap, time_out=0.15):
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        time_start = time.time()
        res = b''
        while True:
            if time.time() - time_start > time_out:
                logger.error("get_anwser timeout {}".format(res.hex(' ')))
                return None
            res = serial_obj.read(2)
            if len(res) == 2:
                break
        dst_len = res[1] + 3
        res = res + serial_obj.read(dst_len - 2)
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        while True:
            if time.time() - time_start > time_out:
                return None
            # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
            res = res.replace(self.header_escape, self.header).replace(self.tail_escape, self.tail)
            rx_len = len(res)
            if rx_len == dst_len:
                if res[0] == self.header[0] and res[-1] == self.tail[0]:
                    return res[6:-1]
            res = res + serial_obj.read(dst_len - len(res))
    
    def ping_rx(self, serial_obj:SerialWrap, time_out=0.3):
        self.send_cmd(serial_obj, bytes.fromhex('02 01 10'))
        # serial_obj.flush()   # 直到发送完毕
        # time.sleep(0.01)
        ret = self.get_anwser(serial_obj, time_out)
        if ret is not None:
            return True
        return False

serial_wrap = SerialWrap()
# logger.info("start time:{}".format(time.time()))

if __name__ == "__main__":
    last_time = time.time()
    # print(time.time())
    serial_wrap.timeout=0.3
    while True:
        # serial_wra
        serial_wrap.reset_buffer()
        ret = serial_wrap.get_anwser(bytes.fromhex('02 02 01 10'))
        # print(ret)
        time.sleep(0.4)
        # serial_wrap.write(bytes.fromhex('FE 10 90 91 5d 3d 02 02 01 10 02 02 02 10 02 02 03 10 FF'))
        # res = serial_wrap.read(23)
        # if(res != b'\xfe\x14\x90\x91]=\x02\x02\x01\x10\x02\x02\x02\x10\x02\x02\x03\x10\x02\x02\x04\n\xff'):
        #     print(res)
        # ret = serial_wrap.get_anwser(bytes.fromhex('02 02 01 F0'))
        # if ret is not None:
        #     logger.info("ret:\'{}\'".format(ret.hex(' ')))

        # fps = 1.0 / (time.time() - last_time)
        # last_time = time.time()
        # logger.info("fps:{}".format(fps))
        # time.sleep(1)
    # logger.info()
