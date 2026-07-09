"""MC602 串口封装。

源码 1:1 对齐 baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/serial_wrap.py
的 MC602 class + ping_port 逻辑。删除 MC601/MC602Wireness 不需要的部分,
保留 SDK 风格的全局单例 + lock + 自动端口扫描。
"""
from __future__ import annotations

import time
from threading import Lock

import serial
from serial.tools import list_ports


class MC602(serial.Serial):
    """MC602 串口包装,SDK 风格(继承 pyserial)。

    Args:
        port: 串口设备路径,默认 None(由 ping_port 自动扫描)。
        baud: 波特率,默认 1_000_000(SDK 默认)。
        timeout: 串口读超时,默认 0.03s。
    """

    HEADER = bytes.fromhex('77 68')
    TAIL = bytes.fromhex('0A')

    def __init__(self, port=None, baud=1_000_000, timeout=0.03):
        super().__init__(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=timeout,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        self.lock = Lock()
        self.connect_flag = False

    def send_cmd(self, cmd: bytes) -> None:
        """发送一帧数据,自动加头/长/尾。

        对齐 SDK MC602.send_cmd(serial_wrap.py:215-219):cmd_len = len(cmd)+4,
        frame = HEADER + cmd_len + cmd + TAIL。
        """
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        frame = self.HEADER + cmd_len + cmd + self.TAIL
        self.write(frame)

    def get_anwser(self, cmd: bytes | None = None, time_out: float = 0.1) -> bytes | None:
        """发送 cmd(可选)+ 读 MC602 响应。

        对齐 SDK SerialWrap.get_anwser(serial_wrap.py:70-80):lock 保护下,如
        cmd 不为 None 先 send_cmd,再读响应。
        协议:读 3 字节 → 解 dst_len = res[2] → 读剩余 → 校验头尾 → 返回 res[3:-1]。

        Real-hardware recovery:真硬件密集测试中(>100 calls),pyserial 内部
        buffer 状态偶尔 corruption,导致 read(3) 立即返回 0 字节。SDK-direct
        (每次新建 SerialWrap) 不受影响,但 singleton 受污染。Fix:
        连续 N 次 read 0 字节后,自动 close + reopen 重置 OS state。
        """
        with self.lock:
            try:
                if cmd is not None:
                    # 注意:SDK 的 reset_buffer() 是 INPUT+OUTPUT 一起清;
                    # 我们的 reset_input_buffer + reset_output_buffer 在
                    # 连续 send+read 序列里偶尔会破坏 pyserial 内部 wait state。
                    # 保守起见,只在 cmd 不为 None 时清 output(避免丢 write),
                    # 永远不清 input(让旧的响应有机会被读到)。
                    try:
                        self.reset_output_buffer()
                    except Exception:
                        pass
                    self.send_cmd(cmd)
                time_start = time.time()
                res = self.read(3)
                if len(res) != 3:
                    # Auto-recovery:port state corruption,close + reopen
                    self._recover_port()
                    return None
                dst_len = res[2]
                res = res + self.read(dst_len - 3)
                while True:
                    if time.time() - time_start > time_out:
                        return None
                    if len(res) == dst_len:
                        if res[0] == self.HEADER[0] and res[-1] == self.TAIL[0]:
                            return res[3:-1]
                        return None
                    res = res + self.read(dst_len - len(res))
            except Exception:
                return None

    def _recover_port(self) -> None:
        """close + reopen port 以重置 pyserial 内部状态。

        真硬件密集测试中,singleton 偶尔进入"read 立即返回 0 字节"状态。
        SDK-direct(每次新 SerialWrap)无此问题。重开 port 是最便宜的恢复方式。
        """
        try:
            saved_port = self.port
            saved_baud = self.baudrate
            if self.is_open:
                self.close()
            time.sleep(0.05)
            self.port = saved_port
            self.baudrate = saved_baud
            self.open()
        except Exception:
            pass  # best-effort recovery

    def ping_rx(self, time_out: float = 0.05) -> bool:
        """探测 MC602 是否在响应。SDK `serial_wrap.py:248-257`。

        SDK 关键细节:ping 帧 `77 68 04 00 01 CA 01 0A` 必须通过 send_cmd() 再
        包一层(变成 `77 68 0C 77 68 04 00 01 CA 01 0A 0A`),不能直接 write。
        真硬件验证:SDK 1:1 字节通过 3 次 beep 测试。
        """
        time_start = time.time()
        while time.time() - time_start < time_out:
            self.reset_input_buffer()
            self.reset_output_buffer()
            # 通过 send_cmd() 包装 ping 帧(SDK serial_wrap.py:201 行为)
            self.send_cmd(bytes.fromhex('77 68 04 00 01 CA 01 0A'))
            res = self.get_anwser(cmd=None, time_out=0.03)
            if res is not None:
                # 关闭 MC601 省电模式(SDK 行为,serial_wrap.py:205)
                self.send_cmd(bytes.fromhex('77 68 03 00 02 67 0A'))
                return True
        return False

    def ping_port(self, baud: int = 1_000_000) -> str | None:
        """扫描 CH340/USB 串口,找到第一个能 ping 通的端口并打开。

        对齐 SDK `serial_wrap.py:113-142`。返回打开的端口名,失败 None。
        真硬件测试:SDK 的 ping time_out=0.05 太短,首字节经常迟到 100ms+。
        这里用 0.3s 兜底,确保不误判端口为"未响应"。
        """
        port_list = list_ports.comports()
        port_list = [p for p in port_list if 'CH340' in p[1] or 'USB' in p[1]]
        port_list.sort(key=lambda x: 'CH340' not in x[1])  # CH340 优先
        for p in port_list:
            try:
                self.port = p[0]
                self.baudrate = baud
                self.open()
                self.connect_flag = True
                if self.ping_rx(time_out=0.3):
                    return p[0]
                # 没 ping 通:不要关 port(可能 caller 还想用),让 is_open 留 True
                # 但要 reset 一下 buffer 给下次机会
                try:
                    self.reset_input_buffer()
                    self.reset_output_buffer()
                except Exception:
                    pass
            except Exception:
                try:
                    self.close()
                except Exception:
                    pass
                self.connect_flag = False
        return None

    def open(self) -> bool:
        """打开串口,带状态记录。"""
        try:
            if self.port is None:
                return False
            self.connect_flag = True
            super().open()
            return True
        except Exception:
            self.connect_flag = False
            return False


# 全局单例(SDK 风格,所有设备类共享)
serial_mc602 = MC602()
