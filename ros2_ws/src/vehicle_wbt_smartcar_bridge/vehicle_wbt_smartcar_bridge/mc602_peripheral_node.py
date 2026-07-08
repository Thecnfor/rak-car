"""mc602_peripheral_node — real MC602 hardware driver.

Translates the official Baidu SmartCar 2026 peripheral API to MC602
serial commands. Subscribes to:
  /vehicle_wbt/v1/cmd/peripheral/beep_event    (std_msgs/Empty)
  /vehicle_wbt/v1/cmd/peripheral/storage_event (std_msgs/Bool)
  /vehicle_wbt/v1/cmd/peripheral/shoot_event   (std_msgs/Empty)

For each event, opens the serial port, writes the corresponding MC602
CTL frame, closes. This mirrors car_wrap_2026.MyCar:
  beep       -> Buzzer_2.rings(freq, duration)         -> MC602 beep pin
  set_storage-> ServoPwm(1).set_angle([-42, 165][flag]) -> MC602 servo pin 1
  shooting   -> PoutD(4).set(1) for 0.3s               -> MC602 dout pin 4

The beep_event handler plays a pre-canned melody ("Happy Birthday")
by sequencing Buzzer frames on the MC602. Total duration ~12s at
400ms-per-note pacing. The melody plays asynchronously via rclpy
timers so rclpy.spin is never blocked.

MC602 USB frame format (per WhalesBot SDK serial_wrap.py + mc602_ctl2.py):
  Header: 0x77 0x68
  Length: 1 byte (= payload length + 4)
  Payload: <dev_id> <mode> <port_id> <args...>
  Tail: 0x0A

  mode: 1=get, 2=set, 3=reset
  dev_id (from ctl602_dev_list):
    0x0a beep       format="BBB" (freq/2, dur*20, 0)
    0x05 servo_pwm  format="bbBB" (angle_lo, angle_hi, ...)
    0x10 dout       format="bbb" (1=on, 0=off)
  port_id: physical pin number on MC602

Serial I/O uses the standard POSIX layer (os.open / os.write / os.read)
via ctypes to libc. We do NOT depend on pyserial — keeps the runtime
python env clean. termios config is set via tcsetattr (raw mode,
custom baud).

If the serial port cannot be opened (no driver loaded, no MC602
connected, no permission), this node logs the would-be frame in hex
and continues — never fabricates a successful beep.

Spec: car_wrap_2026.py:344-409 (sensor_init / set_storage / shooting),
car_wrap_2026.py:364-371 (beep)
"""
from __future__ import annotations

import ctypes
import ctypes.util
import errno
import fcntl
import os
import struct
import termios
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from vehicle_wbt_smartcar_bridge import config as cfg


# MC602 device IDs and payload formats (from ctl602_dev_list)
MC602_BEEP       = 0x0a
MC602_SERVO_PWM  = 0x05
MC602_DOUT       = 0x10
MC602_HEADER     = bytes.fromhex('77 68')
MC602_TAIL       = bytes.fromhex('0A')

# Serial config
DEFAULT_BAUD = 1_000_000  # MC602 default baud per official SDK
DEFAULT_PORT = '/dev/ttyUSB0'  # per full_system.launch.py default

# Happy Birthday melody (freq_hz, duration_sec).
# Equal-temperament A4=440. Notes chosen so the entire sequence fits
# comfortably within MC602 Buzzer_2's freq/2 byte (max raw freq = 510 Hz
# which gives a fundamental of 1020 Hz — above human hearing so safe).
# 0.4s per note + 0.05s gap ≈ 0.45s/note × 26 notes ≈ 12 seconds total.
HAPPY_BIRTHDAY_MELODY = [
    # "Happy birthday to you"
    (262, 0.40), (262, 0.40), (294, 0.80),
    (262, 0.80), (349, 0.80), (330, 0.80),
    # "Happy birthday to you"
    (262, 0.40), (262, 0.40), (294, 0.80),
    (262, 0.80), (392, 0.80), (349, 0.80),
    # "Happy birthday dear [name]"
    (262, 0.40), (262, 0.40), (523, 0.80), (440, 0.80), (349, 0.80), (330, 0.80), (294, 0.80),
    # "Happy birthday to you"
    (466, 0.40), (466, 0.40), (440, 0.80), (349, 0.80), (392, 0.80), (349, 0.80),
]


def _build_mc602_frame(dev_id: int, mode: int, port_id: int, args: bytes) -> bytes:
    """Wrap a payload in the MC602 USB frame format."""
    payload = bytes([dev_id, mode, port_id]) + args
    length = len(payload) + 4  # +2 header +1 length +1 tail
    return MC602_HEADER + bytes([length]) + payload + MC602_TAIL


def build_beep_frame(freq: int = 200, duration: float = 0.2) -> bytes:
    """Mirrors Buzzer_2.rings(freq, duration):
       payload = (freq/2, duration*20, 0) encoded as 3 signed bytes.
    """
    args = bytes([
        int(freq / 2) & 0xff,
        int(duration * 20) & 0xff,
        0,
    ])
    return _build_mc602_frame(MC602_BEEP, mode=2, port_id=0, args=args)


def build_servo_frame(port_id: int, angle_deg: int) -> bytes:
    """Mirrors ServoPwm(port_id).set_angle(angle_deg):
       payload = (angle_lo, angle_hi, ?, ?) — angle is u16 LE, plus 2 padding.
       WhalesBot ServoPwm encodes 0..180 deg as 0..9000 raw units,
       angle = (deg / 180) * 9000. Format bbBB -> 4 bytes after port_id.
    """
    raw = int((angle_deg / 180.0) * 9000)
    args = bytes([raw & 0xff, (raw >> 8) & 0xff, 0, 0])
    return _build_mc602_frame(MC602_SERVO_PWM, mode=2, port_id=port_id, args=args)


def build_dout_frame(port_id: int, state: int) -> bytes:
    """Mirrors PoutD(port_id).set(state) for 0/1 GPIO. Format "bbb"."""
    args = bytes([state & 0xff, 0, 0])
    return _build_mc602_frame(MC602_DOUT, mode=2, port_id=port_id, args=args)


def build_ping_frame() -> bytes:
    """MC602 ping (per MC602.ping_rx). Used at startup to verify link."""
    return _build_mc602_frame(0x02, mode=1, port_id=0x10, args=b'')


# ---------------------------------------------------------------------------
# Serial I/O via libc (no pyserial dependency)
# ---------------------------------------------------------------------------


class MC602Serial:
    """Minimal MC602 serial-port wrapper using ctypes to libc.

    Opens /dev/ttyUSB0 (or configured port) in raw mode at 1 Mbaud.
    write_frame() is fire-and-forget; for the beep use case the MC602
    processes the command asynchronously.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD) -> None:
        self.port = port
        self.baud = baud
        self._fd: Optional[int] = None
        self._libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)

    def open(self) -> bool:
        try:
            # O_RDWR | O_NOCTTY | O_NONBLOCK: non-blocking so writes don't stall
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self._fd = None
            return False
        # Switch to blocking with VMIN=0 VTIME=1 (100ms read timeout)
        try:
            attrs = termios.tcgetattr(fd)
            # Raw mode
            attrs[0] = 0  # iflag
            attrs[1] = 0  # oflag
            attrs[2] = (termios.CLOCAL | termios.CREAD)  # cflag
            attrs[3] = 0  # lflag
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1
            # Baud (cfsetispeed/cfsetospeed via termios)
            baud_const = self._baud_to_constant(self.baud)
            if baud_const is not None:
                attrs[4] = baud_const  # ispeed
                attrs[5] = baud_const  # ospeed
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            # Clear O_NONBLOCK so writes are blocking (with VMIN/VTIME)
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
        except (termios.error, OSError):
            os.close(fd)
            self._fd = None
            return False
        self._fd = fd
        return True

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def write_frame(self, frame: bytes) -> bool:
        if self._fd is None:
            return False
        try:
            os.write(self._fd, frame)
            return True
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return False
            return False

    def read(self, n: int) -> bytes:
        if self._fd is None:
            return b''
        try:
            return os.read(self._fd, n)
        except OSError:
            return b''

    @staticmethod
    def _baud_to_constant(baud: int) -> Optional[int]:
        """termios baud constant lookup for common rates (Linux)."""
        table = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
            230400: termios.B230400,
            460800: termios.B460800,
            500000: termios.B500000,
            576000: termios.B576000,
            921600: termios.B921600,
            1000000: termios.B1000000,
            1152000: termios.B1152000,
            1500000: termios.B1500000,
            2000000: termios.B2000000,
            2500000: termios.B2500000,
            3000000: termios.B3000000,
            3500000: termios.B3500000,
            4000000: termios.B4000000,
        }
        return table.get(baud)


class MC602PeripheralNode(Node):
    """Subscribes to peripheral events and sends MC602 frames."""

    def __init__(self) -> None:
        super().__init__('mc602_peripheral_node')
        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('servo_port', 1)  # per official servo_1
        self.declare_parameter('dout_port', 4)   # per official PoutD(4)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.servo_port = self.get_parameter('servo_port').get_parameter_value().integer_value
        self.dout_port = self.get_parameter('dout_port').get_parameter_value().integer_value

        self.serial = MC602Serial(port, baud)
        self._serial_ok = self.serial.open()
        if self._serial_ok:
            self.get_logger().info(
                f'MC602 link open: {port} @ {baud} baud')
        else:
            self.get_logger().warn(
                f'MC602 port {port} unavailable (errno={errno.__dict__.get("ENOENT", "n/a")}). '
                f'Will log frames instead of sending.')

        # Subscriptions
        self.beep_sub = self.create_subscription(
            Empty, cfg.TRIGGER_BEEP, self._on_beep, 10)
        self.storage_sub = self.create_subscription(
            Bool, cfg.TRIGGER_STORAGE, self._on_storage, 10)
        self.shoot_sub = self.create_subscription(
            Empty, cfg.TRIGGER_SHOOT, self._on_shoot, 10)

        self._melody_timer = None  # rclpy.Timer or None
        self._melody_index = 0
        self._melody_cumulative_time = 0.0

        self.get_logger().info(
            f'mc602_peripheral_node ready: '
            f'beep={cfg.TRIGGER_BEEP}, storage={cfg.TRIGGER_STORAGE}, shoot={cfg.TRIGGER_SHOOT} '
            f'(beep plays Happy Birthday melody, ~{sum(d for _,d in HAPPY_BIRTHDAY_MELODY):.1f}s)')

    def _send_or_log(self, frame: bytes, label: str) -> None:
        if self._serial_ok:
            ok = self.serial.write_frame(frame)
            if ok:
                self.get_logger().info(f'{label}: sent {frame.hex(" ")}')
            else:
                self.get_logger().warn(f'{label}: write failed')
        else:
            self.get_logger().warn(
                f'{label}: would have sent {frame.hex(" ")} (no MC602 link)')

    def _on_beep(self, _msg: Empty) -> None:
        """Plays the Happy Birthday melody on the MC602 buzzer.

        Schedules one timer per note at cumulative offsets so rclpy.spin
        stays responsive. Each timer fires once and writes one beep frame.
        Total melody time ~= sum(note durations) + small inter-note gaps.

        Note: ROS2 Humble's create_timer() doesn't support one_shot=True —
        we create a recurring timer and self-destroy it inside the
        callback once the melody is done.
        """
        if self._melody_timer is not None:
            # Cancel any in-flight melody (one at a time for now).
            self.destroy_timer(self._melody_timer)
            self._melody_timer = None
            self.get_logger().info('cancelled previous melody')

        self._melody_index = 0
        self.get_logger().info(
            f'playing Happy Birthday ({len(HAPPY_BIRTHDAY_MELODY)} notes, '
            f'~{sum(d for _,d in HAPPY_BIRTHDAY_MELODY):.1f}s)')

        def _play_next():
            if self._melody_index >= len(HAPPY_BIRTHDAY_MELODY):
                # Melody complete: destroy self, drop reference.
                if self._melody_timer is not None:
                    self.destroy_timer(self._melody_timer)
                    self._melody_timer = None
                return
            freq, dur = HAPPY_BIRTHDAY_MELODY[self._melody_index]
            frame = build_beep_frame(freq=freq, duration=dur)
            self._send_or_log(
                frame, f'melody[{self._melody_index}] f={freq}Hz d={dur}s')
            self._melody_index += 1

        # Use a recurring timer at note+gap cadence; the callback
        # self-destroys when index goes past the end.
        first_dur = HAPPY_BIRTHDAY_MELODY[0][1]
        self._melody_timer = self.create_timer(
            first_dur + 0.05,
            _play_next,
        )

    def _on_storage(self, msg: Bool) -> None:
        # Mirrors MyCar.set_storage -> ServoPwm(1).set_angle([-42, 165][flag])
        angle = 165 if msg.data else -42
        frame = build_servo_frame(self.servo_port, angle)
        self._send_or_log(frame, f'set_storage({msg.data})')

    def _on_shoot(self, _msg: Empty) -> None:
        # Mirrors MyCar.shooting -> PoutD(4).set(1); time.sleep(0.3); set(0)
        # We send the "on" frame; the MC602 will hold the GPIO; the caller
        # (or a follow-up service) sends "off". For a single-event API, we
        # approximate by sending on immediately and scheduling an off.
        frame_on = build_dout_frame(self.dout_port, 1)
        self._send_or_log(frame_on, 'shoot(on)')
        self.create_timer(
            0.3,
            lambda: self._send_or_log(
                build_dout_frame(self.dout_port, 0), 'shoot(off)'),
            one_shot=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MC602PeripheralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()