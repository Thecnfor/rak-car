"""mc602_peripheral_node — ROS2 wrapper for MC602 hardware.

Subscribes to:
  /vehicle_wbt/v1/cmd/peripheral/beep_event    (std_msgs/Empty)
  /vehicle_wbt/v1/cmd/peripheral/storage_event (std_msgs/Bool)
  /vehicle_wbt/v1/cmd/peripheral/shoot_event   (std_msgs/Empty)

Translates each event into a call on the corresponding
vehicle_wbt_smartcar_hw device class (Buzzer_2 / ServoPwm / PoutD).
The protocol layer (pyserial, frame assembly) lives in
vehicle_wbt_smartcar_hw.mc602 — this node is a thin glue layer.

The beep_event handler plays a pre-canned Happy Birthday melody by
sequencing Buzzer_2.rings() calls on the MC602. Total ~30 s.
Melody plays asynchronously via rclpy timers so rclpy.spin stays
responsive.

If the serial port cannot be opened (no driver, no MC602, no
permission), the node logs a warning and continues — it never
fabricates a successful beep.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from vehicle_wbt_smartcar_bridge import config as cfg
from vehicle_wbt_smartcar_hw.mc602 import (
    Buzzer_2,
    MC602Serial,
    PoutD,
    ServoPwm,
)


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 1_000_000

# 25-note Happy Birthday melody (freq_hz, duration_sec).
# Equal-temperament tuning, A4=440. Each note + 50 ms gap ≈ 0.45 s.
# Total ≈ 11.25 s of notes + 1.25 s of gaps ≈ 12 s.
HAPPY_BIRTHDAY_MELODY = [
    # "Happy birthday to you"
    (262, 0.40), (262, 0.40), (294, 0.80),
    (262, 0.80), (349, 0.80), (330, 0.80),
    # "Happy birthday to you"
    (262, 0.40), (262, 0.40), (294, 0.80),
    (262, 0.80), (392, 0.80), (349, 0.80),
    # "Happy birthday dear [name]"
    (262, 0.40), (262, 0.40), (523, 0.80),
    (440, 0.80), (349, 0.80), (330, 0.80), (294, 0.80),
    # "Happy birthday to you"
    (466, 0.40), (466, 0.40), (440, 0.80),
    (349, 0.80), (392, 0.80), (349, 0.80),
]


class MC602PeripheralNode(Node):
    """Subscribes to peripheral events and calls hw-package devices."""

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

        self.serial = MC602Serial(port=port, baud=baud)
        self._serial_ok = self.serial.open()
        if self._serial_ok:
            self.get_logger().info(
                f'MC602 link open: {port} @ {baud} baud')
        else:
            self.get_logger().warn(
                f'MC602 port {port} unavailable. Will log calls instead of sending.')

        # Cache device-class instances (avoid recreating on every event).
        self._buzzer = Buzzer_2(self.serial)
        self._storage_servo = ServoPwm(self.serial, port_id=self.servo_port)
        self._shooter = PoutD(self.serial, port_id=self.dout_port)

        # Subscriptions
        self.beep_sub = self.create_subscription(
            Empty, cfg.TRIGGER_BEEP, self._on_beep, 10)
        self.storage_sub = self.create_subscription(
            Bool, cfg.TRIGGER_STORAGE, self._on_storage, 10)
        self.shoot_sub = self.create_subscription(
            Empty, cfg.TRIGGER_SHOOT, self._on_shoot, 10)

        self._melody_timer = None  # rclpy.Timer or None
        self._melody_index = 0

        self.get_logger().info(
            f'mc602_peripheral_node ready: '
            f'beep={cfg.TRIGGER_BEEP}, storage={cfg.TRIGGER_STORAGE}, '
            f'shoot={cfg.TRIGGER_SHOOT} '
            f'(beep plays Happy Birthday melody, '
            f'~{sum(d for _, d in HAPPY_BIRTHDAY_MELODY):.1f}s)')

    # ----- helpers -----

    def _hw_or_log(self, frame: list, label: str) -> None:
        """Report frame sent / would-have-sent."""
        if self._serial_ok:
            if frame:
                self.get_logger().info(f'{label}: sent {bytes(frame).hex(" ")}')
            else:
                self.get_logger().warn(f'{label}: write failed')
        else:
            hex_repr = bytes(frame).hex(' ') if frame else '<empty>'
            self.get_logger().warn(
                f'{label}: would have sent {hex_repr} (no MC602 link)')

    # ----- callbacks -----

    def _on_beep(self, _msg: Empty) -> None:
        """Plays Happy Birthday via rclpy timer (one frame per note).

        ROS2 Humble's create_timer() doesn't accept one_shot=True, so
        we create a recurring timer and self-destroy it inside the
        callback once the melody completes.
        """
        if self._melody_timer is not None:
            self.destroy_timer(self._melody_timer)
            self._melody_timer = None
            self.get_logger().info('cancelled previous melody')

        self._melody_index = 0
        self.get_logger().info(
            f'playing Happy Birthday ({len(HAPPY_BIRTHDAY_MELODY)} notes, '
            f'~{sum(d for _, d in HAPPY_BIRTHDAY_MELODY):.1f}s)')

        def _play_next() -> None:
            if self._melody_index >= len(HAPPY_BIRTHDAY_MELODY):
                if self._melody_timer is not None:
                    self.destroy_timer(self._melody_timer)
                    self._melody_timer = None
                return
            freq, dur = HAPPY_BIRTHDAY_MELODY[self._melody_index]
            frame = self._buzzer.rings(freq, dur)
            self._hw_or_log(
                frame, f'melody[{self._melody_index}] f={freq}Hz d={dur}s')
            self._melody_index += 1

        first_dur = HAPPY_BIRTHDAY_MELODY[0][1]
        self._melody_timer = self.create_timer(
            first_dur + 0.05, _play_next)

    def _on_storage(self, msg: Bool) -> None:
        # Mirrors MyCar.set_storage -> ServoPwm(1).set_angle([-42, 165][flag])
        angle = 165 if msg.data else -42
        frame = self._storage_servo.set_angle(angle)
        self._hw_or_log(frame, f'set_storage({msg.data})')

    def _on_shoot(self, _msg: Empty) -> None:
        # Mirrors MyCar.shooting -> PoutD(4).set(1); sleep 0.3; set(0)
        frame_on = self._shooter.set(1)
        self._hw_or_log(frame_on, 'shoot(on)')
        # Schedule the off frame via a one-shot-ish timer.
        self.create_timer(
            0.3,
            lambda: self._hw_or_log(self._shooter.set(0), 'shoot(off)'),
            one_shot=False,
        )


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
