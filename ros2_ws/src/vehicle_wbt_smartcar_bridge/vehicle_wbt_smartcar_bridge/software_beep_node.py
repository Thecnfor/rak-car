"""software_beep_node — temporary audible feedback for the beep service.

Subscribes to /vehicle_wbt/v1/cmd/peripheral/beep_event (emitted by
PeripheralService when the beep() service is called). On each event,
generates a short sine-wave beep in-memory as a WAV file and plays it
via `aplay` through the Jetson HDMI audio output.

This is a *placeholder* until the real MC602 peripheral node exists —
that node will subscribe to the same topic and drive the actual
WhalesBot buzzer pin instead. The replacement is a drop-in swap.

Replace me by:
  ros2 run vehicle_wbt_platform_cpp mc602_peripheral_node --buzzer-event beep_event
"""
from __future__ import annotations

import math
import struct
import subprocess
import tempfile
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from vehicle_wbt_smartcar_bridge import config as cfg


# Audio params — short chirp is more recognizable than a long tone.
BEEP_FREQ_HZ = 880.0      # A5 — pleasant, audible above fan noise
BEEP_DURATION_S = 0.18
BEEP_SAMPLE_RATE = 22050
BEEP_AMPLITUDE = 0.45      # 0..1, leave headroom for HDMI


def make_beep_wav(path: str) -> None:
    """Generate a single sine-wave chirp as 16-bit mono PCM WAV."""
    n_samples = int(BEEP_SAMPLE_RATE * BEEP_DURATION_S)
    frames = []
    for i in range(n_samples):
        # Apply a short attack/release envelope to avoid click artifacts
        env = min(1.0, i / 200.0) * min(1.0, (n_samples - i) / 200.0)
        sample = BEEP_AMPLITUDE * env * math.sin(
            2.0 * math.pi * BEEP_FREQ_HZ * i / BEEP_SAMPLE_RATE)
        frames.append(struct.pack('<h', int(sample * 32767)))
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(BEEP_SAMPLE_RATE)
        w.writeframes(b''.join(frames))


class SoftwareBeepNode(Node):
    """Listens to beep_event and plays a tone via aplay."""

    def __init__(self) -> None:
        super().__init__('software_beep_node')
        self.sub = self.create_subscription(
            Empty, cfg.TRIGGER_BEEP, self._on_beep, 10)
        self.get_logger().info(
            f'software_beep_node ready: subscribing to {cfg.TRIGGER_BEEP}, '
            f'playing {BEEP_FREQ_HZ:.0f} Hz for {BEEP_DURATION_S:.2f}s via aplay')

    def _on_beep(self, _msg: Empty) -> None:
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                make_beep_wav(tf.name)
                wav_path = tf.name
            # -q: quiet (no aplay status), default device
            subprocess.run(['aplay', '-q', wav_path], check=False, timeout=5)
            self.get_logger().info('beep!')
        except Exception as e:
            self.get_logger().error(f'beep failed: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SoftwareBeepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()