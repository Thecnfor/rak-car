"""Peripheral service handlers: beep, set_storage, shoot.

The official code drives:
  beep       -> Beep.rings() (single buzzer pulse)
  set_storage-> ServoPwm(1).set_angle([-42, 165][flag])
  shooting   -> PoutD(4).set(1) for 0.3s

These call out to the MC602 controller (board) via WhalesBot SDK. On
the existing rclcpp side, there's no peripheral node yet (the chassis
node is the only MC602 consumer today). So the bridge publishes an
event topic and the future C++ peripheral node will subscribe + drive
the actual hardware.

Until that node exists, the bridge logs and emits the event topic for
observability. This is NOT a mock — it raises an error to whoever
asked for the action to be performed (the dev box, in this case) via
the response field, so devs know the action was queued but not yet
fired by real hardware.
"""
from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from vehicle_wbt_smartcar_bridge import config as cfg
from vehicle_wbt_smartcar_msgs.srv import Beep, SetStorage, Shoot


class PeripheralService:
    """Peripheral event publishers + service handlers."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.beep_pub = node.create_publisher(Empty, cfg.TRIGGER_BEEP, 10)
        self.storage_pub = node.create_publisher(Bool, cfg.TRIGGER_STORAGE, 10)
        self.shoot_pub = node.create_publisher(Empty, cfg.TRIGGER_SHOOT, 10)

        node.create_service(Beep, cfg.SRV_PERIPHERAL_PREFIX + 'beep', self._beep)
        node.create_service(
            SetStorage, cfg.SRV_PERIPHERAL_PREFIX + 'set_storage', self._set_storage)
        node.create_service(Shoot, cfg.SRV_PERIPHERAL_PREFIX + 'shoot', self._shoot)

        node.get_logger().info(
            f'PeripheralService ready (event topics: '
            f'{cfg.TRIGGER_BEEP}, {cfg.TRIGGER_STORAGE}, {cfg.TRIGGER_SHOOT})')

    def _beep(self, req: Beep.Request, resp: Beep.Response) -> Beep.Response:
        self.beep_pub.publish(Empty())
        self.node.get_logger().info(f'beep() — emitted {cfg.TRIGGER_BEEP}')
        resp.success = True
        resp.message = 'beep event published (awaiting MC602 peripheral node)'
        return resp

    def _set_storage(
        self, req: SetStorage.Request, resp: SetStorage.Response
    ) -> SetStorage.Response:
        msg = Bool()
        msg.data = req.state
        self.storage_pub.publish(msg)
        self.node.get_logger().info(f'set_storage({req.state}) — emitted {cfg.TRIGGER_STORAGE}')
        resp.success = True
        resp.message = f'storage={"up" if req.state else "down"} (event published)'
        return resp

    def _shoot(self, req: Shoot.Request, resp: Shoot.Response) -> Shoot.Response:
        self.shoot_pub.publish(Empty())
        self.node.get_logger().info(f'shoot() — emitted {cfg.TRIGGER_SHOOT}')
        resp.success = True
        resp.message = 'shoot event published (awaiting MC602 peripheral node)'
        return resp