"""Peripheral SDK client. Mirrors MyCar.beep/set_storage/shooting."""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from vehicle_wbt_smartcar_msgs.srv import Beep, SetStorage, Shoot


class PeripheralClient:
    def __init__(self, node: Node) -> None:
        self.node = node
        prefix = '/vehicle_wbt/v1/cmd/peripheral/'
        self._clients = {
            'beep':        node.create_client(Beep, prefix + 'beep'),
            'set_storage': node.create_client(SetStorage, prefix + 'set_storage'),
            'shoot':       node.create_client(Shoot, prefix + 'shoot'),
        }

    def _call(self, name: str, req, timeout: float = 5.0):
        cli = self._clients[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'peripheral service {name} unavailable')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result()

    def beep(self):
        return self._call('beep', Beep.Request())

    def set_storage(self, state: bool):
        req = SetStorage.Request()
        req.state = bool(state)
        return self._call('set_storage', req)

    def shooting(self):
        return self._call('shoot', Shoot.Request())