"""Arm SDK client. Mirrors car_wrap_2026.ArmController."""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from vehicle_wbt_smartcar_msgs.srv import (
    ArmReset, ArmSetPose, ArmGrasp, ArmMoveX, ArmMoveY,
    ArmSetArmAngle, ArmSetHandAngle,
)


class ArmClient:
    """Calls /vehicle_wbt/v1/cmd/arm/* services."""

    def __init__(self, node: Node) -> None:
        self.node = node
        prefix = '/vehicle_wbt/v1/cmd/arm/'
        self._clients = {
            'reset_position': node.create_client(ArmReset, prefix + 'reset_position'),
            'set_pose':       node.create_client(ArmSetPose, prefix + 'set_pose'),
            'grasp':          node.create_client(ArmGrasp, prefix + 'grasp'),
            'move_x':         node.create_client(ArmMoveX, prefix + 'move_x'),
            'move_y':         node.create_client(ArmMoveY, prefix + 'move_y'),
            'set_arm_angle':  node.create_client(ArmSetArmAngle, prefix + 'set_arm_angle'),
            'set_hand_angle': node.create_client(ArmSetHandAngle, prefix + 'set_hand_angle'),
        }

    def _call(self, name: str, req, timeout: float = 30.0):
        cli = self._clients[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'arm service {name} unavailable')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result()

    # Mirrors MyCar.arm.*
    def reset_position(self):
        return self._call('reset_position', ArmReset.Request())

    def set_arm_pose(self, x=0.0, y=0.0, arm: str = 'LEFT', hand: str = 'UP'):
        """Mirrors ArmController.set_arm_pose.

        The official API takes (arm_id, pitch, arm, hand). The SDK
        convention here mirrors car_task_function.py usage where x/y are
        the dominant control. arm_id is taken from x and pitch from y
        (you can also pass the canonical form via set_pose_canonical).
        """
        req = ArmSetPose.Request()
        req.arm_id = int(round(x)) if x is not None else 0
        req.pitch = float(y) if y is not None else 0.0
        req.arm_dir = arm
        req.hand_dir = hand
        return self._call('set_pose', req)

    def set_pose_canonical(self, arm_id: int, pitch: float, arm: str, hand: str):
        """Canonical form: explicit (arm_id, pitch, arm_dir, hand_dir)."""
        req = ArmSetPose.Request()
        req.arm_id = int(arm_id)
        req.pitch = float(pitch)
        req.arm_dir = arm
        req.hand_dir = hand
        return self._call('set_pose', req)

    def grasp(self, state: bool):
        req = ArmGrasp.Request()
        req.state = bool(state)
        return self._call('grasp', req)

    def move_x_position(self, x: float, out_time: float = 0.0):
        req = ArmMoveX.Request()
        req.x = float(x)
        req.out_time = float(out_time)
        return self._call('move_x', req)

    def move_y_position(self, y: float, out_time: float = 0.0):
        req = ArmMoveY.Request()
        req.y = float(y)
        req.out_time = float(out_time)
        return self._call('move_y', req)

    def reset_x(self):
        return self._call('reset_position', ArmReset.Request())  # reuse reset

    def set_arm_angle(self, angle: int):
        req = ArmSetArmAngle.Request()
        req.angle = int(angle)
        return self._call('set_arm_angle', req)

    def set_hand_angle(self, angle):
        """Official API accepts int OR string ('UP'/'DOWN'/'MID')."""
        if isinstance(angle, str):
            # String convention from car_wrap_2026.py
            mapping = {'UP': 10, 'DOWN': -10, 'MID': 0}
            angle = mapping.get(angle.upper(), 0)
        req = ArmSetHandAngle.Request()
        req.angle = int(angle)
        return self._call('set_hand_angle', req)

    def x_get_position(self) -> float:
        """Read current x from the bridge's /vehicle_wbt/v1/state/arm/pose."""
        # Lightweight: instantiate a transient subscription. Real code on
        # the dev box usually caches this once per task, so the cost is
        # negligible.
        from vehicle_wbt_smartcar_msgs.msg import ArmPose
        latest = {'pose': None}

        def cb(msg: ArmPose):
            latest['pose'] = msg

        sub = self.node.create_subscription(
            ArmPose, '/vehicle_wbt/v1/state/arm/pose', cb, 10)
        deadline = __import__('time').time() + 1.0
        import rclpy
        while __import__('time').time() < deadline and latest['pose'] is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)
        if latest['pose'] is None:
            raise TimeoutError('arm pose not received')
        return float(latest['pose'].x)