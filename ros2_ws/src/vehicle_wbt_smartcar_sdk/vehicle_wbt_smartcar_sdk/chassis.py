"""Chassis SDK client.

Calls /vehicle_wbt/v1/cmd/chassis/* services on the bridge. Each method
is blocking (waits for the service to return).
"""
from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node

from vehicle_wbt_smartcar_msgs.srv import (
    MoveFor, MoveToPosition, MoveTime, MoveDistance, ResetOdometry,
    LaneBase, LaneTime, LaneDis, LaneDisOffset,
)


class ChassisClient:
    """Thin wrapper that calls chassis services on the bridge."""

    def __init__(self, node: Node) -> None:
        self.node = node
        prefix = '/vehicle_wbt/v1/cmd/chassis/'
        # Each call spawns a fresh client (cheap; rclpy caches connections
        # internally). This avoids stale handles if the bridge restarts.
        self._clients = {
            'move_for':        node.create_client(MoveFor, prefix + 'move_for'),
            'move_to_position':node.create_client(MoveToPosition, prefix + 'move_to_position'),
            'move_time':       node.create_client(MoveTime, prefix + 'move_time'),
            'move_distance':   node.create_client(MoveDistance, prefix + 'move_distance'),
            'reset_odometry':  node.create_client(ResetOdometry, prefix + 'reset_odometry'),
            'lane_base':       node.create_client(LaneBase, prefix + 'lane_base'),
            'lane_time':       node.create_client(LaneTime, prefix + 'lane_time'),
            'lane_dis':        node.create_client(LaneDis, prefix + 'lane_dis'),
            'lane_dis_offset': node.create_client(LaneDisOffset, prefix + 'lane_dis_offset'),
        }

    def _call(self, name: str, req, timeout: float = 30.0):
        cli = self._clients[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'chassis service {name} unavailable')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f'chassis service {name} timed out')
        return future.result()

    def move_for(self, vec: Sequence[float]):
        req = MoveFor.Request()
        req.dx, req.dy, req.dz = float(vec[0]), float(vec[1]), float(vec[2])
        return self._call('move_for', req)

    def move_to_position(self, coords: Sequence[float]):
        req = MoveToPosition.Request()
        req.x, req.y, req.z = float(coords[0]), float(coords[1]), float(coords[2])
        return self._call('move_to_position', req)

    def move_time(self, speed_vec: Sequence[float], duration: float):
        req = MoveTime.Request()
        req.vx, req.vy, req.wz = float(speed_vec[0]), float(speed_vec[1]), float(speed_vec[2])
        req.duration = float(duration)
        return self._call('move_time', req)

    def move_distance(self, speed: float, distance: float, direction: int = 0):
        req = MoveDistance.Request()
        req.speed = float(speed)
        req.distance = float(distance)
        req.direction = int(direction)
        return self._call('move_distance', req)

    def reset_position(self):
        return self._call('reset_odometry', ResetOdometry.Request())

    def lane_base(self, speed: float):
        req = LaneBase.Request()
        req.speed = float(speed)
        return self._call('lane_base', req)

    def lane_time(self, speed: float, duration: float):
        req = LaneTime.Request()
        req.speed = float(speed)
        req.duration = float(duration)
        return self._call('lane_time', req)

    def lane_dis(self, speed: float, distance: float):
        req = LaneDis.Request()
        req.speed = float(speed)
        req.distance = float(distance)
        return self._call('lane_dis', req)

    def lane_dis_offset(self, speed: float, dis_hold: float):
        req = LaneDisOffset.Request()
        req.speed = float(speed)
        req.dis_hold = float(dis_hold)
        return self._call('lane_dis_offset', req)