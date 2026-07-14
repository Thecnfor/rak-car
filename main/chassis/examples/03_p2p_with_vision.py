"""main/chassis/examples/03_p2p_with_vision.py
外环跑 50Hz 巡线 → 触达终点线 → 让位给 car.move_to_detection_target 做视觉终点微调。

注意：这是占位骨架；底盘组按任务需要把"看到目标 / 走到目标位置"替换成自己的判定。
"""
from main.chassis import ChassisClient, DoubleLoopRunner, StanleyOuterLoop
from main.chassis.tasks import track_target


def main() -> None:
    api = ChassisClient.connect()
    api.start_lane_feed(hz=20.0)
    try:
        runner = DoubleLoopRunner(
            api=api,
            outer=StanleyOuterLoop(vx=0.3, k=0.6),
            hz=50.0,
        )
        # 这里示范性跑 5 秒；底盘组替换成"已到目标位置"的判定
        runner.run(max_seconds=5.0)
    finally:
        # 让位前先 zero out，然后交给车端内环 PID
        api.stop_wheel_speeds()
        track_target(api, label=None, time_out=3.0)
        api.stop_lane_feed()


if __name__ == "__main__":
    main()
