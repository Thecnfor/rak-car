"""main/chassis/examples/02_stanley_lane.py
Stanley 控制律版本：把 POuterLoop 换成 StanleyOuterLoop。
其他完全相同。
"""
from main.chassis import ChassisClient, DoubleLoopRunner, StanleyOuterLoop


def main(max_seconds: float = 20.0, vx: float = 0.3, k: float = 0.6) -> None:
    api = ChassisClient.connect()
    api.start_lane_feed(hz=20.0)
    runner = DoubleLoopRunner(
        api=api,
        outer=StanleyOuterLoop(vx=vx, k=k),
        hz=50.0,
    )
    runner.run(max_seconds=max_seconds)
    api.stop_lane_feed()


if __name__ == "__main__":
    main()
