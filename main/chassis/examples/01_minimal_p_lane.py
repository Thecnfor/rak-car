"""main/chassis/examples/01_minimal_p_lane.py
最简起步：10 行内跑通 client 端外环。
- 车端：client.start_lane_feed() 持续刷 lane_state
- 客户端：DoubleLoopRunner 50Hz 拉 lane_state, POuterLoop 算 4 轮线速度
- 退出：自动 zero out
"""
from main.chassis import ChassisClient, DoubleLoopRunner, POuterLoop


def main(max_seconds: float = 15.0, vx: float = 0.3) -> None:
    api = ChassisClient.connect()
    api.start_lane_feed(hz=50.0)
    runner = DoubleLoopRunner(api=api, outer=POuterLoop(vx=vx), hz=50.0)
    runner.run(max_seconds=max_seconds)
    api.stop_lane_feed()


if __name__ == "__main__":
    main()
