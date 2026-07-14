"""main/lane_outer_loop.py
lane 外环 demo：用 WebSocket 订阅 lane_state push + 下发轮速。

用法：
    1) 确认 runtime 在线、car 已初始化
    2) python3 main/lane_outer_loop.py
    3) Ctrl-C 退出（自动 unsubscribe_lane + stop lane_feed + 关 WS）

完整 API：见 runtime/VISION_API.md "外环专用" 一节。
"""
from __future__ import annotations

import json
import sys
import time

try:
    import websocket  # type: ignore
except ImportError:
    print("缺少 websocket-client 依赖: pip3 install websocket-client")
    sys.exit(1)

try:
    from main.settings import DEFAULT_API_HOST
except ImportError:
    DEFAULT_API_HOST = "http://192.168.3.60:5050"


def _http(method: str, path: str, payload=None, timeout: float = 5.0):
    import urllib.request
    url = DEFAULT_API_HOST + path
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main():
    # 1) 启动车端 lane_feed 守护线程
    print("[1] 启动 lane_feed (POST /v1/vision/lane/feed)...")
    try:
        r = _http("POST", "/v1/vision/lane/feed", {"hz": 20})
    except Exception as exc:
        print(f"  启动失败: {exc}")
        print("  确认 runtime 在跑 + car 已初始化（GET /v1/health 看 initialized）")
        sys.exit(1)
    print(f"  {r}")

    # 2) 连 WS，订阅 lane_state
    ws_url = DEFAULT_API_HOST.replace("http://", "ws://").replace("https://", "wss://") + "/v1/ws"
    print(f"\n[2] 连 WS {ws_url}")
    ws = websocket.create_connection(ws_url, timeout=5)
    welcome = json.loads(ws.recv())
    print(f"  welcome ok={welcome['ok']} usage={list(welcome['usage'].keys())}")

    ws.send(json.dumps({"op": "subscribe_lane"}))
    ack = json.loads(ws.recv())
    print(f"  subscribe_lane: subscribed={ack.get('subscribed')} hz={ack.get('hz')}")

    # 3) 外环：收 push → 控制律 → 下发轮速
    push_count = 0
    send_count = 0
    t_start = time.time()
    print("\n[3] 外环运行中...（Ctrl-C 退出）")
    try:
        while True:
            msg = json.loads(ws.recv())
            op = msg.get("op")
            if op != "lane_state":
                # 可能是别的 ping/health 响应，跳过
                continue
            push_count += 1
            d = msg["data"]
            ey = d.get("error_y")
            ea = d.get("error_angle")
            if ey is None or ea is None:
                continue
            # === 你的控制律 ===
            vx = 0.30
            vy = -0.5 * ey
            wz = -0.8 * ea
            # 直接发 (vx, vy, wz)，runtime 服务端 IK 反算 4 轮速、绕开 set_velocity 里程计耦合
            ws.send(json.dumps({
                "op": "realtime/chassis_velocity",
                "vx": vx, "vy": vy, "wz": wz,
            }))
            send_count += 1
            if push_count <= 3 or push_count % 40 == 0:
                age = round(time.time() - d["updated_at"], 3)
                print(
                    f"  push #{push_count} age={age}s  "
                    f"error_y={ey:+.5f} error_angle={ea:+.5f}  "
                    f"vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}"
                )
    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        print(
            f"\n[Ctrl-C] 跑了 {elapsed:.1f}s，"
            f"收 {push_count} 次 lane_state，下发 {send_count} 次轮速 "
            f"({push_count / max(elapsed, 0.001):.1f} Hz)"
        )
    finally:
        # 4) 清理
        print("\n[4] 清理...")
        try:
            ws.send(json.dumps({"op": "unsubscribe_lane"}))
            ws.recv()
        except Exception:
            pass
        ws.close()
        try:
            _http("POST", "/v1/vision/lane/feed/stop")
            print("  lane_feed 已停止")
        except Exception as exc:
            print(f"  停止 lane_feed 失败: {exc}")


if __name__ == "__main__":
    main()