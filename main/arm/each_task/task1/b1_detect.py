#!/usr/bin/python3
"""task1 / step b1 —— 视觉找种子 + 底盘对齐

动作:
  1) 侧摄识别种子(按大小降序,先拿最大的 10cm)
  2) 底盘微调:move_to_detection_target 让目标在视野中心
  3) 返回目标种子的 (x_mm, y_mm) 业务坐标

依赖:b1 之前已经到达播种区(阶段A 完成)
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b1_detect(client: ArmClient, runner: ArmRunner) -> dict:
    """B1:找种子,返回种子信息。"""
    print("=== [B1] 视觉找种子 + 底盘对齐 ===")

    # ---- 视觉 ----
    print("  [视觉] get_detection_results (sort_pos=size_desc)")
    # det = client._call_car("get_detection_results", sort_pos="size_desc")
    # boxes = det.get("result", [])
    #  boxes 里每项: [cls_id, det_id, label, score, x_c, y_c, w, h]
    # 按 size 排后,boxes[0] 是最大的 10cm 圆柱

    # 模拟一个检测结果(等真实视觉接好后改)
    seed = {
        "label": "seed_large",
        "diameter_mm": 100.0,
        "x_mm": 120.0,    # 业务坐标,需要像素→mm 标定
        "y_mm": -5.0,      # 落地高度
    }

    # ---- 底盘对齐 ----
    print(f"  [底盘] move_to_detection_target(label='{seed['label']}')")
    # client._call_car("move_to_detection_target", label=seed["label"])

    print(f"  -> 目标种子: {seed['label']}  x={seed['x_mm']:.0f}mm  y={seed['y_mm']:.0f}mm")
    print("=== [B1] 完成 ===\n")
    return {"seed": seed}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b1_detect(client, runner)


if __name__ == "__main__":
    main()
