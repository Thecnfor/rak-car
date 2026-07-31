"""PID + depth-aware gain 单测 — find_target_pid 行为验证."""
import unittest
from unittest.mock import MagicMock
from main.arm.vision import ArmVisionClient, TargetSelector


def _det_dict(dx=0.0, dy=0.0, label="h_dou_jiao", score=0.9, height=100):
    """构造一个 detection dict (与 runtime 实际返回格式一致)."""
    return {
        "label": label, "score": score, "det_id": 1, "cls_id": 1,
        "bbox_norm": {
            "x_center": dx, "y_center": dy,
            "width": 0.05, "height": 0.05,
        },
        "bbox_pixels": {
            "x1": 0, "y1": 0, "x2": height, "y2": height,
            "width": height, "height": height,
        },
    }


def _make_http_with_dets(dicts):
    """每次 get_vision_task_cache 返回新的 dict 列表 (避免 list 复用)."""
    http = MagicMock()
    counter = [0]
    pool = [d for d in dicts]

    def _next_state():
        counter[0] += 1
        # 深拷贝: 重新构造 dict, 避免 _parse_cache 改 list 元素后污染下次
        return {
            "task_state": {
                "detections": [{k: v for k, v in d.items()} for d in pool],
                "updated_at": float(counter[0]),
            }
        }

    http.get_vision_task_cache.side_effect = _next_state
    return http


class TestPIDDepth(unittest.TestCase):
    def test_pure_p_no_depth(self):
        """无 depth: dx_mm = -dx_norm * mm_per_norm_base = -0.1 * 30 = -3.0"""
        det = _det_dict(dx=0.1, dy=0.0, label="h_dou_jiao", height=0)  # bbox 高度 0 走 fallback
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("h_dou_jiao")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05,
            settle_stable_frames=99,  # 不达稳定
            target_real_height_m=None,  # 不走 depth
            mm_per_norm_base=30.0,
            timeout=0.5, max_iter=2,
        )
        self.assertFalse(result.converged)
        self.assertGreater(len(result.trace), 0)
        # 第一步 x_mm ≈ -3.0 (dx=0.1, pure P, mm_per_norm_eff=30)
        self.assertAlmostEqual(result.trace[0].x_mm, -3.0, places=1)

    def test_depth_aware_gain_skipped_without_bbox_pixels(self):
        """cache 路径 (_parse_cache) 不提供 bbox_pixels → depth-aware 跳过, 走 mm_per_norm_base.

        注: depth-aware 在 vision servo 真实生产中不触发, 因为 task_feed 30Hz cache
        无 bbox_pixels 字段. 深度估计只在用户调用 snap (POST /v1/vision/task) 时可用.
        本测试只验 PID 路径不破; compute_depth 单元测试在 test_servo_depth.py 覆盖.
        """
        det = _det_dict(dx=0.1, dy=0.0, label="cylinder_1", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("cylinder_1")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05,
            settle_stable_frames=99,
            target_real_height_m=0.30,
            focal_length_px=600.0,
            ref_depth_m=0.30,
            mm_per_norm_base=30.0,
            timeout=0.5, max_iter=2,
        )
        # bbox_pixels=None → mm_per_norm_eff = mm_per_norm_base = 30
        # dx_mm = -0.1 * 30 = -3.0
        self.assertAlmostEqual(result.trace[0].x_mm, -3.0, places=1)

    def test_compute_depth_formula(self):
        """compute_depth 公式: depth = real_height * focal / bbox_h
        (这是 depth-aware 真正生效时的换算, 单独单元测试, 不依赖 find_target_pid).
        """
        from main.arm.vision import ArmVisionClient
        from main.arm.vision.types import BBoxPixels
        bp = BBoxPixels(0, 0, 100, 100, 100, 100)
        depth = ArmVisionClient.compute_depth(bp, 0.30, 600.0)
        # 0.30 * 600 / 100 = 1.8m
        self.assertAlmostEqual(depth, 1.8, places=2)
        # mm_per_norm_eff = 30 * 1.8/0.30 = 180; dx_mm = -0.1 * 180 = -18
        mm_per_norm_eff = 30.0 * (depth / 0.30)
        dx_mm = -0.1 * mm_per_norm_eff
        self.assertAlmostEqual(dx_mm, -18.0, places=1)

    def test_pid_kd_dampens(self):
        """kd>0: D 项阻尼, 第二步位移不会比第一步大很多."""
        det = _det_dict(dx=0.1, dy=0.0, label="cylinder_1", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("cylinder_1")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=1.0, ki=0.0, kd=0.5,
            settle_tol_norm=0.05,
            settle_stable_frames=99,
            target_real_height_m=0.30,
            timeout=0.5, max_iter=3,
        )
        self.assertGreaterEqual(len(result.trace), 2)
        # 第二步 x_mm 不应继续 -18.0 走太远 (D 阻尼)
        self.assertLessEqual(abs(result.trace[1].x_mm), abs(result.trace[0].x_mm) + 5.0)

    def test_pid_ki_integration(self):
        """ki>0: PID 路径在 dx=0.10 / min_step_mm=0.001 时第一帧 dx_mm 应非 0.

        验证 ki 项确实参与计算, 不被纯 kp=0 短路成 0.
        """
        det = _det_dict(dx=0.10, dy=0.0, label="cylinder_1", height=100)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        sel = TargetSelector.for_label("cylinder_1")
        result = client.find_target_pid(
            sel, x_mm=0.0, y_mm=-100.0,
            kp=0.0, ki=1.0, kd=0.0,
            settle_tol_norm=0.02,
            settle_stable_frames=99,
            target_real_height_m=None,
            mm_per_norm_base=30.0,
            min_step_mm=0.001,  # 死区非常小, 让 ki 累积生效
            timeout=0.5, max_iter=2,
        )
        # 第一步 x_mm 应非 0 (PID 输出 → dx_mm 被推到死区外)
        self.assertGreater(len(result.trace), 0)
        self.assertNotAlmostEqual(result.trace[0].x_mm, 0.0)
