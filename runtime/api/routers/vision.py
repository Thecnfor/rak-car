#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""视觉端点：/v1/vision/*。

lane / task / ocr 推理都走 `_execute_sync`（同步 job），overlay 预览
直接读 streamer 帧缓存做 cv2 画字/画框。
"""
try:
    from fastapi import APIRouter, Body, HTTPException, Query
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

try:
    from fastapi.responses import Response
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

try:
    import cv2  # 仅 /v1/vision/*/preview.jpg 用，单独 try 避免污染启动路径
    _HAS_CV2 = True
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    _HAS_CV2 = False

from runtime.core import settings

from ._helpers import _execute_sync, _format_detection, _frame_shape, _vision_models_payload


def build_vision_router(service, camera_stream_service):
    router_v1 = APIRouter(prefix=settings.get_api_v1_prefix(), tags=["runtime"])

    @router_v1.get("/vision/models")
    def v1_vision_models():
        return {"ok": True, **_vision_models_payload()}

    @router_v1.post("/vision/lane")
    def v1_vision_lane(payload: dict = Body(default={})):
        timeout = payload.get("timeout", 20)
        result = _execute_sync(
            service,
            target="car",
            name="get_lane_results",
            timeout=timeout,
        )
        return {
            "ok": True,
            "model": "lane",
            "camera": "cam1",
            "frame_url": "/stream/frame/cam1.jpg",
            "preview_url": "/stream/",
            "state_url": "/v1/vision/lane/state",
            "result": {
                "error": result[0],
                "angle": result[1],
            },
            "frame_shape": _frame_shape(camera_stream_service, "cam1"),
        }

    @router_v1.get("/vision/lane/state")
    def v1_vision_lane_state():
        return {"ok": True, **camera_stream_service.get_lane_state()}

    # lane_feed 守护线程：runtime init 默认自动启动（详见 runtime_service._create_car_locked），
    # 这里不暴露 start/stop 端点 —— 比赛阶段 lane_state 必须持续更新。

    @router_v1.get("/vision/lane/preview.jpg")
    def v1_vision_lane_preview_jpg(cam_id: str = Query(default="cam1")):
        """cam1 帧 + 车道误差 overlay 一次性 JPEG。

        设计目的：lane_feed 守护线程已经不再 cv2.putText 主帧流，
        但调试时仍然想看到 "d_e:... d_a:..." 字样。这个端点：
          1) 读 streamer.frames[cam_id] 缓存（不读摄像头，不抢 _capture_loop）
          2) 读 lane_state 拿 (error_y, error_angle)
          3) 调 cv2.putText × 2 一次性画字（白底+绿字，和之前调试期一致）
          4) imencode JPEG 返回
        端点不抢摄像头、不污染 cam1 主帧流，前端 <img> 默认走
        /video_feed/cam1 拿干净流；想看叠加就切到本端点。
        """
        if not _HAS_CV2:
            raise HTTPException(status_code=503, detail="cv2 不可用，无法生成 overlay")
        normalized = camera_stream_service.normalize_cam_id(cam_id)
        frame = camera_stream_service.get_frame(normalized)
        if frame is None:
            raise HTTPException(status_code=409, detail="摄像头当前没有可用的帧")
        # 拷贝再画字，避免污染缓存
        try:
            drawn = frame.copy()
        except Exception:
            drawn = frame
        state = camera_stream_service.get_lane_state() or {}
        ey = state.get("error_y")
        ea = state.get("error_angle")
        if ey is not None and ea is not None:
            try:
                label = f"d_e:{float(ey):7.5f}  d_a:{float(ea):7.5f}"
                # 与 get_lane_results 旧实现一致：白底厚 + 绿字薄（双重 putText 防锯齿）
                cv2.putText(drawn, label, (20, 40),
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
                cv2.putText(drawn, label, (20, 40),
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1, cv2.LINE_AA)
            except cv2.error:
                # cv2 putText 偶发在 copy 后的非连续数组上失败，忽略 overlay 即可
                drawn = frame
        try:
            ret, buf = cv2.imencode(
                ".jpg", drawn,
                [int(cv2.IMWRITE_JPEG_QUALITY), 80],
            )
        except cv2.error:
            raise HTTPException(status_code=500, detail="overlay JPEG 编码失败")
        if not ret:
            raise HTTPException(status_code=500, detail="overlay JPEG 编码失败")
        return Response(content=buf.tobytes(), media_type="image/jpeg")

    @router_v1.get("/vision/task/preview.jpg")
    def v1_vision_task_preview_jpg(cam_id: str = Query(default="cam2")):
        """cam2 帧 + 侧摄目标检测 bbox overlay 一次性 JPEG。

        设计目的：task_feed 守护线程在后台持续刷新 streamer.task_state，
        但默认 /video_feed/cam2 只走干净流。调试 / 比赛时希望叠加 bbox，
        这个端点：
          1) 读 streamer.frames[cam_id] 缓存（不读摄像头，不抢 _capture_loop）
          2) 读 task_state 取最新 detections（bbox_norm = 中心点+宽高，归一化 0~1）
          3) 把每个检测画矩形框 + 类别/置信度文字
          4) imencode JPEG 返回
        端点不抢摄像头、不污染 cam2 主帧流，前端 <img> 默认走
        /video_feed/cam2 拿干净流；想看叠加就切到本端点。
        """
        if not _HAS_CV2:
            raise HTTPException(status_code=503, detail="cv2 不可用，无法生成 overlay")
        normalized = camera_stream_service.normalize_cam_id(cam_id)
        frame = camera_stream_service.get_frame(normalized)
        if frame is None:
            raise HTTPException(status_code=409, detail="摄像头当前没有可用的帧")
        try:
            drawn = frame.copy()
        except Exception:
            drawn = frame
        state = camera_stream_service.get_task_state() or {}
        detections = state.get("detections") or []
        if detections:
            try:
                h, w = drawn.shape[:2]
                for det in detections:
                    bbox = det.get("bbox_norm") or {}
                    # backend 通过 YoloeInfer.predict(..., normalize_out=True)
                    # + DetectResult.tolist_nomoralize 返回相对图中心的归一化坐标:
                    #   x_c, y_c ∈ [-1, +1], width, height ∈ [0, 2]
                    # 注意归一化基准是 backend resize 后的 416x416 输入,与 cam2
                    # MJPEG 帧分辨率不同;此处 img_w/img_h 用当前帧尺寸,等比
                    # resize 下中心点位置仍正确,只有 bbox 宽高会有拉压。
                    x_c = float(bbox.get("x_center", 0.0))
                    y_c = float(bbox.get("y_center", 0.0))
                    box_w = float(bbox.get("width", 0.0))
                    box_h = float(bbox.get("height", 0.0))
                    center_x = (x_c + 1.0) / 2.0 * w
                    center_y = (y_c + 1.0) / 2.0 * h
                    bw = box_w * w / 2.0
                    bh = box_h * h / 2.0
                    x1 = int(round(center_x - bw / 2.0))
                    y1 = int(round(center_y - bh / 2.0))
                    x2 = int(round(center_x + bw / 2.0))
                    y2 = int(round(center_y + bh / 2.0))
                    x1 = max(x1, 0)
                    y1 = max(y1, 0)
                    x2 = min(x2, w - 1)
                    y2 = min(y2, h - 1)
                    if x2 <= x1 or y2 <= y1:
                        continue
                    cls_id = det.get("cls_id")
                    # 按 cls_id 散到固定调色板，避免重复类全黑/全绿
                    palette = [
                        (0, 255, 0),    # 绿
                        (255, 200, 0),  # 橙
                        (255, 80, 200), # 粉
                        (0, 200, 255),  # 青
                        (180, 130, 255),# 紫
                        (255, 255, 0),  # 黄
                    ]
                    try:
                        color = palette[int(cls_id) % len(palette)] if cls_id is not None else (0, 255, 0)
                    except Exception:
                        color = (0, 255, 0)
                    cv2.rectangle(drawn, (x1, y1), (x2, y2), color, 2)
                    label = str(det.get("label", ""))
                    score = float(det.get("score", 0.0))
                    text = "{} {:.2f}".format(label, score) if label else "{:.2f}".format(score)
                    # 文字画在框上方；放不下就改画框内左上
                    ty = y1 - 6
                    if ty < 14:
                        ty = y1 + 14
                    cv2.putText(drawn, text, (x1, ty),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(drawn, text, (x1, ty),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                # 顶部画个 count 角标，方便一眼看出识别数量
                count = len(detections)
                banner = "task detections: {}".format(count)
                cv2.putText(drawn, banner, (12, 22),
                            cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255, 255, 255), 3, cv2.LINE_AA)
                cv2.putText(drawn, banner, (12, 22),
                            cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
            except cv2.error:
                # 偶发在 copy 后非连续数组上 putText 失败，回退到干净帧
                drawn = frame
        try:
            ret, buf = cv2.imencode(
                ".jpg", drawn,
                [int(cv2.IMWRITE_JPEG_QUALITY), 80],
            )
        except cv2.error:
            raise HTTPException(status_code=500, detail="overlay JPEG 编码失败")
        if not ret:
            raise HTTPException(status_code=500, detail="overlay JPEG 编码失败")
        return Response(content=buf.tobytes(), media_type="image/jpeg")

    @router_v1.post("/vision/task")
    def v1_vision_task(payload: dict = Body(default={})):
        timeout = payload.get("timeout", 20)
        sort_pos = payload.get("sort_pos", [0, 0])
        limit_x = payload.get("limit_x", 1)
        limit_y = payload.get("limit_y", 1)
        result = _execute_sync(
            service,
            target="car",
            name="get_detection_results",
            kwargs={
                "sort_pos": sort_pos,
                "limit_x": limit_x,
                "limit_y": limit_y,
            },
            timeout=timeout,
        )
        image_shape = _frame_shape(camera_stream_service, "cam2")
        detections = [
            _format_detection(det, index=index, image_shape=image_shape)
            for index, det in enumerate(result)
        ]
        return {
            "ok": True,
            "model": "task",
            "camera": "cam2",
            "frame_url": "/stream/frame/cam2.jpg",
            "preview_url": "/stream/",
            "filters": {
                "sort_pos": sort_pos,
                "limit_x": limit_x,
                "limit_y": limit_y,
            },
            "count": len(detections),
            "detections": detections,
            "frame_shape": image_shape,
        }

    @router_v1.post("/vision/ocr")
    def v1_vision_ocr(payload: dict = Body(default={})):
        timeout = payload.get("timeout", 20)
        label = payload.get("label")
        sort_pos = payload.get("sort_pos", [0, 0])
        limit_x = payload.get("limit_x", 1)
        limit_y = payload.get("limit_y", 1)
        detections = _execute_sync(
            service,
            target="car",
            name="get_detection_results",
            kwargs={
                "sort_pos": sort_pos,
                "limit_x": limit_x,
                "limit_y": limit_y,
            },
            timeout=timeout,
        )
        image_shape = _frame_shape(camera_stream_service, "cam2")
        formatted_detections = [
            _format_detection(det, index=index, image_shape=image_shape)
            for index, det in enumerate(detections)
        ]
        matched_index = None
        matched_det = None
        for index, det in enumerate(detections):
            det_label = det[2]
            if label is None and det_label in {"order", "name"}:
                matched_index = index
                matched_det = det
                break
            if label is not None and det_label == label:
                matched_index = index
                matched_det = det
                break
        if matched_det is None:
            return {
                "ok": True,
                "model": "ocr",
                "camera": "cam2",
                "frame_url": "/stream/frame/cam1.jpg",
                "preview_url": "/stream/",
                "label": label,
                "text": None,
                "matched_detection": None,
                "detections": formatted_detections,
                "message": "当前画面未找到匹配的 OCR 检测框",
            }
        text = _execute_sync(
            service,
            target="car",
            name="get_det_ocr",
            args=[matched_det],
            kwargs={"label": label, "time_out": timeout},
            timeout=timeout,
        )
        return {
            "ok": True,
            "model": "ocr",
            "camera": "cam2",
            "frame_url": "/stream/frame/cam1.jpg",
            "source_frame_url": "/stream/frame/cam2.jpg",
            "preview_url": "/stream/",
            "label": label,
            "text": text,
            "matched_detection": formatted_detections[matched_index],
            "detections": formatted_detections,
        }

    return router_v1
