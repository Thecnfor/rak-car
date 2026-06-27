# 摄像头系统

## Camera 类

`camera/base/camera.py`

### 初始化

```python
Camera(index, width=640, height=480)
```

- **Linux：** 打开 `/dev/cam{index}`（例如 `/dev/cam1`）
- **Windows：** 打开 `cv2.VideoCapture(index, cv2.CAP_DSHOW)`

### 线程模型

```
主线程                          后台守护线程
   │                               │
   │ camera.read()                 │ while not stop_flag:
   │ ──────→ 返回 self.frame       │     ret, frame = cap.read()
   │                               │     self.frame = frame
   │                               │     time.sleep(0.01)
```

- 后台线程持续抓帧，存入 `self.frame`
- `read()` 返回最新帧，不阻塞
- 无线程锁（依赖 GIL 保证引用原子性）

### 错误恢复

```python
def update(self):  # 后台线程
    while not self.stop_flag:
        try:
            ret, frame = self.cap.read()
            self.frame = frame
        except Exception as e:
            self.cap.release()
            self.init()  # 无限重试打开摄像头
            self.set_size()
```

**⚠️ `init()` 中重试是无限循环，没有超时退出。**

### 设备约定

| 设备 | 配置项 | 默认值 | 用途 |
|------|--------|:------:|------|
| 前方摄像头 | `config_car.yml → camera.front` | 2 | 车道线跟随、前方检测 |
| 侧方摄像头 | `config_car.yml → camera.side` | 1 | 目标检测、OCR |

### 分辨率

默认 640×480，可通过 `set_size(width, height)` 修改。

### 使用示例

```python
from camera import Camera

cap_front = Camera(2, 640, 480)  # 前方摄像头
cap_side = Camera(1, 640, 480)   # 侧方摄像头

frame = cap_front.read()          # 获取最新帧
cap_front.close()                 # 关闭
```
