# 操作手册

## 推荐顺序

### 1. 先看串口

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py ports
```

### 2. 再探测 program 模式

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py probe
```

### 3. 如果 probe 失败，再测 bootloader

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py boot-ping --port /dev/ttyUSB0
```

### 4. 如果 bootloader 在场，尝试拉起 program

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py recover --port /dev/ttyUSB0
```

### 5. 验证最小原始协议

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py raw "02 01 10" --port /dev/ttyUSB0
```

### 6. 单设备只读验证

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py sensor infrared --device-port 1 --port /dev/ttyUSB0
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py sensor board-key --port /dev/ttyUSB0
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py sensor bluetooth --port /dev/ttyUSB0
```

### 7. 真实动作验证

先蜂鸣，再低速底盘，最后急停：

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py beep --dangerous --port /dev/ttyUSB0
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py chassis --lf 5 --rf 5 --lr 5 --rr 5 --dangerous --port /dev/ttyUSB0
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py chassis-stop --dangerous --port /dev/ttyUSB0
```

舵机和步进也建议从最小值开始：

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py servo pwm --device-port 1 --angle 90 --speed 30 --dangerous --port /dev/ttyUSB0
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py stepper --device-port 1 --velocity 20 --position 50 --dangerous --port /dev/ttyUSB0
```

### 8. 下载/烧录

先确认已经进入 bootloader 或至少能正常 ping：

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py ping-control --port /dev/ttyUSB0
```

然后再下载：

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py download \
  --port /dev/ttyUSB0 \
  --file /absolute/path/to/Run.bin \
  --slot RunA \
  --dangerous \
  --yes-download
```

## 常见现象

- `未找到候选串口`
  - 检查 USB 线、供电、`/dev/ttyUSB*`
- `bootloader 未响应`
  - 说明不在 bootloader，也可能端口不对
- `控制器已在 program 模式`
  - 不需要先走下载链
- `分块下载失败`
  - 重点看该 offset 的 CRC、写缓存与 RAM2FLASH 阶段
- `该命令会触发真实硬件动作`
  - 这是安全保护，需要显式传 `--dangerous`

