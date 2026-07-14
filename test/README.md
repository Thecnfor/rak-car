# 独立下位机通信测试

本目录提供一套完全独立于现有业务代码的下位机通信与操作工具，目标是研究 MC602 的串口协议、bootloader 拉起、下载/烧录与最小动作控制。

## 安装

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pip install -r /home/jetson/workspace/rak-car/test/requirements.txt
```

## 最常用命令

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py ports
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py probe
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py raw "02 01 10"
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py boot-ping --port /dev/ttyUSB0
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py recover --port /dev/ttyUSB0
```

## 风险命令

以下命令会触发真实硬件动作，必须显式加 `--dangerous`：

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py beep --dangerous
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py chassis --lf 5 --rf 5 --lr 5 --rr 5 --dangerous
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py chassis-stop --dangerous
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py servo pwm --device-port 1 --angle 90 --dangerous
```

下载/烧录除了 `--dangerous`，还必须显式加 `--yes-download`：

```bash
python3 /home/jetson/workspace/rak-car/test/run_controller_lab.py download \
  --port /dev/ttyUSB0 \
  --file /absolute/path/to/Run.bin \
  --slot RunA \
  --dangerous \
  --yes-download
```

## 目录说明

- `controller_lab/constants.py`：协议常量、地址表、设备字典
- `controller_lab/serial_utils.py`：串口、校验、十六进制工具
- `controller_lab/protocol.py`：MC602 program 帧封装与收包
- `controller_lab/probe.py`：候选串口探测
- `controller_lab/bootloader.py`：bootloader 探测与 `RUNCODE`
- `controller_lab/downloader.py`：独立下载/烧录最小实现
- `controller_lab/devices.py`：通用设备命令层
- `controller_lab/actions.py`：最小动作封装
- `controller_lab/cli.py`：统一 CLI

更多协议细节见 `PROTOCOL_NOTES.md`，推荐操作顺序见 `OPERATION_GUIDE.md`。

