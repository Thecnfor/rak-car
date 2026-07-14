# 协议笔记

## 1. Program 模式

MC602 的业务协议帧格式：

```text
77 68 <len> <payload...> 0A
```

- `77 68`：帧头
- `<len>`：整帧长度，等于 `payload_len + 4`
- `payload`：实际业务数据
- `0A`：帧尾

program 握手 payload：

```text
02 01 10
```

完整握手帧：

```text
77 68 07 02 01 10 0A
```

## 2. Bootloader 模式

bootloader 探测帧：

```text
55 AA 00 01 08 00 00 F7
```

`RUNCODE` 拉起帧：

```text
55 AA 00 40 0B 00 00 D0 00 08 DD
```

bootloader 类协议结构：

```text
55 AA <index> <cmd> <len_lo> <len_hi> <payload...> <checksum>
```

回包头为：

```text
66 BB
```

校验规则：

- 对除最后一个字节外的所有字节求和
- 取低 8 位
- 按位取反

## 3. 下载流程

最小下载闭环：

1. `PING`
2. 读取 bin
3. 按 4K 分块
4. 对目标 flash 地址做 CRC 查询
5. 若 CRC 不同，则：
   - `WRITEBUFFER`
   - `RAM2FLASH`
6. 全部分块完成后：
   - 可选 `SAVEFILENAME`
   - 可选 `RUNCODE`

## 4. 设备命令模型

设备命令统一抽象为：

```text
dev_id + mode + port + args
```

常见 `mode`：

- `1`：get
- `2`：set
- `3`：reset

## 5. 已内置设备字典

- `motor4`
- `motor`
- `encoder4`
- `encoder`
- `servo_pwm`
- `servo_bus`
- `sensor_analog`
- `sensor_infrared`
- `bluetooth`
- `beep`
- `board_key`
- `dout`
- `stepper`

