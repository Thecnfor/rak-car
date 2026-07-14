"""MC602 独立测试常量定义。"""

from __future__ import annotations

FLASH_BASE = 0x08000000
PROGRAM_GCC = FLASH_BASE + (52 * 1024)
MC602_APP1_ADDRESS = FLASH_BASE + (384 * 1024)
MC602_APP2_ADDRESS = FLASH_BASE + (512 * 1024)
MC602_APP3_ADDRESS = FLASH_BASE + (612 * 1024)
MC602_APP4_ADDRESS = FLASH_BASE + (712 * 1024)
MC602_APP5_ADDRESS = FLASH_BASE + (812 * 1024)
MC602_APP6_ADDRESS = FLASH_BASE + (912 * 1024)

RUN_SLOT_TO_ADDRESS = {
    "RunA": MC602_APP1_ADDRESS,
    "RunB": MC602_APP2_ADDRESS,
    "RunC": MC602_APP3_ADDRESS,
    "RunD": MC602_APP4_ADDRESS,
    "RunE": MC602_APP5_ADDRESS,
    "RunF": MC602_APP6_ADDRESS,
    "debug": PROGRAM_GCC,
}

USB_VID = 6790
USB_PID = 29987
PORT_KEYWORDS = ("CH340", "USB")

MC602_BAUDRATE = 1_000_000
MC601_BAUDRATE = 380_400
SERIAL_TIMEOUT_S = 0.03
WRITE_TIMEOUT_S = 0.2

PROGRAM_FRAME_HEAD = bytes.fromhex("77 68")
PROGRAM_FRAME_TAIL = bytes.fromhex("0A")
PROGRAM_PING_PAYLOAD = bytes.fromhex("02 01 10")
MC601_PING_FRAME = bytes.fromhex("77 68 04 00 01 CA 01 0A")

BOOT_PING = bytes.fromhex("55 AA 00 01 08 00 00 F7")
RUN_CODE = bytes.fromhex("55 AA 00 40 0B 00 00 D0 00 08 DD")

CMD_PC_2_M32_HEAD0 = 0x55
CMD_PC_2_M32_HEAD1 = 0xAA
CMD_M32_2_PC_HEAD0 = 0x66
CMD_M32_2_PC_HEAD1 = 0xBB

CMD_PING = 0x01
CMD_SERVO = 0x02
CMD_DUMMY = 0x00
CMD_WRITEBUFFER = 0x10
CMD_RAM2FLASH = 0x20
CMD_RAM2FLASH_SUCCESS = 0x21
CMD_SAVEFILENAME = 0x30
CMD_SAVEFILENAME_SUCCESS = 0x31
CMD_RUNCODE = 0x40
CMD_RUNCODE_SUCCESS = 0x41
CMD_CRC = 0x50
CMD_CRC_SUCCESS = 0x51

CMD_PING_SEND_SIZE = 8
CMD_PING_REV_SIZE = 10
CMD_BUFFER_4K_LEN = 4 * 1024
CMD_DW_SEND_SIZE = CMD_BUFFER_4K_LEN + 8
CMD_DW_REV_SIZE = 8
CMD_RAM2FLASH_SEND_SIZE = 11
CMD_RAM2FLASH_REV_SIZE = 11
CMD_SAVEFILENAME_SEND_SIZE = 16
CMD_SAVEFILENAME_REV_SIZE = 16
CMD_RUNCODE_SEND_SIZE = 11
CMD_RUNCODE_REV_SIZE = 11

PORT_STABLE_FOR_S = 0.8
PORT_STABLE_TIMEOUT_S = 3.5
RECOVERY_WINDOW_S = 4.5
RUNCODE_MAX_ATTEMPTS = 8
RUNCODE_ACK_TIMEOUT_S = 0.18
PROGRAM_PING_AFTER_RUNCODE_S = 0.8
RECOVERY_COOLDOWN_S = 0.18

BOOTLOADER_RETRY_COUNT = 5
DOWNLOAD_RETRY_COUNT = 5

DEVICE_DEFS = {
    "motor4": {"dev_id": 0x01, "format": "bbbbb"},
    "motor": {"dev_id": 0x02, "format": "bbb"},
    "encoder4": {"dev_id": 0x03, "format": "biiii"},
    "encoder": {"dev_id": 0x04, "format": "bbi"},
    "servo_pwm": {"dev_id": 0x05, "format": "bbBB"},
    "servo_bus": {"dev_id": 0x06, "format": "bbbbh"},
    "sensor_analog": {"dev_id": 0x07, "mode": 0, "format": "bbH"},
    "sensor_infrared": {"dev_id": 0x07, "mode": 1, "format": "bbH"},
    "sensor_touch": {"dev_id": 0x07, "mode": 2, "format": "bbH"},
    "sensor_ultrasonic": {"dev_id": 0x07, "mode": 3, "format": "bbH"},
    "sensor_ambient_light": {"dev_id": 0x07, "mode": 4, "format": "bbH"},
    "sensor_analog_a": {"dev_id": 0x08, "mode": 0, "format": "bbH"},
    "bluetooth": {"dev_id": 0x09, "format": "BBBBi"},
    "beep": {"dev_id": 0x0A, "format": "BBB"},
    "led_show": {"dev_id": 0x0B, "format": "b" * 101},
    "power": {"dev_id": 0x0C, "format": "bi"},
    "board_key": {"dev_id": 0x0D, "format": "bbb"},
    "led_light": {"dev_id": 0x0E, "format": "bbBBBB"},
    "nixietube": {"dev_id": 0x0F, "format": "bbi"},
    "dout": {"dev_id": 0x10, "format": "bbb"},
    "stepper": {"dev_id": 0x11, "format": "bbii"},
}

