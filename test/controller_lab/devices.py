"""独立设备命令层。"""

from __future__ import annotations

import struct

from .constants import DEVICE_DEFS
from .protocol import exchange_program_payload


class StructData:
    def __init__(self, fmt: str = "") -> None:
        self.format = "<b" + fmt
        self.size = struct.calcsize(self.format)
        self.len = len(self.format) - 1

    def pack(self, values):
        return struct.pack(self.format, *values)

    def unpack(self, raw: bytes, offset: int):
        return list(struct.unpack(self.format, raw[offset : offset + self.size]))

    def __len__(self) -> int:
        return self.len


class DeviceCommand:
    def __init__(self, device_name: str, port_id: int | None = None):
        if device_name not in DEVICE_DEFS:
            raise KeyError(f"未知设备类型: {device_name}")
        definition = DEVICE_DEFS[device_name]
        self.device_name = device_name
        self.dev_id = definition["dev_id"]
        self.mode = definition.get("mode")
        self.port_id = port_id
        self.data_struct = StructData(definition["format"])
        self.arg_reg = 1

    def build_payload(self, *args, mode: int | None = None, port_id: int | None = None) -> bytes:
        values = [self.dev_id]
        self.arg_reg = 3
        final_mode = self.mode if mode is None else mode
        if final_mode is not None:
            values.append(final_mode)
        else:
            self.arg_reg -= 1
            values.append(0)
        final_port = self.port_id if port_id is None else port_id
        if final_port is not None:
            values.append(final_port)
        else:
            self.arg_reg -= 1
        arg_values = list(args)
        expected_arg_count = len(self.data_struct) - len(values)
        while len(arg_values) > expected_arg_count:
            arg_values.pop(0)
        while len(arg_values) < expected_arg_count:
            arg_values.append(0)
        return self.data_struct.pack(values + arg_values)

    def parse_result(self, payload: bytes, offset: int = 0):
        unpacked = self.data_struct.unpack(payload, offset)[self.arg_reg :]
        if len(unpacked) == 1:
            return unpacked[0]
        return unpacked

    def exchange(self, serial_obj, *args, mode: int | None = None, port_id: int | None = None):
        payload = self.build_payload(*args, mode=mode, port_id=port_id)
        reply = exchange_program_payload(serial_obj, payload)
        return self.parse_result(reply)

    def set(self, serial_obj, *args, port_id: int | None = None):
        return self.exchange(serial_obj, *args, mode=2, port_id=port_id)

    def get(self, serial_obj, *args, port_id: int | None = None):
        return self.exchange(serial_obj, *args, mode=1, port_id=port_id)

    def reset(self, serial_obj, *args, port_id: int | None = None):
        return self.exchange(serial_obj, *args, mode=3, port_id=port_id)

    def act_mode(self, serial_obj, *args, mode: int | None = None, port_id: int | None = None):
        return self.exchange(serial_obj, *args, mode=mode, port_id=port_id)


class BatchDeviceCommand:
    def __init__(self, device_commands):
        self.device_commands = device_commands

    def get_all(self, serial_obj, args_list, mode: int = 1):
        payload = b"".join(
            device.build_payload(args_list[index], mode=mode)
            for index, device in enumerate(self.device_commands)
        )
        reply = exchange_program_payload(serial_obj, payload)
        results = []
        offset = 0
        for device in self.device_commands:
            results.append(device.parse_result(reply, offset))
            offset += device.data_struct.size
        return results

