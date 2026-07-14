"""MC602 最小下载器。"""

from __future__ import annotations

import os
import time

from .bootloader import boot_ping, send_runcode, wait_program_ready
from .constants import (
    BOOTLOADER_RETRY_COUNT,
    CMD_BUFFER_4K_LEN,
    CMD_CRC,
    CMD_CRC_SUCCESS,
    CMD_DUMMY,
    CMD_M32_2_PC_HEAD0,
    CMD_M32_2_PC_HEAD1,
    CMD_PC_2_M32_HEAD0,
    CMD_PC_2_M32_HEAD1,
    CMD_RAM2FLASH,
    CMD_RAM2FLASH_REV_SIZE,
    CMD_RAM2FLASH_SUCCESS,
    CMD_RUNCODE,
    CMD_RUNCODE_REV_SIZE,
    CMD_RUNCODE_SUCCESS,
    CMD_SAVEFILENAME,
    CMD_SAVEFILENAME_REV_SIZE,
    CMD_SAVEFILENAME_SUCCESS,
    CMD_WRITEBUFFER,
    CMD_DW_REV_SIZE,
    CMD_PING,
    CMD_PING_REV_SIZE,
    DOWNLOAD_RETRY_COUNT,
    MC602_BAUDRATE,
    PROGRAM_GCC,
    RUN_SLOT_TO_ADDRESS,
)
from .serial_utils import checksum8, open_serial, read_exact, write_slow

_CRC32_TABLE = [0 for _ in range(256)]


def _init_crc32_table():
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ 0x04C11DB7) if (c & 0x80000000) else (c << 1)
        _CRC32_TABLE[i] = c & 0xFFFFFFFF


_init_crc32_table()


def crc32_stm(data: bytes) -> int:
    length = len(data)
    offset = 0
    crc = 0xFFFFFFFF
    while length >= 4:
        value = (
            ((data[offset] << 24) & 0xFF000000)
            | ((data[offset + 1] << 16) & 0x00FF0000)
            | ((data[offset + 2] << 8) & 0x0000FF00)
            | (data[offset + 3] & 0x000000FF)
        )
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ value)]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ (value >> 8))]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ (value >> 16))]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ (value >> 24))]
        offset += 4
        length -= 4
    if length > 0:
        value = 0
        for i in range(length):
            value |= data[offset + i] << (24 - i * 8)
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ value)]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ (value >> 8))]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ (value >> 16))]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC32_TABLE[0xFF & ((crc >> 24) ^ (value >> 24))]
    return crc


def _boot_frame(cmd: int, payload: bytes = b"", frame_len: int | None = None) -> bytes:
    if frame_len is None:
        frame_len = 7 + len(payload) + 1
    frame = bytearray(frame_len)
    frame[0] = CMD_PC_2_M32_HEAD0
    frame[1] = CMD_PC_2_M32_HEAD1
    frame[2] = 0
    frame[3] = cmd
    frame[4] = frame_len & 0xFF
    frame[5] = (frame_len >> 8) & 0xFF
    if payload:
        frame[6 : 6 + len(payload)] = payload
    else:
        frame[6] = CMD_DUMMY
    frame[-1] = checksum8(frame)
    return bytes(frame)


def _expect_boot_reply(serial_obj, size: int, cmd_ok: int) -> bytes:
    frame = read_exact(serial_obj, size, 0.3)
    if len(frame) != size:
        raise RuntimeError("bootloader 回包长度错误")
    if frame[0] != CMD_M32_2_PC_HEAD0 or frame[1] != CMD_M32_2_PC_HEAD1:
        raise RuntimeError("bootloader 回包头错误")
    if frame[3] != cmd_ok:
        raise RuntimeError(f"bootloader 回包命令错误: 0x{frame[3]:02x}")
    if frame[-1] != checksum8(frame):
        raise RuntimeError("bootloader 回包校验错误")
    return frame


def ping_control(port_name: str) -> dict:
    with open_serial(port_name, baudrate=MC602_BAUDRATE, timeout_s=0.2) as serial_obj:
        frame = _boot_frame(CMD_PING, bytes([CMD_DUMMY]), frame_len=8)
        write_slow(serial_obj, frame)
        reply = read_exact(serial_obj, CMD_PING_REV_SIZE, 0.2)
        if reply[0] != CMD_M32_2_PC_HEAD0 or reply[1] != CMD_M32_2_PC_HEAD1 or reply[3] != CMD_PING:
            raise RuntimeError("PING 回包格式错误")
        if reply[-1] != checksum8(reply):
            raise RuntimeError("PING 回包校验错误")
        control_type = reply[6] + (reply[7] << 8)
        return {"port": port_name, "control_type": control_type, "reply_hex": reply.hex()}


def load_bin_file(file_path: str) -> bytes:
    with open(file_path, "rb") as file_obj:
        return file_obj.read()


def _send_write_buffer(serial_obj, chunk: bytes) -> bytes:
    payload = bytes([CMD_DUMMY]) + chunk
    frame = _boot_frame(CMD_WRITEBUFFER, payload=payload, frame_len=len(chunk) + 8)
    serial_obj.write(frame)
    return _expect_boot_reply(serial_obj, CMD_DW_REV_SIZE, CMD_WRITEBUFFER)


def _send_ram_to_flash(serial_obj, flash_address: int) -> bytes:
    payload = flash_address.to_bytes(4, "little")
    frame = _boot_frame(CMD_RAM2FLASH, payload=payload, frame_len=11)
    serial_obj.write(frame)
    reply = _expect_boot_reply(serial_obj, CMD_RAM2FLASH_REV_SIZE, CMD_RAM2FLASH_SUCCESS)
    if reply[6:10] != payload:
        raise RuntimeError("RAM2FLASH 地址回显错误")
    return reply


def _send_crc(serial_obj, flash_address: int) -> bytes:
    payload = flash_address.to_bytes(4, "little")
    frame = _boot_frame(CMD_CRC, payload=payload, frame_len=11)
    serial_obj.write(frame)
    return _expect_boot_reply(serial_obj, CMD_RAM2FLASH_REV_SIZE, CMD_CRC_SUCCESS)


def _flash_crc_matches(serial_obj, flash_address: int, chunk_crc: int) -> bool:
    reply = _send_crc(serial_obj, flash_address)
    flash_crc = int.from_bytes(reply[6:10], "little")
    return flash_crc == chunk_crc


def save_run_name(port_name: str, run_name: str) -> dict:
    pure_name = run_name.split(".")[0][:8]
    name_bytes = pure_name.encode("utf-8")
    payload = name_bytes.ljust(8, b"\x00")
    with open_serial(port_name, baudrate=MC602_BAUDRATE, timeout_s=0.2) as serial_obj:
        frame = _boot_frame(CMD_SAVEFILENAME, payload=payload, frame_len=16)
        write_slow(serial_obj, frame)
        reply = _expect_boot_reply(serial_obj, CMD_SAVEFILENAME_REV_SIZE, CMD_SAVEFILENAME_SUCCESS)
    return {"ok": True, "reply_hex": reply.hex(), "run_name": pure_name}


def run_code(port_name: str, flash_address: int) -> dict:
    payload = flash_address.to_bytes(4, "little")
    last_reply = b""
    for _ in range(10):
        with open_serial(port_name, baudrate=MC602_BAUDRATE, timeout_s=0.2) as serial_obj:
            frame = _boot_frame(CMD_RUNCODE, payload=payload, frame_len=11)
            write_slow(serial_obj, frame)
            try:
                reply = _expect_boot_reply(serial_obj, CMD_RUNCODE_REV_SIZE, CMD_RUNCODE_SUCCESS)
                if reply[6:10] != payload:
                    raise RuntimeError("RUNCODE 地址回显错误")
                last_reply = reply
                return {"ok": True, "reply_hex": reply.hex()}
            except Exception as exc:
                last_reply = str(exc).encode("utf-8", errors="ignore")
        time.sleep(0.05)
    return {"ok": False, "reply_hex": last_reply.hex() if isinstance(last_reply, bytes) else str(last_reply)}


def ensure_bootloader(port_name: str) -> dict:
    last_detail = "bootloader 未响应"
    for _ in range(BOOTLOADER_RETRY_COUNT):
        with open_serial(port_name, baudrate=MC602_BAUDRATE, timeout_s=0.2) as serial_obj:
            ok, frame = boot_ping(serial_obj)
            if ok:
                return {"ok": True, "stage": "boot_ping", "reply_hex": frame.hex()}
            ok, frame = send_runcode(serial_obj)
            if ok and wait_program_ready(port_name, timeout_s=0.5):
                last_detail = "控制器当前在 program 模式，不处于 bootloader"
            else:
                last_detail = f"bootloader 探测失败: {frame.hex()}"
        time.sleep(0.05)
    return {"ok": False, "stage": "boot_ping", "detail": last_detail}


def download_file(port_name: str, file_path: str, slot: str = "RunA", run_after_download: bool = False, write_name: bool = True) -> dict:
    if slot not in RUN_SLOT_TO_ADDRESS:
        raise ValueError(f"未知槽位: {slot}")
    target_address = PROGRAM_GCC if run_after_download else RUN_SLOT_TO_ADDRESS[slot]
    code = load_bin_file(file_path)
    result = {
        "ok": False,
        "port": port_name,
        "file": os.path.abspath(file_path),
        "slot": slot,
        "target_address": hex(target_address),
        "size": len(code),
        "chunks": [],
    }
    ping_control(port_name)
    with open_serial(port_name, baudrate=MC602_BAUDRATE, timeout_s=0.2) as serial_obj:
        offset = 0
        while offset <= len(code):
            chunk = bytearray(CMD_BUFFER_4K_LEN)
            source = code[offset : offset + CMD_BUFFER_4K_LEN]
            chunk[: len(source)] = source
            if len(source) < CMD_BUFFER_4K_LEN:
                chunk[len(source) :] = b"\xFF" * (CMD_BUFFER_4K_LEN - len(source))
            flash_address = target_address + offset
            chunk_crc = crc32_stm(chunk)
            chunk_result = {
                "offset": offset,
                "flash_address": hex(flash_address),
                "skipped_by_crc": False,
            }
            if not _flash_crc_matches(serial_obj, flash_address, chunk_crc):
                ok = False
                for _ in range(DOWNLOAD_RETRY_COUNT):
                    _send_write_buffer(serial_obj, bytes(chunk))
                    _send_ram_to_flash(serial_obj, flash_address)
                    if _flash_crc_matches(serial_obj, flash_address, chunk_crc):
                        ok = True
                        break
                if not ok:
                    raise RuntimeError(f"分块下载失败: offset={offset}")
            else:
                chunk_result["skipped_by_crc"] = True
            result["chunks"].append(chunk_result)
            offset += CMD_BUFFER_4K_LEN
            if offset > len(code):
                break
    if write_name and not run_after_download:
        save_run_name(port_name, slot)
    if run_after_download:
        run_result = run_code(port_name, PROGRAM_GCC)
        if not run_result["ok"]:
            raise RuntimeError("下载成功，但运行 GCC 程序失败")
    result["ok"] = True
    return result

