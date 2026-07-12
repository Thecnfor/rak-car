#!/usr/bin/python3
# -*- coding: utf-8 -*-
import importlib.util
import threading
from pathlib import Path


_DOWNLOAD_LOCK = threading.Lock()
_DOWNLOAD_MODULE = None


def _get_download_module():
    global _DOWNLOAD_MODULE
    if _DOWNLOAD_MODULE is not None:
        return _DOWNLOAD_MODULE
    module_path = (
        Path(__file__).resolve().parents[2]
        / "smartcar"
        / "whalesbot"
        / "vehicle"
        / "base"
        / "pydownload.py"
    )
    spec = importlib.util.spec_from_file_location(
        "rak_car_controller_pydownload",
        str(module_path),
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("无法加载 pydownload.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _DOWNLOAD_MODULE = module
    return module


def download_mc602_program(run_name="RunA", isrun=True):
    with _DOWNLOAD_LOCK:
        module = _get_download_module()
        return module.Scratch_Download_MC602P(run_name, isrun=isrun)
