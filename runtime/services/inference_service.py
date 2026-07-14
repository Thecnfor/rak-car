#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml
import zmq

from runtime.core import settings


class InferBackendService:
    def __init__(self):
        self.project_root = Path(__file__).resolve().parents[2]
        self.script_path = Path(settings.get_infer_backend_script())
        self.state_lock = threading.Lock()
        self._process = None
        self._supervisor = None
        self._supervisor_started = False
        self._configs = None
        self._status = "stopped"
        self._last_error = None
        self._last_start_at = None
        self._last_ready_at = None
        self._last_probe_at = None
        self._last_probe_ok_at = None
        self._last_probe = None

    def _load_configs(self):
        if self._configs is not None:
            return self._configs
        config_path = self.project_root / "config_car.yml"
        with config_path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
        self._configs = list(config.get("infer_cfg") or [])
        return self._configs

    def start_background(self):
        if self._supervisor_started:
            return
        self._supervisor_started = True
        self._supervisor = threading.Thread(target=self._supervisor_loop, daemon=True)
        self._supervisor.start()

    def _supervisor_loop(self):
        while True:
            try:
                if settings.get_infer_auto_start():
                    self.ensure_started()
                    self.probe()
            except Exception as exc:  # pragma: no cover
                with self.state_lock:
                    self._last_error = str(exc)
            time.sleep(settings.get_infer_poll_interval())

    def _set_state(self, status=None, last_error=None, last_probe=None):
        with self.state_lock:
            if status is not None:
                self._status = status
            if last_error is not None:
                self._last_error = last_error
            if last_probe is not None:
                self._last_probe = last_probe

    def _current_process(self):
        process = self._process
        if process is not None and process.poll() is None:
            return process
        self._process = None
        return None

    def _find_existing_pid(self):
        try:
            result = subprocess.run(
                ["pgrep", "-f", str(self.script_path)],
                check=False,
                capture_output=True,
                text=True,
            )
        except Exception:
            return None
        if result.returncode != 0:
            return None
        for line in (result.stdout or "").splitlines():
            line = line.strip()
            if not line:
                continue
            pid = int(line)
            if pid != os.getpid():
                return pid
        return None

    def _get_live_pid(self):
        process = self._current_process()
        if process is not None:
            return process.pid
        return self._find_existing_pid()

    def _spawn_process(self):
        env = os.environ.copy()
        env["RAK_CAR_INFER_MANAGED"] = "1"
        command = [sys.executable, str(self.script_path)]
        process = subprocess.Popen(
            command,
            cwd=str(self.project_root),
            env=env,
        )
        self._process = process
        now = time.time()
        with self.state_lock:
            self._status = "starting"
            self._last_start_at = now
            self._last_error = None
        return process

    def ensure_started(self):
        if self._get_live_pid() is not None:
            return self._process
        snapshot = self.probe()
        if snapshot.get("status") in {"ready", "starting"}:
            return self._process
        return self._spawn_process()

    def stop(self):
        process = self._current_process()
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
            finally:
                self._process = None
                with self.state_lock:
                    self._status = "stopped"
            return
        pid = self._find_existing_pid()
        if pid is None:
            return
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pid = None
        deadline = time.time() + 3
        while pid is not None and time.time() < deadline:
            if self._find_existing_pid() is None:
                pid = None
                break
            time.sleep(0.2)
        if pid is not None:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        with self.state_lock:
            self._status = "stopped"

    def _probe_port(self, port, timeout_s):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, int(timeout_s * 1000))
        socket.setsockopt(zmq.SNDTIMEO, int(timeout_s * 1000))
        socket.connect(f"tcp://127.0.0.1:{int(port)}")
        try:
            socket.send(b"ATATA")
            response = socket.recv()
            payload = json.loads(response)
            return {"ok": True, "payload": payload}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        finally:
            socket.close(0)
            context.term()

    def probe(self):
        configs = self._load_configs()
        timeout_s = settings.get_infer_health_timeout()
        now = time.time()
        models = []
        all_ready = True
        errors = []
        for conf in configs:
            result = self._probe_port(conf.get("port"), timeout_s)
            model_state = {
                "name": conf.get("name"),
                "port": conf.get("port"),
                "ready": result["ok"],
            }
            if result["ok"]:
                model_state["response"] = result["payload"]
            else:
                all_ready = False
                model_state["error"] = result["error"]
                errors.append(f"{conf.get('name')}:{result['error']}")
            models.append(model_state)

        pid = self._get_live_pid()
        status = "ready" if all_ready else ("starting" if pid is not None else "stopped")
        detail = None if all_ready else "; ".join(errors) if errors else "推理服务未就绪"
        snapshot = {
            "status": status,
            "managed": True,
            "process_running": pid is not None,
            "pid": pid,
            "models": models,
            "last_probe_at": now,
            "last_error": detail,
            "last_start_at": self._last_start_at,
            "last_ready_at": self._last_ready_at,
        }
        with self.state_lock:
            self._last_probe_at = now
            self._last_probe = snapshot
            self._status = status
            self._last_error = detail
            if all_ready:
                self._last_ready_at = now
                self._last_probe_ok_at = now
                snapshot["last_ready_at"] = now
            else:
                snapshot["last_probe_ok_at"] = self._last_probe_ok_at
        return snapshot

    def ensure_ready(self, timeout=None):
        timeout = (
            settings.get_infer_ready_timeout() if timeout is None else float(timeout)
        )
        deadline = time.time() + timeout
        self.ensure_started()
        last_snapshot = None
        while time.time() < deadline:
            last_snapshot = self.probe()
            if last_snapshot.get("status") == "ready":
                return last_snapshot
            time.sleep(settings.get_infer_poll_interval())
        detail = "推理服务未就绪"
        if last_snapshot is not None:
            detail = last_snapshot.get("last_error") or detail
        raise RuntimeError(detail)

    def get_state(self):
        pid = self._get_live_pid()
        with self.state_lock:
            snapshot = {
                "status": self._status,
                "managed": True,
                "process_running": pid is not None,
                "pid": pid,
                "last_error": self._last_error,
                "last_start_at": self._last_start_at,
                "last_ready_at": self._last_ready_at,
                "last_probe_at": self._last_probe_at,
                "last_probe_ok_at": self._last_probe_ok_at,
                "models": [],
            }
            if self._last_probe is not None:
                snapshot["models"] = list(self._last_probe.get("models") or [])
        return snapshot
