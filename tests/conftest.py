"""Shared pytest fixtures for vehicle_wbt main tests.

Works around the 'import-time side effect' from CLAUDE.md:
importing log_info triggers RotatingFileHandler to open
log_info/base/logs/<date>-all.log. We pre-create the directory
so the import doesn't fail.
"""
import os
import sys
from pathlib import Path

import pytest

# Repo root = parent of tests/
_REPO_ROOT = Path(__file__).resolve().parent.parent
_LOG_DIR = _REPO_ROOT / "log_info" / "base" / "logs"
_LOG_DIR.mkdir(parents=True, exist_ok=True)

# Make 'import task_func' work without sys.path hacks
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


@pytest.fixture(scope="session", autouse=True)
def ensure_log_dir():
    """Recreate log dir before each test session (in case it was wiped)."""
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    yield
