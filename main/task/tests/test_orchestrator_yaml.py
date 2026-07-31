"""main/task/tests/test_orchestrator_yaml.py

验证 task_config.yml 的 waypoints 段能被加载,且 8 个 task + 1 finish 全在。
"""
from pathlib import Path
import sys

# 路径: main/task/tests/ → repo_root
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# 走 importlib 直接加载 _config.py, 绕开 main/task/__init__.py
# (后者在 task3/4/5/7 stub 落位前会 import 失败, 但本测试只关心 yaml 段)
import importlib.util as _il  # noqa: E402
_cfg_spec = _il.spec_from_file_location(
    "_main_task_config_under_test",
    _REPO_ROOT / "main/task/_config.py",
)
_config = _il.module_from_spec(_cfg_spec)
_cfg_spec.loader.exec_module(_config)
load_waypoints = _config.load_waypoints  # noqa: E402


def test_load_waypoints_returns_list_of_dicts():
    wp = load_waypoints()
    assert isinstance(wp, list)
    assert len(wp) >= 8  # 7 tasks + 1 finish
    for w in wp:
        assert isinstance(w, dict)
        assert "name" in w
        assert "task_id" in w or w.get("is_finish")


def test_load_waypoints_covers_all_7_tasks():
    wp = load_waypoints()
    task_ids = {w.get("task_id") for w in wp if "task_id" in w}
    assert task_ids == {1, 2, 3, 4, 5, 6, 7}, f"missing tasks: {set(range(1,8)) - task_ids}"


def test_load_waypoints_includes_finish():
    wp = load_waypoints()
    finish = [w for w in wp if w.get("is_finish")]
    assert len(finish) == 1
    assert "dis_at_least_m" in finish[0]


def test_load_waypoints_threshold_fields():
    """每个 waypoint 至少有一个触发条件 (IR 或 odom)。"""
    wp = load_waypoints()
    for w in wp:
        if w.get("is_finish"):
            continue  # finish 只看 odom
        assert (w.get("ir_threshold_m") is not None or
                w.get("dis_at_least_m") is not None), \
            f"{w.get('name')} 缺触发条件"
