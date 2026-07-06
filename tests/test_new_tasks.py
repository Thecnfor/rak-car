"""Smoke tests for new 8.10 比赛 task functions.

We cannot `import task_func` because it triggers hardware scan
(serial port probing via controller_wrap.py:37). Instead we use
AST parsing to verify function signatures without executing imports.

CLAUDE.md 'Import-time side effects': import vehicle cannot run
without hardware connected. So this test uses ast.parse() to
verify the source structure.
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent
_TASK_FUNC = _REPO_ROOT / "task_func.py"


@pytest.fixture(scope="module")
def task_func_ast() -> ast.Module:
    """Parse task_func.py into AST (no imports executed)."""
    source = _TASK_FUNC.read_text(encoding="utf-8")
    return ast.parse(source)


def _find_function(ast_module: ast.Module, name: str) -> ast.FunctionDef:
    """Find a top-level function by name."""
    for node in ast_module.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"function {name!r} not found in {_TASK_FUNC}")


def _function_param_names(fn: ast.FunctionDef) -> list[str]:
    """Extract parameter names (skipping self/cls). Python 3.14 safe."""
    args = fn.args
    params = [a.arg for a in args.posonlyargs + args.args + args.kwonlyargs]
    if args.vararg is not None:
        params.append(args.vararg.arg)
    if args.kwarg is not None:
        params.append(args.kwarg.arg)
    return params


def _function_param_defaults(fn: ast.FunctionDef) -> dict[str, object]:
    """Map parameter name -> default value AST node (None if no default).

    Python 3.14 AST: defaults are stored in fn.args.defaults (positional
    list, aligned from right) and fn.args.kw_defaults (kwonly dict).
    """
    args = fn.args
    result = {}
    # Positional + posonly defaults (right-aligned)
    all_args = list(args.posonlyargs) + list(args.args)
    n_args = len(all_args)
    n_defaults = len(args.defaults)
    if n_defaults > 0:
        offset = n_args - n_defaults
        for i, default in enumerate(args.defaults):
            result[all_args[offset + i].arg] = default
    # kwonly defaults
    for a, default in zip(args.kwonlyargs, args.kw_defaults):
        if default is not None:
            result[a.arg] = default
    return result


# =================================================================
# Test 1-6: Each new task function exists with right signature
# =================================================================

def test_seeding_task_signature(task_func_ast):
    fn = _find_function(task_func_ast, "seeding_task")
    params = _function_param_names(fn)
    assert "task" in params
    assert "car" in params
    assert "stations" in params  # optional kwarg
    defaults = _function_param_defaults(fn)
    # Default is the AST constant `None` (literal None in source)
    default_node = defaults.get("stations")
    assert default_node is not None
    assert isinstance(default_node, ast.Constant)
    assert default_node.value is None  # uses hardcoded fallback


def test_pest_scout_task_signature(task_func_ast):
    fn = _find_function(task_func_ast, "pest_scout_task")
    params = _function_param_names(fn)
    assert "car" in params
    assert "scan_passes" in params


def test_shoot_pest_task_signature(task_func_ast):
    fn = _find_function(task_func_ast, "shoot_pest_task")
    params = _function_param_names(fn)
    assert set(params) >= {"task", "car", "pests"}


def test_harvest_task_signature(task_func_ast):
    fn = _find_function(task_func_ast, "harvest_task")
    params = _function_param_names(fn)
    assert set(params) >= {"task", "car", "crop_stations"}


def test_read_order_task_signature(task_func_ast):
    fn = _find_function(task_func_ast, "read_order_task")
    params = _function_param_names(fn)
    assert "car" in params
    assert "ocr_service" in params


def test_delivery_task_signature(task_func_ast):
    fn = _find_function(task_func_ast, "delivery_task")
    params = _function_param_names(fn)
    assert set(params) >= {"task", "car", "order_items", "station_coords"}


def test_mission_main_signature(task_func_ast):
    fn = _find_function(task_func_ast, "mission_main")
    params = _function_param_names(fn)
    for opt in ("run_seeding", "run_watering", "run_shooting",
                "run_harvest", "run_sort", "run_read_order", "run_delivery"):
        assert opt in params, f"missing {opt}"


# =================================================================
# Test 7: All new functions defined at module level
# =================================================================

def test_new_functions_module_level(task_func_ast):
    new_funcs = ("seeding_task", "pest_scout_task", "shoot_pest_task",
                 "harvest_task", "read_order_task", "delivery_task",
                 "mission_main")
    defined = {node.name for node in task_func_ast.body
               if isinstance(node, ast.FunctionDef)}
    missing = [f for f in new_funcs if f not in defined]
    assert not missing, f"missing module-level functions: {missing}"


# =================================================================
# Test 8: Legacy functions preserved (regression check)
# =================================================================

def test_legacy_functions_preserved(task_func_ast):
    """The 8 legacy test functions and 2 classes must still exist."""
    legacy_funcs = ("bmi_test", "cylinder_test", "ingredients_test",
                    "pick_ingredients_test", "answer_test", "food_test",
                    "eject_test", "task_reset")
    legacy_classes = ("Ejection", "MyTask")
    defined = {node.name for node in task_func_ast.body
               if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    for name in legacy_funcs + legacy_classes:
        assert name in defined, f"legacy {name} was removed!"


# =================================================================
# Test 9: Config template exists
# =================================================================

def test_config_mission_example_exists():
    """config_mission.yml.example should be a real YAML file with required keys."""
    import yaml
    config_file = _REPO_ROOT / "config_mission.yml.example"
    assert config_file.exists(), f"missing {config_file}"
    cfg = yaml.safe_load(config_file.read_text())
    for key in ("seeding_stations", "crop_stations", "delivery_stations",
                "mission", "ocr"):
        assert key in cfg, f"missing top-level key {key!r} in config_mission.yml.example"
    assert len(cfg["seeding_stations"]) == 3, "seeding_stations should have 3 entries"
    assert len(cfg["delivery_stations"]) == 3, "delivery_stations should have 3 stations"


# =================================================================
# Test 10: 8.10 mission orchestrator calls each new task
# =================================================================

def test_mission_main_calls_all_tasks(task_func_ast):
    """mission_main body must call all 6 new task functions."""
    fn = _find_function(task_func_ast, "mission_main")
    calls = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name):
                calls.add(node.func.id)
            elif isinstance(node.func, ast.Attribute):
                calls.add(node.func.attr)
    for required in ("seeding_task", "pest_scout_task", "shoot_pest_task",
                     "harvest_task", "read_order_task", "delivery_task"):
        assert required in calls, f"mission_main does not call {required}"


# =================================================================
# Test 11: New tasks have docstrings
# =================================================================

def test_new_tasks_have_docstrings(task_func_ast):
    """Every new task function must have a docstring for documentation."""
    new_funcs = ("seeding_task", "pest_scout_task", "shoot_pest_task",
                 "harvest_task", "read_order_task", "delivery_task",
                 "mission_main")
    for name in new_funcs:
        fn = _find_function(task_func_ast, name)
        docstring = ast.get_docstring(fn)
        assert docstring is not None, f"{name}() missing docstring"
        assert len(docstring.strip()) > 20, f"{name}() docstring too short"


# =================================================================
# Test 12: All 14 tasks exposed via __main__ CLI (argparse choices)
# =================================================================

def test_main_argparse_choices(task_func_ast):
    """The --op argparse choices should include all 6 new tasks."""
    # Find the if __name__ == '__main__' block
    main_block = None
    for node in task_func_ast.body:
        if isinstance(node, ast.If):
            test = node.test
            if (isinstance(test, ast.Compare) and
                isinstance(test.left, ast.Name) and test.left.id == "__name__"):
                main_block = node
                break
    assert main_block is not None, "no __main__ block found"
    # Find the argparse add_argument('--op', choices=...)
    found_choices = None
    for node in ast.walk(main_block):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute) and func.attr == "add_argument":
                # Check first arg is '--op' (positional, not keyword)
                if (node.args and
                        isinstance(node.args[0], ast.Constant) and
                        node.args[0].value == "--op"):
                    for kw in node.keywords:
                        if kw.arg == "choices":
                            if isinstance(kw.value, (ast.List, ast.Tuple)):
                                # Python 3.14 stores as ast.List
                                found_choices = []
                                for elt in kw.value.elts:
                                    if isinstance(elt, ast.Constant):
                                        found_choices.append(elt.value)
    assert found_choices is not None, "--op argument not found in argparse"
    for task_name in ("seeding", "pest_scout", "shoot_pest", "harvest",
                      "read_order", "delivery", "mission"):
        assert task_name in found_choices, \
            f"--op argparse missing {task_name!r}; got {found_choices}"
