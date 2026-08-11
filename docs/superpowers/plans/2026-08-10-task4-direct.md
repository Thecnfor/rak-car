# Task4 Direct Hardware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add an independent task4 runner that directly uses `MyCar` and never imports `main/` or a network client.

**Architecture:** A small `Task4Direct` state machine owns search, detection, pickup, placement, and bounded termination. It consumes a narrow injected car facade, allowing fake-car unit tests and real `MyCar` execution through the same code path. Relative chassis motion uses `move_for` only.

**Tech Stack:** Python 3, stdlib `unittest`, runtime `MyCar`, existing smartcar hardware APIs.

## Global Constraints

- The new runner imports no module from `main/`.
- No HTTP or WebSocket calls are made.
- Chassis translation uses relative `move_for`.
- Hardware safety and serial arbitration remain in `MyCar`.

### Task 1: Add direct task4 state machine

**Files:**
- Create: `runtime/tasks/__init__.py`
- Create: `runtime/tasks/task4_direct.py`

**Interfaces:**
- `run(car=None, *, dry_run=False, max_seconds=120.0) -> dict`
- `Task4Direct(car, max_seconds=120.0)` with `run() -> dict`
- Car facade methods: `get_detection_results()`, `get_all_ir_distance()`, `get_odometry()`, `move_for(vector, stop=True)`, `move_to_detection_target(...)`, `arm.move_x_position(mm)`, `arm.move_y_position(mm)`, `arm.grasp(bool)`.

- [ ] Define normalized detection selection and IR/odom extraction helpers.
- [ ] Implement search loop with 0.12 m/s relative creep, 0.8 m cumulative bound, and stop checks.
- [ ] Implement one target cycle: align using the direct facade, move arm to configured pickup pose, close gripper, retract, move to bin pose, release.
- [ ] Return structured result with `ok`, `picked`, `reason`, and `elapsed_s`.
- [ ] Add CLI flags `--dry-run` and `--max-seconds`; default path constructs `MyCar` and closes it in `finally`.

### Task 2: Add offline tests

**Files:**
- Create: `runtime/tests/test_task4_direct.py`

- [ ] Fake the car and assert no-target exits at the distance bound.
- [ ] Assert a matching detection causes direct alignment and arm/gripper calls.
- [ ] Assert `dry_run` does not instantiate or call hardware.
- [ ] Run focused unittest module.

### Task 3: Verify and commit

- [ ] Compile new modules with `/usr/bin/python3 -m py_compile`.
- [ ] Run focused tests and existing runtime tests that do not require hardware.
- [ ] Inspect imports to prove no `main` dependency.
- [ ] Commit with a focused message and report branch/commit.
