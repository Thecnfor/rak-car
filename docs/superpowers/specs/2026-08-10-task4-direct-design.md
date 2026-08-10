# Task4 Direct Hardware Rewrite

## Goal

Provide a standalone task4 implementation that runs inside the runtime process and does not depend on `main/`, HTTP, WebSocket, or `RuntimeApiClient`.

## Design

Add `runtime/tasks/task4_direct.py` with a small state-machine entrypoint. The entrypoint accepts an injected car object for offline tests and otherwise constructs `runtime.services.my_car.MyCar`. It reads detections, IR, and odometry through the car facade, drives the chassis with relative `move_for`, and invokes arm/gripper operations directly. Existing safety, serial arbitration, and stop handling remain owned by `MyCar`.

The task preserves the current operational phases: search/creep, target selection and chassis alignment, arm pickup, placement, and termination after successful pickup or a bounded no-target search. A `--dry-run` mode uses a deterministic fake facade and never opens hardware.

## Constraints

- New implementation imports no module from `main/`.
- No network client, HTTP request, or WebSocket is used.
- Chassis translation uses relative `move_for`; no world-coordinate target is introduced.
- Hardware calls are isolated behind a small protocol-like facade so tests can fake them.
- Existing `main/` task files remain untouched.

## Validation

Run focused unit tests for the direct state machine and Python compilation for the new module. Hardware execution is not attempted in offline CI.
