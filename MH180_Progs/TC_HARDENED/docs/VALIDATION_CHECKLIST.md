## TOOLCHANGE_V2 Validation Checklist

### Entry Point
- Production entry point: `TOOLCHANGE_V2` (call with requested tool as arg #1).
- Complete station flow entry point: `TC_FULL_CHANGE` (arg #1 = requested tool, includes approach/retract motion).

### Pre-Check
- Confirm `TC_CHK_READY`, `TC_SET_TOOL_INDEX`, `TC_VERIFY_TOOL`, `TC_RECOVER`, `TC_PHASE1_UNLOAD`, `TC_PHASE2_LOAD`, and `TOOLCHANGE_V2` are loaded on controller.
- Confirm `TC_FULL_CHANGE` is loaded.
- Confirm `TC_FULL_CHANGE` calls `TOOLCHANGE_V2`.
- Confirm EtherNet/IP mapping matches `TCVars_DX200MH180.xlsx`:
  - `IN#(204)` at-position
  - `IN#(205)` home complete
  - `IN#(206)` motor enabled
  - `IN#(207)` fault
  - `IN#(208)` in motion
  - `IG#(30)` current tool

### Test 1: No-Change Request (same tool)
- Set `IG#(30)` to requested tool (non-zero), with `IN#(194)=ON` and `IN#(196)=OFF`.
- Run `TOOLCHANGE_V2` with same tool request.
- Expected:
  - Immediate return.
  - No execute pulse to `OT#(209)`.
  - `B098 = 0`.

### Test 2: Empty Spindle Load-Only
- Force physical empty spindle state (`IN#(196)=ON` or `IN#(194)=OFF`).
- Request valid tool (1..6) with `TOOLCHANGE_V2`.
- Expected:
  - Job does not short-circuit even if stale `B010` was non-zero before entry.
  - `OT#(209)` executes.
  - `IG#(30)` reads back requested tool.
  - `B010` updates to requested tool and `B098 = 0`.

### Test 3: Fault Recovery Path
- Force `IN#(207)=ON` before calling `TOOLCHANGE_V2`.
- Expected:
  - `TC_RECOVER` pulse fault clear (`OT#(212)`).
  - If fault persists, `TC_COLDSTART` is called.
  - Readiness recheck is attempted before motion command.

### Test 4: Verify Mismatch Retry
- Induce mismatch where `IG#(30)` does not equal requested tool after first command.
- Expected:
  - first `TC_VERIFY_TOOL` sets non-zero `B098`.
  - recovery and readiness re-check run.
  - command/verify executes one more time.
  - if still mismatch, terminal message path `S012`.

### Test 5: Not Ready Blocks Motion
- Force any readiness condition false (`IN#(204)=OFF`, `IN#(205)=OFF`, or `IN#(206)=OFF`).
- Expected:
  - `TC_CHK_READY` returns non-zero status code.
  - recovery is attempted.
  - if still not ready after recovery, job returns without command motion.

### Test 6: Phase 1 Unload (with tool loaded)
- Start with tool physically loaded (`IN#(194)=ON`, `IN#(196)=OFF`).
- Run `TC_PHASE1_UNLOAD`.
- Expected:
  - drawbar extends and release sequence runs.
  - `IN#(196)` transitions ON (spindle empty).
  - `B098 = 0`.

### Test 7: Phase 2 Load/Lock
- Start after index change with spindle positioned for target tool pickup.
- Run `TC_PHASE2_LOAD`.
- Expected:
  - `IN#(194)` transitions ON (tool locked).
  - `IN#(196)` transitions OFF (tool loaded).
  - `B098 = 0`.

### Test 8: Complete Station Program (`TC_FULL_CHANGE`)
- Call `TC_FULL_CHANGE` with valid tool request arg (1..6) from a safe approach condition.
- Expected:
  - robot executes approach, exchange, and retreat moves.
  - `TC_PHASE1_UNLOAD` and `TC_PHASE2_LOAD` both run (or unload is skipped if already empty).
  - `TOOLCHANGE_V2` runs and returns success (`B098=0`).
  - final `B098 = 0` and spindle reports tool locked (`IN#194=ON`, `IN#196=OFF`).

### Result Logging Template
- Date/Time:
- Requested tool:
- Initial `B010`:
- Final `B010`:
- Final `B098`:
- Fault bits (`IN#207`, `IN#212`, `IN#213`):
- Outcome (pass/fail):
- Notes:
