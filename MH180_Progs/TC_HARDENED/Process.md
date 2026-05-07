## Hardened Tool Changer Process (DX200 MH180)

### Goals
- deterministic tool-change sequence
- one retry on command/handshake failure
- escalate to `TC_COLDSTART` if retry fails
- support empty spindle by doing load-only behavior
- support explicit 2-phase station flow (unload then load)

### I/O and Register Baseline (from `TCVars_DX200MH180.xlsx`)
- `OG#(26)`: requested tool number command (output group value)
- `OT#(209)`: execute move pulse
- spindle commands from pendant map:
  - `OT#(185)`: DB extend
  - `OT#(186)`: DB return
- `IN#(204)`: at-position/ready
- `IN#(205)`: home complete
- `IN#(206)`: motor enabled
- `IN#(207)`: fault present
- `IN#(208)`: in motion
- `IG#(30)` -> `B010`: current/reported tool number
- spindle states from pendant map:
  - `IN#(193)`: drawbar extended
  - `IN#(194)`: tool locked
  - `IN#(196)`: no tool loaded (return without tool)

### Variable Conventions
- `B009`: requested tool number
- `B010`: current/reported tool number
- `B098`: status/result code from helper jobs (0 means OK)

### Normal Flow
1. entry (`TOOLCHANGE_V2`) gets requested tool and stores to `B009`
2. synchronize spindle state first:
   - read `IG#(30)` into `B010`
   - force `B010=0` if `IN#(196)=ON` (no tool loaded) or `IN#(194)=OFF` (not locked)
3. if `B009 = B010` and `B010 <> 0`, exit early (already correct tool)
4. call `TC_CHK_READY`
5. if not ready, call `TC_RECOVER`, then re-check ready
6. call `TC_SET_TOOL_INDEX`
7. call `TC_VERIFY_TOOL`
8. if verify passes, complete successfully

### 2-Phase Station Flow (`SAMPLE_TC`)
1. approach station and move to pre-exchange pose
2. call `TC_PHASE1_UNLOAD`:
   - if spindle already empty (`IN#196=ON`), skip unload actions
   - else release/put-away active tool and confirm empty spindle
3. move to exchange pose and call `TOOLCHANGE_V2` to index requested pocket
4. retract to pre-exchange pose
5. call `TC_PHASE2_LOAD`:
   - close drawbar/return path
   - confirm tool locked (`IN#194=ON`) and not empty (`IN#196=OFF`)
6. leave station

### Retry and Recovery Flow
1. if `TC_VERIFY_TOOL` fails, run recovery (`TC_RECOVER`)
2. re-check with `TC_CHK_READY`
3. retry `TC_SET_TOOL_INDEX`
4. retry `TC_VERIFY_TOOL`
5. if still failed, stop and report via message/alarm path

### Empty Spindle Policy
- Empty spindle is treated as load-only.
- Practical behavior: physical spindle inputs (`IN#(194)`, `IN#(196)`) override stale `B010`.
- continue through command and verify path to force valid loaded state.

### Job/File Layout
- `TC_HARDENED/TC_CHK_READY.JBI`: readiness checks and status code set
- `TC_HARDENED/TC_SET_TOOL_INDEX.JBI`: execute move command and handshakes
- `TC_HARDENED/TC_VERIFY_TOOL.JBI`: read back tool and validate
- `TC_HARDENED/TC_RECOVER.JBI`: fault clear and coldstart escalation
- `TC_HARDENED/TC_PHASE1_UNLOAD.JBI`: phase 1 unload/empty confirmation
- `TC_HARDENED/TC_PHASE2_LOAD.JBI`: phase 2 load/lock confirmation
- `TC_HARDENED/TOOLCHANGE_V2.JBI`: main orchestrator

### Integration
- `TC_HARDENED/SAMPLE_TC.JBI` calls `TOOLCHANGE_V2`
- `TC_HARDENED/ARGPASS.JBI` calls `TOOLCHANGE_V2`
- `TC_HARDENED/TOOLCHANGE.JBI` remains available for rollback safety
