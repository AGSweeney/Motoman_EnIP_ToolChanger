## Spindle I/O Mapping

This mapping is confirmed from DX200 pendant Universal Input/Output screens.

### Signals in use

- `OT#(185)` -> Drawbar Extend Command
  - `ON` before `WAIT IN#(193)=ON`
  - `OFF` after tool-change command

- `IN#(193)` -> Drawbar Extended
  - waited after `OT#(185)=ON`

- `OT#(186)` -> DB Return Command
  - set `OFF` before engage
  - set `ON` during return/recovery phase
  - output command (not status feedback)

- `IN#(194)` -> Tool Locked
  - waited after `OT#(186)=ON`

- `IN#(196)` -> No Tool Loaded / Return Without Tool
  - true when spindle did not retain a tool

### Practical mapping for requested states

- Tool-locked (feedback): `IN#(194)`
- Drawbar extend (command): `OT#(185)` (`ON` = extend)
- Return without tool (feedback): `IN#(196)`

### Recommended confirmation

Verify with PLC ladder/electrical I/O list before production:
- confirm whether `IN#(195)` (Spindle Stopped) should be enforced before drawbar actions
