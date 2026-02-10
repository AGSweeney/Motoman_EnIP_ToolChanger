# Tool Changer Process — Cold Start & Robot Programs

EtherNet/IP rotary tool changer for Yaskawa Motoman DX200 / YRC1000 controllers.  
I/O mapping: Output Group #26 (OUT#201), Input Group #26 (IN#201).

---

## 1. Cold Start Procedure

### 1.1 Prerequisites

- [ ] E-stop released (A-10 NC circuit closed)
- [ ] ClearCore powered, Ethernet link established
- [ ] DX200/YRC1000 EtherNet/IP connection to tool changer configured and established
- [ ] Tool turret clear of obstructions

### 1.2 Hardware Enable (DI-8)

| Step | Action | Notes |
|------|--------|-------|
| 1 | Release E-stop | Ensure A-10 is high (circuit closed) |
| 2 | Assert DI-8 | Hardware enable input to ClearCore — motor will enable |
| 3 | Wait for Motor Enabled | IN#206 (byte 0, bit 5) = ON. Allow up to 2 s for HLFB |
| 4 | If fault: clear first | DI-8 rising edge is ignored while faulted. Use Clear Faults (OUT#212) or A-9 button |

*If Motor Enabled never asserts: check DI-8 wiring, E-stop, ClearPath motor connection, and HLFB.*

### 1.3 Homing

| Step | Action | Notes |
|------|--------|-------|
| 1 | Set Home Request | OUT#210 = ON (byte 1, bit 1). Rising-edge triggers homing |
| 2 | Wait for Homing Active | State = HOMING (1). IN#212 = ON |
| 3 | Wait for Home Complete | IN#205 = ON (byte 0, bit 4) |
| 4 | Wait for State = IDLE | State = 2. Homed and ready |
| 5 | Clear Home Request | OUT#210 = OFF. Avoid re-triggering on next cycle |

*Homing timeout: 30 s. If Home Failed or Home Sensor Stuck, address DI-6 and mechanical issues, then Clear Faults.*

### 1.4 Cold Start Complete

After homing:

- Current tool = 1 (home position)
- State = IDLE
- At Position = ON
- Ready for tool change commands

---

## 2. Robot-Side Tool Change Process

The robot must **put away its current tool** (if any) into the correct pocket before grabbing the next tool. Tool-in-spindle is checked via `<Placeholder HAS_TOOL>`. The carousel rotates to present the desired pocket at the spindle interface.

### 2.1 Variables

| Variable | Purpose |
|----------|---------|
| B009 | New tool number requested (1–6) |
| B010 | Tool pocket position in carousel (1–6) |
| B011 | Current tool in spindle (1–6, or 0 if empty) |

### 2.2 Positions (Define in job)

| Position | Description |
|----------|-------------|
| P000 (P_LOW_OUT) | Low and out — safe retracted position away from tool changer |
| P001 (P_LOW_IN) | Low and in — at the tool changer interface for place/retrieve |
| P002 (P_HIGH_IN) | High in — hovering above the tool in the carousel (approach from above for grab) |

*Use `MOVL Pxxx V=xx PL=0` for position moves.*

*Per TC Motion Notes: Put away = low out → low in → release → high in. Grab = high in → low in → grab → low out.*

### 2.3 State & Status Checks

Before commanding a tool change:

| Check | Input | Condition |
|-------|-------|-----------|
| No fault | IN#207 | OFF |
| Motor enabled | IN#206 | ON |
| Home complete | IN#205 | ON |
| Ready (IDLE or AT_TOOL) | State bits IN#209–211 | 2 or 4 |

### 2.4 Execute Move Behavior

- **Trigger**: Carousel move triggers on **tool number change while Execute is held high**.
- **Execute rising edge alone**: Does *not* trigger a move.
- Hold Execute high, write tool number, wait for At Position, then write next tool number.

### 2.5 Tool Change Sequence (High Level)

1. Move to **P_LOW_OUT** (safe position).
2. **If** `<Placeholder HAS_TOOL>` (tool in spindle):
   - B011 = current tool in spindle.
   - Rotate carousel to pocket B011 (put-away pocket).
   - Wait for At Position.
   - **If** pocket not empty (Tool In Pocket = 1): abort — pocket occupied.
   - Move low out → **P_LOW_IN**.
   - Release tool (spindle release).
   - Move to **P_HIGH_IN** (hovering above tool in carousel).
   - Rotate carousel to B009 (new tool).
   - Wait for At Position.
3. **Else** (no tool in spindle):
   - Rotate carousel to B009.
   - Wait for At Position.
4. **Retrieve new tool** (spindle must be empty):
   - Move to **P_HIGH_IN** (approach from above).
   - Move to **P_LOW_IN** (descend, grab tool).
   - Deactivate spindle release.
   - Wait for `<Placeholder HAS_TOOL>` (spindle confirms tool loaded).
   - Retract to **P_LOW_OUT**.

### 2.6 Grab Safety Rule

*Never grab when the spindle is loaded.* All grab paths verify spindle empty before proceeding:
- **TC_CHANGE_TOOL**: Reaches retrieve only after put-away (spindle emptied) or no-tool path. Defensive check at *RETRIEVE aborts if HAS_TOOL=ON.
- **TC_GRAB_TOOL**: Explicit check aborts if HAS_TOOL=ON; calls put away first.

---

## 3. Inform Language Example Programs (.jbi Pseudo-code)

*I/O mapping: OUT#201–224 (Output Groups 26–28), IN#201–248 (Input Groups 26–31).*

### 3.1 Constants (User I/O or Variables)

```
! Tool changer I/O mapping (EtherNet/IP)
! Write: DOUT #OG<group> <value>   Read: DIN <variable> IG#<group>
!
! Output Group 26 (OG26): Byte 0 = Tool Select (1-6)
! Output Group 27 (OG27): Byte 1 = Execute(1), Home(2), Clear Faults(8)
! Output Group 28 (OG28): Byte 2 = Aux outputs
! Input Group 26 (IG26):  Byte 0 = status
! Input Group 27 (IG27):  Byte 1 = state, info
!
! Key outputs (DOUT #OG26/OG27):
!   OG26 = tool number (1-6)
!   OG27 = 1 Execute, 2 Home, 8 Clear Faults
!
! Key inputs (DIN Bxxx IG#26/IG#27): read byte into variable, then mask bits
!   IG#26: bits 0-2 Current Tool, 3 At Position, 4 Home Complete, 5 Motor Enabled, 6 Fault, 7 In Motion
!   IG#27: bits 0-2 State, 3 Invalid Tool, 4 Not Ready, 5 E-Stop, 6 Tool In Pocket
! IN#204 = At Position
! IN#205 = Home Complete
! IN#206 = Motor Enabled
! IN#207 = Fault Present
! IN#215 = Tool In Pocket (1 = tool present in current pocket)
! IN#209-211 = State (0=DISABLED, 1=HOMING, 2=IDLE, 3=MOVING, 4=AT_TOOL, 5=FAULTED)
!
! Variables: B009 = new tool (1-6), B010 = pocket position, B011 = current tool in spindle
! <Placeholder HAS_TOOL> = spindle tool-loaded check (replace with actual condition)
```

### 3.2 Cold Start Job

<pre><code>/JOB
//NAME TC_COLD_START
NOP
<span style="color:magenta">! --- Pre-check: no fault ---</span>
*WAIT_FAULT_OFF
WAIT IN#(207)=OFF  <span style="color:magenta">//Fault Present</span>
TIMER T=0.05
*WAIT_MOTOR
WAIT IN#(206)=ON  <span style="color:magenta">//Motor Enabled</span>
TIMER T=0.2
DOUT #OG27 0
TIMER T=0.05
DOUT #OG27 2  <span style="color:magenta">//Home Request</span>
TIMER T=0.1
DOUT #OG27 0
*WAIT_HOME_DONE
WAIT IN#(205)=ON  <span style="color:magenta">//Home Complete</span>
TIMER T=0.1
*WAIT_IDLE
WAIT IN#(209)=OFF  <span style="color:magenta">//State bit 0</span>
WAIT IN#(210)=ON   <span style="color:magenta">//State bit 1 (IDLE=2)</span>
WAIT IN#(211)=OFF  <span style="color:magenta">//State bit 2</span>
TIMER T=0.2
<span style="color:magenta">! --- Cold start complete ---</span>
END</code></pre>

### 3.3 Change Tool Job (Combined: Put Away + Grab)

*Single job for full tool change. Alternative: call TC_PUT_AWAY_TOOL then TC_GRAB_TOOL (see 3.5–3.6).*  
*Before calling: set B009 = new tool (1–6). If spindle has a tool, set B011 = current tool.*  
*Replace `<Placeholder HAS_TOOL>` with your spindle tool-loaded check (e.g. `IN#(xxx)`).*  
*Replace `OT#(xxx)` with your spindle release output.*

<pre><code>/JOB
//NAME TC_CHANGE_TOOL
NOP
<span style="color:magenta">! B009 = new tool requested (1-6)</span>
<span style="color:magenta">! B010 = pocket position, B011 = current tool in spindle</span>
<span style="color:magenta">! --- Pre-check: fault, motor, home ---</span>
*CHECK_READY
IF IN#(207)=ON,JMP *FAULT_HANDLE  <span style="color:magenta">//Fault Present</span>
IF IN#(206)=OFF,JMP *NOT_ENABLED  <span style="color:magenta">//Motor Enabled</span>
IF IN#(205)=OFF,JMP *NOT_HOMED  <span style="color:magenta">//Home Complete</span>
IF B009<1,JMP *INVALID_TOOL
IF B009>6,JMP *INVALID_TOOL
<span style="color:blue">MOVL</span> P000 V=100 PL=0  <span style="color:magenta">//P_LOW_OUT</span>
<span style="color:magenta">! --- Sync B011 with spindle: if no tool, clear stale value ---</span>
*CHECK_HAS_TOOL
IF &lt;Placeholder HAS_TOOL&gt;=ON,JMP *HAS_TOOL
B011 = 0  <span style="color:magenta">//Sync: spindle empty, clear stale tool number</span>
JMP *ROTATE_TO_NEW
*HAS_TOOL
<span style="color:magenta">! B011 = current tool in spindle (must be set by caller when HAS_TOOL)</span>
B010 = B011
<span style="color:magenta">! Rotate carousel to put-away pocket</span>
DOUT #OG27 1  <span style="color:magenta">//Execute</span>
TIMER T=0.02
DOUT #OG26 B010  <span style="color:magenta">//Tool Select</span>
*WAIT_PUTAWAY_POS
WAIT IN#(204)=ON  <span style="color:magenta">//At Position</span>
TIMER T=0.05
IF IN#(215)=ON,JMP *POCKET_OCCUPIED  <span style="color:magenta">//Tool In Pocket (must be empty)</span>
<span style="color:blue">MOVL</span> P001 V=100 PL=0  <span style="color:magenta">//P_LOW_IN</span>
DOUT OT#(xxx) ON  <span style="color:magenta">//Spindle Release</span>
TIMER T=0.5
<span style="color:blue">MOVL</span> P002 V=100 PL=0  <span style="color:magenta">//P_HIGH_IN</span>
DOUT #OG26 B009  <span style="color:magenta">//Tool Select</span>
*WAIT_NEW_POS
WAIT IN#(204)=ON  <span style="color:magenta">//At Position</span>
TIMER T=0.05
JMP *RETRIEVE
*ROTATE_TO_NEW
DOUT #OG27 1  <span style="color:magenta">//Execute</span>
TIMER T=0.02
DOUT #OG26 B009  <span style="color:magenta">//Tool Select</span>
*WAIT_NEW_POS2
WAIT IN#(204)=ON  <span style="color:magenta">//At Position</span>
TIMER T=0.05
*RETRIEVE
IF &lt;Placeholder HAS_TOOL&gt;=ON,JMP *SPINDLE_NOT_EMPTY  <span style="color:magenta">//Defensive: spindle must be empty to grab</span>
<span style="color:blue">MOVL</span> P002 V=100 PL=0  <span style="color:magenta">//P_HIGH_IN (approach from above)</span>
<span style="color:blue">MOVL</span> P001 V=100 PL=0  <span style="color:magenta">//P_LOW_IN (descend, grab)</span>
DOUT OT#(xxx) OFF  <span style="color:magenta">//Spindle Grab</span>
*WAIT_TOOL_LOADED
WAIT &lt;Placeholder HAS_TOOL&gt;=ON
TIMER T=0.2
<span style="color:blue">MOVL</span> P000 V=100 PL=0  <span style="color:magenta">//P_LOW_OUT</span>
DOUT #OG27 0  <span style="color:magenta">//Execute off</span>
TIMER T=0.05
B011 = B009
RET
*FAULT_HANDLE
<span style="color:magenta">! Handle fault - clear via TC_CLEAR_FAULTS</span>
RET
*NOT_ENABLED
<span style="color:magenta">! Assert DI-8 to enable motor</span>
RET
*NOT_HOMED
<span style="color:magenta">! Run cold start first</span>
RET
*INVALID_TOOL
RET
*POCKET_OCCUPIED
<span style="color:magenta">! Pocket has tool already - cannot put away</span>
RET
*SPINDLE_NOT_EMPTY
<span style="color:magenta">! Spindle has tool - put away first (should not reach here)</span>
RET
END</code></pre>

### 3.4 Approach: Separate Jobs vs Combined

*Choose during implementation:*

| Approach | Jobs | Use when |
|----------|------|----------|
| **Separate** | TC_PUT_AWAY_TOOL, TC_GRAB_TOOL | Flexibility: put away only, grab only, or call both in sequence |
| **Combined** | TC_CHANGE_TOOL | Convenience: single call for full tool change |

*Separate sequence example:* `CALL JOB:TC_PUT_AWAY_TOOL` then `CALL JOB:TC_GRAB_TOOL` (with B009 set).

### 3.5 Put Away Tool Only

*Puts the current tool into its pocket and retracts. Does not grab a new tool.*  
*Before calling: set B011 = current tool in spindle (1–6).*

<pre><code>/JOB
//NAME TC_PUT_AWAY_TOOL
NOP
<span style="color:magenta">! --- Pre-check: fault, motor, home ---</span>
*CHECK_READY
IF IN#(207)=ON,JMP *FAULT_HANDLE  <span style="color:magenta">//Fault Present</span>
IF IN#(206)=OFF,JMP *NOT_ENABLED  <span style="color:magenta">//Motor Enabled</span>
IF IN#(205)=OFF,JMP *NOT_HOMED  <span style="color:magenta">//Home Complete</span>
IF &lt;Placeholder HAS_TOOL&gt;=ON,JMP *HAS_TOOL
B011 = 0  <span style="color:magenta">//Sync: spindle empty, clear stale tool number</span>
JMP *DONE
*HAS_TOOL
IF B011<1,JMP *INVALID_TOOL
IF B011>6,JMP *INVALID_TOOL
<span style="color:blue">MOVL</span> P000 V=100 PL=0  <span style="color:magenta">//P_LOW_OUT</span>
DOUT #OG27 1  <span style="color:magenta">//Execute</span>
TIMER T=0.02
DOUT #OG26 B011  <span style="color:magenta">//Tool Select (put-away pocket)</span>
*WAIT_PUTAWAY_POS
WAIT IN#(204)=ON  <span style="color:magenta">//At Position</span>
TIMER T=0.05
IF IN#(215)=ON,JMP *POCKET_OCCUPIED  <span style="color:magenta">//Tool In Pocket (must be empty)</span>
<span style="color:blue">MOVL</span> P001 V=100 PL=0  <span style="color:magenta">//P_LOW_IN</span>
DOUT OT#(xxx) ON  <span style="color:magenta">//Spindle Release</span>
TIMER T=0.5
<span style="color:blue">MOVL</span> P002 V=100 PL=0  <span style="color:magenta">//P_HIGH_IN</span>
<span style="color:blue">MOVL</span> P000 V=100 PL=0  <span style="color:magenta">//P_LOW_OUT</span>
DOUT #OG27 0  <span style="color:magenta">//Execute off</span>
B011 = 0  <span style="color:magenta">//Sync: spindle now empty after put away</span>
*DONE
RET
*FAULT_HANDLE
*NOT_ENABLED
*NOT_HOMED
*INVALID_TOOL
*POCKET_OCCUPIED
RET
END</code></pre>

### 3.6 Grab Tool Only

*Rotates to the requested tool and grabs it. Verifies spindle is empty before proceeding. Approaches from high in, then descends to low in.*  
*Before calling: set B009 = tool to grab (1–6).*

<pre><code>/JOB
//NAME TC_GRAB_TOOL
NOP
<span style="color:magenta">! --- Pre-check: fault, motor, home ---</span>
*CHECK_READY
IF IN#(207)=ON,JMP *FAULT_HANDLE  <span style="color:magenta">//Fault Present</span>
IF IN#(206)=OFF,JMP *NOT_ENABLED  <span style="color:magenta">//Motor Enabled</span>
IF IN#(205)=OFF,JMP *NOT_HOMED  <span style="color:magenta">//Home Complete</span>
IF &lt;Placeholder HAS_TOOL&gt;=ON,JMP *SPINDLE_NOT_EMPTY  <span style="color:magenta">//Spindle must be empty - put away first</span>
B011 = 0  <span style="color:magenta">//Sync: spindle empty, clear stale tool number</span>
IF B009<1,JMP *INVALID_TOOL
IF B009>6,JMP *INVALID_TOOL
<span style="color:blue">MOVL</span> P000 V=100 PL=0  <span style="color:magenta">//P_LOW_OUT</span>
DOUT #OG27 1  <span style="color:magenta">//Execute</span>
TIMER T=0.02
DOUT #OG26 B009  <span style="color:magenta">//Tool Select</span>
*WAIT_AT_POS
WAIT IN#(204)=ON  <span style="color:magenta">//At Position</span>
TIMER T=0.05
<span style="color:blue">MOVL</span> P002 V=100 PL=0  <span style="color:magenta">//P_HIGH_IN (approach from above)</span>
<span style="color:blue">MOVL</span> P001 V=100 PL=0  <span style="color:magenta">//P_LOW_IN (descend, grab)</span>
DOUT OT#(xxx) OFF  <span style="color:magenta">//Spindle Grab</span>
*WAIT_TOOL_LOADED
WAIT &lt;Placeholder HAS_TOOL&gt;=ON
TIMER T=0.2
<span style="color:blue">MOVL</span> P000 V=100 PL=0  <span style="color:magenta">//P_LOW_OUT</span>
DOUT #OG27 0  <span style="color:magenta">//Execute off</span>
B011 = B009
RET
*FAULT_HANDLE
*NOT_ENABLED
*NOT_HOMED
*SPINDLE_NOT_EMPTY
<span style="color:magenta">! Spindle has tool - call TC_PUT_AWAY_TOOL first</span>
RET
*INVALID_TOOL
RET
END</code></pre>

### 3.7 Clear Faults (Rising Edge)

<pre><code>/JOB
//NAME TC_CLEAR_FAULTS
NOP
DOUT #OG27 0
TIMER T=0.05
DOUT #OG27 8  <span style="color:magenta">//Clear Faults</span>
TIMER T=0.1
DOUT #OG27 0
*WAIT_CLEAR
WAIT IN#(207)=OFF  <span style="color:magenta">//Fault Present</span>
TIMER T=0.2
RET
END</code></pre>

### 3.8 Group I/O Reference

*Write multiple bits: `DOUT #OG<group> <value>`*  
*Read multiple bits: `DIN <variable> IG#<group>`*

| Group | Dir | Byte | Value / Bits | Meaning |
|-------|-----|------|--------------|---------|
| OG26 | out | 0 | 0–6 | Tool Select (1–6) |
| OG27 | out | 1 | 0 | All commands off |
| OG27 | out | 1 | 1 | Execute Move |
| OG27 | out | 1 | 2 | Home Request |
| OG27 | out | 1 | 8 | Clear Faults |
| IG26 | in | 0 | bits 0–7 | Current Tool, At Position, Home Complete, Motor Enabled, Fault, In Motion |
| IG27 | in | 1 | bits 0–7 | State, Invalid Tool, Not Ready, E-Stop, Tool In Pocket |

---

## 4. Fault Recovery

| Fault | Input Byte 2–3 | Action |
|-------|----------------|-------|
| Motor Alert | 0x0001 | Check ClearPath motor; clear via MSP if needed |
| HLFB Timeout | 0x0002 | Check motor wiring, enable, move timeout |
| Home Failed | 0x0004 | Find DI-6 within 30 s; check sensor |
| Position Error | 0x0008 | Mechanical issue; re-home after clear |
| Home Sensor Stuck | 0x0020 | DI-6 did not clear; check sensor/geometry |

1. Diagnose cause.
2. Clear faults: rising edge on OUT#212 (or A-9 button).
3. Cycle DI-8 (low then high) if needed after clear.
4. Re-run cold start.

---

## 5. Quick Reference

| Signal | Type | Address | Description |
|--------|------|---------|-------------|
| Tool Select | OUT | 201–203 | Bits 0–2: tool 1–6 |
| Execute | OUT | 209 | Triggers move |
| Home Request | OUT | 210 | Rising edge starts homing |
| Clear Faults | OUT | 212 | Rising edge clears faults |
| Current Tool | IN | 201–203 | Bits 0–2 |
| At Position | IN | 204 | Turret settled |
| Home Complete | IN | 205 | Homing done |
| Motor Enabled | IN | 206 | HLFB asserted |
| Fault Present | IN | 207 | Fault active |
| In Motion | IN | 208 | Turret rotating |
| State | IN | 209–211 | 0–5 (DISABLED…FAULTED) |
| Tool In Pocket | IN | 215 | 1 = tool present in current pocket |
