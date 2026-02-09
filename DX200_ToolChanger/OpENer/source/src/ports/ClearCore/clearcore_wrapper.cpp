/*
 * DX200 Rotary Tool Changer -- ClearCore Hardware Wrapper Implementation
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#ifdef CLEARCORE
#include "ClearCore.h"
#include "NvmManager.h"
#include "ports/ClearCore/clearcore_wrapper.h"
#include "lwip/opt.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/netifapi.h"
#include "ports/ClearCore/networkconfig.h"
#include "ciptcpipinterface.h"
#include "trace.h"
#include "lwip/inet.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <core_cm4.h>

extern "C" {
unsigned long GetMillis(void) {
    return Milliseconds();
}

void ConnectorLed_SetState(int state) {
    ConnectorLed.State(state != 0);
}

void ConnectorIO0_Initialize(void) {
    ConnectorIO0.Mode(Connector::OUTPUT_DIGITAL);
}

void ConnectorIO0_SetState(int state) {
    ConnectorIO0.State(state != 0);
}

void ConnectorIO1_Initialize(void) {
    ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);
}

void ConnectorIO1_SetState(int state) {
    ConnectorIO1.State(state != 0);
}

void ConnectorIO2_Initialize(void) {
    ConnectorIO2.Mode(Connector::OUTPUT_DIGITAL);
}

void ConnectorIO2_SetState(int state) {
    ConnectorIO2.State(state != 0);
}

void ConnectorIO3_Initialize(void) {
    ConnectorIO3.Mode(Connector::OUTPUT_DIGITAL);
}

void ConnectorIO3_SetState(int state) {
    ConnectorIO3.State(state != 0);
}

void ConnectorIO4_Initialize(void) {
    ConnectorIO4.Mode(Connector::OUTPUT_DIGITAL);
}

void ConnectorIO4_SetState(int state) {
    ConnectorIO4.State(state != 0);
}

void ConnectorIO5_Initialize(void) {
    ConnectorIO5.Mode(Connector::OUTPUT_DIGITAL);
}

void ConnectorIO5_SetState(int state) {
    ConnectorIO5.State(state != 0);
}

void ConnectorDI6_Initialize(void) {
    ConnectorDI6.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorDI6_GetState(void) {
    return ConnectorDI6.State() ? 1 : 0;
}

void ConnectorDI7_Initialize(void) {
    ConnectorDI7.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorDI7_GetState(void) {
    return ConnectorDI7.State() ? 1 : 0;
}

void ConnectorDI8_Initialize(void) {
    ConnectorDI8.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorDI8_GetState(void) {
    return ConnectorDI8.State() ? 1 : 0;
}

void ConnectorA9_Initialize(void) {
    ConnectorA9.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorA9_GetState(void) {
    return ConnectorA9.State() ? 1 : 0;
}

void ConnectorA10_Initialize(void) {
    ConnectorA10.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorA10_GetState(void) {
    return ConnectorA10.State() ? 1 : 0;
}

void ConnectorA11_Initialize(void) {
    ConnectorA11.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorA11_GetState(void) {
    return ConnectorA11.State() ? 1 : 0;
}

void ConnectorA12_Initialize(void) {
    ConnectorA12.Mode(Connector::INPUT_DIGITAL);
}

int ConnectorA12_GetState(void) {
    return ConnectorA12.State() ? 1 : 0;
}

/* NOTE: Not reentrant -- uses a static buffer.  Safe on bare-metal
 * single-core (all callers are main-loop context), but must NOT be
 * called from ISR if interrupt-driven logging is ever added. */
void ClearCoreTraceOutput(const char *format, ...) {
    static char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    /* vsnprintf returns the number of characters that WOULD have been
     * written.  Clamp to the actual buffer capacity. */
    if (len < 0) return;
    if (len >= (int)sizeof(buffer)) {
        len = (int)sizeof(buffer) - 1;
    }
    buffer[len] = '\0';  /* guaranteed in-bounds now */

    /* Ensure proper \r\n line ending for serial terminal.
     * Strip any trailing \n the caller already appended, then add \r\n. */
    while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
        len--;
    }
    if (len < (int)sizeof(buffer) - 2) {
        buffer[len]     = '\r';
        buffer[len + 1] = '\n';
        buffer[len + 2] = '\0';
    } else {
        buffer[len] = '\0';
    }

    ConnectorUsb.Send(buffer);
    ConnectorUsb.Flush();
}
}

/* =========================================================================
 * Tool Changer -- ClearPath SDSK on M-0, 3600 ppr, 10:1 gearbox
 *
 * 36 000 counts / full turret revolution
 *  6 000 counts / tool pocket  (6 pockets, evenly spaced)
 *
 * DI-6 is used as the home proximity sensor input.
 * ========================================================================= */

/* Motor connector alias */
extern "C" {
#define TC_MOTOR        ConnectorM0

/* Motion parameters (pre-gearbox motor values converted to pulses/sec)
 *   Max velocity:       500 RPM      -> 500  * 3600 / 60 = 30 000 pulses/sec
 *   Max acceleration: 1 000 RPM/s^2  -> 1000 * 3600 / 60 = 60 000 pulses/sec^2
 *   Homing velocity:    ~33 RPM      ->   33 * 3600 / 60 =  2 000 pulses/sec   */
static const int32_t TC_VEL_MAX         = 10000;   /* 500/3 RPM   pre-gearbox      */
static const int32_t TC_ACCEL_MAX       = 60000;   /* 1000 RPM/s^2 pre-gearbox   */
static const int32_t TC_HOME_VEL        = 2000;    /* ~33 RPM  pre-gearbox       */

/* Absolute positions for each tool pocket (0-indexed internally) */
static const int32_t toolPositions[TC_TOOL_COUNT] = {
    0,      /* Tool 1 */
    6000,   /* Tool 2 */
    12000,  /* Tool 3 */
    18000,  /* Tool 4 */
    24000,  /* Tool 5 */
    30000   /* Tool 6 */
};

/* ---------- internal state ---------- */
static ToolChangerState tc_state       = TC_STATE_DISABLED;
static uint8_t          tc_current_tool = 0;   /* 1-6, 0 = unknown       */
static uint8_t          tc_target_tool  = 0;   /* requested tool 1-6     */
static uint16_t         tc_fault_code   = TC_FAULT_NONE;
static uint8_t          tc_info_code    = TC_INFO_NONE;  /* non-fatal warnings */
static int              tc_home_done    = 0;
static int32_t          tc_position     = 0;   /* normalised 0 .. 35999  */

/* Track accumulated motor position for relative-move strategy */
static int32_t          tc_motor_abs_pos = 0;
static int8_t           tc_move_dir      = 0;  /* +1 = CW, -1 = CCW, 0 = stopped */

/* Homing sub-phase:
 *   0 = move off sensor (if already on it)
 *   1 = seek sensor (normal homing search) */
static int              tc_home_phase   = 0;

/* DI-8 hardware enable tracking */
static int              tc_prev_di8     = 0;   /* previous DI-8 state    */

/* A-9 physical fault reset button -- rising-edge detection */
static int              tc_prev_a9      = 0;

/* A-10 E-stop input (normally closed: HIGH = normal, LOW = E-stop active).
 * Fail-safe: a broken wire reads LOW, same as E-stop pressed. */
static int              tc_estop_active = 0;

/* Homing timeout tracking */
static unsigned long    tc_home_start_ms = 0;
#define TC_HOME_TIMEOUT_MS  30000  /* 30 s -- ample for full revolution at homing speed */

/* HLFB timeout tracking -- three scenarios:
 *   1. Post-enable: HLFB must assert within TC_HLFB_ENABLE_TIMEOUT_MS
 *      after the motor is enabled.
 *   2. Move completion: move must finish within TC_MOVE_TIMEOUT_MS.
 *   3. Unexpected drop: HLFB must not stay deasserted for more than
 *      TC_HLFB_DROP_DEBOUNCE_MS while in IDLE or AT_TOOL. */
#define TC_HLFB_ENABLE_TIMEOUT_MS  2000  /* max time for HLFB after enable */
#define TC_MOVE_TIMEOUT_MS         5000  /* max time for a tool-change move */
#define TC_HLFB_DROP_DEBOUNCE_MS    100  /* debounce for unexpected HLFB loss */

static unsigned long    tc_enable_request_ms = 0;  /* timestamp of last enable  */
static int              tc_awaiting_hlfb     = 0;  /* 1 = enabled, waiting for HLFB */
static unsigned long    tc_move_start_ms     = 0;  /* timestamp of move start   */
static unsigned long    tc_hlfb_drop_ms      = 0;  /* first deassert seen (0=ok)*/

/* Fault-clear override: when non-zero, DI-8 is temporarily ignored
 * while the enable is being cycled to clear a ClearPath fault. */
static int              tc_fault_clear_active = 0;

/* ---------- helpers ---------- */

/** Normalise an arbitrary count value into the range [0, TC_COUNTS_PER_REV). */
static int32_t tc_normalise(int32_t pos) {
    pos = pos % TC_COUNTS_PER_REV;
    if (pos < 0) pos += TC_COUNTS_PER_REV;
    return pos;
}

/** Compute the shortest signed distance from current position to target
 *  position on a circular turret (range -17999 .. +18000). */
static int32_t tc_shortest_path(int32_t from, int32_t to) {
    int32_t diff = tc_normalise(to) - tc_normalise(from);
    if (diff > TC_COUNTS_PER_REV / 2) {
        diff -= TC_COUNTS_PER_REV;
    } else if (diff < -(TC_COUNTS_PER_REV / 2)) {
        diff += TC_COUNTS_PER_REV;
    }
    return diff;
}

/** Determine the tool number (1-6) for a given normalised position.
 *  Returns 0 if the position is not close enough to any pocket centre. */
static uint8_t tc_tool_at_position(int32_t pos) {
    pos = tc_normalise(pos);
    for (uint8_t i = 0; i < TC_TOOL_COUNT; i++) {
        int32_t diff = pos - toolPositions[i];
        /* Handle wrap-around for tool 1 vs tool 6 boundary */
        if (diff > TC_COUNTS_PER_REV / 2) diff -= TC_COUNTS_PER_REV;
        if (diff < -(TC_COUNTS_PER_REV / 2)) diff += TC_COUNTS_PER_REV;
        /* Allow a small window (+-50 counts) to account for settling */
        if (diff >= -50 && diff <= 50) {
            return (uint8_t)(i + 1);
        }
    }
    return 0;  /* not at a known tool pocket */
}

/* ---------- public API ---------- */

void ToolChanger_Initialize(void) {
    /* NOTE: Motor hardware setup (MotorModeSet, HLFB, VelMax, AccelMax)
     * is done early in main() before opener_init() -- per ClearCore
     * requirements.  This function only resets the state machine. */

    /* Initial state */
    tc_state              = TC_STATE_DISABLED;
    tc_current_tool       = 0;
    tc_target_tool        = 0;
    tc_fault_code         = TC_FAULT_NONE;
    tc_info_code          = TC_INFO_NONE;
    tc_home_done          = 0;
    tc_motor_abs_pos      = 0;
    tc_home_phase         = 0;
    tc_prev_di8           = 0;
    tc_prev_a9            = 0;
    tc_estop_active       = ConnectorA10.State() ? 0 : 1;  /* read initial state */
    tc_home_start_ms      = 0;
    tc_enable_request_ms  = 0;
    tc_awaiting_hlfb      = 0;
    tc_move_start_ms      = 0;
    tc_hlfb_drop_ms       = 0;
    tc_fault_clear_active = 0;

    TC_LOG("TC_Init: State machine reset "
                      "(DI-8 = enable, A-9 = fault reset, A-10 = E-stop [NC], "
                      "E-stop %s)\n",
                      tc_estop_active ? "ACTIVE" : "clear");
}

void ToolChanger_EnableMotor(int enable) {
    if (enable) {
        TC_MOTOR.EnableRequest(true);
        tc_awaiting_hlfb     = 1;
        tc_enable_request_ms = GetMillis();
        TC_LOG("TC: Motor enable requested, "
                          "awaiting HLFB (timeout %lu ms)\n",
                          (unsigned long)TC_HLFB_ENABLE_TIMEOUT_MS);
    } else {
        /* Stop any in-progress motion before disabling */
        if (tc_state == TC_STATE_MOVING || tc_state == TC_STATE_HOMING) {
            TC_MOTOR.MoveStopAbrupt();
        }
        TC_MOTOR.EnableRequest(false);
        tc_awaiting_hlfb = 0;  /* no longer waiting for HLFB */

        /* Preserve FAULTED state so the user must explicitly clear it.
         * Only transition to DISABLED if we weren't already faulted. */
        if (tc_state != TC_STATE_FAULTED) {
            tc_state = TC_STATE_DISABLED;
            tc_home_done = 0;
        }
        TC_LOG("TC: Motor disabled (state=%d)\n", tc_state);
    }
}

int ToolChanger_IsMotorReady(void) {
    return (TC_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED &&
            !TC_MOTOR.StatusReg().bit.AlertsPresent) ? 1 : 0;
}

int ToolChanger_GetMoveDirection(void) {
    return (int)tc_move_dir;
}

int ToolChanger_GetHlfbState(void) {
    MotorDriver::HlfbStates s = TC_MOTOR.HlfbState();
    if (s == MotorDriver::HLFB_ASSERTED)        return 1;
    if (s == MotorDriver::HLFB_HAS_MEASUREMENT) return 2;
    return 0;  /* HLFB_DEASSERTED or HLFB_UNKNOWN */
}

int ToolChanger_GetTorquePercent(void) {
    /* HlfbPercent() returns -100.0 to +100.0 when in HLFB_HAS_MEASUREMENT */
    return (int)TC_MOTOR.HlfbPercent();
}

void ToolChanger_HomeStart(void) {
    if (tc_state == TC_STATE_FAULTED) {
        TC_LOG("TC_HomeStart: Cannot home while faulted\n");
        return;
    }

    /* Motor must be enabled via DI-8 before homing can start */
    if (!ToolChanger_IsMotorReady()) {
        TC_LOG("TC_HomeStart: Motor not ready "
                          "(assert DI-8 to enable)\n");
        return;
    }

    tc_state        = TC_STATE_HOMING;
    tc_home_done    = 0;
    tc_home_start_ms = GetMillis();

    TC_MOTOR.VelMax(TC_HOME_VEL);

    if (ConnectorDI6.State()) {
        /* Already on the home sensor -- move off it first (positive direction).
         * Limited to one tool pocket width (6000 counts).  If the sensor does
         * not clear within that distance, a HOME_SENSOR_STUCK fault is raised. */
        tc_home_phase = 0;
        TC_MOTOR.Move(TC_COUNTS_PER_TOOL, MotorDriver::MOVE_TARGET_REL_END_POSN);
        TC_LOG("TC_HomeStart: Already on sensor, moving off "
                          "(max %ld counts)\n", (long)TC_COUNTS_PER_TOOL);
    } else {
        /* Not on the sensor -- seek it directly */
        tc_home_phase = 1;
        TC_MOTOR.MoveVelocity(-TC_HOME_VEL);
        TC_LOG("TC_HomeStart: Homing started, seeking DI-6 sensor\n");
    }
}

int ToolChanger_IsHomeComplete(void) {
    return tc_home_done;
}

void ToolChanger_SelectTool(uint8_t tool_number) {
    if (tool_number < 1 || tool_number > TC_TOOL_COUNT) {
        /* Invalid tool number -- set informational code but do NOT fault
         * the state machine.  The command is simply ignored. */
        tc_info_code |= TC_INFO_INVALID_TOOL;
        TC_LOG("TC_SelectTool: Ignored invalid tool %d "
                          "(must be 1-%d)\n", tool_number, TC_TOOL_COUNT);
        return;
    }

    /* Valid tool requested -- clear the invalid-tool info bit */
    tc_info_code &= (uint8_t)~TC_INFO_INVALID_TOOL;

    if (tc_state != TC_STATE_IDLE && tc_state != TC_STATE_AT_TOOL) {
        tc_info_code |= TC_INFO_NOT_READY;
        TC_LOG("TC_SelectTool: Not ready (state=%d)\n", tc_state);
        return;
    }

    /* Command accepted -- clear all info codes */
    tc_info_code = TC_INFO_NONE;

    tc_target_tool = tool_number;

    /* Calculate shortest rotary path */
    int32_t target_pos = toolPositions[tool_number - 1];
    int32_t current_normalised = tc_normalise(tc_motor_abs_pos);
    int32_t delta = tc_shortest_path(current_normalised, target_pos);

    if (delta == 0) {
        /* Already at the requested tool */
        tc_state = TC_STATE_AT_TOOL;
        tc_current_tool = tool_number;
        TC_LOG("TC_SelectTool: Already at tool %d\n", tool_number);
        return;
    }

    /* Restore full-speed velocity and issue relative move */
    TC_MOTOR.VelMax(TC_VEL_MAX);
    TC_MOTOR.AccelMax(TC_ACCEL_MAX);
    TC_MOTOR.Move(delta, MotorDriver::MOVE_TARGET_REL_END_POSN);

    tc_motor_abs_pos += delta;
    tc_move_dir      = (delta > 0) ? 1 : -1;
    tc_state         = TC_STATE_MOVING;
    tc_move_start_ms = GetMillis();

    TC_LOG("TC_SelectTool: Moving to tool %d (delta=%ld counts, dir=%s)\n",
                      tool_number, (long)delta, delta > 0 ? "CW" : "CCW");
}

void ToolChanger_Cyclic(void) {
    /* --- A-10 E-stop monitoring (normally closed, highest priority) ---
     * NC circuit: HIGH = normal operation, LOW = E-stop active or wire broken.
     * When active: immediately stop all motion and disable the motor.
     * While active: block all motor enable attempts.
     * When released: system stays DISABLED, DI-8 must be cycled to re-enable. */
    {
        int prev_estop = tc_estop_active;
        tc_estop_active = ConnectorA10.State() ? 0 : 1;  /* NC: LOW = active */

        if (tc_estop_active && !prev_estop) {
            /* E-stop just activated */
            if (tc_state == TC_STATE_MOVING || tc_state == TC_STATE_HOMING) {
                TC_MOTOR.MoveStopAbrupt();
            }
            TC_MOTOR.EnableRequest(false);
            tc_awaiting_hlfb = 0;
            tc_move_dir      = 0;

            if (tc_state != TC_STATE_FAULTED) {
                tc_state     = TC_STATE_DISABLED;
                tc_home_done = 0;
            }
            TC_LOG("TC_Cyclic: E-STOP ACTIVE (A-10 low) "
                              "-> motor disabled\n");
        } else if (!tc_estop_active && prev_estop) {
            /* E-stop released -- stay DISABLED, require DI-8 cycle */
            TC_LOG("TC_Cyclic: E-stop released (A-10 high), "
                              "cycle DI-8 to re-enable\n");
            /* Force DI-8 edge tracker low so the next DI-8 high is seen
             * as a rising edge even if DI-8 was held high during E-stop. */
            tc_prev_di8 = 0;
        }
    }

    /* --- DI-8 hardware enable monitoring ---
     * DI-8 is the physical motor enable input.  Rising edge enables the
     * motor (unless faulted or E-stopped), falling edge immediately disables
     * with an emergency stop.  Monitoring is paused during the fault-clear
     * sequence so the enable can be cycled without DI-8 interference. */
    if (!tc_fault_clear_active) {
        int di8 = ConnectorDI8.State() ? 1 : 0;

        if (di8 && !tc_prev_di8) {
            /* Rising edge on DI-8 */
            if (tc_estop_active) {
                TC_LOG("TC_Cyclic: DI-8 rising edge ignored "
                                  "(E-stop active)\n");
            } else if (tc_state == TC_STATE_DISABLED) {
                ToolChanger_EnableMotor(1);
                TC_LOG("TC_Cyclic: DI-8 high -> motor enabled\n");
            } else if (tc_state == TC_STATE_FAULTED) {
                TC_LOG("TC_Cyclic: DI-8 rising edge ignored "
                                  "(clear faults first)\n");
            }
        } else if (!di8 && tc_prev_di8) {
            /* Falling edge on DI-8 -- emergency stop and disable */
            if (tc_state == TC_STATE_MOVING || tc_state == TC_STATE_HOMING) {
                TC_MOTOR.MoveStopAbrupt();
            }
            TC_MOTOR.EnableRequest(false);
            tc_move_dir = 0;

            if (tc_state != TC_STATE_FAULTED) {
                tc_state     = TC_STATE_DISABLED;
                tc_home_done = 0;
            }
            /* If faulted, hardware is disabled but fault state is preserved
             * so the user must still issue a Clear Faults command. */
            TC_LOG("TC_Cyclic: DI-8 low -> motor disabled\n");
        }

        tc_prev_di8 = di8;
    }

    /* --- A-9 physical fault reset button (rising-edge) ---
     * Cooperates with the EIP Clear Faults command -- both paths call
     * the same ToolChanger_ClearFaults() function. */
    {
        int a9 = ConnectorA9.State() ? 1 : 0;
        if (a9 && !tc_prev_a9 && tc_state == TC_STATE_FAULTED) {
            TC_LOG("TC_Cyclic: A-9 button pressed -> "
                              "clearing faults\n");
            ToolChanger_ClearFaults();
        }
        tc_prev_a9 = a9;
    }

    /* --- Post-enable HLFB timeout (Scenario 1) ---
     * After EnableRequest(true), HLFB should assert within
     * TC_HLFB_ENABLE_TIMEOUT_MS.  If it doesn't, the motor may not
     * be connected, wired incorrectly, or have a drive-level fault. */
    if (tc_awaiting_hlfb && tc_state == TC_STATE_DISABLED) {
        if (ToolChanger_IsMotorReady()) {
            tc_awaiting_hlfb = 0;
            TC_LOG("TC_Cyclic: HLFB asserted after enable "
                              "(%lu ms)\n",
                              (unsigned long)(GetMillis() - tc_enable_request_ms));
        } else if ((GetMillis() - tc_enable_request_ms) >=
                   TC_HLFB_ENABLE_TIMEOUT_MS) {
            tc_awaiting_hlfb = 0;
            TC_MOTOR.EnableRequest(false);
            tc_fault_code |= TC_FAULT_HLFB_TIMEOUT;
            tc_state = TC_STATE_FAULTED;
            TC_LOG("TC_Cyclic: HLFB TIMEOUT after enable "
                              "-- HLFB did not assert within %lu ms\n",
                              (unsigned long)TC_HLFB_ENABLE_TIMEOUT_MS);
        }
    }

    /* Check for ClearPath motor faults regardless of state */
    if (TC_MOTOR.StatusReg().bit.AlertsPresent && tc_state != TC_STATE_FAULTED) {
        tc_fault_code |= TC_FAULT_MOTOR_ALERT;
        tc_state = TC_STATE_FAULTED;
        TC_LOG("TC_Cyclic: Motor alert detected! Faulted.\n");
        return;
    }

    /* Update normalised position */
    tc_position = tc_normalise(tc_motor_abs_pos);

    switch (tc_state) {
        case TC_STATE_DISABLED:
            /* Nothing to do -- waiting for enable + home command */
            break;

        case TC_STATE_HOMING:
            if (tc_home_phase == 0) {
                /* Phase 0: moving OFF the sensor (positive direction),
                 * bounded to one tool pocket width (6000 counts). */
                if (!ConnectorDI6.State()) {
                    /* Cleared the sensor -- stop the bounded move and
                     * reverse to seek the sensor edge slowly. */
                    TC_MOTOR.MoveStopAbrupt();
                    tc_home_phase = 1;
                    TC_MOTOR.MoveVelocity(-TC_HOME_VEL);
                    TC_LOG("TC_Cyclic: Cleared home sensor, "
                                      "now seeking edge\n");
                } else if (TC_MOTOR.StepsComplete() &&
                           TC_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
                    /* Moved a full pocket width and sensor is still active
                     * -- the sensor is stuck or wired incorrectly. */
                    TC_MOTOR.MoveStopAbrupt();
                    tc_fault_code |= TC_FAULT_HOME_SENSOR_STUCK;
                    tc_state = TC_STATE_FAULTED;
                    TC_LOG("TC_Cyclic: HOME SENSOR STUCK -- "
                                      "DI-6 did not clear within %ld counts\n",
                                      (long)TC_COUNTS_PER_TOOL);
                }
            } else {
                /* Phase 1: seeking the sensor (negative direction).
                 * Wait until DI-6 goes active. */
                if (ConnectorDI6.State()) {
                    /* Stop the motor immediately */
                    TC_MOTOR.MoveStopAbrupt();

                    /* Zero position -- this is the home reference */
                    tc_motor_abs_pos = 0;
                    tc_position      = 0;
                    tc_current_tool  = 1;  /* Home position = Tool 1 */
                    tc_home_done     = 1;

                    /* Restore full-speed limits */
                    TC_MOTOR.VelMax(TC_VEL_MAX);
                    TC_MOTOR.AccelMax(TC_ACCEL_MAX);

                    tc_state = TC_STATE_IDLE;
                    TC_LOG("TC_Cyclic: Home sensor detected. "
                                      "Position zeroed. At Tool 1.\n");
                }
            }

            /* Overall homing timeout -- covers both phases.
             * At homing speed (2000 pulses/sec) a full revolution takes
             * 18 seconds, so 30 s is generous.
             * Guard: only fire if we're still in HOMING (the phase logic
             * above may have already transitioned to IDLE or FAULTED). */
            if (tc_state == TC_STATE_HOMING &&
                (GetMillis() - tc_home_start_ms) >= TC_HOME_TIMEOUT_MS) {
                TC_MOTOR.MoveStopAbrupt();
                tc_fault_code |= TC_FAULT_HOME_FAILED;
                tc_state = TC_STATE_FAULTED;
                TC_LOG("TC_Cyclic: HOMING TIMEOUT -- "
                                  "sensor not found within %lu ms\n",
                                  (unsigned long)TC_HOME_TIMEOUT_MS);
            }
            break;

        case TC_STATE_IDLE:
            /* Ready and waiting for a tool select command */
            tc_current_tool = tc_tool_at_position(tc_position);

            /* Unexpected HLFB loss (Scenario 3) -- HLFB should be solidly
             * asserted while idle.  Debounce to avoid noise false triggers. */
            if (TC_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED) {
                if (tc_hlfb_drop_ms == 0) {
                    tc_hlfb_drop_ms = GetMillis();
                } else if ((GetMillis() - tc_hlfb_drop_ms) >=
                           TC_HLFB_DROP_DEBOUNCE_MS) {
                    tc_fault_code |= TC_FAULT_HLFB_TIMEOUT;
                    tc_state = TC_STATE_FAULTED;
                    TC_LOG("TC_Cyclic: HLFB LOST in IDLE "
                                      "-- deasserted for >%lu ms\n",
                                      (unsigned long)TC_HLFB_DROP_DEBOUNCE_MS);
                }
            } else {
                tc_hlfb_drop_ms = 0;
            }
            break;

        case TC_STATE_MOVING:
            /* Check if the move has completed */
            if (TC_MOTOR.StepsComplete() &&
                TC_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {

                tc_position     = tc_normalise(tc_motor_abs_pos);
                tc_current_tool = tc_tool_at_position(tc_position);

                if (tc_current_tool == tc_target_tool) {
                    tc_state    = TC_STATE_AT_TOOL;
                    tc_move_dir = 0;
                    TC_LOG("TC_Cyclic: Arrived at tool %d "
                                      "(pos=%ld)\n",
                                      tc_current_tool, (long)tc_position);
                } else {
                    /* Position error -- not at expected tool */
                    tc_fault_code |= TC_FAULT_POSITION_ERROR;
                    tc_state    = TC_STATE_FAULTED;
                    tc_move_dir = 0;
                    TC_LOG("TC_Cyclic: Position error! "
                                      "Expected tool %d, at tool %d (pos=%ld)\n",
                                      tc_target_tool, tc_current_tool,
                                      (long)tc_position);
                }
            }

            /* Move completion timeout (Scenario 2).
             * Guard: only fire if still in MOVING (move may have
             * succeeded or faulted above on this same scan). */
            if (tc_state == TC_STATE_MOVING &&
                (GetMillis() - tc_move_start_ms) >= TC_MOVE_TIMEOUT_MS) {
                TC_MOTOR.MoveStopAbrupt();
                tc_fault_code |= TC_FAULT_HLFB_TIMEOUT;
                tc_state    = TC_STATE_FAULTED;
                tc_move_dir = 0;
                TC_LOG("TC_Cyclic: MOVE TIMEOUT -- "
                                  "move did not complete within %lu ms\n",
                                  (unsigned long)TC_MOVE_TIMEOUT_MS);
            }
            break;

        case TC_STATE_AT_TOOL:
            /* Stable at tool pocket -- nothing to do until next command.
             * Unexpected HLFB loss (Scenario 3) -- same check as IDLE. */
            if (TC_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED) {
                if (tc_hlfb_drop_ms == 0) {
                    tc_hlfb_drop_ms = GetMillis();
                } else if ((GetMillis() - tc_hlfb_drop_ms) >=
                           TC_HLFB_DROP_DEBOUNCE_MS) {
                    tc_fault_code |= TC_FAULT_HLFB_TIMEOUT;
                    tc_state = TC_STATE_FAULTED;
                    TC_LOG("TC_Cyclic: HLFB LOST in AT_TOOL "
                                      "-- deasserted for >%lu ms\n",
                                      (unsigned long)TC_HLFB_DROP_DEBOUNCE_MS);
                }
            } else {
                tc_hlfb_drop_ms = 0;
            }
            break;

        case TC_STATE_FAULTED:
            /* Latched fault -- waiting for ClearFaults command */
            break;
    }
}

void ToolChanger_ClearFaults(void) {
    /* Pause DI-8 monitoring while we cycle the enable to clear faults.
     * This prevents the DI-8 handler from fighting our enable toggle. */
    tc_fault_clear_active = 1;

    /* 1. Stop any motion that might still be in progress */
    TC_MOTOR.MoveStopAbrupt();

    /* 2. Always cycle the motor enable -- this is required to clear
     *    ClearPath alerts, and is harmless for non-motor faults.
     *    Keep delays minimal to avoid starving the EIP stack.
     *    ClearPath minimum disable pulse is ~1ms; 2ms+1ms is safe. */
    TC_MOTOR.EnableRequest(false);
    Delay_ms(2);
    TC_MOTOR.ClearAlerts();
    Delay_ms(1);

    /* 3. Reset all tool changer state */
    tc_fault_code    = TC_FAULT_NONE;
    tc_info_code     = TC_INFO_NONE;
    tc_state         = TC_STATE_DISABLED;
    tc_home_done     = 0;
    tc_home_phase    = 0;
    tc_move_dir      = 0;
    tc_awaiting_hlfb = 0;
    tc_hlfb_drop_ms  = 0;

    /* 4. Re-enable motor only if DI-8 is asserted AND E-stop is clear.
     *    Synchronise the DI-8 edge tracker either way.
     *    If re-enabling, start the HLFB assertion timeout. */
    int di8 = ConnectorDI8.State() ? 1 : 0;
    tc_estop_active = ConnectorA10.State() ? 0 : 1;  /* refresh E-stop */
    if (di8 && !tc_estop_active) {
        TC_MOTOR.EnableRequest(true);
        tc_awaiting_hlfb     = 1;
        tc_enable_request_ms = GetMillis();
    }
    tc_prev_di8 = di8;

    tc_fault_clear_active = 0;

    TC_LOG("TC_ClearFaults: Faults cleared, "
                      "state -> DISABLED, DI-8=%d, E-stop=%s, motor %s\n",
                      di8, tc_estop_active ? "ACTIVE" : "clear",
                      (di8 && !tc_estop_active) ? "re-enabled" : "disabled");
}

ToolChangerState ToolChanger_GetState(void) {
    return tc_state;
}

extern "C" { extern volatile int g_eip_scanner_connected; }
int EIP_IsScannerConnected(void) {
    return g_eip_scanner_connected;
}

uint8_t ToolChanger_GetCurrentTool(void) {
    return tc_current_tool;
}

uint8_t ToolChanger_GetTargetTool(void) {
    return tc_target_tool;
}

int32_t ToolChanger_GetPosition(void) {
    return tc_position;
}

int ToolChanger_IsMoving(void) {
    return (tc_state == TC_STATE_MOVING || tc_state == TC_STATE_HOMING) ? 1 : 0;
}

uint16_t ToolChanger_GetFaultCode(void) {
    return tc_fault_code;
}

uint8_t ToolChanger_GetInfoCode(void) {
    return tc_info_code;
}

int ToolChanger_IsEstopActive(void) {
    return tc_estop_active;
}

}  /* extern "C" -- end of tool changer block */

/* =========================================================================
 * NVM / EEPROM helpers
 * ========================================================================= */

int ClearCoreEepromRead(uint16_t address, uint8_t *data, size_t length) {
    if (data == NULL || length == 0) {
        return -1;
    }
    
    if (address + length > 8192) {
        return -1;
    }
    
    ClearCore::NvmManager &nvm = ClearCore::NvmManager::Instance();
    nvm.BlockRead((ClearCore::NvmManager::NvmLocations)address, (int)length, data);
    
    return 0;
}

int ClearCoreEepromWrite(uint16_t address, const uint8_t *data, size_t length) {
    if (data == NULL || length == 0) {
        return -1;
    }
    
    if (address + length > 8192) {
        return -1;
    }
    
    ClearCore::NvmManager &nvm = ClearCore::NvmManager::Instance();
    bool success = nvm.BlockWrite((ClearCore::NvmManager::NvmLocations)address, (int)length, data);
    
    return success ? 0 : -1;
}

extern "C" {
EipStatus IfaceApplyConfiguration(TcpIpInterface *iface, CipTcpIpObject *tcpip) {
    if (iface == NULL || tcpip == NULL) {
        return kEipStatusError;
    }
    
    OPENER_TRACE_INFO("IfaceApplyConfiguration: CALLED - This should only happen at startup!\n");
    
    CipDword config_method = tcpip->config_control & kTcpipCfgCtrlMethodMask;
    
    if (config_method == kTcpipCfgCtrlDhcp) {
        OPENER_TRACE_INFO("IfaceApplyConfiguration: Switching to DHCP mode\n");
        bool dhcp_success = EthernetMgr.DhcpBegin();
        if (!dhcp_success) {
            OPENER_TRACE_ERR("IfaceApplyConfiguration: DHCP failed\n");
            return kEipStatusError;
        }
    } else if (config_method == kTcpipCfgCtrlStaticIp) {
        OPENER_TRACE_INFO("IfaceApplyConfiguration: Setting static IP configuration\n");
        
        CipUdint ip_addr = ntohl(tcpip->interface_configuration.ip_address);
        CipUdint netmask = ntohl(tcpip->interface_configuration.network_mask);
        CipUdint gateway = ntohl(tcpip->interface_configuration.gateway);
        
        IpAddress ip = IpAddress((ip_addr >> 24) & 0xFF, 
                                 (ip_addr >> 16) & 0xFF,
                                 (ip_addr >> 8) & 0xFF,
                                 ip_addr & 0xFF);
        IpAddress nm = IpAddress((netmask >> 24) & 0xFF,
                                 (netmask >> 16) & 0xFF,
                                 (netmask >> 8) & 0xFF,
                                 netmask & 0xFF);
        IpAddress gw = IpAddress((gateway >> 24) & 0xFF,
                                 (gateway >> 16) & 0xFF,
                                 (gateway >> 8) & 0xFF,
                                 gateway & 0xFF);
        
        EthernetMgr.LocalIp(ip);
        EthernetMgr.NetmaskIp(nm);
        EthernetMgr.GatewayIp(gw);
        EthernetMgr.Setup();
        
        ip4_addr_t ip4_addr, ip4_netmask, ip4_gateway;
        IP4_ADDR(&ip4_addr, (ip_addr >> 24) & 0xFF, (ip_addr >> 16) & 0xFF, 
                 (ip_addr >> 8) & 0xFF, ip_addr & 0xFF);
        IP4_ADDR(&ip4_netmask, (netmask >> 24) & 0xFF, (netmask >> 16) & 0xFF,
                 (netmask >> 8) & 0xFF, netmask & 0xFF);
        IP4_ADDR(&ip4_gateway, (gateway >> 24) & 0xFF, (gateway >> 16) & 0xFF,
                 (gateway >> 8) & 0xFF, gateway & 0xFF);
        
#if LWIP_NETIF_API && LWIP_IPV4
        netifapi_netif_set_addr(iface, &ip4_addr, &ip4_netmask, &ip4_gateway);
#else
        netif_set_addr(iface, &ip4_addr, &ip4_netmask, &ip4_gateway);
#endif
        
        OPENER_TRACE_INFO("IfaceApplyConfiguration: Static IP set to %d.%d.%d.%d\n",
                          (ip_addr >> 24) & 0xFF, (ip_addr >> 16) & 0xFF,
                          (ip_addr >> 8) & 0xFF, ip_addr & 0xFF);
    }
    
    if (tcpip->hostname.string != NULL && tcpip->hostname.length > 0) {
#if LWIP_NETIF_HOSTNAME
        /* Always copy into a safe buffer to guarantee null termination
         * without reading past the declared hostname length. */
        static char hostname_buffer[65];
        size_t copy_len = (tcpip->hostname.length < 64) ? tcpip->hostname.length : 64;
        memcpy(hostname_buffer, tcpip->hostname.string, copy_len);
        hostname_buffer[copy_len] = '\0';
        netif_set_hostname(iface, hostname_buffer);
        OPENER_TRACE_INFO("IfaceApplyConfiguration: Hostname set to '%s'\n", hostname_buffer);
#endif
    }
    
    tcpip->status &= ~kTcpipStatusIfaceCfgPend;
    
    return kEipStatusOk;
}

void ClearCoreRebootDevice(void) {
    OPENER_TRACE_INFO("ClearCoreRebootDevice: Rebooting device...\n");
    ConnectorUsb.Flush();
    Delay_ms(100);
    NVIC_SystemReset();
}

void ClearCoreClearNvram(void) {
    OPENER_TRACE_INFO("ClearCoreClearNvram: Clearing NVRAM...\n");
    
    uint8_t zero_magic[4] = {0, 0, 0, 0};
    ClearCore::NvmManager &nvm = ClearCore::NvmManager::Instance();
    
    bool success = nvm.BlockWrite((ClearCore::NvmManager::NvmLocations)0x0100, 4, zero_magic);
    if (success) {
        OPENER_TRACE_INFO("ClearCoreClearNvram: NVRAM cleared successfully\n");
    } else {
        OPENER_TRACE_ERR("ClearCoreClearNvram: Failed to clear NVRAM\n");
    }
    
    ConnectorUsb.Flush();
}
}

#endif

