/*
 * DX200 Rotary Tool Changer -- ClearCore Hardware Wrapper API
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef CLEARCORE_WRAPPER_H_
#define CLEARCORE_WRAPPER_H_

#ifdef CLEARCORE

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * System
 * --------------------------------------------------------------------------- */
unsigned long GetMillis(void);
void ConnectorLed_SetState(int state);

/* ---------------------------------------------------------------------------
 * Tool Changer Logging -- always prints, independent of OPENER_TRACE_LEVEL.
 * Use TC_LOG(...) for tool changer state changes, faults, and actions.
 * --------------------------------------------------------------------------- */
void ClearCoreTraceOutput(const char *format, ...);
#define TC_LOG(...)  ClearCoreTraceOutput(__VA_ARGS__)

/* ---------------------------------------------------------------------------
 * Digital I/O  (IO-0 .. IO-5 outputs, DI-6..DI-8 / A-9..A-12 inputs)
 * --------------------------------------------------------------------------- */
void ConnectorIO0_Initialize(void);
void ConnectorIO0_SetState(int state);
void ConnectorIO1_Initialize(void);
void ConnectorIO1_SetState(int state);
void ConnectorIO2_Initialize(void);
void ConnectorIO2_SetState(int state);
void ConnectorIO3_Initialize(void);
void ConnectorIO3_SetState(int state);
void ConnectorIO4_Initialize(void);
void ConnectorIO4_SetState(int state);
void ConnectorIO5_Initialize(void);
void ConnectorIO5_SetState(int state);
void ConnectorDI6_Initialize(void);
int ConnectorDI6_GetState(void);
void ConnectorDI7_Initialize(void);
int ConnectorDI7_GetState(void);
void ConnectorDI8_Initialize(void);
int ConnectorDI8_GetState(void);
void ConnectorA9_Initialize(void);
int ConnectorA9_GetState(void);
void ConnectorA10_Initialize(void);
int ConnectorA10_GetState(void);
void ConnectorA11_Initialize(void);
int ConnectorA11_GetState(void);
void ConnectorA12_Initialize(void);
int ConnectorA12_GetState(void);

/* ---------------------------------------------------------------------------
 * NVM / EEPROM helpers
 * --------------------------------------------------------------------------- */
int ClearCoreEepromRead(uint16_t address, uint8_t *data, size_t length);
int ClearCoreEepromWrite(uint16_t address, const uint8_t *data, size_t length);
void ClearCoreRebootDevice(void);
void ClearCoreClearNvram(void);

/* ---------------------------------------------------------------------------
 * Tool Changer  --  ClearPath SDSK motor on M-0
 *
 *   Motor : 3600 ppr
 *   Gearbox: 10:1
 *   => 36 000 counts / turret revolution
 *   => 6 000 counts / tool pocket  (6 pockets)
 * --------------------------------------------------------------------------- */

/* Configuration constants exposed to C code */
#define TC_MOTOR_PPR              3600
#define TC_GEARBOX_RATIO          10
#define TC_COUNTS_PER_REV         (TC_MOTOR_PPR * TC_GEARBOX_RATIO)  /* 36000 */
#define TC_TOOL_COUNT             6
#define TC_COUNTS_PER_TOOL        (TC_COUNTS_PER_REV / TC_TOOL_COUNT) /* 6000 */

/* Tool changer state machine states */
typedef enum {
    TC_STATE_DISABLED  = 0,   /* Motor disabled, no motion allowed              */
    TC_STATE_HOMING    = 1,   /* Homing sequence in progress                    */
    TC_STATE_IDLE      = 2,   /* Homed and ready, no motion in progress         */
    TC_STATE_MOVING    = 3,   /* Rotating to a requested tool pocket            */
    TC_STATE_AT_TOOL   = 4,   /* At the requested tool pocket, move complete    */
    TC_STATE_FAULTED   = 5    /* Fault detected, motion inhibited               */
} ToolChangerState;

/* Fault codes reported in the input assembly */
typedef enum {
    TC_FAULT_NONE               = 0x0000,
    TC_FAULT_MOTOR_ALERT        = 0x0001,  /* ClearPath alert register active   */
    TC_FAULT_HLFB_TIMEOUT       = 0x0002,  /* HLFB did not assert in time       */
    TC_FAULT_HOME_FAILED        = 0x0004,  /* Homing did not complete            */
    TC_FAULT_POSITION_ERROR     = 0x0008,  /* Position outside expected range    */
    /* 0x0010 reserved */
    TC_FAULT_HOME_SENSOR_STUCK  = 0x0020   /* Home sensor did not clear within
                                            * one tool pocket width (6000 counts) */
} ToolChangerFault;

/* Informational codes -- reported in the input assembly but do NOT stop
 * the state machine.  Self-clearing: each bit is cleared when the
 * condition is no longer present. */
typedef enum {
    TC_INFO_NONE                = 0x00,
    TC_INFO_INVALID_TOOL        = 0x01,   /* Tool number out of range (not 1-6) */
    TC_INFO_NOT_READY           = 0x02    /* Command ignored: not in IDLE/AT_TOOL */
} ToolChangerInfo;

/* Initialise the ClearPath SDSK motor on connector M-0.
 * Sets step-and-direction mode, HLFB, velocity, and acceleration. */
void ToolChanger_Initialize(void);

/* Enable / disable the motor drive.
 * enable=1 to enable, enable=0 to disable. */
void ToolChanger_EnableMotor(int enable);

/* Returns non-zero if the motor is enabled and HLFB is asserted. */
int  ToolChanger_IsMotorReady(void);

/* Returns the move direction: +1 = CW, -1 = CCW, 0 = not moving. */
int  ToolChanger_GetMoveDirection(void);

/* Returns non-zero if an EtherNet/IP scanner has an active I/O connection. */
int  EIP_IsScannerConnected(void);

/* Returns the HLFB state: 0 = off/deasserted, 1 = asserted (servo locked),
 * 2 = has measurement (PWM torque data available during motion). */
int  ToolChanger_GetHlfbState(void);

/* Returns the measured torque as a percentage of peak torque (-100 to +100).
 * Only valid when ToolChanger_GetHlfbState() returns 2. */
int  ToolChanger_GetTorquePercent(void);

/* Start homing sequence.  Uses DI-6 as the home sensor input.
 * Homing moves the turret slowly until the sensor triggers, then
 * zeroes the position counter. */
void ToolChanger_HomeStart(void);

/* Returns non-zero once the homing sequence has finished successfully. */
int  ToolChanger_IsHomeComplete(void);

/* Command a tool change.  tool_number is 1-6.
 * The turret will rotate via the shortest path to the requested pocket. */
void ToolChanger_SelectTool(uint8_t tool_number);

/* Cyclic processing -- MUST be called every scan from the main loop.
 * Drives the state machine, monitors faults, updates status. */
void ToolChanger_Cyclic(void);

/* Clear any latched faults and transition back to DISABLED state
 * so the motor can be re-enabled and re-homed. */
void ToolChanger_ClearFaults(void);

/* --- Status query functions (safe to call from C / assembly callbacks) --- */

/* Returns the current state machine state (ToolChangerState). */
ToolChangerState ToolChanger_GetState(void);

/* Returns the tool number (1-6) that the turret is currently at,
 * or 0 if the position is unknown / between tools. */
uint8_t ToolChanger_GetCurrentTool(void);

/* Returns the tool number (1-6) that was last commanded, or 0 if none. */
uint8_t ToolChanger_GetTargetTool(void);

/* Returns the current motor position in counts (normalised 0-35999). */
int32_t ToolChanger_GetPosition(void);

/* Returns non-zero if the turret is currently in motion. */
int  ToolChanger_IsMoving(void);

/* Returns the active fault code bitmask (ToolChangerFault). */
uint16_t ToolChanger_GetFaultCode(void);

/* Returns the active informational code bitmask (ToolChangerInfo).
 * These are non-fatal warnings that do not affect the state machine. */
uint8_t ToolChanger_GetInfoCode(void);

/* Returns non-zero if the E-stop is active (A-10 normally-closed input is LOW).
 * While active, motor enable is blocked and all motion is inhibited. */
int  ToolChanger_IsEstopActive(void);

#ifdef __cplusplus
}
#endif

#endif  /* CLEARCORE */

#endif  /* CLEARCORE_WRAPPER_H_ */

