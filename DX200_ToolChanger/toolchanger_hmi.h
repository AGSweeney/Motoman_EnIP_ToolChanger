/*
 * DX200 Rotary Tool Changer -- Status HMI for Gen4-uLCD-70DCT-CLB
 *
 * Draws and manages the tool changer status display on a 4D Systems
 * 800x480 Diablo16 display using the SPE serial protocol.
 *
 * Layout (800 x 480 landscape):
 * ┌──────────────────── TOP BAR (800x32) ─────────────────────┐
 * │ DX200 ROTARY TOOL CHANGER      │ STATE │ EtherNet/IP      │
 * ├────────────┬──────────────────────────────────────────────┤
 * │ LEFT       │  RIGHT                                       │
 * │ Tool       │  Status Dashboard                            │
 * │ Position   │  STATE / POSITION / MOTOR                    │
 * │ Indicator  │  E-STOP / FAULT / INFO                       │
 * │ (250px)    │  (550px)                                     │
 * ├────────────┴──────────────────────────────────────────────┤
 * │ BOTTOM BAR (800x26) -- System info / Fault text           │
 * └───────────────────────────────────────────────────────────┘
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef TOOLCHANGER_HMI_H_
#define TOOLCHANGER_HMI_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Initialisation
 * ========================================================================= */

/**
 * Initialise the HMI display on COM-0 (TTL serial).
 *
 * Sets up the serial port, waits for the display to boot (~3 seconds),
 * configures landscape mode, and draws the full initial screen.
 *
 * @param baudRate  Preferred baud rate for COM-0 (actual may differ).
 */
void HMI_Init(uint32_t baudRate);

/* =========================================================================
 * Cyclic Processing
 * ========================================================================= */

/**
 * HMI cyclic update -- call every main-loop scan (~10 ms).
 *
 * Reads tool changer state via ToolChanger_*() API and updates
 * changed display elements (dirty-flag based).
 */
void HMI_Cyclic(void);

/* =========================================================================
 * Power Control
 * ========================================================================= */

/**
 * Power off hook (no-op -- IO-3 now controlled by EIP assembly).
 */
void HMI_PowerOff(void);

#ifdef __cplusplus
}
#endif

#endif /* TOOLCHANGER_HMI_H_ */
