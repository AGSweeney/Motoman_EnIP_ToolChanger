/*
 * DX200 Rotary Tool Changer -- MAX7219 7-Segment Display Driver
 *
 * Drives a MAX7219-based 7-segment display over hardware SPI on COM-1.
 * Designed to show current tool pocket, state, or fault codes.
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef MAX7219_DISPLAY_H_
#define MAX7219_DISPLAY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

/**
 * Initialise COM-1 in SPI mode and configure the MAX7219.
 *
 * Sets up: SPI Mode 0 (CPOL=0, CPHA=0), MSB-first, 1 MHz clock.
 * Configures the MAX7219 for BCD decode on all digits, medium intensity,
 * scan limit matching digit_count, and display ON.
 *
 * @param digit_count  Number of physical digits on the display (1-8).
 *                     Only these digits will be scanned/updated.
 */
void Display_Init(uint8_t digit_count);

/* -------------------------------------------------------------------------
 * Display control
 * ------------------------------------------------------------------------- */

/**
 * Display a decimal integer.  Right-justified, leading digits blanked.
 * Negative values show a dash on the leftmost digit.
 *
 * @param value  Integer to display (-9 .. 99999999 for 8 digits).
 */
void Display_ShowNumber(int32_t value);

/**
 * Display a raw character on a specific digit position.
 *
 * @param digit  Digit position (0 = rightmost, 7 = leftmost).
 * @param value  BCD value (0-9) or blank (0x0F).
 * @param dp     If non-zero, the decimal point is lit.
 */
void Display_SetDigit(uint8_t digit, uint8_t value, uint8_t dp);

/**
 * Blank (clear) the entire display.
 */
void Display_Clear(void);

/**
 * Set display brightness.
 *
 * @param intensity  0 (dimmest) to 15 (brightest).
 */
void Display_SetBrightness(uint8_t intensity);

/**
 * Turn the display on or off.  When off, digit data is retained.
 *
 * @param on  Non-zero = on, zero = off.
 */
void Display_Enable(int on);

/**
 * Display a dash pattern on all digits (e.g. for fault indication).
 */
void Display_ShowDashes(void);

/**
 * Display a two-character state code on a 2+ digit display.
 * Uses segments to approximate letters where possible.
 *
 * Supported codes (case-insensitive in implementation):
 *   "HO" = Homing, "Id" = Idle, "At" = At tool,
 *   "Ft" = Faulted, "dS" = Disabled, "  " = blank
 *
 * @param code  Two-character null-terminated string.
 */
void Display_ShowState(const char *code);

/* -------------------------------------------------------------------------
 * Low-level access
 * ------------------------------------------------------------------------- */

/**
 * Write a raw register/data pair to the MAX7219.
 *
 * @param reg   Register address (0x00 - 0x0F).
 * @param data  Data byte.
 */
void Display_WriteReg(uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* MAX7219_DISPLAY_H_ */
