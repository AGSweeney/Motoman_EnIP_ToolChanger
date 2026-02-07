/*
 * DX200 Rotary Tool Changer -- MAX7219 7-Segment Display Driver
 *
 * Hardware SPI on COM-1.  See max7219_display.h for API documentation.
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#include "ClearCore.h"
#include "max7219_display.h"

/* =========================================================================
 * MAX7219 register addresses
 * ========================================================================= */
#define MAX7219_REG_NOOP         0x00
#define MAX7219_REG_DIGIT0       0x01
#define MAX7219_REG_DIGIT1       0x02
#define MAX7219_REG_DIGIT2       0x03
#define MAX7219_REG_DIGIT3       0x04
#define MAX7219_REG_DIGIT4       0x05
#define MAX7219_REG_DIGIT5       0x06
#define MAX7219_REG_DIGIT6       0x07
#define MAX7219_REG_DIGIT7       0x08
#define MAX7219_REG_DECODE_MODE  0x09
#define MAX7219_REG_INTENSITY    0x0A
#define MAX7219_REG_SCAN_LIMIT   0x0B
#define MAX7219_REG_SHUTDOWN     0x0C
#define MAX7219_REG_DISPLAY_TEST 0x0F

/* BCD decode special values */
#define MAX7219_BCD_DASH    0x0A
#define MAX7219_BCD_BLANK   0x0F

/* Decimal-point flag (OR with digit data) */
#define MAX7219_DP           0x80

/* =========================================================================
 * Raw segment patterns for non-BCD characters.
 *
 * Segment mapping:     a
 *                    -----
 *                   |     |
 *                 f |     | b
 *                   |  g  |
 *                    -----
 *                   |     |
 *                 e |     | c
 *                   |     |
 *                    -----  . dp
 *                      d
 *
 * Bit layout: dp-a-b-c-d-e-f-g  (bit 7 = dp, bit 6 = a, ... bit 0 = g)
 * ========================================================================= */
#define SEG_A  0x40
#define SEG_B  0x20
#define SEG_C  0x10
#define SEG_D  0x08
#define SEG_E  0x04
#define SEG_F  0x02
#define SEG_G  0x01

/* Letter approximations for state display */
static const uint8_t SEG_CHAR_H = SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;          /* H */
static const uint8_t SEG_CHAR_O = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;  /* O (= 0) */
static const uint8_t SEG_CHAR_I = SEG_E | SEG_F;                                    /* I (= 1-like) */
static const uint8_t SEG_CHAR_d = SEG_B | SEG_C | SEG_D | SEG_E | SEG_G;           /* d */
static const uint8_t SEG_CHAR_A = SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;  /* A */
static const uint8_t SEG_CHAR_t = SEG_D | SEG_E | SEG_F | SEG_G;                   /* t */
static const uint8_t SEG_CHAR_F = SEG_A | SEG_E | SEG_F | SEG_G;                   /* F */
static const uint8_t SEG_CHAR_S = SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;           /* S (= 5) */
static const uint8_t SEG_CHAR_DASH = SEG_G;                                         /* - */
static const uint8_t SEG_CHAR_BLANK = 0x00;                                          /* blank */

/* =========================================================================
 * Module state
 * ========================================================================= */
static uint8_t g_digit_count  = 1;
static bool    g_bcd_mode     = true;  /* true = BCD decode active */

/* =========================================================================
 * Low-level SPI transfer
 * ========================================================================= */

/**
 * Send a 16-bit command (register + data) to the MAX7219.
 * COM-1's built-in SS line handles chip-select automatically.
 */
void Display_WriteReg(uint8_t reg, uint8_t data) {
    ConnectorCOM1.SpiTransferData(reg);
    ConnectorCOM1.SpiTransferData(data);

    /* The MAX7219 latches data on CS rising edge.  Toggle SS by
     * sending a brief deselect/reselect cycle.  The SpiSsMode is
     * set to LINE_ON which keeps SS asserted during transfers.
     * We toggle it manually between commands. */
    ConnectorCOM1.SpiSsMode(SerialBase::LINE_OFF);
    ConnectorCOM1.SpiSsMode(SerialBase::LINE_ON);
}

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * Switch between BCD decode mode and raw segment mode.
 * BCD mode: digits 0-9, dash, blank via 4-bit codes.
 * Raw mode: direct segment control via 8-bit patterns.
 */
static void display_set_decode(bool bcd) {
    if (bcd == g_bcd_mode) return;
    g_bcd_mode = bcd;
    Display_WriteReg(MAX7219_REG_DECODE_MODE, bcd ? 0xFF : 0x00);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void Display_Init(uint8_t digit_count) {
    if (digit_count < 1) digit_count = 1;
    if (digit_count > 8) digit_count = 8;
    g_digit_count = digit_count;

    /* Configure COM-1 for SPI */
    ConnectorCOM1.Mode(Connector::SPI);
    ConnectorCOM1.Speed(1000000);  /* 1 MHz -- well within MAX7219's 10 MHz limit */
    ConnectorCOM1.DataOrder(SerialBase::COM_MSB_FIRST);
    ConnectorCOM1.SpiClock(SerialBase::SCK_LOW, SerialBase::LEAD_SAMPLE);  /* Mode 0 */
    ConnectorCOM1.SpiSsMode(SerialBase::LINE_ON);
    ConnectorCOM1.PortOpen();

    /* Bring MAX7219 out of shutdown */
    Display_WriteReg(MAX7219_REG_SHUTDOWN, 0x01);

    /* Disable display test mode */
    Display_WriteReg(MAX7219_REG_DISPLAY_TEST, 0x00);

    /* Set scan limit to number of digits - 1 */
    Display_WriteReg(MAX7219_REG_SCAN_LIMIT, digit_count - 1);

    /* Enable BCD decode on all digits */
    Display_WriteReg(MAX7219_REG_DECODE_MODE, 0xFF);
    g_bcd_mode = true;

    /* Medium brightness */
    Display_WriteReg(MAX7219_REG_INTENSITY, 0x07);

    /* Clear all digit registers */
    Display_Clear();
}

void Display_ShowNumber(int32_t value) {
    display_set_decode(true);

    bool negative = false;
    if (value < 0) {
        negative = true;
        value = -value;
    }

    /* Extract digits right-to-left */
    for (uint8_t i = 0; i < g_digit_count; i++) {
        uint8_t reg = MAX7219_REG_DIGIT0 + i;

        if (i == 0 || value > 0) {
            uint8_t digit = (uint8_t)(value % 10);
            Display_WriteReg(reg, digit);
            value /= 10;
        } else if (negative) {
            Display_WriteReg(reg, MAX7219_BCD_DASH);
            negative = false;  /* only one dash */
        } else {
            Display_WriteReg(reg, MAX7219_BCD_BLANK);
        }
    }
}

void Display_SetDigit(uint8_t digit, uint8_t value, uint8_t dp) {
    if (digit >= g_digit_count) return;
    display_set_decode(true);

    uint8_t data = value & 0x0F;
    if (dp) data |= MAX7219_DP;
    Display_WriteReg(MAX7219_REG_DIGIT0 + digit, data);
}

void Display_Clear(void) {
    display_set_decode(true);
    for (uint8_t i = 0; i < 8; i++) {
        Display_WriteReg(MAX7219_REG_DIGIT0 + i, MAX7219_BCD_BLANK);
    }
}

void Display_SetBrightness(uint8_t intensity) {
    if (intensity > 15) intensity = 15;
    Display_WriteReg(MAX7219_REG_INTENSITY, intensity);
}

void Display_Enable(int on) {
    Display_WriteReg(MAX7219_REG_SHUTDOWN, on ? 0x01 : 0x00);
}

void Display_ShowDashes(void) {
    display_set_decode(true);
    for (uint8_t i = 0; i < g_digit_count; i++) {
        Display_WriteReg(MAX7219_REG_DIGIT0 + i, MAX7219_BCD_DASH);
    }
}

void Display_ShowState(const char *code) {
    if (code == 0 || g_digit_count < 2) return;

    /* Switch to raw segment mode for letter rendering */
    display_set_decode(false);

    /* Map two-character codes to segment patterns.
     * Digit 1 (tens) = left character, Digit 0 (ones) = right character. */
    uint8_t left = SEG_CHAR_BLANK;
    uint8_t right = SEG_CHAR_BLANK;

    char c0 = code[0];
    char c1 = code[1];

    /* Left character */
    switch (c0) {
        case 'H': case 'h': left = SEG_CHAR_H; break;
        case 'I': case 'i': left = SEG_CHAR_I; break;
        case 'A': case 'a': left = SEG_CHAR_A; break;
        case 'F': case 'f': left = SEG_CHAR_F; break;
        case 'D': case 'd': left = SEG_CHAR_d; break;
        case 'S': case 's': left = SEG_CHAR_S; break;
        case '-':           left = SEG_CHAR_DASH; break;
        case ' ':           left = SEG_CHAR_BLANK; break;
        default:            left = SEG_CHAR_DASH; break;
    }

    /* Right character */
    switch (c1) {
        case 'O': case 'o': right = SEG_CHAR_O; break;
        case 'D': case 'd': right = SEG_CHAR_d; break;
        case 'T': case 't': right = SEG_CHAR_t; break;
        case 'S': case 's': right = SEG_CHAR_S; break;
        case 'A': case 'a': right = SEG_CHAR_A; break;
        case 'F': case 'f': right = SEG_CHAR_F; break;
        case '-':           right = SEG_CHAR_DASH; break;
        case ' ':           right = SEG_CHAR_BLANK; break;
        default:            right = SEG_CHAR_DASH; break;
    }

    /* Write: digit 1 = left char, digit 0 = right char */
    Display_WriteReg(MAX7219_REG_DIGIT1, left);
    Display_WriteReg(MAX7219_REG_DIGIT0, right);

    /* Blank any remaining higher digits */
    for (uint8_t i = 2; i < g_digit_count; i++) {
        Display_WriteReg(MAX7219_REG_DIGIT0 + i, SEG_CHAR_BLANK);
    }
}
