/*
 * DX200 Rotary Tool Changer -- Diablo16 Serial/SPE Display Driver
 *
 * Implementation of the Diablo16 SPE serial protocol for ClearCore.
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#include "ClearCore.h"
#include "diablo16_serial.h"
#include <string.h>

/* =========================================================================
 * Construction & Initialisation
 * ========================================================================= */

Diablo16Serial::Diablo16Serial()
    : m_serial(nullptr)
    , m_timeoutMs(D16_DEFAULT_TIMEOUT_MS)
    , m_error(D16_ERR_OK)
    , m_lastNak(0)
    , m_nakReceived(false)
    , m_cachedFont(0xFFFF)
    , m_cachedFGcolour(0xBEEF)  /* sentinel != D16_WHITE (0xFFFF) */
    , m_cachedOpacity(0xFFFF)
    , m_cachedWidth(0xFFFF)
    , m_cachedHeight(0xFFFF)
{
}

void Diablo16Serial::Begin(ClearCore::ISerial &serial) {
    m_serial = &serial;
    m_error  = D16_ERR_OK;
    m_lastNak = 0;
    resetTextCache();
}

void Diablo16Serial::resetTextCache(void) {
    m_cachedFont     = 0xFFFF;
    m_cachedFGcolour = 0xBEEF;  /* sentinel != D16_WHITE (0xFFFF) */
    m_cachedOpacity  = 0xFFFF;
    m_cachedWidth    = 0xFFFF;
    m_cachedHeight   = 0xFFFF;
}

/* =========================================================================
 * Low-Level I/O Helpers
 * ========================================================================= */

void Diablo16Serial::sendWord(uint16_t w) {
    if (m_serial == nullptr) return;
    m_serial->SendChar((uint8_t)(w >> 8));
    m_serial->SendChar((uint8_t)(w & 0xFF));
}

void Diablo16Serial::sendCmd(uint16_t cmd) {
    sendWord(cmd);
}

void Diablo16Serial::sendCmdWord(uint16_t cmd, uint16_t w) {
    sendWord(cmd);
    sendWord(w);
}

void Diablo16Serial::sendCmdWords(uint16_t cmd, const uint16_t *words, uint16_t count) {
    sendWord(cmd);
    for (uint16_t i = 0; i < count; i++) {
        sendWord(words[i]);
    }
}

void Diablo16Serial::sendChars(const char *str) {
    if (m_serial == nullptr || str == nullptr) return;
    do {
        m_serial->SendChar((uint8_t)*str);
    } while (*str++);
    /* NOTE: loop sends the char first, then checks -- so null terminator
     * IS sent (matching the official 4D Systems WriteChars behaviour). */
}

void Diablo16Serial::drainRx(void) {
    if (m_serial == nullptr) return;
    while (m_serial->AvailableForRead() > 0) {
        m_serial->CharGet();
    }
}

void Diablo16Serial::getAckNonBlocking(void) {
    /* For draw commands: wait for the ACK/NAK byte (flow control).
     * At 9600 baud, a command takes ~1-15ms to transmit depending on
     * arg count, then the display needs time to execute the graphics
     * operation before sending ACK.  We allow up to 50ms which prevents
     * overwhelming the Diablo16 processor while keeping the main loop
     * responsive enough for Ethernet/IP. */
    if (m_serial == nullptr) return;
    m_error = D16_ERR_OK;
    m_nakReceived = false;

    uint32_t startTime = Milliseconds();
    while (Milliseconds() - startTime < 50) {
        if (m_serial->AvailableForRead() > 0) {
            uint8_t rxByte = (uint8_t)(m_serial->CharGet() & 0xFF);
            if (rxByte == D16_NAK) {
                m_nakReceived = true;
                m_lastNak = rxByte;
            }
            /* Drain any additional bytes (e.g. stale responses) */
            while (m_serial->AvailableForRead() > 0) {
                m_serial->CharGet();
            }
            return;
        }
    }
    /* Timeout -- no response within 10ms.  Not fatal; command was sent. */
}

void Diablo16Serial::getAck(void) {
    if (m_serial == nullptr) {
        m_error = D16_ERR_TIMEOUT;
        return;
    }

    /* Ensure all TX bytes have actually left the wire before polling
     * for the response.  ClearCore SendChar() queues into a ring
     * buffer -- WaitForTransmitIdle() blocks until the UART shift
     * register is empty. */
    m_serial->WaitForTransmitIdle();

    /* Wait for the ACK/NAK response byte.
     * Matches the official 4D Systems GetAck() behaviour: tolerate
     * up to 2 stale / garbage bytes before declaring an error.  This
     * lets us self-recover from a 1-byte protocol desync. */
    m_error = D16_ERR_OK;
    m_nakReceived = false;
    uint32_t startTime = Milliseconds();
    uint8_t retries = 0;

    while (Milliseconds() - startTime < m_timeoutMs) {
        if (m_serial->AvailableForRead() > 0) {
            uint8_t rxByte = (uint8_t)(m_serial->CharGet() & 0xFF);

            if (rxByte == D16_ACK) {
                m_error = D16_ERR_OK;
                return;
            }
            if (rxByte == D16_NAK) {
                /* NAK -- display rejected the command.  Flag it so
                 * getWord() uses a short timeout (no data after NAK). */
                m_error = D16_ERR_OK;
                m_nakReceived = true;
                m_lastNak = rxByte;
                return;
            }

            /* Neither ACK nor NAK -- stale data byte left from a
             * previous response.  The official 4D library skips up
             * to 2 such bytes before giving up. */
            retries++;
            if (retries >= 2) {
                m_error   = D16_ERR_NAK;
                m_lastNak = rxByte;
                return;
            }
            /* else: keep reading -- the real ACK may be next */
        }
    }

    m_error = D16_ERR_TIMEOUT;
}

uint16_t Diablo16Serial::getWord(void) {
    if (m_serial == nullptr || m_error != D16_ERR_OK) {
        return 0;
    }

    /* If the previous ACK was actually a NAK, the display won't send
     * data.  Use a very short timeout (5ms) so we don't block the
     * main loop -- at 256000 baud, 2 bytes take <0.1ms to arrive. */
    uint32_t effectiveTimeout = m_nakReceived ? 5 : m_timeoutMs;

    uint8_t buf[2];
    uint8_t count = 0;
    uint32_t startTime = Milliseconds();

    while (count < 2 && (Milliseconds() - startTime < effectiveTimeout)) {
        if (m_serial->AvailableForRead() > 0) {
            buf[count++] = (uint8_t)(m_serial->CharGet() & 0xFF);
        }
    }

    if (count < 2) {
        if (m_nakReceived) {
            /* Expected: no data after NAK.  Not a real error. */
            return 0;
        }
        m_error = D16_ERR_TIMEOUT;
        return 0;
    }

    return ((uint16_t)buf[0] << 8) | buf[1];
}

uint16_t Diablo16Serial::getAckResp(void) {
    getAck();
    return getWord();
}

uint16_t Diablo16Serial::getAckResStr(char *outStr, uint16_t maxLen) {
    getAck();
    uint16_t len = getWord();
    if (outStr != nullptr && len > 0 && m_error == D16_ERR_OK) {
        uint16_t toRead = (len < maxLen - 1) ? len : (maxLen - 1);
        getBytes((uint8_t *)outStr, toRead);
        outStr[toRead] = '\0';
        /* Discard remaining *bytes* (not words) if the string was truncated.
         * The SPE response contains single-byte characters, so each extra
         * character is one byte -- using getWord() here would consume 2x
         * too many bytes and desync the protocol. */
        for (uint16_t i = toRead; i < len && m_error == D16_ERR_OK; i++) {
            uint8_t discard;
            getBytes(&discard, 1);
        }
    }
    return len;
}

void Diablo16Serial::getBytes(uint8_t *data, uint16_t size) {
    if (m_serial == nullptr || m_error != D16_ERR_OK) return;

    uint16_t count = 0;
    uint32_t startTime = Milliseconds();

    while (count < size && (Milliseconds() - startTime < m_timeoutMs)) {
        if (m_serial->AvailableForRead() > 0) {
            data[count++] = (uint8_t)(m_serial->CharGet() & 0xFF);
        }
    }

    if (count < size) {
        m_error = D16_ERR_TIMEOUT;
    }
}

/* =========================================================================
 * Graphics Commands
 * ========================================================================= */

void Diablo16Serial::gfx_Cls(void) {
    sendCmd(F_gfx_Cls);
    getAck();
}

void Diablo16Serial::gfx_ChangeColour(uint16_t oldCol, uint16_t newCol) {
    uint16_t args[] = { oldCol, newCol };
    sendCmdWords(F_gfx_ChangeColour, args, 2);
    getAck();
}

void Diablo16Serial::gfx_Circle(uint16_t x, uint16_t y, uint16_t radius, uint16_t colour) {
    uint16_t args[] = { x, y, radius, colour };
    sendCmdWords(F_gfx_Circle, args, 4);
    getAck();
}

void Diablo16Serial::gfx_CircleFilled(uint16_t x, uint16_t y, uint16_t radius, uint16_t colour) {
    uint16_t args[] = { x, y, radius, colour };
    sendCmdWords(F_gfx_CircleFilled, args, 4);
    getAck();
}

void Diablo16Serial::gfx_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour) {
    uint16_t args[] = { x1, y1, x2, y2, colour };
    sendCmdWords(F_gfx_Line, args, 5);
    getAck();
}

void Diablo16Serial::gfx_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour) {
    uint16_t args[] = { x1, y1, x2, y2, colour };
    sendCmdWords(F_gfx_Rectangle, args, 5);
    getAck();
}

void Diablo16Serial::gfx_RectangleFilled(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour) {
    uint16_t args[] = { x1, y1, x2, y2, colour };
    sendCmdWords(F_gfx_RectangleFilled, args, 5);
    getAck();
}

void Diablo16Serial::gfx_Triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                                    uint16_t x3, uint16_t y3, uint16_t colour) {
    uint16_t args[] = { x1, y1, x2, y2, x3, y3, colour };
    sendCmdWords(F_gfx_Triangle, args, 7);
    getAck();
}

void Diablo16Serial::gfx_Ellipse(uint16_t x, uint16_t y, uint16_t xrad, uint16_t yrad, uint16_t colour) {
    uint16_t args[] = { x, y, xrad, yrad, colour };
    sendCmdWords(F_gfx_Ellipse, args, 5);
    getAck();
}

void Diablo16Serial::gfx_EllipseFilled(uint16_t x, uint16_t y, uint16_t xrad, uint16_t yrad, uint16_t colour) {
    uint16_t args[] = { x, y, xrad, yrad, colour };
    sendCmdWords(F_gfx_EllipseFilled, args, 5);
    getAck();
}

void Diablo16Serial::gfx_PutPixel(uint16_t x, uint16_t y, uint16_t colour) {
    uint16_t args[] = { x, y, colour };
    sendCmdWords(F_gfx_PutPixel, args, 3);
    getAck();
}

void Diablo16Serial::gfx_MoveTo(uint16_t x, uint16_t y) {
    uint16_t args[] = { x, y };
    sendCmdWords(F_gfx_MoveTo, args, 2);
    getAck();
}

void Diablo16Serial::gfx_LineTo(uint16_t x, uint16_t y) {
    uint16_t args[] = { x, y };
    sendCmdWords(F_gfx_LineTo, args, 2);
    getAck();
}

void Diablo16Serial::gfx_Clipping(uint16_t onOff) {
    sendCmdWord(F_gfx_Clipping, onOff);
    getAck();
}

void Diablo16Serial::gfx_ClipWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint16_t args[] = { x1, y1, x2, y2 };
    sendCmdWords(F_gfx_ClipWindow, args, 4);
    getAck();
}

void Diablo16Serial::gfx_Set(uint16_t func, uint16_t value) {
    uint16_t args[] = { func, value };
    sendCmdWords(F_gfx_Set, args, 2);
    getAck();
}

uint16_t Diablo16Serial::gfx_Get(uint16_t mode) {
    sendCmdWord(F_gfx_Get, mode);
    return getAckResp();
}

uint16_t Diablo16Serial::gfx_BGcolour(uint16_t colour) {
    sendCmdWord(F_gfx_BGcolour, colour);
    return getAckResp();
}

uint16_t Diablo16Serial::gfx_Contrast(uint16_t contrast) {
    sendCmdWord(F_gfx_Contrast, contrast);
    return getAckResp();
}

uint16_t Diablo16Serial::gfx_ScreenMode(uint16_t mode) {
    sendCmdWord(F_gfx_ScreenMode, mode);
    return getAckResp();
}

uint16_t Diablo16Serial::gfx_OutlineColour(uint16_t colour) {
    sendCmdWord(F_gfx_OutlineColour, colour);
    return getAckResp();
}

uint16_t Diablo16Serial::gfx_LinePattern(uint16_t pattern) {
    sendCmdWord(F_gfx_LinePattern, pattern);
    return getAckResp();
}

void Diablo16Serial::gfx_ScreenCopyPaste(uint16_t xs, uint16_t ys, uint16_t xd, uint16_t yd,
                                           uint16_t width, uint16_t height) {
    uint16_t args[] = { xs, ys, xd, yd, width, height };
    sendCmdWords(F_gfx_ScreenCopyPaste, args, 6);
    getAck();
}

void Diablo16Serial::gfx_Panel(uint16_t raised, uint16_t x, uint16_t y,
                                 uint16_t width, uint16_t height, uint16_t colour) {
    uint16_t args[] = { raised, x, y, width, height, colour };
    sendCmdWords(F_gfx_Panel, args, 6);
    getAck();
}

void Diablo16Serial::gfx_Button(uint16_t state, uint16_t x, uint16_t y,
                                  uint16_t btnColour, uint16_t txtColour,
                                  uint16_t font, uint16_t txtWidth, uint16_t txtHeight,
                                  const char *text) {
    uint16_t args[] = { state, x, y, btnColour, txtColour, font, txtWidth, txtHeight };
    sendCmdWords(F_gfx_Button, args, 8);
    sendChars(text);
    getAck();
}

uint16_t Diablo16Serial::gfx_Slider(uint16_t mode, uint16_t x1, uint16_t y1,
                                      uint16_t x2, uint16_t y2, uint16_t colour,
                                      uint16_t scale, uint16_t value) {
    uint16_t args[] = { mode, x1, y1, x2, y2, colour, scale, value };
    sendCmdWords(F_gfx_Slider, args, 8);
    return getAckResp();
}

/* =========================================================================
 * Text & String Commands
 * ========================================================================= */

void Diablo16Serial::txt_MoveCursor(uint16_t line, uint16_t column) {
    uint16_t args[] = { line, column };
    sendCmdWords(F_txt_MoveCursor, args, 2);
    getAck();
}

void Diablo16Serial::putCH(uint16_t ch) {
    sendCmdWord(F_putCH, ch);
    getAck();
}

uint16_t Diablo16Serial::putstr(const char *str) {
    sendCmd(F_putstr);
    sendChars(str);
    return getAckResp();  /* ACK + word (count of chars printed) */
}

/* All txt_* attribute commands return ACK + previous-value word (per SPE spec).
 * Must use getAckResp() to consume both bytes, otherwise the leftover response
 * word corrupts the next command's ACK parsing. */

void Diablo16Serial::txt_FGcolour(uint16_t colour) {
    sendCmdWord(F_txt_FGcolour, colour);
    getAckResp();
}

void Diablo16Serial::txt_BGcolour(uint16_t colour) {
    sendCmdWord(F_txt_BGcolour, colour);
    getAckResp();
}

void Diablo16Serial::txt_FontID(uint16_t fontId) {
    sendCmdWord(F_txt_FontID, fontId);
    getAckResp();
}

void Diablo16Serial::txt_Width(uint16_t multiplier) {
    sendCmdWord(F_txt_Width, multiplier);
    getAckResp();
}

void Diablo16Serial::txt_Height(uint16_t multiplier) {
    sendCmdWord(F_txt_Height, multiplier);
    getAckResp();
}

void Diablo16Serial::txt_Xgap(uint16_t pixels) {
    sendCmdWord(F_txt_Xgap, pixels);
    getAckResp();
}

void Diablo16Serial::txt_Ygap(uint16_t pixels) {
    sendCmdWord(F_txt_Ygap, pixels);
    getAckResp();
}

void Diablo16Serial::txt_Bold(uint16_t mode) {
    sendCmdWord(F_txt_Bold, mode);
    getAckResp();
}

void Diablo16Serial::txt_Italic(uint16_t mode) {
    sendCmdWord(F_txt_Italic, mode);
    getAckResp();
}

void Diablo16Serial::txt_Inverse(uint16_t mode) {
    sendCmdWord(F_txt_Inverse, mode);
    getAckResp();
}

void Diablo16Serial::txt_Opacity(uint16_t mode) {
    sendCmdWord(F_txt_Opacity, mode);
    getAckResp();
}

void Diablo16Serial::txt_Underline(uint16_t mode) {
    sendCmdWord(F_txt_Underline, mode);
    getAckResp();
}

void Diablo16Serial::txt_Attributes(uint16_t attribs) {
    sendCmdWord(F_txt_Attributes, attribs);
    getAckResp();
}

void Diablo16Serial::txt_Wrap(uint16_t position) {
    sendCmdWord(F_txt_Wrap, position);
    getAckResp();
}

/* =========================================================================
 * Touch Screen Commands
 * ========================================================================= */

void Diablo16Serial::touch_DetectRegion(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint16_t args[] = { x1, y1, x2, y2 };
    sendCmdWords(F_touch_DetectRegion, args, 4);
    getAck();
}

uint16_t Diablo16Serial::touch_Get(uint16_t mode) {
    sendCmdWord(F_touch_Get, mode);
    return getAckResp();
}

void Diablo16Serial::touch_Set(uint16_t mode) {
    sendCmdWord(F_touch_Set, mode);
    getAck();
}

/* =========================================================================
 * System Commands
 * ========================================================================= */

bool Diablo16Serial::setbaudWait(uint16_t newRate) {
    sendCmdWord(F_setbaudWait, newRate);
    getAck();
    return (m_error == D16_ERR_OK);
}

uint16_t Diablo16Serial::sys_GetModel(char *modelStr, uint16_t maxLen) {
    sendCmd(F_sys_GetModel);
    return getAckResStr(modelStr, maxLen);
}

uint16_t Diablo16Serial::sys_GetVersion(void) {
    sendCmd(F_sys_GetVersion);
    return getAckResp();
}

uint16_t Diablo16Serial::sys_GetPmmC(void) {
    sendCmd(F_sys_GetPmmC);
    return getAckResp();
}

uint16_t Diablo16Serial::peekM(uint16_t address) {
    sendCmdWord(F_peekM, address);
    return getAckResp();
}

void Diablo16Serial::pokeM(uint16_t address, uint16_t value) {
    uint16_t args[] = { address, value };
    sendCmdWords(F_pokeM, args, 2);
    getAck();
}

/* =========================================================================
 * Media Commands
 * ========================================================================= */

uint16_t Diablo16Serial::media_Init(void) {
    sendCmd(F_media_Init);
    return getAckResp();
}

void Diablo16Serial::media_SetAdd(uint16_t hiWord, uint16_t loWord) {
    uint16_t args[] = { hiWord, loWord };
    sendCmdWords(F_media_SetAdd, args, 2);
    getAck();
}

void Diablo16Serial::media_SetSector(uint16_t hiWord, uint16_t loWord) {
    uint16_t args[] = { hiWord, loWord };
    sendCmdWords(F_media_SetSector, args, 2);
    getAck();
}

void Diablo16Serial::media_Image(uint16_t x, uint16_t y) {
    uint16_t args[] = { x, y };
    sendCmdWords(F_media_Image, args, 2);
    getAckResp();
}

void Diablo16Serial::media_Video(uint16_t x, uint16_t y) {
    uint16_t args[] = { x, y };
    sendCmdWords(F_media_Video, args, 2);
    getAck();
}

void Diablo16Serial::media_VideoFrame(uint16_t x, uint16_t y, uint16_t frame) {
    uint16_t args[] = { x, y, frame };
    sendCmdWords(F_media_VideoFrame, args, 3);
    getAckResp();
}

/* =========================================================================
 * Convenience: Draw Text at Pixel Position
 *
 * Uses gfx_MoveTo to set the origin, then prints text with the given
 * font, colour, and size multipliers.  Resets width/height to 1 after.
 * ========================================================================= */

void Diablo16Serial::drawTextAt(uint16_t x, uint16_t y, const char *str,
                                  uint16_t colour, uint16_t font,
                                  uint16_t wMul, uint16_t hMul) {
    /* Only send text attribute commands when the value has actually changed.
     * This dramatically reduces the command count (from 7 down to 2-3 in
     * the common case) and prevents Diablo16 stack overflow / Address Trap. */

    if (font != m_cachedFont) {
        txt_FontID(font);
        if (m_error != D16_ERR_OK) return;
        m_cachedFont = font;
    }
    if (colour != m_cachedFGcolour) {
        txt_FGcolour(colour);
        if (m_error != D16_ERR_OK) return;
        m_cachedFGcolour = colour;
    }
    if (D16_TRANSPARENT != m_cachedOpacity) {
        txt_Opacity(D16_TRANSPARENT);
        if (m_error != D16_ERR_OK) return;
        m_cachedOpacity = D16_TRANSPARENT;
    }
    if (wMul != m_cachedWidth) {
        txt_Width(wMul);
        if (m_error != D16_ERR_OK) return;
        m_cachedWidth = wMul;
    }
    if (hMul != m_cachedHeight) {
        txt_Height(hMul);
        if (m_error != D16_ERR_OK) return;
        m_cachedHeight = hMul;
    }

    /* Move text/graphics origin to pixel position (always needed) */
    gfx_MoveTo(x, y);
    if (m_error != D16_ERR_OK) return;
    putstr(str);             /* ACK + count */
}
