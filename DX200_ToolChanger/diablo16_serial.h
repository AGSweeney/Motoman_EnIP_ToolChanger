/*
 * DX200 Rotary Tool Changer -- Diablo16 Serial/SPE Display Driver
 *
 * Implements the 4D Systems Diablo16 Serial Platform Environment (SPE)
 * protocol for Gen4 display modules (e.g. Gen4-uLCD-70DCT-CLB).
 * Adapted for the Teknic ClearCore platform using the ISerial API.
 *
 * Based on the official Diablo16 Serial Command Set documentation and
 * the 4D Systems Diablo16-Serial-Arduino-Library (GPL-3.0).
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef DIABLO16_SERIAL_H_
#define DIABLO16_SERIAL_H_

#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * Diablo16 SPE Protocol Constants
 * ========================================================================= */

/* ACK / NAK byte values */
#define D16_ACK                     0x06
#define D16_NAK                     0x15

/* Error codes */
#define D16_ERR_OK                  0
#define D16_ERR_TIMEOUT             1
#define D16_ERR_NAK                 2

/* Default command timeout (ms) */
#define D16_DEFAULT_TIMEOUT_MS      2000

/* =========================================================================
 * SPE Command Opcodes (Diablo16 Serial Command Set)
 *
 * Each command is a 16-bit word sent MSB first over UART.
 * All multi-byte parameters are also 16-bit words, MSB first.
 * ========================================================================= */

/* --- Graphics Commands --------------------------------------------------- */
#define F_gfx_Cls                   0xFF82
#define F_gfx_ChangeColour          0xFF5E
#define F_gfx_Circle                0xFF7B
#define F_gfx_CircleFilled          0xFF76
#define F_gfx_Line                  0xFF7D
#define F_gfx_Rectangle             0xFF7A
#define F_gfx_RectangleFilled       0xFF79
#define F_gfx_Triangle              0xFF77
#define F_gfx_TriangleFilled        0xFF68
#define F_gfx_Ellipse               0xFF7E
#define F_gfx_EllipseFilled         0xFF75
#define F_gfx_PutPixel              0xFF7C
#define F_gfx_GetPixel              0xFF98
#define F_gfx_MoveTo                0xFF81  /* Move Origin -- official SPE opcode */
#define F_gfx_LineTo                0xFF87
#define F_gfx_Clipping              0xFF84
#define F_gfx_ClipWindow            0xFF85
#define F_gfx_SetClipRegion         0xFF86
#define F_gfx_Set                   0xFF96
#define F_gfx_Get                   0xFF97
#define F_gfx_BGcolour              0xFF6E
#define F_gfx_Contrast              0xFF66
#define F_gfx_ScreenMode            0xFF9E
#define F_gfx_OutlineColour         0xFF9D
#define F_gfx_LinePattern           0xFF9B
#define F_gfx_FrameDelay            0xFF99
#define F_gfx_Panel                 0xFF9F
#define F_gfx_Button                0xFF54
#define F_gfx_Slider                0xFF57
#define F_gfx_BevelShadow           0xFF55
#define F_gfx_BevelWidth            0xFF56
#define F_gfx_ScreenCopyPaste       0xFF6C
#define F_gfx_Transparency          0xFF60
#define F_gfx_TransparentColour     0xFF61
#define F_gfx_Orbit                 0x0003
#define F_gfx_Polygon               0x0013
#define F_gfx_PolygonFilled         0x0014
#define F_gfx_Polyline              0x0015

/* --- Text & String Commands ---------------------------------------------- */
#define F_txt_MoveCursor            0xFFF0
#define F_putCH                     0xFFFE
#define F_putstr                    0x0018
#define F_charwidth                 0x001E
#define F_charheight                0x001D
#define F_txt_FGcolour              0xFFEE
#define F_txt_BGcolour              0xFFED
#define F_txt_FontID                0xFFEC
#define F_txt_Width                 0xFFEB
#define F_txt_Height                0xFFEA
#define F_txt_Xgap                  0xFFE9
#define F_txt_Ygap                  0xFFE8
#define F_txt_Bold                  0xFFE5
#define F_txt_Italic                0xFFE4
#define F_txt_Inverse               0xFFE3
#define F_txt_Opacity               0xFFE6
#define F_txt_Underline             0xFFE2
#define F_txt_Attributes            0xFFE1
#define F_txt_Wrap                  0xFFDD
#define F_txt_Set                   0xFFE7

/* --- Touch Screen Commands ----------------------------------------------- */
#define F_touch_DetectRegion        0x0039
#define F_touch_Get                 0x0037
#define F_touch_Set                 0x0038

/* --- System Commands ----------------------------------------------------- */
#define F_setbaudWait               0x0026
#define F_sys_GetModel              0x001A
#define F_sys_GetVersion            0x001B
#define F_sys_GetPmmC               0x001C
#define F_sys_Sleep                 0xFF3B
#define F_peekM                     0xFF58
#define F_pokeM                     0xFF59

/* --- Media Commands ------------------------------------------------------ */
#define F_media_Init                0xFF89
#define F_media_SetAdd              0xFF93
#define F_media_SetSector           0xFF92
#define F_media_ReadByte            0xFF90
#define F_media_ReadWord            0xFF91
#define F_media_WriteByte           0xFF94
#define F_media_WriteWord           0xFF95
#define F_media_Image               0xFF8B
#define F_media_Video               0xFF8C
#define F_media_VideoFrame          0xFF8D
#define F_media_Flush               0xFF8A

/* --- Bus I/O Commands ---------------------------------------------------- */
#define F_bus_Read8                 0xFF70
#define F_bus_Write8                0xFF71

/* =========================================================================
 * RGB565 Colour Constants
 * ========================================================================= */
#define D16_BLACK                   0x0000
#define D16_WHITE                   0xFFFF
#define D16_RED                     0xF800
#define D16_GREEN                   0x07E0
#define D16_BLUE                    0x001F
#define D16_YELLOW                  0xFFE0
#define D16_CYAN                    0x07FF
#define D16_MAGENTA                 0xF81F
#define D16_ORANGE                  0xFD20
#define D16_NAVY                    0x0010
#define D16_DARKGREEN               0x03E0
#define D16_DARKCYAN                0x03EF
#define D16_MAROON                  0x7800
#define D16_PURPLE                  0x780F
#define D16_OLIVE                   0x7BE0
#define D16_LIGHTGREY               0xC618
#define D16_DARKGREY                0x7BEF
#define D16_GREY                    0xAD55

/* Custom dark theme colours */
#define D16_DARK_BG                 0x18E3  /* Dark blue-grey (#1B1B2F) */
#define D16_PANEL_BG                0x2124  /* Slightly lighter panel */
#define D16_ACCENT_BLUE             0x2D7F  /* Accent blue (#2980B9) */
#define D16_ACCENT_GREEN            0x2E68  /* Status green (#27AE60) */
#define D16_ACCENT_ORANGE           0xFCA0  /* Warning orange (#F39C12) */
#define D16_ACCENT_RED              0xE8A4  /* Error red (#E74C3C) */
#define D16_TEXT_DIM                0x7BEF  /* Dimmed text (#808080) */
#define D16_TEXT_BRIGHT             0xE71C  /* Bright text (#E0E0E0) */

/* Inline colour builder: 5-bit R, 6-bit G, 5-bit B */
#define D16_RGB565(r8, g8, b8) \
    ((uint16_t)(((r8) & 0xF8) << 8) | (((g8) & 0xFC) << 3) | (((b8) & 0xF8) >> 3))

/* =========================================================================
 * gfx_Set() Parameter Constants
 * ========================================================================= */
#define GFX_PEN_SIZE                16
#define GFX_BACKGROUND_COLOUR       17
#define GFX_OBJECT_COLOUR           18
#define GFX_CLIPPING                19
#define GFX_TRANSPARENT_COLOUR      20
#define GFX_TRANSPARENCY            21
#define GFX_FRAME_DELAY             22
#define GFX_SCREEN_MODE             23
#define GFX_OUTLINE_COLOUR          24
#define GFX_CONTRAST                25
#define GFX_LINE_PATTERN            26
#define GFX_BEVEL_RADIUS            27
#define GFX_BEVEL_WIDTH             28
#define GFX_BEVEL_SHADOW            29
#define GFX_X_ORIGIN                30
#define GFX_Y_ORIGIN                31

#define GFX_SOLID                   0
#define GFX_OUTLINE                 1

#define GFX_LANDSCAPE               0
#define GFX_LANDSCAPE_R             1
#define GFX_PORTRAIT                2
#define GFX_PORTRAIT_R              3

/* =========================================================================
 * Font ID Constants
 * ========================================================================= */
#define D16_FONT_1                  1   /* System 5x7   */
#define D16_FONT_2                  2   /* System 8x8   */
#define D16_FONT_3                  3   /* System 8x12  (default) */
#define D16_FONT_4                  4   /* System 12x16 */
#define D16_FONT_5                  5   /* MS SanSerif 8x12 */
#define D16_FONT_6                  6   /* DejaVu Sans 9pt */
#define D16_FONT_7                  7   /* DejaVu Sans Bold 9pt */
#define D16_FONT_8                  8   /* DejaVu Sans Condensed 9pt */
#define D16_FONT_9                  9   /* System 3x6 */
#define D16_FONT_11                 11  /* EGA 8x12 */

/* =========================================================================
 * Text Constants
 * ========================================================================= */
#define D16_TRANSPARENT             0
#define D16_OPAQUE                  1

/* =========================================================================
 * Touch Constants
 * ========================================================================= */
#define D16_TOUCH_ENABLE            0
#define D16_TOUCH_DISABLE           1
#define D16_TOUCH_REGIONDEFAULT     2

#define D16_TOUCH_STATUS            0
#define D16_TOUCH_GETX              1
#define D16_TOUCH_GETY              2
#define D16_NOTOUCH                 0
#define D16_TOUCH_PRESSED           1
#define D16_TOUCH_RELEASED          2
#define D16_TOUCH_MOVING            3

/* =========================================================================
 * Button/Panel/Slider State Constants
 * ========================================================================= */
#define D16_BUTTON_DOWN             0
#define D16_BUTTON_UP               1
#define D16_BUTTON_HIDE             2

#define D16_PANEL_SUNKEN            0
#define D16_PANEL_RAISED            1
#define D16_PANEL_HIDE              2

/* =========================================================================
 * Forward Declarations
 * ========================================================================= */
namespace ClearCore { class ISerial; }

/* =========================================================================
 * Diablo16Serial Class
 *
 * Low-level driver for Diablo16 SPE serial protocol.
 * Sends 2-byte commands + word parameters, waits for ACK/response.
 * ========================================================================= */

class Diablo16Serial {
public:
    Diablo16Serial();

    /**
     * Initialise the driver with a ClearCore serial port reference.
     * The port MUST already be configured (mode, baud, opened).
     */
    void Begin(ClearCore::ISerial &serial);

    /* --- Error state --- */
    int     GetError(void) const    { return m_error; }
    uint8_t GetLastNak(void) const  { return m_lastNak; }
    bool    WasNakd(void) const     { return m_nakReceived; }
    void    ClearError(void)        { m_error = D16_ERR_OK; m_lastNak = 0; m_nakReceived = false; }

    /* Timeout for command responses (ms) */
    void     SetTimeout(uint32_t ms) { m_timeoutMs = ms; }
    uint32_t GetTimeout(void) const  { return m_timeoutMs; }

    /* --- Graphics Commands ----------------------------------------------- */
    void     gfx_Cls(void);
    void     gfx_ChangeColour(uint16_t oldCol, uint16_t newCol);
    void     gfx_Circle(uint16_t x, uint16_t y, uint16_t radius, uint16_t colour);
    void     gfx_CircleFilled(uint16_t x, uint16_t y, uint16_t radius, uint16_t colour);
    void     gfx_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour);
    void     gfx_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour);
    void     gfx_RectangleFilled(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour);
    void     gfx_Triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                           uint16_t x3, uint16_t y3, uint16_t colour);
    void     gfx_Ellipse(uint16_t x, uint16_t y, uint16_t xrad, uint16_t yrad, uint16_t colour);
    void     gfx_EllipseFilled(uint16_t x, uint16_t y, uint16_t xrad, uint16_t yrad, uint16_t colour);
    void     gfx_PutPixel(uint16_t x, uint16_t y, uint16_t colour);
    void     gfx_MoveTo(uint16_t x, uint16_t y);
    void     gfx_LineTo(uint16_t x, uint16_t y);
    void     gfx_Clipping(uint16_t onOff);
    void     gfx_ClipWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void     gfx_Set(uint16_t func, uint16_t value);
    uint16_t gfx_Get(uint16_t mode);
    uint16_t gfx_BGcolour(uint16_t colour);
    uint16_t gfx_Contrast(uint16_t contrast);
    uint16_t gfx_ScreenMode(uint16_t mode);
    uint16_t gfx_OutlineColour(uint16_t colour);
    uint16_t gfx_LinePattern(uint16_t pattern);
    void     gfx_ScreenCopyPaste(uint16_t xs, uint16_t ys, uint16_t xd, uint16_t yd,
                                  uint16_t width, uint16_t height);
    void     gfx_Panel(uint16_t raised, uint16_t x, uint16_t y,
                        uint16_t width, uint16_t height, uint16_t colour);
    void     gfx_Button(uint16_t state, uint16_t x, uint16_t y,
                         uint16_t btnColour, uint16_t txtColour,
                         uint16_t font, uint16_t txtWidth, uint16_t txtHeight,
                         const char *text);
    uint16_t gfx_Slider(uint16_t mode, uint16_t x1, uint16_t y1,
                          uint16_t x2, uint16_t y2, uint16_t colour,
                          uint16_t scale, uint16_t value);

    /* --- Text & String Commands ------------------------------------------ */
    void     txt_MoveCursor(uint16_t line, uint16_t column);
    void     putCH(uint16_t ch);
    uint16_t putstr(const char *str);
    void     txt_FGcolour(uint16_t colour);
    void     txt_BGcolour(uint16_t colour);
    void     txt_FontID(uint16_t fontId);
    void     txt_Width(uint16_t multiplier);
    void     txt_Height(uint16_t multiplier);
    void     txt_Xgap(uint16_t pixels);
    void     txt_Ygap(uint16_t pixels);
    void     txt_Bold(uint16_t mode);
    void     txt_Italic(uint16_t mode);
    void     txt_Inverse(uint16_t mode);
    void     txt_Opacity(uint16_t mode);
    void     txt_Underline(uint16_t mode);
    void     txt_Attributes(uint16_t attribs);
    void     txt_Wrap(uint16_t position);

    /* --- Touch Screen Commands ------------------------------------------- */
    void     touch_DetectRegion(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    uint16_t touch_Get(uint16_t mode);
    void     touch_Set(uint16_t mode);

    /* --- System Commands ------------------------------------------------- */
    bool     setbaudWait(uint16_t newRate);
    uint16_t sys_GetModel(char *modelStr, uint16_t maxLen);
    uint16_t sys_GetVersion(void);
    uint16_t sys_GetPmmC(void);
    uint16_t peekM(uint16_t address);
    void     pokeM(uint16_t address, uint16_t value);

    /* --- Media Commands -------------------------------------------------- */
    uint16_t media_Init(void);
    void     media_SetAdd(uint16_t hiWord, uint16_t loWord);
    void     media_SetSector(uint16_t hiWord, uint16_t loWord);
    void     media_Image(uint16_t x, uint16_t y);
    void     media_Video(uint16_t x, uint16_t y);
    void     media_VideoFrame(uint16_t x, uint16_t y, uint16_t frame);

    /* --- Convenience: draw text at pixel position using gfx_MoveTo ------- */
    void     drawTextAt(uint16_t x, uint16_t y, const char *str,
                         uint16_t colour, uint16_t font = D16_FONT_3,
                         uint16_t wMul = 1, uint16_t hMul = 1);

    /* --- Protocol helpers (public for HMI sync recovery) ----------------- */
    void     drainRx(void);         /* consume all pending RX bytes */

    /* Reset cached text attributes (call after gfx_Cls or screen mode change) */
    void     resetTextCache(void);

private:
    ClearCore::ISerial *m_serial;
    uint32_t m_timeoutMs;
    int      m_error;
    uint8_t  m_lastNak;
    bool     m_nakReceived;    /* true if last getAck() got 0x15 instead of 0x06 */

    /* Cached text attributes -- avoids resending unchanged values.
     * 0xFFFF means "unknown / not yet set", forcing the first send. */
    uint16_t m_cachedFont;
    uint16_t m_cachedFGcolour;
    uint16_t m_cachedOpacity;
    uint16_t m_cachedWidth;
    uint16_t m_cachedHeight;

    /* Low-level I/O helpers */
    void     sendWord(uint16_t w);
    void     sendCmd(uint16_t cmd);
    void     sendCmdWord(uint16_t cmd, uint16_t w);
    void     sendCmdWords(uint16_t cmd, const uint16_t *words, uint16_t count);
    void     sendChars(const char *str);

    void     getAck(void);
    void     getAckNonBlocking(void); /* fire-and-forget: drain RX, don't wait */
    uint16_t getWord(void);
    uint16_t getAckResp(void);
    uint16_t getAckResStr(char *outStr, uint16_t maxLen);
    void     getBytes(uint8_t *data, uint16_t size);
};

#endif /* DIABLO16_SERIAL_H_ */
