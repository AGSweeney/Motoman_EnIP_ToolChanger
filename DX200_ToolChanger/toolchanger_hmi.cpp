/*
 * DX200 Rotary Tool Changer -- Status HMI Implementation
 *
 * Draws and manages the tool changer status display on a 4D Systems
 * Gen4-uLCD-70DCT-CLB (800x480, Diablo16) display using the SPE
 * serial protocol via ClearCore COM-0.
 *
 * Status-only display -- no touch input.
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#include "ClearCore.h"
#include "toolchanger_hmi.h"
#include "diablo16_serial.h"
#include <stdio.h>
#include <string.h>

/* Pull in tool changer API for live status queries */
#ifdef CLEARCORE
#include "ports/ClearCore/clearcore_wrapper.h"
#endif

/* =========================================================================
 * Screen Geometry  (800 x 480 landscape)
 * ========================================================================= */

#define SCREEN_W    800
#define SCREEN_H    480

/* Top bar */
#define TOP_Y       0
#define TOP_H       32

/* Bottom bar */
#define BOT_H       26
#define BOT_Y       (SCREEN_H - BOT_H)

/* Content area between top and bottom bars */
#define CONTENT_Y   (TOP_Y + TOP_H)
#define CONTENT_H   (BOT_Y - CONTENT_Y)

/* Left panel -- tool position indicator (wide, dominant) */
#define LEFT_X      0
#define LEFT_W      480

/* Right panel -- status dashboard (compact, remaining width) */
#define RIGHT_X     LEFT_W
#define RIGHT_W     (SCREEN_W - LEFT_W)

/* Tool pocket grid: 3 columns x 2 rows -- sized to fill the left panel */
#define POCKET_MARGIN   12
#define POCKET_X0       (LEFT_X + POCKET_MARGIN)
#define POCKET_Y0       (CONTENT_Y + 28)
#define POCKET_W        ((LEFT_W - 4 * POCKET_MARGIN) / 3)  /* 3 cols, ~144px each */
#define POCKET_GAP_X    POCKET_MARGIN
#define POCKET_GAP_Y    12
#define POCKET_H        ((BOT_Y - POCKET_Y0 - POCKET_MARGIN - POCKET_GAP_Y) / 2)  /* ~185px */

/* Status row geometry (right panel) */
#define STATUS_X        (RIGHT_X + 10)
#define STATUS_LABEL_W  110
#define STATUS_VAL_X    (STATUS_X + STATUS_LABEL_W)
#define STATUS_ROW_H    40
#define STATUS_Y0       (CONTENT_Y + 10)

/* =========================================================================
 * Colour Palette (Light Motoman / Yaskawa theme)
 * ========================================================================= */

#define COL_BG          D16_RGB565(0xE8, 0xE8, 0xEC)   /* Light grey background */
#define COL_PANEL       D16_RGB565(0xFF, 0xFF, 0xFF)   /* White panels          */
#define COL_TOP_BAR     D16_RGB565(0x00, 0x54, 0xA6)   /* Yaskawa blue          */
#define COL_BOT_BAR     D16_RGB565(0x00, 0x54, 0xA6)   /* Yaskawa blue          */
#define COL_BORDER      D16_RGB565(0xA0, 0xA0, 0xA8)   /* Medium grey borders   */

#define COL_TEXT        D16_RGB565(0x20, 0x20, 0x20)   /* Dark charcoal text    */
#define COL_TEXT_DIM    D16_RGB565(0x70, 0x70, 0x78)   /* Medium grey labels    */
#define COL_TEXT_TITLE  D16_WHITE                       /* White (on blue bar)   */

#define COL_GREEN       D16_RGB565(0x27, 0xAE, 0x60)   /* Good / Ready         */
#define COL_YELLOW      D16_RGB565(0xFF, 0xE0, 0x00)   /* Bright yellow        */
#define COL_RED         D16_RGB565(0xE7, 0x4C, 0x3C)   /* Fault / E-stop       */
#define COL_GREEN_BRT   D16_RGB565(0x00, 0xFF, 0x60)   /* Bright green (bar)   */
#define COL_RED_BRT     D16_RGB565(0xFF, 0x40, 0x40)   /* Bright red   (bar)   */
#define COL_BLUE        D16_RGB565(0x00, 0x54, 0xA6)   /* Yaskawa blue         */
#define COL_ORANGE      D16_RGB565(0xE6, 0x7E, 0x22)   /* Homing               */

/* Pocket colours for the tool grid */
#define COL_POCKET_EMPTY    D16_RGB565(0xD0, 0xD0, 0xD8)   /* Silver            */
#define COL_POCKET_CURRENT  D16_RGB565(0x00, 0x54, 0xA6)   /* Yaskawa blue      */
#define COL_POCKET_TARGET   COL_YELLOW                      /* Yellow while moving */

/* =========================================================================
 * Internal State
 * ========================================================================= */

static Diablo16Serial gDisp;

/* Cached state to detect changes (dirty-flag updates) */
static struct {
    ToolChangerState state;
    uint8_t  currentTool;
    uint8_t  targetTool;
    int32_t  position;
    uint16_t faultCode;
    uint8_t  infoCode;
    bool     estop;
    int      hlfbState;    /* 0=off, 1=asserted, 2=has measurement */
    int      torquePercent;
    bool     linkUp;
    bool     scannerConnected;
} gPrev;

static bool     gInitialDrawDone = false;
static uint32_t gLastUpdate      = 0;

/* =========================================================================
 * Forward Declarations
 * ========================================================================= */

static void drawTopBar(void);
static void drawBottomBar(void);
static void drawLeftPanel(void);
static void drawRightPanel(void);

static void drawPocket(uint8_t tool, uint16_t bgCol, uint16_t txCol);
static void updateToolIndicator(uint8_t currentTool, uint8_t targetTool);
static void updateStatusRow(uint8_t row, const char *value, uint16_t valueCol);
static void updateState(ToolChangerState state);
static void updateBottomBar(uint16_t faultCode, uint8_t infoCode, bool scannerConnected);
static void updateEthernetStatus(bool linkUp, bool scannerConnected);

static uint16_t stateColour(ToolChangerState s);
static const char *stateName(ToolChangerState s);
static void drawTextCentred(uint16_t x, uint16_t y, uint16_t w, const char *str,
                             uint16_t colour, uint16_t font, uint16_t wm, uint16_t hm);

/* =========================================================================
 * Public API
 * ========================================================================= */

void HMI_Init(uint32_t baudRate) {
    char dbg[80];

    /* Configure COM-0 as TTL serial at 9600 (SPE default). */
    ConnectorCOM0.Mode(Connector::TTL);
    ConnectorCOM0.Speed(9600);
    ConnectorCOM0.Parity(SerialBase::PARITY_N);
    ConnectorCOM0.StopBits(1);
    ConnectorCOM0.PortOpen();

    /* Hardware reset via RTS line (if RESET-EN switch is ON on adapter).
     * 4D adapter inverts: HIGH on RTS = reset active, LOW = reset released.
     * ClearCore: LINE_OFF = HIGH, LINE_ON = LOW */
    ConnectorUsb.SendLine("HMI: Sending hardware reset via RTS...");
    ConnectorUsb.Flush();
    ConnectorCOM0.RtsMode(SerialBase::LINE_OFF); /* Pull reset HIGH (active) */
    Delay_ms(50);
    ConnectorCOM0.RtsMode(SerialBase::LINE_ON);  /* Release reset LOW (inactive) */

    /* Wait for Diablo16 to boot after reset (~1.5s typical) */
    ConnectorUsb.SendLine("HMI: Waiting for display boot...");
    ConnectorUsb.Flush();
    Delay_ms(1500);

    /* Flush any startup bytes the display may have sent */
    while (ConnectorCOM0.AvailableForRead() > 0) {
        ConnectorCOM0.CharGet();
    }

    /* Attach the serial driver. Use a longer timeout for baud probing. */
    gDisp.Begin(ConnectorCOM0);
    gDisp.SetTimeout(200);

    /* ---- Probe display ---- */
    uint32_t probeBauds[] = { 256000, 115200, 9600, 57600 };
    const uint8_t numBauds = sizeof(probeBauds) / sizeof(probeBauds[0]);
    uint32_t currentBaud = 9600;
    uint16_t version = 0;
    bool found = false;

    for (uint8_t i = 0; i < numBauds && !found; i++) {
        currentBaud = probeBauds[i];
        snprintf(dbg, sizeof(dbg), "HMI: Probing display at %lu baud...", (unsigned long)currentBaud);
        ConnectorUsb.SendLine(dbg);
        ConnectorUsb.Flush();

        ConnectorCOM0.PortClose();
        ConnectorCOM0.Speed(currentBaud);
        ConnectorCOM0.PortOpen();
        Delay_ms(100);
        while (ConnectorCOM0.AvailableForRead() > 0) ConnectorCOM0.CharGet();

        gDisp.ClearError();
        version = gDisp.sys_GetVersion();
        snprintf(dbg, sizeof(dbg), "HMI: sys_GetVersion at %lu -> err=%d, version=0x%04X",
                 (unsigned long)currentBaud, gDisp.GetError(), version);
        ConnectorUsb.SendLine(dbg);
        ConnectorUsb.Flush();

        if (gDisp.GetError() != D16_ERR_TIMEOUT) {
            found = true;
            snprintf(dbg, sizeof(dbg), "HMI: Display responding at %lu baud.", (unsigned long)currentBaud);
            ConnectorUsb.SendLine(dbg);
            ConnectorUsb.Flush();
        }
    }

    if (!found) {
        ConnectorUsb.SendLine("HMI: No response -- display not detected.");
        ConnectorUsb.Flush();
        return;
    }

    if (baudRate != currentBaud) {
        snprintf(dbg, sizeof(dbg), "HMI: Using %lu baud.", (unsigned long)currentBaud);
        ConnectorUsb.SendLine(dbg);
        ConnectorUsb.Flush();
    }

    /* ---- Configure display ---- */
    gDisp.SetTimeout(50);
    gDisp.ClearError();

    /* Drain any pending RX bytes */
    while (ConnectorCOM0.AvailableForRead() > 0) ConnectorCOM0.CharGet();

    gDisp.gfx_ScreenMode(GFX_LANDSCAPE);
    snprintf(dbg, sizeof(dbg), "HMI: gfx_ScreenMode -> err=%d", gDisp.GetError());
    ConnectorUsb.SendLine(dbg); ConnectorUsb.Flush();

    gDisp.gfx_BGcolour(COL_BG);
    gDisp.gfx_Cls();
    gDisp.resetTextCache();  /* gfx_Cls resets display state */
    snprintf(dbg, sizeof(dbg), "HMI: gfx_Cls -> err=%d", gDisp.GetError());
    ConnectorUsb.SendLine(dbg); ConnectorUsb.Flush();

    /* ---- Draw the full HMI interface ---- */
    ConnectorUsb.SendLine("HMI: Drawing initial screen...");
    ConnectorUsb.Flush();
    gDisp.SetTimeout(50);

    drawTopBar();
    drawBottomBar();
    drawLeftPanel();
    drawRightPanel();

    /* Draw initial state values */
    updateState(TC_STATE_DISABLED);
    updateToolIndicator(0, 0);
    updateStatusRow(1, "0", COL_TEXT);
    updateStatusRow(2, "OFF", COL_TEXT_DIM);
    updateStatusRow(3, "CLEAR", COL_GREEN);
    updateStatusRow(4, "NONE", COL_GREEN);
    updateStatusRow(5, "OK", COL_GREEN);
    updateEthernetStatus(false, false);

    /* Sync gPrev so first HMI_Cyclic pass doesn't redundantly redraw */
    gPrev.state            = TC_STATE_DISABLED;
    gPrev.currentTool      = 0;
    gPrev.targetTool       = 0;
    gPrev.position         = 0;
    gPrev.faultCode        = 0;
    gPrev.infoCode         = 0;
    gPrev.estop            = false;
    gPrev.hlfbState        = -1;  /* force first update */
    gPrev.torquePercent    = -999;
    gPrev.linkUp           = false;
    gPrev.scannerConnected = false;

    gInitialDrawDone = true;
    /* At 256000 baud the display responds in <1ms.  10ms is plenty. */
    gDisp.SetTimeout(10);

    ConnectorUsb.SendLine("HMI: Initial screen complete.");
    ConnectorUsb.Flush();
}

void HMI_Cyclic(void) {
    if (!gInitialDrawDone) return;

#ifdef CLEARCORE
    /* Throttle display updates to ~4 Hz (250ms).  The display serial link
     * is shared with the main loop -- updating too often starves EIP and
     * motor control, making the TC feel unresponsive. */
    uint32_t now = Milliseconds();
    if (now - gLastUpdate < 250) return;
    gLastUpdate = now;

    /* Read live state */
    ToolChangerState curState = ToolChanger_GetState();
    uint8_t  curTool    = ToolChanger_GetCurrentTool();
    int32_t  curPos     = ToolChanger_GetPosition();
    uint16_t curFault   = ToolChanger_GetFaultCode();
    uint8_t  curInfo    = ToolChanger_GetInfoCode();
    bool     curEstop   = ToolChanger_IsEstopActive() != 0;
    int      curHlfb    = ToolChanger_GetHlfbState();
    int      curTorque  = (curHlfb == 2) ? ToolChanger_GetTorquePercent() : 0;
    bool     curLink    = EthernetMgr.PhyLinkActive();
    bool     curScanner = EIP_IsScannerConnected() != 0;
    /* Target tool -- show yellow highlight while moving */
    uint8_t  curTarget  = (curState == TC_STATE_MOVING) ? ToolChanger_GetTargetTool() : 0;

    /* ---- Dirty-flag update: only redraw what changed ---- */

    /* State badge */
    if (curState != gPrev.state) {
        updateState(curState);
        gPrev.state = curState;
    }

    /* Tool position indicator */
    if (curTool != gPrev.currentTool || curTarget != gPrev.targetTool) {
        updateToolIndicator(curTool, curTarget);
        gPrev.currentTool = curTool;
        gPrev.targetTool  = curTarget;
    }

    /* Position row */
    if (curPos != gPrev.position) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%ld", (long)curPos);
        updateStatusRow(1, buf, COL_TEXT);
        gPrev.position = curPos;
    }

    /* Motor status row */
    if (curHlfb != gPrev.hlfbState || (curHlfb == 2 && curTorque != gPrev.torquePercent)) {
        if (curHlfb == 1) {
            /* HLFB asserted -- servo locked in position */
            updateStatusRow(2, "IN POSITION", COL_GREEN);
        } else if (curHlfb == 2) {
            /* HLFB has measurement -- motor active, show torque */
            char buf[16];
            snprintf(buf, sizeof(buf), "%d%% TQ", curTorque);
            updateStatusRow(2, buf, COL_ORANGE);
        } else {
            /* HLFB deasserted -- motor truly off */
            updateStatusRow(2, "OFF", COL_TEXT_DIM);
        }
        gPrev.hlfbState     = curHlfb;
        gPrev.torquePercent = curTorque;
    }

    /* E-stop row */
    if (curEstop != gPrev.estop) {
        updateStatusRow(3, curEstop ? "ACTIVE" : "CLEAR",
                         curEstop ? COL_RED : COL_GREEN);
        gPrev.estop = curEstop;
    }

    /* Fault + Info rows */
    if (curFault != gPrev.faultCode || curInfo != gPrev.infoCode) {
        /* Fault row */
        if (curFault == 0) {
            updateStatusRow(4, "NONE", COL_GREEN);
        } else {
            /* Show the highest-priority fault name */
            const char *faultName = "FAULT";
            if (curFault & 0x0001) faultName = "MOTOR ALERT";
            else if (curFault & 0x0002) faultName = "HLFB TIMEOUT";
            else if (curFault & 0x0004) faultName = "HOME FAILED";
            else if (curFault & 0x0008) faultName = "POS ERROR";
            else if (curFault & 0x0020) faultName = "SENSOR STUCK";
            updateStatusRow(4, faultName, COL_RED);
        }
        updateBottomBar(curFault, curInfo, curScanner);

        /* Info row -- show fault detail when faulted, otherwise normal info */
        if (curFault != 0) {
            char buf[20];
            snprintf(buf, sizeof(buf), "FAULT 0x%04X", curFault);
            updateStatusRow(5, buf, COL_RED);
        } else if (curInfo == 0) {
            updateStatusRow(5, "OK", COL_GREEN);
        } else if (curInfo & 0x01) {
            updateStatusRow(5, "BAD TOOL #", COL_YELLOW);
        } else if (curInfo & 0x02) {
            updateStatusRow(5, "NOT READY", COL_YELLOW);
        } else {
            updateStatusRow(5, "WARNING", COL_YELLOW);
        }

        gPrev.faultCode = curFault;
        gPrev.infoCode  = curInfo;
    }

    /* Ethernet status (top-right dot + bottom bar text) */
    if (curLink != gPrev.linkUp || curScanner != gPrev.scannerConnected) {
        updateEthernetStatus(curLink, curScanner);
        updateBottomBar(curFault, curInfo, curScanner);
        gPrev.linkUp           = curLink;
        gPrev.scannerConnected = curScanner;
    }
#endif /* CLEARCORE */
}

void HMI_PowerOff(void) {
    /* IO-3 is now controlled by EIP assembly -- nothing to do here */
}

/* =========================================================================
 * Top Bar
 * ========================================================================= */

static void drawTopBar(void) {
    gDisp.ClearError();
    gDisp.drainRx();

    /* Background */
    gDisp.gfx_RectangleFilled(0, TOP_Y, SCREEN_W - 1, TOP_Y + TOP_H - 1, COL_TOP_BAR);
    gDisp.gfx_Line(0, TOP_Y + TOP_H - 1, SCREEN_W - 1, TOP_Y + TOP_H - 1, COL_BORDER);

    /* Title -- centred in top bar */
    drawTextCentred(0, TOP_Y + 8, SCREEN_W, "MOTOMAN DX200 ETHERNET/IP TOOL CHANGER",
                     D16_WHITE, D16_FONT_4, 1, 1);

    ConnectorUsb.SendLine("HMI: Top bar drawn");
    ConnectorUsb.Flush();
}

/* =========================================================================
 * Bottom Bar
 * ========================================================================= */

static void drawBottomBar(void) {
    gDisp.ClearError();
    gDisp.drainRx();

    gDisp.gfx_RectangleFilled(0, BOT_Y, SCREEN_W - 1, SCREEN_H - 1, COL_BOT_BAR);
    gDisp.gfx_Line(0, BOT_Y, SCREEN_W - 1, BOT_Y, COL_BORDER);

    gDisp.drawTextAt(10, BOT_Y + 6, "Ethernet/IP: ",
                      D16_WHITE, D16_FONT_3, 1, 1);
    gDisp.drawTextAt(10 + 14 * 8, BOT_Y + 6, "Disconnected",
                      COL_RED_BRT, D16_FONT_3, 1, 1);

    gDisp.drawTextAt(SCREEN_W - 248, BOT_Y + 6, "(c) 2026 Van Voorst Lumber Co.",
                      D16_WHITE, D16_FONT_3, 1, 1);

    ConnectorUsb.SendLine("HMI: Bottom bar drawn");
    ConnectorUsb.Flush();
}

static void updateBottomBar(uint16_t faultCode, uint8_t infoCode,
                            bool scannerConnected) {
    (void)infoCode;
    gDisp.gfx_RectangleFilled(0, BOT_Y + 1, SCREEN_W - 1, SCREEN_H - 1, COL_BOT_BAR);

    if (faultCode != 0) {
        char buf[96];
        snprintf(buf, sizeof(buf), "FAULT: 0x%04X", faultCode);
        if (faultCode & 0x0001) snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " [MOTOR ALERT]");
        if (faultCode & 0x0002) snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " [HLFB TIMEOUT]");
        if (faultCode & 0x0004) snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " [HOME FAIL]");
        if (faultCode & 0x0008) snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " [POS ERROR]");
        if (faultCode & 0x0020) snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " [SENSOR STUCK]");
        gDisp.drawTextAt(10, BOT_Y + 6, buf, D16_WHITE, D16_FONT_3, 1, 1);
    } else {
        gDisp.drawTextAt(10, BOT_Y + 6, "Ethernet/IP: ",
                          D16_WHITE, D16_FONT_3, 1, 1);
        gDisp.drawTextAt(10 + 14 * 8, BOT_Y + 6,
                          scannerConnected ? "Connected" : "Disconnected",
                          scannerConnected ? COL_GREEN_BRT : COL_RED_BRT,
                          D16_FONT_3, 1, 1);
    }

    /* Copyright -- always visible */
    gDisp.drawTextAt(SCREEN_W - 248, BOT_Y + 6, "(c) 2026 Van Voorst Lumber Co.",
                      D16_WHITE, D16_FONT_3, 1, 1);
}

/* =========================================================================
 * Left Panel -- Tool Position Indicator
 * ========================================================================= */

static void drawLeftPanel(void) {
    gDisp.ClearError();
    gDisp.drainRx();

    /* Panel background */
    gDisp.gfx_RectangleFilled(LEFT_X, CONTENT_Y, LEFT_X + LEFT_W - 1, BOT_Y - 1, COL_PANEL);
    gDisp.gfx_Rectangle(LEFT_X, CONTENT_Y, LEFT_X + LEFT_W - 1, BOT_Y - 1, COL_BORDER);

    /* "TOOL POSITION" header */
    drawTextCentred(LEFT_X, CONTENT_Y + 4, LEFT_W, "TOOL POSITION",
                     COL_TEXT_DIM, D16_FONT_3, 1, 1);

    /* Draw all 6 pockets in their default (empty) state */
    for (uint8_t i = 1; i <= 6; i++) {
        drawPocket(i, COL_POCKET_EMPTY, COL_TEXT);
    }

    ConnectorUsb.SendLine("HMI: Left panel drawn");
    ConnectorUsb.Flush();
}

/* Helper: draw a single pocket tile at grid position (tool 1-6) */
static void drawPocket(uint8_t tool, uint16_t bgCol, uint16_t txCol) {
    uint8_t col_idx = (uint8_t)((tool - 1) % 3);
    uint8_t row_idx = (uint8_t)((tool - 1) / 3);
    uint16_t bx = POCKET_X0 + col_idx * (POCKET_W + POCKET_GAP_X);
    uint16_t by = POCKET_Y0 + row_idx * (POCKET_H + POCKET_GAP_Y);

    gDisp.ClearError();
    gDisp.gfx_RectangleFilled(bx, by, bx + POCKET_W - 1, by + POCKET_H - 1, bgCol);
    gDisp.gfx_Rectangle(bx, by, bx + POCKET_W - 1, by + POCKET_H - 1, COL_BORDER);

    char num[4];
    snprintf(num, sizeof(num), "T%d", tool);
    drawTextCentred(bx, by + (POCKET_H - 48) / 2, POCKET_W, num,
                     txCol, D16_FONT_4, 3, 3);
}

static void updateToolIndicator(uint8_t currentTool, uint8_t targetTool) {
    /* Only redraw pockets that actually changed role.
     * Possible roles: current (blue), target (orange), empty (silver).
     * We restore old current/target to empty, then draw new ones. */

    uint8_t oldCur = gPrev.currentTool;
    uint8_t oldTgt = gPrev.targetTool;

    /* Restore old current pocket to empty (if it changed) */
    if (oldCur >= 1 && oldCur <= 6 && oldCur != currentTool && oldCur != targetTool) {
        drawPocket(oldCur, COL_POCKET_EMPTY, COL_TEXT);
    }

    /* Restore old target pocket to empty (if it changed) */
    if (oldTgt >= 1 && oldTgt <= 6 && oldTgt != currentTool && oldTgt != targetTool && oldTgt != oldCur) {
        drawPocket(oldTgt, COL_POCKET_EMPTY, COL_TEXT);
    }

    /* Draw new current pocket */
    if (currentTool >= 1 && currentTool <= 6) {
        drawPocket(currentTool, COL_POCKET_CURRENT, D16_WHITE);
    }

    /* Draw new target pocket (if different from current) */
    if (targetTool >= 1 && targetTool <= 6 && targetTool != currentTool) {
        drawPocket(targetTool, COL_POCKET_TARGET, D16_BLACK);
    }
}

/* =========================================================================
 * Right Panel -- Status Dashboard
 * ========================================================================= */

static void drawRightPanel(void) {
    gDisp.ClearError();
    gDisp.drainRx();

    /* Panel background */
    gDisp.gfx_RectangleFilled(RIGHT_X, CONTENT_Y, SCREEN_W - 1, BOT_Y - 1, COL_PANEL);
    gDisp.gfx_Rectangle(RIGHT_X, CONTENT_Y, SCREEN_W - 1, BOT_Y - 1, COL_BORDER);

    /* "STATUS DASHBOARD" header */
    drawTextCentred(RIGHT_X, CONTENT_Y + 4, RIGHT_W, "STATUS DASHBOARD",
                     COL_TEXT_DIM, D16_FONT_3, 1, 1);

    /* Separator line */
    gDisp.gfx_Line(RIGHT_X + 10, CONTENT_Y + 20, SCREEN_W - 10, CONTENT_Y + 20, COL_BORDER);

    /* Draw static labels for each status row */
    const char *labels[] = { "STATE", "POSITION", "MOTOR", "E-STOP", "FAULT", "INFO" };
    for (int i = 0; i < 6; i++) {
        uint16_t y = STATUS_Y0 + 18 + (uint16_t)i * STATUS_ROW_H;

        gDisp.ClearError();
        gDisp.drawTextAt(STATUS_X, y + 10, labels[i], COL_TEXT_DIM, D16_FONT_4, 1, 1);

        /* Row separator */
        if (i < 5) {
            gDisp.gfx_Line(RIGHT_X + 10, y + STATUS_ROW_H - 2,
                            SCREEN_W - 10, y + STATUS_ROW_H - 2, COL_BORDER);
        }
    }

    ConnectorUsb.SendLine("HMI: Right panel drawn");
    ConnectorUsb.Flush();
}

static void updateStatusRow(uint8_t row, const char *value, uint16_t valueCol) {
    uint16_t y = STATUS_Y0 + 18 + (uint16_t)row * STATUS_ROW_H;

    /* Clear value area */
    gDisp.gfx_RectangleFilled(STATUS_VAL_X, y + 2,
                                SCREEN_W - 16, y + STATUS_ROW_H - 4, COL_PANEL);

    /* Draw new value */
    gDisp.drawTextAt(STATUS_VAL_X, y + 10, value, valueCol, D16_FONT_4, 1, 1);
}

static void updateState(ToolChangerState state) {
    if (state == TC_STATE_HOMING || state == TC_STATE_MOVING) {
        /* Red text on a tight yellow badge */
        uint16_t y = STATUS_Y0 + 18 + 0 * STATUS_ROW_H;
        const char *label;
        if (state == TC_STATE_HOMING) {
            label = "HOMING";
        } else {
            int dir = ToolChanger_GetMoveDirection();
            label = (dir > 0) ? "MOVING CW" : (dir < 0) ? "MOVING CCW" : "MOVING";
        }
        uint16_t textW = (uint16_t)(strlen(label) * 12);  /* Font 4 = 12px/char */
        uint16_t pad = 6;

        /* Clear entire value area first, then draw tight badge */
        gDisp.gfx_RectangleFilled(STATUS_VAL_X, y + 2,
                                    SCREEN_W - 16, y + STATUS_ROW_H - 4, COL_PANEL);
        gDisp.gfx_RectangleFilled(STATUS_VAL_X, y + 2,
                                    STATUS_VAL_X + textW + 2 * pad, y + STATUS_ROW_H - 4, COL_YELLOW);
        gDisp.drawTextAt(STATUS_VAL_X + pad, y + 10, label,
                          COL_RED, D16_FONT_4, 1, 1);
    } else {
        updateStatusRow(0, stateName(state), stateColour(state));
    }
}

static void updateEthernetStatus(bool linkUp, bool scannerConnected) {
    uint16_t x = 740;
    uint16_t y = TOP_Y + 4;
    uint16_t h = TOP_H - 8;

    /* Status dot:
     *   Green  = scanner connected (active I/O link)
     *   Yellow = Ethernet link up but no scanner
     *   Red    = no Ethernet link                    */
    uint16_t col;
    if (scannerConnected) col = COL_GREEN;
    else if (linkUp)      col = COL_YELLOW;
    else                  col = COL_RED;

    gDisp.gfx_RectangleFilled(x - 2, y, SCREEN_W - 2, y + h, COL_TOP_BAR);
    gDisp.gfx_CircleFilled(x + 8, y + h / 2, 5, col);
}

/* =========================================================================
 * Utility Helpers
 * ========================================================================= */

static uint16_t stateColour(ToolChangerState s) {
    switch (s) {
        case TC_STATE_DISABLED: return COL_TEXT_DIM;
        case TC_STATE_HOMING:   return COL_ORANGE;
        case TC_STATE_IDLE:     return COL_GREEN;
        case TC_STATE_MOVING:   return COL_ORANGE;
        case TC_STATE_AT_TOOL:  return COL_GREEN;
        case TC_STATE_FAULTED:  return COL_RED;
        default:                return COL_TEXT_DIM;
    }
}

static const char *stateName(ToolChangerState s) {
    switch (s) {
        case TC_STATE_DISABLED: return "DISABLED";
        case TC_STATE_HOMING:   return "HOMING";
        case TC_STATE_IDLE:     return "IDLE";
        case TC_STATE_MOVING:   return "MOVING";
        case TC_STATE_AT_TOOL:  return "AT TOOL";
        case TC_STATE_FAULTED:  return "FAULTED";
        default:                return "UNKNOWN";
    }
}

static void drawTextCentred(uint16_t x, uint16_t y, uint16_t w, const char *str,
                             uint16_t colour, uint16_t font, uint16_t wm, uint16_t hm) {
    uint16_t charW = (font >= D16_FONT_4) ? 12 : 8;
    uint16_t textW = (uint16_t)strlen(str) * charW * wm;
    uint16_t cx = x + (w > textW ? (w - textW) / 2 : 0);
    gDisp.drawTextAt(cx, y, str, colour, font, wm, hm);
}
