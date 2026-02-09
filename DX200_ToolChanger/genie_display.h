/*
 * DX200 Rotary Tool Changer -- 4D Systems ViSi-Genie Display Driver
 *
 * Implements the ViSi-Genie serial protocol for 4D Systems displays,
 * adapted for the Teknic ClearCore platform (UART via COM-0 or COM-1).
 *
 * Based on genieArduino v1.5.3 by 4D Systems Pty Ltd (LGPL v3).
 * Ported to ClearCore by removing Arduino dependencies and using
 * the ClearCore SerialDriver / ISerial API.
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#ifndef GENIE_DISPLAY_H_
#define GENIE_DISPLAY_H_

#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * ViSi-Genie Protocol Constants
 * ========================================================================= */

/* ACK / NAK */
#define GENIE_ACK               0x06
#define GENIE_NAK               0x15

/* Timeout & resync periods (milliseconds) */
#define GENIE_TIMEOUT_PERIOD    1000
#define GENIE_RESYNC_PERIOD     100

/* Command / reply codes */
#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11
#define GENIE_WRITE_INH_LABEL   12

/* =========================================================================
 * ViSi-Genie Object Type IDs
 *
 * NOTE from 4D Systems: Object IDs may change with future releases;
 * it is not advisable to code their values as constants.
 * ========================================================================= */
#define GENIE_OBJ_DIPSW                 0
#define GENIE_OBJ_KNOB                  1
#define GENIE_OBJ_ROCKERSW              2
#define GENIE_OBJ_ROTARYSW              3
#define GENIE_OBJ_SLIDER                4
#define GENIE_OBJ_TRACKBAR              5
#define GENIE_OBJ_WINBUTTON             6
#define GENIE_OBJ_ANGULAR_METER         7
#define GENIE_OBJ_COOL_GAUGE            8
#define GENIE_OBJ_CUSTOM_DIGITS         9
#define GENIE_OBJ_FORM                  10
#define GENIE_OBJ_GAUGE                 11
#define GENIE_OBJ_IMAGE                 12
#define GENIE_OBJ_KEYBOARD              13
#define GENIE_OBJ_LED                   14
#define GENIE_OBJ_LED_DIGITS            15
#define GENIE_OBJ_METER                 16
#define GENIE_OBJ_STRINGS               17
#define GENIE_OBJ_THERMOMETER           18
#define GENIE_OBJ_USER_LED              19
#define GENIE_OBJ_VIDEO                 20
#define GENIE_OBJ_STATIC_TEXT           21
#define GENIE_OBJ_SOUND                 22
#define GENIE_OBJ_TIMER                 23
#define GENIE_OBJ_SPECTRUM              24
#define GENIE_OBJ_SCOPE                 25
#define GENIE_OBJ_TANK                  26
#define GENIE_OBJ_USERIMAGES            27
#define GENIE_OBJ_PINOUTPUT             28
#define GENIE_OBJ_PININPUT              29
#define GENIE_OBJ_4DBUTTON              30
#define GENIE_OBJ_ANIBUTTON             31
#define GENIE_OBJ_COLORPICKER           32
#define GENIE_OBJ_USERBUTTON            33
/* 34 reserved for magic functions */
#define GENIE_OBJ_SMARTGAUGE            35
#define GENIE_OBJ_SMARTSLIDER           36
#define GENIE_OBJ_SMARTKNOB             37
#define GENIE_OBJ_ISMARTGAUGE           35  /* legacy alias */
#define GENIE_OBJ_ISMARTSLIDER          36  /* legacy alias */
#define GENIE_OBJ_ISMARTKNOB            37  /* legacy alias */
#define GENIE_OBJ_ILED_DIGITS_H         38
#define GENIE_OBJ_IANGULAR_METER        39
#define GENIE_OBJ_IGAUGE                40
#define GENIE_OBJ_ILABELB               41
#define GENIE_OBJ_IUSER_GAUGE           42
#define GENIE_OBJ_IMEDIA_GAUGE          43
#define GENIE_OBJ_IMEDIA_THERMOMETER    44
#define GENIE_OBJ_ILED                  45
#define GENIE_OBJ_IMEDIA_LED            46
#define GENIE_OBJ_ILED_DIGITS_L         47
#define GENIE_OBJ_ILED_DIGITS           47
#define GENIE_OBJ_INEEDLE               48
#define GENIE_OBJ_IRULER                49
#define GENIE_OBJ_ILED_DIGIT            50
#define GENIE_OBJ_IBUTTOND              51
#define GENIE_OBJ_IBUTTONE              52
#define GENIE_OBJ_IMEDIA_BUTTON         53
#define GENIE_OBJ_ITOGGLE_INPUT         54
#define GENIE_OBJ_IDIAL                 55
#define GENIE_OBJ_IMEDIA_ROTARY         56
#define GENIE_OBJ_IROTARY_INPUT         57
#define GENIE_OBJ_ISWITCH               58
#define GENIE_OBJ_ISWITCHB              59
#define GENIE_OBJ_ISLIDERE              60
#define GENIE_OBJ_IMEDIA_SLIDER         61
#define GENIE_OBJ_ISLIDERH              62
#define GENIE_OBJ_ISLIDERG              63
#define GENIE_OBJ_ISLIDERF              64
#define GENIE_OBJ_ISLIDERD              65
#define GENIE_OBJ_ISLIDERC              66
#define GENIE_OBJ_ILINEAR_INPUT         67

/* =========================================================================
 * Frame & queue sizes
 * ========================================================================= */
#define GENIE_FRAME_SIZE        6
#define MAX_GENIE_EVENTS        16      /* MUST be a power of 2 */
#define MAX_GENIE_FATALS        10
#define MAX_LINK_STATES         20

/* =========================================================================
 * Link states
 * ========================================================================= */
#define GENIE_LINK_IDLE           0
#define GENIE_LINK_WFAN           1     /* waiting for ACK or NAK */
#define GENIE_LINK_WF_RXREPORT   2     /* waiting for report frame */
#define GENIE_LINK_RXREPORT      3     /* receiving a report frame */
#define GENIE_LINK_RXEVENT       4     /* receiving an event frame */
#define GENIE_LINK_SHDN          5     /* shut down */
#define GENIE_LINK_RXMBYTES      6     /* receiving magic bytes */
#define GENIE_LINK_RXMDBYTES     7     /* receiving magic double-bytes */

/* =========================================================================
 * DoEvents return values
 * ========================================================================= */
#define GENIE_EVENT_NONE    0
#define GENIE_EVENT_RXCHAR  1

/* =========================================================================
 * Error codes
 * ========================================================================= */
#define GENIE_ERROR_NONE           0
#define GENIE_ERROR_TIMEOUT       -1
#define GENIE_ERROR_NOHANDLER     -2
#define GENIE_ERROR_NOCHAR        -3
#define GENIE_ERROR_NAK           -4
#define GENIE_ERROR_REPLY_OVR     -5
#define GENIE_ERROR_RESYNC        -6
#define GENIE_ERROR_NODISPLAY     -7
#define GENIE_ERROR_BAD_CS        -8

/* =========================================================================
 * Frame structures
 * ========================================================================= */

/** Report object frame layout (5 data bytes + checksum). */
struct GenieFrameReportObj {
    uint8_t cmd;
    uint8_t object;
    uint8_t index;
    uint8_t data_msb;
    uint8_t data_lsb;
};

/** Magic report header. */
struct GenieMagicReportHeader {
    uint8_t cmd;
    uint8_t index;
    uint8_t length;
};

/** Union for 32-bit float / long / word-pair access. */
union GenieFloatLongFrame {
    float    floatValue;
    int32_t  longValue;
    uint32_t ulongValue;
    int16_t  wordValue[2];
};

/**
 * ViSi-Genie frame.
 *
 * The union allows referencing data as a raw byte array or as
 * structured fields:
 *   frame.bytes[4]  ==  frame.reportObject.data_lsb
 */
union GenieFrame {
    uint8_t             bytes[GENIE_FRAME_SIZE];
    GenieFrameReportObj reportObject;
};

/** Circular event queue. */
struct GenieEventQueue {
    GenieFrame frames[MAX_GENIE_EVENTS];
    uint8_t    rd_index;
    uint8_t    wr_index;
    uint8_t    n_events;
};

/* =========================================================================
 * Callback typedefs
 * ========================================================================= */
typedef void (*GenieEventHandler)(void);
typedef void (*GenieMagicByteHandler)(uint8_t index, uint8_t length);
typedef void (*GenieMagicDoubleByteHandler)(uint8_t index, uint8_t length);

/* =========================================================================
 * Forward declaration -- ClearCore serial base class
 * ========================================================================= */
namespace ClearCore {
    class ISerial;
}

/* =========================================================================
 * GenieDisplay class
 *
 * Drives a 4D Systems display running ViSi-Genie firmware over a
 * ClearCore serial port (UART).  Replaces the Arduino Genie class
 * with ClearCore-native serial calls.
 * ========================================================================= */
class GenieDisplay {

public:
    GenieDisplay();

    /* -----------------------------------------------------------------
     * Initialisation
     * ----------------------------------------------------------------- */

    /**
     * Initialise the display driver on a ClearCore serial port.
     *
     * The caller is responsible for configuring the serial port mode
     * (e.g. TTL) and baud rate, and calling PortOpen() before this.
     *
     * Typical setup:
     *   ConnectorCOM0.Mode(Connector::TTL);
     *   ConnectorCOM0.Speed(200000);
     *   ConnectorCOM0.Parity(SerialBase::PARITY_N);
     *   ConnectorCOM0.StopBits(1);
     *   ConnectorCOM0.PortOpen();
     *   genie.Begin(ConnectorCOM0);
     *
     * @param serial  Reference to a ClearCore ISerial port.
     */
    void Begin(ClearCore::ISerial &serial);

    /* -----------------------------------------------------------------
     * Core protocol -- read / write objects
     * ----------------------------------------------------------------- */

    /**
     * Request a read of an object on the display.
     * The reply will arrive asynchronously and be queued by DoEvents().
     */
    bool ReadObject(uint16_t object, uint16_t index);

    /**
     * Write a 16-bit value to a display object.
     */
    uint16_t WriteObject(uint16_t object, uint16_t index, uint16_t data);

    /* -----------------------------------------------------------------
     * String writing
     * ----------------------------------------------------------------- */

    /** Write a null-terminated ASCII string to a Strings object. */
    uint16_t WriteStr(uint16_t index, const char *string);

    /** Write a signed integer as a string. */
    uint16_t WriteStr(uint16_t index, int32_t number);

    /** Write an unsigned integer as a string. */
    uint16_t WriteStr(uint16_t index, uint32_t number);

    /** Write a floating-point number as a string. */
    uint16_t WriteStr(uint16_t index, double number, int digits = 2);

    /** Write a null-terminated Unicode string to a Strings object. */
    uint16_t WriteStrU(uint16_t index, const uint16_t *string);

    /* -----------------------------------------------------------------
     * Inherent label writing
     * ----------------------------------------------------------------- */

    /** Refresh an inherent label (redraw from flash). */
    uint16_t WriteInhLabel(uint16_t index);

    /** Write a null-terminated ASCII string to an inherent label. */
    uint16_t WriteInhLabel(uint16_t index, const char *string);

    /** Write a signed integer to an inherent label. */
    uint16_t WriteInhLabel(uint16_t index, int32_t number);

    /** Write an unsigned integer to an inherent label. */
    uint16_t WriteInhLabel(uint16_t index, uint32_t number);

    /** Write a floating-point number to an inherent label. */
    uint16_t WriteInhLabel(uint16_t index, double number, int digits = 2);

    /* -----------------------------------------------------------------
     * Internal LED digits (32-bit values)
     * ----------------------------------------------------------------- */

    /** Write a 16-bit integer to internal LED digits. */
    uint16_t WriteIntLedDigits(uint16_t index, int16_t data);

    /** Write a 32-bit float to internal LED digits. */
    uint16_t WriteIntLedDigits(uint16_t index, float data);

    /** Write a 32-bit integer to internal LED digits. */
    uint16_t WriteIntLedDigits(uint16_t index, int32_t data);

    /* -----------------------------------------------------------------
     * Display control
     * ----------------------------------------------------------------- */

    /**
     * Set display contrast / backlight level.
     * @param value  0-15 (display dependent).
     */
    void WriteContrast(uint16_t value);

    /* -----------------------------------------------------------------
     * Magic functions (ViSi-Genie Pro only)
     * ----------------------------------------------------------------- */

    /** Write an array of bytes to a Magic object. */
    uint16_t WriteMagicBytes(uint16_t index, const uint8_t *bytes,
                             uint16_t len);

    /** Write an array of 16-bit values to a Magic object. */
    uint16_t WriteMagicDBytes(uint16_t index, const uint16_t *shorts,
                              uint16_t len);

    /** Read the next byte from the serial port (blocking). */
    uint8_t GetNextByte(void);

    /** Read the next 16-bit value from the serial port (blocking). */
    uint16_t GetNextDoubleByte(void);

    /* -----------------------------------------------------------------
     * Event processing
     * ----------------------------------------------------------------- */

    /**
     * Process incoming serial data from the display.
     *
     * MUST be called frequently from the main loop (every scan).
     * Receives frames, manages the protocol state machine, and
     * dispatches queued events to the user handler.
     *
     * @param doHandler  If true (default), call the user event handler
     *                   when events are queued and no new data arrived.
     * @return GENIE_EVENT_NONE or GENIE_EVENT_RXCHAR.
     */
    uint16_t DoEvents(bool doHandler = true);

    /**
     * Register a callback invoked when display events are queued.
     * Inside the handler, call DequeueEvent() to retrieve events.
     */
    void AttachEventHandler(GenieEventHandler handler);

    /** Register a callback for Magic byte reports (Pro only). */
    void AttachMagicByteReader(GenieMagicByteHandler handler);

    /** Register a callback for Magic double-byte reports (Pro only). */
    void AttachMagicDoubleByteReader(GenieMagicDoubleByteHandler handler);

    /* -----------------------------------------------------------------
     * Event query helpers
     * ----------------------------------------------------------------- */

    /**
     * Check if an event matches the given command, object, and index.
     */
    static bool EventIs(GenieFrame *e, uint8_t cmd,
                        uint8_t object, uint8_t index);

    /**
     * Extract the 16-bit data value from an event frame.
     */
    static uint16_t GetEventData(GenieFrame *e);

    /**
     * Dequeue the next event into the caller's buffer.
     * @return true if an event was copied, false if queue is empty.
     */
    bool DequeueEvent(GenieFrame *buff);

    /* -----------------------------------------------------------------
     * Diagnostics
     * ----------------------------------------------------------------- */

    /** Return the last error code (GENIE_ERROR_*). */
    int GetError(void) const { return m_error; }

    /** Return the cumulative timeout count. */
    int GetTimeouts(void) const { return m_timeouts; }

    /** Return the cumulative fatal-error count. */
    int GetFatalErrors(void) const { return m_fatalErrors; }

private:
    /* --- Internal helpers --- */
    void        FlushEventQueue(void);
    void        HandleError(void);
    void        SetLinkState(uint16_t newstate);
    uint16_t    GetLinkState(void);
    bool        EnqueueEvent(uint8_t *data);
    uint8_t     Getchar(void);
    uint16_t    GetcharSerial(void);
    void        WaitForIdle(void);
    void        PushLinkState(uint8_t newstate);
    void        PopLinkState(void);
    void        FatalError(void);
    void        FlushSerialInput(void);
    void        Resync(void);

    /* --- State --- */
    ClearCore::ISerial *m_serial;           /* serial port handle */

    GenieEventQueue     m_eventQueue;       /* circular event buffer */

    uint8_t  m_linkStates[MAX_LINK_STATES]; /* link state stack */
    uint8_t *m_linkState;                   /* stack pointer */
    int      m_linkCount;                   /* stack depth */

    int      m_timeout;                     /* GetChar timeout (ms) */
    int      m_timeouts;                    /* cumulative timeout count */
    int      m_error;                       /* last error code */
    uint8_t  m_rxFrameCount;               /* bytes received in current frame */
    int      m_fatalErrors;                 /* cumulative fatal error count */

    GenieEventHandler           m_userHandler;
    GenieMagicByteHandler       m_userByteReader;
    GenieMagicDoubleByteHandler m_userDoubleByteReader;
};

#endif /* GENIE_DISPLAY_H_ */
