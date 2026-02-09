/*
 * DX200 Rotary Tool Changer -- 4D Systems ViSi-Genie Display Driver
 *
 * ClearCore port of genieArduino v1.5.3.
 * See genie_display.h for API documentation.
 *
 * Original library:
 *   Copyright (c) 2012-2022 4D Systems Pty Ltd, Sydney, Australia
 *   Licensed under LGPL v3 -- see genieArduino COPYING for details.
 *
 * ClearCore port:
 *   Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 *   SPDX-License-Identifier: MIT
 */

#include "ClearCore.h"
#include "genie_display.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * Platform helpers
 *
 * ClearCore equivalents for Arduino primitives:
 *   millis()         -> Milliseconds()     (ClearCore global)
 *   Serial.available -> AvailableForRead()
 *   Serial.read      -> CharGet()          (returns SerialBase::EOB on empty)
 *   Serial.write     -> SendChar()
 * ========================================================================= */

/* =========================================================================
 * Constructor
 * ========================================================================= */
GenieDisplay::GenieDisplay()
    : m_serial(nullptr)
    , m_linkState(&m_linkStates[0])
    , m_linkCount(0)
    , m_timeout(GENIE_TIMEOUT_PERIOD)
    , m_timeouts(0)
    , m_error(GENIE_ERROR_NONE)
    , m_rxFrameCount(0)
    , m_fatalErrors(0)
    , m_userHandler(nullptr)
    , m_userByteReader(nullptr)
    , m_userDoubleByteReader(nullptr)
{
    m_linkStates[0] = GENIE_LINK_IDLE;
    m_eventQueue.rd_index = 0;
    m_eventQueue.wr_index = 0;
    m_eventQueue.n_events = 0;
}

/* =========================================================================
 * Begin -- attach to an already-configured serial port
 * ========================================================================= */
void GenieDisplay::Begin(ClearCore::ISerial &serial) {
    m_serial = &serial;
    PushLinkState(GENIE_LINK_IDLE);
    FlushEventQueue();
}

/* =========================================================================
 * Event data extraction
 * ========================================================================= */
uint16_t GenieDisplay::GetEventData(GenieFrame *e) {
    return ((uint16_t)e->reportObject.data_msb << 8)
         | (uint16_t)e->reportObject.data_lsb;
}

/* =========================================================================
 * GetNextByte / GetNextDoubleByte -- blocking reads
 * ========================================================================= */
uint8_t GenieDisplay::GetNextByte(void) {
    while (m_serial->AvailableForRead() < 1) {
        /* spin -- caller is expected to use these only inside
         * magic-byte handlers where data is known to be arriving */
    }
    return (uint8_t)(m_serial->CharGet() & 0xFF);
}

uint16_t GenieDisplay::GetNextDoubleByte(void) {
    uint16_t out;
    while (m_serial->AvailableForRead() < 1) { }
    out = ((uint16_t)(m_serial->CharGet() & 0xFF)) << 8;
    while (m_serial->AvailableForRead() < 1) { }
    out |= (uint16_t)(m_serial->CharGet() & 0xFF);
    return out;
}

/* =========================================================================
 * EventIs -- compare event fields
 * ========================================================================= */
bool GenieDisplay::EventIs(GenieFrame *e, uint8_t cmd,
                           uint8_t object, uint8_t index)
{
    return (e->reportObject.cmd    == cmd    &&
            e->reportObject.object == object &&
            e->reportObject.index  == index);
}

/* =========================================================================
 * WaitForIdle -- block until the link is idle or timeout
 * ========================================================================= */
void GenieDisplay::WaitForIdle(void) {
    uint16_t do_event_result;
    uint32_t timeout = Milliseconds() + (uint32_t)m_timeout;

    while (Milliseconds() < timeout) {
        do_event_result = DoEvents(false);

        /* If we received a character, restart the timeout because
         * DoEvents is in the process of receiving something. */
        if (do_event_result == GENIE_EVENT_RXCHAR) {
            timeout = Milliseconds() + (uint32_t)m_timeout;
        }

        if (GetLinkState() == GENIE_LINK_IDLE) {
            return;
        }
    }

    m_error = GENIE_ERROR_TIMEOUT;
    HandleError();
}

/* =========================================================================
 * Link state stack (FILO)
 * ========================================================================= */
void GenieDisplay::PushLinkState(uint8_t newstate) {
    if (m_linkCount >= MAX_LINK_STATES) {
        Resync();
    }
    m_linkCount++;
    m_linkState++;
    SetLinkState(newstate);
}

void GenieDisplay::PopLinkState(void) {
    if (m_linkState > &m_linkStates[0]) {
        *m_linkState = 0xFF;
        m_linkState--;
        m_linkCount--;
    }
}

void GenieDisplay::SetLinkState(uint16_t newstate) {
    *m_linkState = (uint8_t)newstate;
    if (newstate == GENIE_LINK_RXREPORT ||
        newstate == GENIE_LINK_RXEVENT) {
        m_rxFrameCount = 0;
    }
}

uint16_t GenieDisplay::GetLinkState(void) {
    return *m_linkState;
}

/* =========================================================================
 * DoEvents -- heart of the Genie comms state machine
 * ========================================================================= */
uint16_t GenieDisplay::DoEvents(bool doHandler) {
    uint8_t c;
    static uint8_t  rx_data[6];
    static uint8_t  checksum = 0;
    static GenieMagicReportHeader magicHeader;
    static uint8_t  magicByte = 0;

    c = Getchar();

    /* ---- No character available: call user handler if events queued ---- */
    if (m_error == GENIE_ERROR_NOCHAR) {
        if ((m_eventQueue.n_events > 0) &&
            (m_userHandler != nullptr) && doHandler) {
            m_userHandler();
        }
        return GENIE_EVENT_NONE;
    }

    /* ---- Main state machine ---- */
    switch (GetLinkState()) {

    case GENIE_LINK_IDLE:
        switch (c) {
        case GENIE_REPORT_EVENT:
            PushLinkState(GENIE_LINK_RXEVENT);
            break;
        case GENIEM_REPORT_BYTES:
            magicByte = 0;
            PushLinkState(GENIE_LINK_RXMBYTES);
            break;
        case GENIEM_REPORT_DBYTES:
            magicByte = 0;
            PushLinkState(GENIE_LINK_RXMDBYTES);
            break;
        default:
            return GENIE_EVENT_RXCHAR;
        }
        break;

    case GENIE_LINK_WFAN:
        switch (c) {
        case GENIE_ACK:
            PopLinkState();
            return GENIE_EVENT_RXCHAR;
        case GENIE_NAK:
            PopLinkState();
            m_error = GENIE_ERROR_NAK;
            HandleError();
            return GENIE_EVENT_RXCHAR;
        case GENIE_REPORT_EVENT:
            PushLinkState(GENIE_LINK_RXEVENT);
            break;
        case GENIEM_REPORT_BYTES:
            magicByte = 0;
            PushLinkState(GENIE_LINK_RXMBYTES);
            break;
        case GENIEM_REPORT_DBYTES:
            magicByte = 0;
            PushLinkState(GENIE_LINK_RXMDBYTES);
            break;
        case GENIE_REPORT_OBJ:
        default:
            return GENIE_EVENT_RXCHAR;
        }
        break;

    case GENIE_LINK_WF_RXREPORT:
        switch (c) {
        case GENIE_REPORT_EVENT:
            PushLinkState(GENIE_LINK_RXEVENT);
            break;
        case GENIEM_REPORT_BYTES:
            magicByte = 0;
            PushLinkState(GENIE_LINK_RXMBYTES);
            break;
        case GENIEM_REPORT_DBYTES:
            magicByte = 0;
            PushLinkState(GENIE_LINK_RXMDBYTES);
            break;
        case GENIE_REPORT_OBJ:
            PopLinkState();
            PushLinkState(GENIE_LINK_RXREPORT);
            break;
        case GENIE_ACK:
        case GENIE_NAK:
        default:
            return GENIE_EVENT_RXCHAR;
        }
        /* fall through to frame accumulation */

    case GENIE_LINK_RXREPORT:
    case GENIE_LINK_RXEVENT:
    case GENIE_LINK_RXMBYTES:
    case GENIE_LINK_RXMDBYTES:
    default:
        break;
    }

    /* ---- Report / event frame accumulation ---- */
    if (GetLinkState() == GENIE_LINK_RXREPORT ||
        GetLinkState() == GENIE_LINK_RXEVENT) {

        checksum = (m_rxFrameCount == 0) ? c : checksum ^ c;
        rx_data[m_rxFrameCount] = c;

        if (m_rxFrameCount == GENIE_FRAME_SIZE - 1) {
            if (checksum == 0) {
                EnqueueEvent(rx_data);
                m_rxFrameCount = 0;
                PopLinkState();
                return GENIE_EVENT_RXCHAR;
            } else {
                m_error = GENIE_ERROR_BAD_CS;
                HandleError();
            }
        }

        m_rxFrameCount++;
        return GENIE_EVENT_RXCHAR;
    }

    /* ---- Magic report frame handling ---- */
    if (GetLinkState() == GENIE_LINK_RXMBYTES ||
        GetLinkState() == GENIE_LINK_RXMDBYTES) {

        switch (magicByte) {
        case 0:
            magicHeader.cmd = c;
            magicByte++;
            break;
        case 1:
            magicHeader.index = c;
            magicByte++;
            break;
        case 2:
            magicHeader.length = c;
            magicByte++;

            if (magicHeader.cmd == GENIEM_REPORT_BYTES) {
                if (m_userByteReader != nullptr) {
                    m_userByteReader(magicHeader.index, magicHeader.length);
                } else {
                    /* No handler -- sink the bytes */
                    uint8_t remaining = magicHeader.length;
                    while (remaining-- > 0) {
                        (void)GetNextByte();
                    }
                }
            } else if (magicHeader.cmd == GENIEM_REPORT_DBYTES) {
                if (m_userDoubleByteReader != nullptr) {
                    m_userDoubleByteReader(magicHeader.index,
                                           magicHeader.length);
                } else {
                    uint8_t remaining = magicHeader.length;
                    while (remaining-- > 0) {
                        (void)GetNextDoubleByte();
                    }
                }
            }
            /* Discard the trailing checksum byte */
            (void)GetNextByte();
            PopLinkState();
            break;
        }
        return GENIE_EVENT_RXCHAR;
    }

    return GENIE_EVENT_RXCHAR;
}

/* =========================================================================
 * Getchar / GetcharSerial -- non-blocking single-byte read
 * ========================================================================= */
uint8_t GenieDisplay::Getchar(void) {
    m_error = GENIE_ERROR_NONE;
    return (uint8_t)GetcharSerial();
}

uint16_t GenieDisplay::GetcharSerial(void) {
    if (m_serial == nullptr || m_serial->AvailableForRead() == 0) {
        m_error = GENIE_ERROR_NOCHAR;
        return (uint16_t)GENIE_ERROR_NOCHAR;
    }
    return (uint16_t)(m_serial->CharGet() & 0xFF);
}

/* =========================================================================
 * Error handling
 * ========================================================================= */
void GenieDisplay::FatalError(void) {
    if (m_fatalErrors++ > MAX_GENIE_FATALS) {
        /* Future: could transition to GENIE_LINK_SHDN */
    }
}

void GenieDisplay::HandleError(void) {
    /* Placeholder for future error recovery / debug logging */
}

void GenieDisplay::FlushSerialInput(void) {
    if (m_serial == nullptr) return;
    while (m_serial->CharGet() >= 0) { }
}

void GenieDisplay::Resync(void) {
    FlushSerialInput();
    FlushEventQueue();
    m_timeouts  = 0;
    m_linkCount = 0;
    m_linkState = &m_linkStates[0];
    *m_linkState = GENIE_LINK_IDLE;
}

/* =========================================================================
 * Event queue
 * ========================================================================= */
void GenieDisplay::FlushEventQueue(void) {
    m_eventQueue.rd_index = 0;
    m_eventQueue.wr_index = 0;
    m_eventQueue.n_events = 0;
}

bool GenieDisplay::DequeueEvent(GenieFrame *buff) {
    if (m_eventQueue.n_events > 0) {
        memcpy(buff, &m_eventQueue.frames[m_eventQueue.rd_index],
               GENIE_FRAME_SIZE);
        m_eventQueue.rd_index++;
        m_eventQueue.rd_index &= MAX_GENIE_EVENTS - 1;
        m_eventQueue.n_events--;
        return true;
    }
    return false;
}

bool GenieDisplay::EnqueueEvent(uint8_t *data) {
    if (m_eventQueue.n_events < MAX_GENIE_EVENTS - 2) {
        /* De-duplication: if an event with the same cmd/object/index
         * is already queued, update its data in-place. */
        int j = m_eventQueue.wr_index;
        bool found = false;

        for (int i = m_eventQueue.n_events; i > 0; i--) {
            j--;
            if (j < 0) j = MAX_GENIE_EVENTS - 1;

            if (m_eventQueue.frames[j].reportObject.cmd    == data[0] &&
                m_eventQueue.frames[j].reportObject.object == data[1] &&
                m_eventQueue.frames[j].reportObject.index  == data[2]) {
                m_eventQueue.frames[j].reportObject.data_msb = data[3];
                m_eventQueue.frames[j].reportObject.data_lsb = data[4];
                found = true;
                break;
            }
        }

        if (!found) {
            memcpy(&m_eventQueue.frames[m_eventQueue.wr_index],
                   data, GENIE_FRAME_SIZE);
            m_eventQueue.wr_index++;
            m_eventQueue.wr_index &= MAX_GENIE_EVENTS - 1;
            m_eventQueue.n_events++;
            return true;
        }
    } else {
        m_error = GENIE_ERROR_REPLY_OVR;
        HandleError();
        return false;
    }
    return false;
}

/* =========================================================================
 * ReadObject -- send a read-object command
 * ========================================================================= */
bool GenieDisplay::ReadObject(uint16_t object, uint16_t index) {
    uint8_t checksum;

    WaitForIdle();

    m_error = GENIE_ERROR_NONE;
    m_serial->SendChar((uint8_t)GENIE_READ_OBJ);
    checksum  = GENIE_READ_OBJ;
    m_serial->SendChar((uint8_t)object);
    checksum ^= (uint8_t)object;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar(checksum);

    PushLinkState(GENIE_LINK_WF_RXREPORT);
    return true;
}

/* =========================================================================
 * WriteObject -- write a 16-bit value to a display object
 * ========================================================================= */
uint16_t GenieDisplay::WriteObject(uint16_t object, uint16_t index,
                                   uint16_t data)
{
    uint8_t msb = (uint8_t)(data >> 8);
    uint8_t lsb = (uint8_t)(data & 0xFF);
    uint8_t checksum;

    WaitForIdle();

    m_error = GENIE_ERROR_NONE;
    m_serial->SendChar((uint8_t)GENIE_WRITE_OBJ);
    checksum  = GENIE_WRITE_OBJ;
    m_serial->SendChar((uint8_t)object);
    checksum ^= (uint8_t)object;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar(msb);
    checksum ^= msb;
    m_serial->SendChar(lsb);
    checksum ^= lsb;
    m_serial->SendChar(checksum);

    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

/* =========================================================================
 * WriteIntLedDigits -- 16-bit, 32-bit float, 32-bit integer
 * ========================================================================= */
uint16_t GenieDisplay::WriteIntLedDigits(uint16_t index, int16_t data) {
    WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, (uint16_t)data);
    return 1;
}

uint16_t GenieDisplay::WriteIntLedDigits(uint16_t index, float data) {
    GenieFloatLongFrame frame;
    frame.floatValue = data;
    WriteObject(GENIE_OBJ_ILED_DIGITS_H, index, (uint16_t)frame.wordValue[1]);
    WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, (uint16_t)frame.wordValue[0]);
    return 1;
}

uint16_t GenieDisplay::WriteIntLedDigits(uint16_t index, int32_t data) {
    GenieFloatLongFrame frame;
    frame.longValue = data;
    WriteObject(GENIE_OBJ_ILED_DIGITS_H, index, (uint16_t)frame.wordValue[1]);
    WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, (uint16_t)frame.wordValue[0]);
    return 1;
}

/* =========================================================================
 * WriteContrast -- set display backlight level
 * ========================================================================= */
void GenieDisplay::WriteContrast(uint16_t value) {
    uint8_t checksum;

    WaitForIdle();

    m_serial->SendChar((uint8_t)GENIE_WRITE_CONTRAST);
    checksum  = GENIE_WRITE_CONTRAST;
    m_serial->SendChar((uint8_t)value);
    checksum ^= (uint8_t)value;
    m_serial->SendChar(checksum);

    PushLinkState(GENIE_LINK_WFAN);
}

/* =========================================================================
 * WriteStr -- ASCII string
 * ========================================================================= */
uint16_t GenieDisplay::WriteStr(uint16_t index, const char *string) {
    const char *p;
    uint8_t checksum;
    int len = (int)strlen(string);

    if (len > 255) return (uint16_t)-1;

    WaitForIdle();

    m_serial->SendChar((uint8_t)GENIE_WRITE_STR);
    checksum  = GENIE_WRITE_STR;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar((uint8_t)len);
    checksum ^= (uint8_t)len;

    for (p = string; *p; ++p) {
        m_serial->SendChar((uint8_t)*p);
        checksum ^= (uint8_t)*p;
    }

    m_serial->SendChar(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

/* =========================================================================
 * WriteStr -- numeric overloads using snprintf
 * ========================================================================= */
uint16_t GenieDisplay::WriteStr(uint16_t index, int32_t number) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)number);
    return WriteStr(index, buf);
}

uint16_t GenieDisplay::WriteStr(uint16_t index, uint32_t number) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)number);
    return WriteStr(index, buf);
}

uint16_t GenieDisplay::WriteStr(uint16_t index, double number, int digits) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f", digits, number);
    return WriteStr(index, buf);
}

/* =========================================================================
 * WriteStrU -- Unicode string
 * ========================================================================= */
uint16_t GenieDisplay::WriteStrU(uint16_t index, const uint16_t *string) {
    const uint16_t *p;
    uint8_t checksum;
    int len = 0;

    p = string;
    while (*p++) len++;

    if (len > 255) return (uint16_t)-1;

    WaitForIdle();

    m_serial->SendChar((uint8_t)GENIE_WRITE_STRU);
    checksum  = GENIE_WRITE_STRU;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar((uint8_t)len);
    checksum ^= (uint8_t)len;

    p = string;
    while (*p) {
        m_serial->SendChar((uint8_t)(*p >> 8));
        checksum ^= (uint8_t)(*p >> 8);
        m_serial->SendChar((uint8_t)(*p & 0xFF));
        checksum ^= (uint8_t)(*p & 0xFF);
        p++;
    }

    m_serial->SendChar(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

/* =========================================================================
 * WriteInhLabel -- inherent label commands
 * ========================================================================= */
uint16_t GenieDisplay::WriteInhLabel(uint16_t index) {
    WriteObject(GENIE_OBJ_ILABELB, index, (uint16_t)-1);
    return 0;
}

uint16_t GenieDisplay::WriteInhLabel(uint16_t index, const char *string) {
    const char *p;
    uint8_t checksum;
    int len = (int)strlen(string);

    if (len > 255) return (uint16_t)-1;

    WaitForIdle();

    m_serial->SendChar((uint8_t)GENIE_WRITE_INH_LABEL);
    checksum  = GENIE_WRITE_INH_LABEL;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar((uint8_t)len);
    checksum ^= (uint8_t)len;

    for (p = string; *p; ++p) {
        m_serial->SendChar((uint8_t)*p);
        checksum ^= (uint8_t)*p;
    }

    m_serial->SendChar(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

uint16_t GenieDisplay::WriteInhLabel(uint16_t index, int32_t number) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)number);
    return WriteInhLabel(index, buf);
}

uint16_t GenieDisplay::WriteInhLabel(uint16_t index, uint32_t number) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)number);
    return WriteInhLabel(index, buf);
}

uint16_t GenieDisplay::WriteInhLabel(uint16_t index, double number,
                                     int digits)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f", digits, number);
    return WriteInhLabel(index, buf);
}

/* =========================================================================
 * WriteMagicBytes -- byte array to a Magic object
 * ========================================================================= */
uint16_t GenieDisplay::WriteMagicBytes(uint16_t index,
                                       const uint8_t *bytes,
                                       uint16_t len)
{
    uint8_t checksum;

    if (len > 255) return (uint16_t)-1;

    WaitForIdle();

    m_serial->SendChar((uint8_t)GENIEM_WRITE_BYTES);
    checksum  = GENIEM_WRITE_BYTES;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar((uint8_t)len);
    checksum ^= (uint8_t)len;

    for (uint16_t i = 0; i < len; i++) {
        m_serial->SendChar(bytes[i]);
        checksum ^= bytes[i];
    }

    m_serial->SendChar(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

/* =========================================================================
 * WriteMagicDBytes -- 16-bit array to a Magic object
 * ========================================================================= */
uint16_t GenieDisplay::WriteMagicDBytes(uint16_t index,
                                        const uint16_t *shorts,
                                        uint16_t len)
{
    uint8_t checksum;

    if (len > 255) return (uint16_t)-1;

    WaitForIdle();

    m_serial->SendChar((uint8_t)GENIEM_WRITE_DBYTES);
    checksum  = GENIEM_WRITE_DBYTES;
    m_serial->SendChar((uint8_t)index);
    checksum ^= (uint8_t)index;
    m_serial->SendChar((uint8_t)len);
    checksum ^= (uint8_t)len;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t hi = (uint8_t)(shorts[i] >> 8);
        uint8_t lo = (uint8_t)(shorts[i] & 0xFF);
        m_serial->SendChar(hi);
        checksum ^= hi;
        m_serial->SendChar(lo);
        checksum ^= lo;
    }

    m_serial->SendChar(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

/* =========================================================================
 * Callback registration
 * ========================================================================= */
void GenieDisplay::AttachEventHandler(GenieEventHandler handler) {
    m_userHandler = handler;
}

void GenieDisplay::AttachMagicByteReader(GenieMagicByteHandler handler) {
    m_userByteReader = handler;
}

void GenieDisplay::AttachMagicDoubleByteReader(
    GenieMagicDoubleByteHandler handler)
{
    m_userDoubleByteReader = handler;
}
