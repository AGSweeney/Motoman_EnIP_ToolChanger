/*
 * DX200 Rotary Tool Changer -- Main Entry Point
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 */

#include "ClearCore.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "ports/ClearCore/opener.h"
#include "ciptcpipinterface.h"
#include <stdio.h>

/* Tool changer wrapper -- provides ToolChanger_*() API and constants */
#include "ports/ClearCore/clearcore_wrapper.h"

/* State name lookup for debug printing */
static const char *tc_state_names[] = {
    "DISABLED", "HOMING", "IDLE", "MOVING", "AT_TOOL", "FAULTED"
};

int main(void) {
    /* ================================================================
     * Motor hardware setup -- MUST happen first, before any other
     * subsystem initialization, per ClearCore requirements.
     * ================================================================ */
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                           Connector::CPM_MODE_STEP_AND_DIR);

    /* HLFB configuration for ClearPath SDSK */
    ConnectorM0.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    ConnectorM0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    /* Hard-coded motion limits (500 RPM, 1000 RPM/s^2 pre-gearbox) */
    ConnectorM0.VelMax(30000);    /* 500 RPM    = 30000 pulses/sec    */
    ConnectorM0.AccelMax(60000);  /* 1000 RPM/s^2 = 60000 pulses/sec^2 */

    ConnectorUsb.PortOpen();
    Delay_ms(100);

    ConnectorUsb.SendLine("\r\n========================================");
    ConnectorUsb.SendLine("  DX200 Rotary Tool Changer -- ClearCore");
    ConnectorUsb.SendLine("  SDSK 3600ppr | 10:1 Gearbox | 6 Pockets");
    ConnectorUsb.SendLine("========================================\r\n");
    ConnectorUsb.SendLine("Motor M-0 configured (Step+Dir, HLFB bipolar PWM)");
    ConnectorUsb.SendLine("Waiting for Ethernet link...");
    ConnectorUsb.Flush();

    uint32_t linkWaitStart = Milliseconds();
    while (!EthernetMgr.PhyLinkActive()) {
        if (Milliseconds() - linkWaitStart > 5000) {
            ConnectorUsb.SendLine("ERROR: Ethernet link timeout!");
            while (true) {
                Delay_ms(1000);
            }
        }
        Delay_ms(100);
    }

    ConnectorUsb.SendLine("Ethernet link detected!\r\n");
    ConnectorUsb.Flush();

    EthernetMgr.Setup();
    Delay_ms(100);

    ConnectorUsb.SendLine("Initializing network (OpENer will load saved config or use defaults)...");
    bool dhcpSuccess = EthernetMgr.DhcpBegin();
    if (!dhcpSuccess) {
        ConnectorUsb.SendLine("DHCP failed, using default static IP");
        IpAddress ip = IpAddress(192, 168, 1, 100);
        EthernetMgr.LocalIp(ip);
        EthernetMgr.NetmaskIp(IpAddress(255, 255, 255, 0));
        EthernetMgr.GatewayIp(IpAddress(192, 168, 1, 1));
    } else {
        ConnectorUsb.SendLine("DHCP successful (default, may be overridden by saved config)");
    }

    Delay_ms(500);

    struct netif *netif = EthernetMgr.MacInterface();
    if (netif == nullptr) {
        ConnectorUsb.SendLine("ERROR: Failed to get netif pointer!");
        while (true) {
            Delay_ms(1000);
        }
    }

    if (netif->ip_addr.addr != 0) {
        char ipStr[20];
        snprintf(ipStr, sizeof(ipStr), "IP Address: %d.%d.%d.%d",
                 ip4_addr1(&netif->ip_addr),
                 ip4_addr2(&netif->ip_addr),
                 ip4_addr3(&netif->ip_addr),
                 ip4_addr4(&netif->ip_addr));
        ConnectorUsb.SendLine(ipStr);
    } else {
        ConnectorUsb.SendLine("WARNING: IP address not assigned yet");
    }

    ConnectorUsb.SendLine("\r\n--- Initializing OpENer ---\r\n");
    ConnectorUsb.Flush();

    opener_init(netif);

    Delay_ms(500);
    ConnectorUsb.Flush();

    int opener_status = opener_get_status();
    if (opener_status == 0) {
        ConnectorUsb.SendLine("OpENer init: SUCCESS (g_end_stack=0)");
    } else {
        char statusMsg[50];
        snprintf(statusMsg, sizeof(statusMsg), "OpENer init: FAILED (g_end_stack=%d)", opener_status);
        ConnectorUsb.SendLine(statusMsg);
    }
    ConnectorUsb.SendLine("\r\n--- Initialization complete -- entering main loop ---\r\n");
    ConnectorUsb.Flush();

    /* ---- Main loop timing & link state tracking ---- */
    uint32_t lastOpenerCall  = 0;
    uint32_t lastStatusPrint = 0;
    uint32_t lastLedBlink    = 0;
    bool     ledState        = false;
    bool     prevLinkUp      = false;   /* track link transitions */

    while (true) {
        uint32_t currentTime = Milliseconds();

        /* Refresh the Ethernet stack */
        EthernetMgr.Refresh();

        /* OpENer cyclic processing (10 ms interval) */
        if (currentTime - lastOpenerCall >= 10) {
            opener_cyclic();
            lastOpenerCall = currentTime;
        }

        /* LED blink while interface config is pending */
        if ((g_tcpip.status & kTcpipStatusIfaceCfgPend) != 0) {
            if (currentTime - lastLedBlink >= 250) {
                ledState = !ledState;
                ConnectorLed.State(ledState);
                lastLedBlink = currentTime;
            }
        } else {
            if (ledState) {
                ConnectorLed.State(false);
                ledState = false;
            }
        }

        /* --- Network link transition logging (on change only) --- */
        {
            bool linkUp = EthernetMgr.PhyLinkActive();
            if (linkUp && !prevLinkUp) {
                /* Link just came up (or first detection) */
                char statusStr[80];
                if (netif->ip_addr.addr != 0) {
                    snprintf(statusStr, sizeof(statusStr),
                             "NET: Link UP  IP=%d.%d.%d.%d",
                             ip4_addr1(&netif->ip_addr),
                             ip4_addr2(&netif->ip_addr),
                             ip4_addr3(&netif->ip_addr),
                             ip4_addr4(&netif->ip_addr));
                } else {
                    snprintf(statusStr, sizeof(statusStr), "NET: Link UP  (no IP yet)");
                }
                ConnectorUsb.SendLine(statusStr);
            } else if (!linkUp && prevLinkUp) {
                /* Link just went down */
                ConnectorUsb.SendLine("NET: Link DOWN");
            }
            prevLinkUp = linkUp;
        }

        /* Periodic tool changer status (every 5 seconds) */
        if (currentTime - lastStatusPrint >= 5000) {
            ToolChangerState tcState = ToolChanger_GetState();
            uint8_t  tcTool    = ToolChanger_GetCurrentTool();
            int32_t  tcPos     = ToolChanger_GetPosition();
            uint16_t tcFault   = ToolChanger_GetFaultCode();

            const char *stateName = "?";
            if ((int)tcState >= 0 && (int)tcState <= 5) {
                stateName = tc_state_names[(int)tcState];
            }

            char statusStr[120];
            snprintf(statusStr, sizeof(statusStr),
                     "TC:  State=%s Tool=%d Pos=%ld Fault=0x%04X",
                     stateName, tcTool, (long)tcPos, tcFault);
            ConnectorUsb.SendLine(statusStr);

            lastStatusPrint = currentTime;
        }
    }
}