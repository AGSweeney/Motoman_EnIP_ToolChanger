/*******************************************************************************
 * Originally based on OpENer sample application
 * Copyright (c) 2012, Rockwell Automation, Inc.
 * All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause (adapted, see OpENer/license.txt)
 *
 * DX200 Tool Changer modifications and additions:
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * SPDX-License-Identifier: MIT
 *
 ******************************************************************************/

/*******************************************************************************
 * DX200 Rotary Tool Changer -- EtherNet/IP Assembly Definitions
 *
 * ClearPath SDSK motor, 3600 ppr, 10:1 gearbox
 * 36 000 counts per turret revolution, 6 000 counts per tool pocket
 *
 * Motor limits (hard-coded, pre-gearbox):
 *   Max velocity:       500 RPM        = 30 000 pulses/sec
 *   Max acceleration: 1 000 RPM/s^2    = 60 000 pulses/sec^2
 *   Homing velocity:    ~33 RPM        =  2 000 pulses/sec
 *
 * OUTPUT Assembly 150  (DX200 -> Tool Changer, 3 bytes)
 * -------------------------------------------------------
 * Byte 0, bits 0-2 : Tool Select (1-6, 0 = no command)
 * Byte 0, bits 3-7 : Reserved
 * Byte 1, bit 0    : Execute Move (rising-edge, or tool change while held)
 * Byte 1, bit 1    : Home Request (rising-edge starts homing)
 * Byte 1, bit 2    : Reserved (motor enable is via DI-8 hardware input)
 * Byte 1, bit 3    : Clear Faults (rising-edge)
 * Byte 1, bits 4-7 : Reserved
 * Byte 2, bits 0-2 : Reserved (IO-0..IO-2 are stacklight, firmware-driven)
 * Byte 2, bits 3-5 : Auxiliary digital outputs (IO-3..IO-5)
 * Byte 2, bits 6-7 : Reserved
 *
 * INPUT Assembly 100  (Tool Changer -> DX200, 6 bytes)
 * -------------------------------------------------------
 * Byte 0, bits 0-2 : Current Tool (1-6, 0 = unknown)
 * Byte 0, bit 3    : At Position (tool change complete, settled)
 * Byte 0, bit 4    : Home Complete
 * Byte 0, bit 5    : Motor Enabled
 * Byte 0, bit 6    : Fault Present
 * Byte 0, bit 7    : In Motion
 * Byte 1, bits 0-2 : State machine state (ToolChangerState enum, 0-5)
 * Byte 1, bit 3    : Info: Invalid Tool (non-fatal, self-clearing)
 * Byte 1, bit 4    : Info: Not Ready (non-fatal, self-clearing)
 * Byte 1, bit 5    : E-Stop Active (A-10 NC input low)
 * Byte 1, bit 6    : Tool In Pocket (DI-7 sensor, 1 = tool present)
 * Byte 1, bit 7    : Reserved
 * Bytes 2-3        : Fault Code (UINT16, little-endian, ToolChangerFault bitmask)
 * Byte 4           : Current Tool Pocket (1-6, 0 = unknown)
 * Byte 5           : Digital inputs (DI-6..DI-8, A-9..A-12, bits 0-6)
 *                    Note: A-9 doubles as the physical fault reset button
 *                    (rising-edge clears faults when in FAULTED state).
 *
 * CONFIG Assembly 151  (0 bytes, unused)
 ******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "opener_api.h"
#include "appcontype.h"
#include "trace.h"
#include "cipidentity.h"
#include "ciptcpipinterface.h"
#include "cipqos.h"
#include "ports/ClearCore/clearcore_wrapper.h"
#include "cipconnectionmanager.h"
#include "cipethernetlink.h"
#include "ports/ClearCore/dx200_toolchanger/ethlinkcbs.h"

/* ---- Assembly instance numbers ---- */
#define DEMO_APP_INPUT_ASSEMBLY_NUM                100
#define DEMO_APP_OUTPUT_ASSEMBLY_NUM               150
#define DEMO_APP_CONFIG_ASSEMBLY_NUM               151

/* ---- Assembly data buffers ---- */
EipUint8 g_assembly_data_input[6];    /* T->O  (Input, instance 100) */
EipUint8 g_assembly_data_output[3];   /* O->T  (Output, instance 150) */

/* ---- EIP scanner connection state (accessed from clearcore_wrapper) ---- */
volatile int g_eip_scanner_connected = 0;

/* ---- Output assembly bit masks ---- */
/* Byte 0: tool select only */
#define OUT_TOOL_SELECT_MASK    0x07  /* byte 0, bits 0-2: tool number 1-6 */

/* Byte 1: command bits */
#define OUT_EXECUTE_BIT         0x01  /* byte 1, bit 0: execute move */
#define OUT_HOME_BIT            0x02  /* byte 1, bit 1: home request */
/* byte 1, bit 2 (0x04): reserved -- motor enable is via DI-8 hardware input */
#define OUT_CLEAR_FAULTS_BIT    0x08  /* byte 1, bit 3: clear faults */

/* ---- Input assembly bit masks (byte 0) ---- */
#define IN_TOOL_MASK            0x07  /* bits 0-2: current tool 1-6 */
#define IN_AT_POSITION_BIT      0x08  /* bit 3: at commanded position */
#define IN_HOME_COMPLETE_BIT    0x10  /* bit 4: homing finished */
#define IN_MOTOR_ENABLED_BIT    0x20  /* bit 5: motor enabled */
#define IN_FAULT_BIT            0x40  /* bit 6: fault active */
#define IN_IN_MOTION_BIT        0x80  /* bit 7: turret is rotating */

/* ---- Input assembly bit masks (byte 1) ---- */
/* bits 0-2: state machine state (written as integer, not masked) */
#define IN_INFO_INVALID_TOOL    0x08  /* bit 3: invalid tool requested (non-fatal) */
#define IN_INFO_NOT_READY       0x10  /* bit 4: command ignored, not ready (non-fatal) */
#define IN_ESTOP_ACTIVE_BIT     0x20  /* bit 5: E-stop active (A-10 NC input low) */
#define IN_TOOL_IN_POCKET_BIT   0x40  /* bit 6: DI-7 tool-in-pocket sensor */

/* ---- Edge-detection state for rising-edge triggers ---- */
static EipUint8 prev_cmd_byte    = 0;   /* previous byte 1 (command bits) */
static EipUint8 prev_tool_select = 0;   /* track tool number changes */

EipStatus ApplicationInitialization(void) {
  CipRunIdleHeaderSetO2T(true);
  CipRunIdleHeaderSetT2O(false);

  /* -- Initialise digital I/O (retained for auxiliary use) -- */
  OPENER_TRACE_INFO("ApplicationInitialization: Initializing I/O...\n");
  ConnectorIO0_Initialize();
  ConnectorIO1_Initialize();
  ConnectorIO2_Initialize();
  ConnectorIO3_Initialize();
  ConnectorIO4_Initialize();
  ConnectorIO5_Initialize();
  OPENER_TRACE_INFO("ApplicationInitialization: IO-0..IO-5 as digital outputs\n");
  ConnectorDI6_Initialize();
  ConnectorDI7_Initialize();
  ConnectorDI8_Initialize();
  OPENER_TRACE_INFO("ApplicationInitialization: DI-6..DI-8 as digital inputs\n");
  ConnectorA9_Initialize();
  ConnectorA10_Initialize();
  ConnectorA11_Initialize();
  ConnectorA12_Initialize();
  OPENER_TRACE_INFO("ApplicationInitialization: A-9..A-12 as digital inputs\n");

  /* -- Initialise the ClearPath SDSK tool changer motor -- */
  OPENER_TRACE_INFO("ApplicationInitialization: Initializing tool changer motor...\n");
  ToolChanger_Initialize();

  /* -- Create EIP assembly objects -- */
  OPENER_TRACE_INFO("ApplicationInitialization: Creating assembly objects...\n");

  CreateAssemblyObject(DEMO_APP_INPUT_ASSEMBLY_NUM, g_assembly_data_input,
                       sizeof(g_assembly_data_input));
  OPENER_TRACE_INFO("ApplicationInitialization: Created input assembly %d (%d bytes)\n",
                    DEMO_APP_INPUT_ASSEMBLY_NUM, (int)sizeof(g_assembly_data_input));

  CreateAssemblyObject(DEMO_APP_OUTPUT_ASSEMBLY_NUM, g_assembly_data_output,
                       sizeof(g_assembly_data_output));
  OPENER_TRACE_INFO("ApplicationInitialization: Created output assembly %d (%d bytes)\n",
                    DEMO_APP_OUTPUT_ASSEMBLY_NUM, (int)sizeof(g_assembly_data_output));

  CreateAssemblyObject(DEMO_APP_CONFIG_ASSEMBLY_NUM, NULL, 0);
  OPENER_TRACE_INFO("ApplicationInitialization: Created config assembly %d (empty)\n",
                    DEMO_APP_CONFIG_ASSEMBLY_NUM);

  /* -- Configure the exclusive-owner connection point -- */
  ConfigureExclusiveOwnerConnectionPoint(0, DEMO_APP_OUTPUT_ASSEMBLY_NUM,
                                         DEMO_APP_INPUT_ASSEMBLY_NUM,
                                         DEMO_APP_CONFIG_ASSEMBLY_NUM);
  OPENER_TRACE_INFO("ApplicationInitialization: Exclusive owner connection configured\n");

#if defined(OPENER_ETHLINK_CNTRS_ENABLE) && 0 != OPENER_ETHLINK_CNTRS_ENABLE
  {
    CipClass *p_eth_link_class = GetCipClass(kCipEthernetLinkClassCode);
    InsertGetSetCallback(p_eth_link_class,
                         EthLnkPreGetCallback,
                         kPreGetFunc);
    InsertGetSetCallback(p_eth_link_class,
                         EthLnkPostGetCallback,
                         kPostGetFunc);
    for (int idx = 0; idx < OPENER_ETHLINK_INSTANCE_CNT; ++idx)
    {
      CipAttributeStruct *p_eth_link_attr;
      CipInstance *p_eth_link_inst =
        GetCipInstance(p_eth_link_class, idx + 1);
      OPENER_ASSERT(p_eth_link_inst);

      p_eth_link_attr = GetCipAttribute(p_eth_link_inst, 4);
      p_eth_link_attr->attribute_flags |= (kPreGetFunc | kPostGetFunc);
      p_eth_link_attr = GetCipAttribute(p_eth_link_inst, 5);
      p_eth_link_attr->attribute_flags |= (kPreGetFunc | kPostGetFunc);
    }
  }
#endif

  OPENER_TRACE_INFO("ApplicationInitialization: DX200 Tool Changer ready\n");
  return kEipStatusOk;
}

/*******************************************************************************
 * HandleApplication -- called cyclically from the OpENer main loop.
 * Drives the tool changer state machine and stacklight every scan.
 *
 * Stacklight (IO-0..IO-2, firmware-driven):
 *   IO-0 Green  = IDLE / AT_TOOL  (all systems go)
 *   IO-1 Amber  = MOVING (steady), HOMING (blinking 500 ms)
 *   IO-2 Red    = FAULTED (solid), E-STOP (blinking 500 ms)
 *   DISABLED    = all off
 ******************************************************************************/
#define STACKLIGHT_BLINK_MS  500  /* blink period for amber (homing) and red (E-stop) */

void HandleApplication(void) {
  ToolChanger_Cyclic();

  /* --- Stacklight outputs (IO-0 green, IO-1 amber, IO-2 red) --- */
  ToolChangerState st = ToolChanger_GetState();
  int estop = ToolChanger_IsEstopActive();

  ConnectorIO0_SetState(!estop &&
                        (st == TC_STATE_IDLE || st == TC_STATE_AT_TOOL));  /* green */

  /* Amber: steady on while moving, blink while homing, off during E-stop */
  if (!estop && st == TC_STATE_MOVING) {
    ConnectorIO1_SetState(1);
  } else if (!estop && st == TC_STATE_HOMING) {
    ConnectorIO1_SetState((GetMillis() / STACKLIGHT_BLINK_MS) & 1);
  } else {
    ConnectorIO1_SetState(0);
  }

  /* Red: blinking during E-stop, solid during fault, off otherwise */
  if (estop) {
    ConnectorIO2_SetState((GetMillis() / STACKLIGHT_BLINK_MS) & 1);
  } else {
    ConnectorIO2_SetState(st == TC_STATE_FAULTED);
  }
}

void CheckIoConnectionEvent(unsigned int output_assembly_id,
                            unsigned int input_assembly_id,
                            IoConnectionEvent io_connection_event) {

  (void) output_assembly_id;
  (void) input_assembly_id;

  if (io_connection_event == kIoConnectionEventOpened) {
    g_eip_scanner_connected = 1;
    OPENER_TRACE_INFO("CheckIoConnectionEvent: Scanner connected\n");
  }
  else if (io_connection_event == kIoConnectionEventClosed ||
           io_connection_event == kIoConnectionEventTimedOut) {
    g_eip_scanner_connected = 0;
    OPENER_TRACE_INFO("CheckIoConnectionEvent: Connection lost\n");

    memset(g_assembly_data_output, 0, sizeof(g_assembly_data_output));
    prev_cmd_byte    = 0;
    prev_tool_select = 0;
  }
}

/*******************************************************************************
 * AfterAssemblyDataReceived -- process commands from the DX200 (output assembly)
 *
 * OUTPUT Assembly 150 layout (3 bytes):
 *   Byte 0 [2:0] = Tool select 1-6 (tool number only)
 *   Byte 1 [0]   = Execute (rising edge or tool change while held)
 *   Byte 1 [1]   = Home (rising edge)
 *   Byte 1 [2]   = Reserved (enable via DI-8)
 *   Byte 1 [3]   = Clear faults (rising edge)
 *   Byte 2 [5:3] = Auxiliary digital outputs (IO-3..IO-5)
 *   Byte 2 [2:0] = Reserved (stacklight is firmware-driven)
 ******************************************************************************/
EipStatus AfterAssemblyDataReceived(CipInstance *instance) {
  EipStatus status = kEipStatusOk;

  switch (instance->instance_number) {
    case DEMO_APP_OUTPUT_ASSEMBLY_NUM: {
      EipUint8 cmd    = g_assembly_data_output[1];      /* command byte */
      EipUint8 rising = cmd & ~prev_cmd_byte;           /* detect rising edges */
      prev_cmd_byte   = cmd;

      /* Motor enable is handled by DI-8 hardware input in ToolChanger_Cyclic.
       * No enable/disable logic needed here. */

      /* --- Clear Faults (rising edge) --- */
      if (rising & OUT_CLEAR_FAULTS_BIT) {
        OPENER_TRACE_INFO("AfterAssemblyDataReceived: Clear faults requested\n");
        ToolChanger_ClearFaults();
      }

      /* --- Home Request (rising edge) --- */
      if (rising & OUT_HOME_BIT) {
        OPENER_TRACE_INFO("AfterAssemblyDataReceived: Home requested\n");
        ToolChanger_HomeStart();
      }

      /* --- Tool Select + Execute --- */
      /* Trigger ONLY on tool-number CHANGE while Execute is held high.
       * Execute rising edge alone does NOT command a move -- this prevents
       * re-commanding the old tool after homing or re-enabling.
       * The DX200 keeps Execute asserted and simply writes a new tool
       * number to byte 0 to command new moves. */
      {
        uint8_t tool = g_assembly_data_output[0] & OUT_TOOL_SELECT_MASK;
        int tool_changed = (tool != prev_tool_select) && (cmd & OUT_EXECUTE_BIT);

        if (tool != 0 && tool_changed) {
          OPENER_TRACE_INFO("EIP: SelectTool %d (prev=%d cmd=0x%02X)\n",
                            tool, prev_tool_select, cmd);
          ToolChanger_SelectTool(tool);
        }

        /* Only update prev_tool_select while Execute is asserted so that
         * tool changes made while Execute is low are remembered and
         * trigger a move the next cycle Execute is high with a new tool. */
        if (cmd & OUT_EXECUTE_BIT) {
          prev_tool_select = tool;
        }
      }

      /* --- Auxiliary digital outputs (byte 2, bits 3-5 -> IO-3..IO-5) ---
       * IO-0..IO-2 are reserved for the stacklight (driven in HandleApplication).
       * Bits 0-2 of byte 2 are ignored. */
      {
        EipUint8 aux = g_assembly_data_output[2];
        ConnectorIO3_SetState(aux & 0x08 ? 1 : 0);
        ConnectorIO4_SetState(aux & 0x10 ? 1 : 0);
        ConnectorIO5_SetState(aux & 0x20 ? 1 : 0);
      }

      break;
    }
    case DEMO_APP_CONFIG_ASSEMBLY_NUM:
      /* Config assembly is empty -- velocity/accel are hard-coded */
      status = kEipStatusOk;
      break;
    default:
      OPENER_TRACE_INFO("Unknown assembly instance in AfterAssemblyDataReceived\n");
      break;
  }
  return status;
}

/*******************************************************************************
 * BeforeAssemblyDataSend -- populate status for the DX200 (input assembly)
 *
 * INPUT Assembly 100 layout (6 bytes):
 *   Byte 0 [2:0] = Current tool (1-6, 0 = unknown)
 *   Byte 0 [3]   = At position
 *   Byte 0 [4]   = Home complete
 *   Byte 0 [5]   = Motor enabled
 *   Byte 0 [6]   = Fault present
 *   Byte 0 [7]   = In motion
 *   Byte 1 [2:0] = State machine state (enum, 0-5)
 *   Byte 1 [3]   = Info: Invalid tool (non-fatal)
 *   Byte 1 [4]   = Info: Not ready (non-fatal)
 *   Byte 1 [5]   = E-Stop Active (A-10 NC input low)
 *   Byte 1 [6]   = Tool in pocket (DI-7 sensor)
 *   Bytes 2-3    = Fault code (UINT16 LE)
 *   Byte 4       = Current tool pocket (1-6, 0 = unknown)
 *   Byte 5       = Digital inputs (DI-6..DI-8, A-9..A-12)
 ******************************************************************************/
EipBool8 BeforeAssemblyDataSend(CipInstance *pa_pstInstance) {
  if (pa_pstInstance->instance_number == DEMO_APP_INPUT_ASSEMBLY_NUM) {
    ToolChangerState state   = ToolChanger_GetState();
    uint8_t  current_tool    = ToolChanger_GetCurrentTool();
    uint16_t fault_code      = ToolChanger_GetFaultCode();
    int      is_moving       = ToolChanger_IsMoving();
    int      home_complete   = ToolChanger_IsHomeComplete();
    int      motor_ready     = ToolChanger_IsMotorReady();

    /* Clear the buffer before populating */
    memset(g_assembly_data_input, 0, sizeof(g_assembly_data_input));

    /* Byte 0: status bits */
    g_assembly_data_input[0]  = (current_tool & 0x07);
    if (state == TC_STATE_AT_TOOL) {
      g_assembly_data_input[0] |= IN_AT_POSITION_BIT;
    }
    if (home_complete) {
      g_assembly_data_input[0] |= IN_HOME_COMPLETE_BIT;
    }
    if (motor_ready) {
      g_assembly_data_input[0] |= IN_MOTOR_ENABLED_BIT;
    }
    if (fault_code != TC_FAULT_NONE) {
      g_assembly_data_input[0] |= IN_FAULT_BIT;
    }
    if (is_moving) {
      g_assembly_data_input[0] |= IN_IN_MOTION_BIT;
    }

    /* Byte 1: state (bits 0-2) + info codes (bits 3-4)
     *          + E-stop (bit 5) + tool-in-pocket (bit 6) */
    g_assembly_data_input[1] = (EipUint8)state;
    {
      uint8_t info = ToolChanger_GetInfoCode();
      if (info & TC_INFO_INVALID_TOOL) g_assembly_data_input[1] |= IN_INFO_INVALID_TOOL;
      if (info & TC_INFO_NOT_READY)    g_assembly_data_input[1] |= IN_INFO_NOT_READY;
    }
    if (ToolChanger_IsEstopActive()) {
      g_assembly_data_input[1] |= IN_ESTOP_ACTIVE_BIT;
    }
    if (ConnectorDI7_GetState()) {
      g_assembly_data_input[1] |= IN_TOOL_IN_POCKET_BIT;
    }

    /* Bytes 2-3: fault code (UINT16, little-endian) */
    g_assembly_data_input[2] = (EipUint8)(fault_code & 0xFF);
    g_assembly_data_input[3] = (EipUint8)((fault_code >> 8) & 0xFF);

    /* Byte 4: current tool pocket (1-6, 0 = unknown) */
    g_assembly_data_input[4] = current_tool;

    /* Byte 5: digital inputs packed */
    g_assembly_data_input[5] = 0;
    if (ConnectorDI6_GetState()) g_assembly_data_input[5] |= 0x01;
    if (ConnectorDI7_GetState()) g_assembly_data_input[5] |= 0x02;
    if (ConnectorDI8_GetState()) g_assembly_data_input[5] |= 0x04;
    if (ConnectorA9_GetState())  g_assembly_data_input[5] |= 0x08;
    if (ConnectorA10_GetState()) g_assembly_data_input[5] |= 0x10;
    if (ConnectorA11_GetState()) g_assembly_data_input[5] |= 0x20;
    if (ConnectorA12_GetState()) g_assembly_data_input[5] |= 0x40;
  }
  return true;
}

EipStatus ResetDevice(void) {
  OPENER_TRACE_INFO("ResetDevice: Closing connections and updating QoS...\n");
  CloseAllConnections();
  CipQosUpdateUsedSetQosValues();
  OPENER_TRACE_INFO("ResetDevice: Rebooting device...\n");
  ClearCoreRebootDevice();
  return kEipStatusOk;
}

EipStatus ResetDeviceToInitialConfiguration(void) {
  OPENER_TRACE_INFO("ResetDeviceToInitialConfiguration: Resetting to factory defaults...\n");
  g_tcpip.encapsulation_inactivity_timeout = 120;
  CipQosResetAttributesToDefaultValues();
  OPENER_TRACE_INFO("ResetDeviceToInitialConfiguration: Clearing NVRAM...\n");
  ClearCoreClearNvram();
  OPENER_TRACE_INFO("ResetDeviceToInitialConfiguration: Rebooting device...\n");
  ClearCoreRebootDevice();
  return kEipStatusOk;
}

void*
CipCalloc(size_t number_of_elements,
          size_t size_of_element) {
  return calloc(number_of_elements, size_of_element);
}

void CipFree(void *data) {
  free(data);
}

void RunIdleChanged(EipUint32 run_idle_value) {
  OPENER_TRACE_INFO("Run/Idle handler triggered\n");
  if ((0x0001 & run_idle_value) == 1) {
    CipIdentitySetExtendedDeviceStatus(kAtLeastOneIoConnectionInRunMode);
  } else {
    CipIdentitySetExtendedDeviceStatus(
        kAtLeastOneIoConnectionEstablishedAllInIdleMode);
  }
  (void) run_idle_value;
}

