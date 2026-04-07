/*****************************************************************************
* DNP Protocol Module Header
* Filename: DNP.h
*
* Original Author:  Igor Kordunsky
* Date: 2025-01-06
*
* File Description: DNP3 protocol handler declarations
*                   Extracted from main.c for better modularity
*
*****************************************************************************/

#ifndef DNP_H
#define DNP_H

#include "GLOBALS.h"
#include "Structure_defs.h"
#include <string.h> // for memset, memcpy
#ifndef PC
#include "hardware.h"
#include <intrinsics.h>
#endif // PC

/*--------------------------- DNP Protocol Constants -------------------------*/
// Already defined in GLOBALS.H:
// - DNP_FRAME_SIZE, CLASS_0, CLASS_1, etc. (enum DNP_Replies)
// - DLL Function Codes (enum DLL_Function_Codes)
// - APP Function Codes (enum APP_Function_Codes)

/*--------------------------- DNP Global Variables ---------------------------*/
// These are defined in main.c and used by DNP.c

extern uint16 crc;                          // CRC calculation result
extern uint8 wrk_str[];                     // Work string for building DNP messages
extern uint8 iien1;                         // DNP internal indication 1
extern uint8 iien2;                         // DNP internal indication 2
extern uint8 ovr_flow;                      // Buffer overflow flag
extern uint8 events_record_num;             // Number of event records
extern uint8 events_requested;              // Events requested flag
extern uint8 sequence_num;                  // DNP sequence number
extern uint8 all_stations_msg;              // Broadcast message flag
extern uint8 pending_confirm;               // Confirmation pending flag
extern volatile uint8 comm_state;				// works with enum UART_Events

#define CONFIRM_PENDING     1
#define NO_CONFIRM_PENDING  0

#ifdef LAST_GASP
    #define EXTRA_DNP_BYTE 1
#else
    #define EXTRA_DNP_BYTE 0
#endif

/*--------------------------- DNP Function Prototypes ------------------------*/

// Core DNP message handling
void Send_DNP_Msg(uint16 type);

// DNP Data Link Layer functions
void Build_DNP_DLL_Header(uint8* buf, uint8 length, uint8 control);

// DNP utility functions
void Calc_DNPCRC(uint8* p, uint16 count);

int Add_DNP_BinaryLongToWorkString(Int32 value, int start_pos);
int AddBinaryFloatToWorkString(float value, int start_pos);

// UART transmission
void Send_232(uint8 num_bytes);            // Defined in main.c, sends data via UART

// DNP message parsing (if needed - check if these are in DNP.c)
// void Parse_DNP_Request(void);           // Uncomment if DNP.c contains this
// void Process_DNP_Command(void);         // Uncomment if DNP.c contains this

#endif // DNP_H
