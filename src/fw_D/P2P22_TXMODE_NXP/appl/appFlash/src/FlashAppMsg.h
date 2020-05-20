/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2_Gen2_LF/appl/appFlash/src/FlashApplLF.h $
  $LastChangedRevision: 334201 $
  $LastChangedDate: 2015-08-21 08:06:40 -0600 (Fri, 21 Aug 2015) $
  $LastChangedBy: mhahnen $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.3.18.2236
-------------------------------------------------------------------------------

******************************************************************************
* Copyright 2017, Microchip Technology Incorporated and its subsidiaries.     *
*                                                                             *
* This software is owned by the Microchip Technology Incorporated.            *
* Microchip hereby grants to licensee a personal                              *
* non-exclusive, non-transferable license to copy, use, modify, create        *
* derivative works of, and compile the Microchip Source Code and derivative   *
* works for the sole and exclusive purpose of creating custom software in     *
* support of licensee product to be used only in conjunction with a Microchip *
* integrated circuit as specified in the applicable agreement. Any            *        
* reproduction, modification, translation, compilation, or representation of  *
* this software except as specified above is prohibited without the express   *
* written permission of Microchip.                                            *
*                                                                             *
* Disclaimer: MICROCHIP MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,    *
* WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.         *
* Microchip reserves the right to make changes without further notice to the  *
* materials described herein. Microchip does not assume any liability arising *
* out of the application or use of any product or circuit described herein.   *
* Microchip does not authorize its products for use as critical components in *
* life-support systems where a malfunction or failure may reasonably be       *
* expected to result in significant injury to the user. The inclusion of      *
* Microchip products in a life-support systems application implies that the   *
* manufacturer assumes all risk of such use and in doing so indemnifies       *
* Microchip against all charges.                                              *
*                                                                             *
* Use may be limited by and subject to the applicable Microchip software      *
* license agreement.                                                          *
******************************************************************************/

/** \file FlashAppMsg.h
*/

#ifndef FLASHAPPMSG_H
#define FLASHAPPMSG_H


//==============================================================================
// Local Include
//==============================================================================
//#include "utils.h"
//#include "rfmsg.h"
//#include "cfg_msg.h"


//==============================================================================
// Local Defines
//==============================================================================

//==============================================================================
// Local Macro
//==============================================================================

//==============================================================================
// Local Types
//==============================================================================



//==============================================================================
// Public Types
//==============================================================================
// Prepare type and max value for command code.
#if CFG_RKE_ROLLINGCODE_LENGTH == 1
#define RKE_ROLLINGCODE_TYPE UINT8
#elif CFG_RKE_ROLLINGCODE_LENGTH == 2
#define RKE_ROLLINGCODE_TYPE UINT16
#elif CFG_RKE_ROLLINGCODE_LENGTH == 4
#define RKE_ROLLINGCODE_TYPE UINT32
#else
#error Invalid command code size, must be 1, 2 or 4 bytes.
#endif


//==============================================================================
// Public Types - RF PEPS Messages
//==============================================================================
// PEPS message data content for read ID
typedef struct
{
  UINT32 fid;
  UINT8 vid[CFG_PEPS_VID_LENGTH];
  UINT8 cks;
} TX_MSG_PEPS_RD_ID_TS;

// PEPS message data content for parameter messages
typedef struct
{
  UINT8 vid[CFG_PEPS_VID_LENGTH];
  UINT8 param_index;
  UINT8 param_data[16];
  UINT8 cks;
} TX_MSG_PEPS_PARAM_TS;

// PEPS message data content for 2 WAY
typedef struct
{
  UINT8 rssi;
  UINT8 cks;
} TX_MSG_PEPS_2WAY_TS;

// PEPS message data content for unilateral and bilateral authent
typedef struct
{
  UINT8 rssi;
  UINT8 mac[CFG_PEPS_CHALLENGE_LENGTH];
} TX_MSG_PEPS_AUTH_TS;

// PEPS message data content for rssi test
typedef struct
{
  UINT8 norm_x;
  UINT8 norm_y;
  UINT8 norm_z;
  UINT8 ref_x;
  UINT8 ref_y;
  UINT8 ref_z;
  UINT8 ext_x;
  UINT8 ext_y;
  UINT8 ext_z;
  UINT8 int_x;
  UINT8 int_y;
  UINT8 int_z;
  UINT8 rss;
  UINT8 cks;
} TX_MSG_PEPS_LF_TST_TS;

// PEPS message data content for LF Param
typedef struct
{
  UINT8 norm_x;
  UINT8 norm_y;
  UINT8 norm_z;
  UINT8 ref_x;
  UINT8 ref_y;
  UINT8 ref_z;
  UINT8 cks;
} TX_MSG_PEPS_LF_PARAM_TS;

// PEPS message data content for SWID
typedef struct
{
  UINT8 swid[6];
  UINT8 cks;
} TX_MSG_PEPS_SWID_TS;

// PEPS message data content for DIAG
typedef struct
{
  UINT32 fid;
  UINT8 mode;
  UINT8 cks;
} TX_MSG_PEPS_MODE_TS;

// PEPS message type
typedef struct
{
  UINT8 preamble0[CFG_PEPS_PREAMBLE_LENGTH-1];
  UINT8 preamble1;
  UINT8 bat:1;
  UINT8 fidx:3;
  UINT8 cid:4;
  union
  {
    TX_MSG_PEPS_RD_ID_TS data_id;
    TX_MSG_PEPS_PARAM_TS data_param;
    TX_MSG_PEPS_2WAY_TS data_2way;
    TX_MSG_PEPS_AUTH_TS data_authent;
    TX_MSG_PEPS_LF_TST_TS data_lf_tst;
    TX_MSG_PEPS_LF_PARAM_TS data_lf_param;
    TX_MSG_PEPS_SWID_TS data_swid;
    TX_MSG_PEPS_MODE_TS data_mode;
  };
} TX_MSG_PEPS_TS;


//==============================================================================
// Public Types - LF PEPS Messages
//==============================================================================
// PEPS message data content for read ID
typedef struct
{
  UINT8 cks;
} RX_MSG_PEPS_RD_ID_TS;

// PEPS message data content for parameter messages
typedef struct
{
  UINT8 param_index;
  UINT8 cks;
} RX_MSG_PEPS_RD_PARAM_TS;

typedef struct
{
  UINT8 param_index;
  UINT8 param_data[16];
  UINT8 cks;
} RX_MSG_PEPS_WR_PARAM_TS;

// PEPS message data content for unilateral authent
typedef struct
{
  UINT8 challenge[CFG_PEPS_CHALLENGE_LENGTH];
} RX_MSG_PEPS_AUTH_UNI_TS;

// PEPS message data content for bilateral authent
typedef struct
{
  UINT8 challenge[CFG_PEPS_CHALLENGE_LENGTH];
  UINT8 cyph_challenge[CFG_PEPS_CHALLENGE_CYPH_LENGTH];
} RX_MSG_PEPS_AUTH_BI_TS;

// PEPS message data content for LF test
typedef struct
{
  UINT8 cks;
} RX_MSG_PEPS_LF_TST_TS;

// PEPS message data content for LF Param
typedef struct
{
  UINT8 norm_x;
  UINT8 norm_y;
  UINT8 norm_z;
  UINT8 ref_x;
  UINT8 ref_y;
  UINT8 ref_z;
  UINT8 cks;
} RX_MSG_PEPS_LF_PARAM_TS;

// PEPS message data content for SWID
typedef struct
{
  UINT8 cks;
} RX_MSG_PEPS_SWID_TS;

// PEPS message data content for DIAG
typedef struct
{
  UINT32 fid;
  UINT8 mode;
  UINT8 code[4];
  UINT8 cks;
} RX_MSG_PEPS_MODE_TS;

// PEPS message type
typedef struct
{
  UINT8 vid[CFG_PEPS_VID_LENGTH];
  UINT8 bat:1;
  UINT8 fidx:3;
  UINT8 cid:4;
  union
  {
    RX_MSG_PEPS_RD_ID_TS data_id;
    RX_MSG_PEPS_RD_PARAM_TS data_rd_param;
    RX_MSG_PEPS_WR_PARAM_TS data_wr_param;
    RX_MSG_PEPS_AUTH_UNI_TS data_auth_uni;
    RX_MSG_PEPS_AUTH_BI_TS data_auth_bi;
    RX_MSG_PEPS_LF_TST_TS data_lf_tst;
    RX_MSG_PEPS_LF_PARAM_TS data_lf_param;
    RX_MSG_PEPS_SWID_TS data_swid;
    RX_MSG_PEPS_MODE_TS data_mode;
  };
} RX_MSG_PEPS_TS;


//==============================================================================
// Public Types - RKE Messages
//==============================================================================
// RKE message type
typedef struct
{
  UINT8 preamble0[CFG_RKE_PREAMBLE_LENGTH-1];
  UINT8 preamble1;
  UINT8 serialNo[CFG_RKE_SERIAL_NO_LENGTH];
  RKE_ROLLINGCODE_TYPE counterValue;
  UINT8 commandCode[CFG_RKE_COMMAND_LENGTH];
  UINT8 mac[CFG_RKE_MAC_LENGTH];
} MSG_RKE_TS;



//==============================================================================
// Public Types - RF Message
//==============================================================================
// Structure for TX message
typedef union
{
  MSG_RKE_TS rke;
  TX_MSG_PEPS_TS peps;
} n;


//==============================================================================
// Public Defines
//==============================================================================

// size of RF RKE messages
// without preamble
#define RKE_SIZE_MSG          (CFG_RKE_SERIAL_NO_LENGTH+CFG_RKE_COMMAND_LENGTH\
                               +CFG_RKE_ROLLINGCODE_LENGTH+CFG_RKE_MAC_LENGTH) 
// with preamble
#define RKE_SIZE_MSG_FULL     (CFG_RKE_PREAMBLE_LENGTH+RKE_SIZE_MSG)


// size of RF PEPS messages
#define PEPS_SIZE_TX_MSG_COMMON      (1 + CFG_PEPS_PREAMBLE_LENGTH)
#define PEPS_SIZE_TX_MSG_ID          (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_RD_ID_TS))
#define PEPS_SIZE_TX_MSG_PARAM       (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_PARAM_TS))
#define PEPS_SIZE_TX_MSG_2WAY        (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_2WAY_TS))
#define PEPS_SIZE_TX_MSG_AUTH        (PEPS_SIZE_TX_MSG_COMMON+1+CFG_PEPS_CHALLENGE_LENGTH)
#define PEPS_SIZE_TX_MSG_LF_TST      (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_LF_TST_TS))
#define PEPS_SIZE_TX_MSG_LF_PARAM    (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_LF_PARAM_TS))
#define PEPS_SIZE_TX_MSG_SWID        (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_SWID_TS))
#define PEPS_SIZE_TX_MSG_MODE        (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_MODE_TS))

// size of LF PEPS messages
#define PEPS_SIZE_RX_MSG_COMMON   (1+CFG_PEPS_VID_LENGTH)
#define PEPS_SIZE_RX_MSG_ID       (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_RD_ID_TS))
#define PEPS_SIZE_RX_MSG_PARAM_RD (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_RD_PARAM_TS))
#define PEPS_SIZE_RX_MSG_PARAM_WR (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_WR_PARAM_TS))
#define PEPS_SIZE_RX_MSG_2WAY     (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_RD_ID_TS))
#define PEPS_SIZE_RX_MSG_AUTH_UNI (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_AUTH_UNI_TS))
#define PEPS_SIZE_RX_MSG_AUTH_BI  (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_AUTH_BI_TS))
#define PEPS_SIZE_RX_MSG_LF_TST   (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_LF_TST_TS))
#define PEPS_SIZE_RX_MSG_LF_PARAM (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_LF_PARAM_TS))
#define PEPS_SIZE_RX_MSG_SWID     (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_SWID_TS))
#define PEPS_SIZE_RX_MSG_MODE     (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_MODE_TS))



//==============================================================================
// Public Macro
//==============================================================================
#define MSG_TX_DATA  (*(TX_MSG_TU*)(msg_tx_buffer.aub_data))

//==============================================================================
// Public Variables Declarations
//==============================================================================
extern RFMSG_FRAME_TS msg_rx_buffer;
extern RFMSG_FRAME_TS msg_tx_buffer;


//==============================================================================
// Public functions Declarations
//==============================================================================




#endif /*APP_MSG_H_*/



