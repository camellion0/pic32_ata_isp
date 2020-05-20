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

#ifndef FLASHAPPLMSG_H
#define FLASHAPPLMSG_H


//==============================================================================
// Local Include
//==============================================================================
#include "FlashApplPEPS.h"


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
//==============================================================================
// RKE message fields configuration
//==============================================================================
#define CFG_RKE_PREAMBLE_LENGTH    16 // PREAMBLE length (in bytes: min 1) 
#define CFG_RKE_SERIAL_NO_LENGTH   4  // FOB ID length (values allowed: 1/2/4) 
#define CFG_RKE_COMMAND_LENGTH     1  // RKE Command ID (values allowed: 1/2/4)
#define CFG_RKE_ROLLINGCODE_LENGTH 4  // RKE Rolling Code (values allowed: 1/2/4)
#define CFG_RKE_MAC_LENGTH         4  // RKE MAC (values allowed: 1/2/4) 
//==============================================================================
// PEPS messages fields configuration
//==============================================================================
#define CFG_PEPS_PREAMBLE_LENGTH        4  // PREAMBLE length (in bytes: min 1)//Was 16 GR was 4 12 Apr 2016
#define CFG_PEPS_CID_LENGTH             1  // Command ID length (in bytes)
#define CFG_PEPS_RSSI_LENGTH            3  // RSSI length (Value + Flag) (in bytes)
#define CFG_PEPS_FID_LENGTH             4  // Fob ID length (in bytes)
#define CFG_PEPS_CHALLENGE_LENGTH       4  // PEPS CHALLENGE (in bytes)
#define CFG_PEPS_CHALLENGE_CYPH_LENGTH  4  // PEPS CYPHERED CHALLENGE (in bytes)
#define CFG_PEPS_VID_LENGTH             3  // PEPS VEHICLE ID (in bytes)
#define VID_LENGTH                      4
#define CFG_PEPS_PHASEDATA_LENGTH       4  // PEPS COIL PHASE DATA (bytes)

// Max Size of Frame (if != 16)
#define CFG_RFMSG_FRAME_LENGTH_MAX 48
//KarM_CH_20160121_extend the length to 48, because of the two bytes RSSI value;
//#define CFG_RFMSG_FRAME_LENGTH_MAX 34



// disable reception buffer if Fob do not manage reception
#define CFG_RFRX_BUFFER_SIZE 2


   
// size of RF RKE messages
// without preamble
#define RKE_SIZE_MSG          (CFG_RKE_SERIAL_NO_LENGTH+CFG_RKE_COMMAND_LENGTH\
                               +CFG_RKE_ROLLINGCODE_LENGTH+CFG_RKE_MAC_LENGTH) 
// with preamble
#define RKE_SIZE_MSG_FULL     (CFG_RKE_PREAMBLE_LENGTH+RKE_SIZE_MSG)



// Prepare type and max value for command code.
#if CFG_RKE_ROLLINGCODE_LENGTH == 1
#define RKE_ROLLINGCODE_TYPE uint8_t
#elif CFG_RKE_ROLLINGCODE_LENGTH == 2
#define RKE_ROLLINGCODE_TYPE UINT16
#elif CFG_RKE_ROLLINGCODE_LENGTH == 4
#define RKE_ROLLINGCODE_TYPE uint32_t
#else
#error Invalid command code size, must be 1, 2 or 4 bytes.
#endif
typedef struct
{
  uint8_t ub_size;
  uint8_t ub_rssi;
  uint8_t aub_data[CFG_RFMSG_FRAME_LENGTH_MAX]; 
} RFMSG_FRAME_TS;


//==============================================================================
// Public Types - RF PEPS Messages
//==============================================================================
// PEPS message data content for read 
typedef struct
{
  uint32_t fid;
  uint8_t vid[CFG_PEPS_VID_LENGTH];
  //uint8_t cks;
} TX_MSG_PEPS_RD_ID_TS;

// PEPS message data content for parameter messages
typedef struct
{
  uint8_t vid[CFG_PEPS_VID_LENGTH];
  uint8_t param_index;
  uint8_t param_data[16];
  uint8_t cks;
} TX_MSG_PEPS_PARAM_TS;

// PEPS message data content for 2 WAY
typedef struct
{
  uint8_t rssil;
  uint8_t rssih;
  uint8_t rssiFlag;
  uint8_t cks;
} TX_MSG_PEPS_2WAY_TS;

// PEPS message data content for unilateral and bilateral authent
typedef struct
{
  uint8_t rssil;     
  uint8_t rssih;
  uint8_t rssiFlag;
  uint8_t CoilPhase12;     
  uint8_t CoilPhase13;   
  uint8_t CoilPhase23;     
  uint8_t CoilPhase360;   
  uint8_t mac[CFG_PEPS_CHALLENGE_LENGTH];
} TX_MSG_PEPS_AUTH_TS;

// PEPS message data content for rssi test
typedef struct
{
//  uint8_t norm_x;
//  uint8_t norm_y;
//  uint8_t norm_z;
//  uint8_t ref_x;
//  uint8_t ref_y;
//  uint8_t ref_z;
//  uint8_t ext_x;
//  uint8_t ext_y;
// uint8_t ext_z;
//  uint8_t int_x;
//  uint8_t int_y;
//  uint8_t int_z;
// uint8_t rss;
//uint16_t norm_x; 
//uint16_t norm_y; 
//uint16_t norm_z;
uint8_t SD_12;
uint8_t SD_13;
uint8_t SD_23;
uint8_t SD_360;
uint16_t ref_x;
uint16_t ref_y;
uint16_t ref_z;
uint16_t ext_x;
uint16_t ext_y;
uint16_t ext_z;
uint16_t int_x;
uint16_t int_y;
uint16_t int_z;
uint16_t rss;
uint8_t rssiFlag;

} TX_MSG_PEPS_LF_TST_TS;

// PEPS message data content for LF Param
typedef struct
{
  
uint16_t norm_x;
uint16_t norm_y;
uint16_t norm_z;
uint16_t ref_x;
uint16_t ref_y;
uint16_t ref_z;  //Only one byte for ATAK51004-V1 compatability
/*NOt yet GeRy
  uint8_t norm_x;
  uint8_t norm_y;
  uint8_t norm_z;
  uint8_t ref_x;
  uint8_t ref_y;
  uint8_t ref_z;
*/
uint8_t cks;
} TX_MSG_PEPS_LF_PARAM_TS;

// PEPS message data content for SWID
typedef struct
{
  uint8_t swid[6];
  uint8_t cks;
} TX_MSG_PEPS_SWID_TS;

// PEPS message data content for DIAG
typedef struct
{
  uint32_t fid;
  uint8_t mode;
  uint8_t cks;
} TX_MSG_PEPS_MODE_TS;

// PEPS message type
typedef struct
{
  uint8_t preamble0[CFG_PEPS_PREAMBLE_LENGTH-1];
 // uint8_t preamble0[CFG_PEPS_PREAMBLE_LENGTH];
  uint8_t preamble1;
  uint8_t bat:1;
  uint8_t fidx:3;
  uint8_t cid:4;
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

typedef struct
{
 // uint8_t preamble0[CFG_RKE_PREAMBLE_LENGTH-1];
  uint8_t preamble0[CFG_RKE_PREAMBLE_LENGTH];
  uint8_t preamble1;
  uint8_t serialNo[CFG_RKE_SERIAL_NO_LENGTH];
  RKE_ROLLINGCODE_TYPE counterValue;
  uint8_t commandCode[CFG_RKE_COMMAND_LENGTH];
  uint8_t mac[CFG_RKE_MAC_LENGTH];
} MSG_RKE_TS;

typedef union
{
  MSG_RKE_TS rke;
  TX_MSG_PEPS_TS peps;
} TX_MSG_TU;



//==============================================================================
// Public Types - LF PEPS Messages
//==============================================================================
// PEPS message data content for read ID
typedef struct
{
  uint8_t cks;
} RX_MSG_PEPS_RD_ID_TS;

// PEPS message data content for parameter messages
typedef struct
{
  uint8_t param_index;
  uint8_t cks;
} RX_MSG_PEPS_RD_PARAM_TS;

typedef struct
{
  uint8_t param_index;
  uint8_t param_data[16];
  uint8_t cks;
} RX_MSG_PEPS_WR_PARAM_TS;

// PEPS message data content for unilateral authent
typedef struct
{
  uint8_t challenge[CFG_PEPS_CHALLENGE_LENGTH];
} RX_MSG_PEPS_AUTH_UNI_TS;

// PEPS message data content for bilateral authent
typedef struct
{
  uint8_t challenge[CFG_PEPS_CHALLENGE_LENGTH];
  uint8_t cyph_challenge[CFG_PEPS_CHALLENGE_CYPH_LENGTH];
} RX_MSG_PEPS_AUTH_BI_TS;

// PEPS message data content for LF test
typedef struct
{
  uint8_t cks;
} RX_MSG_PEPS_LF_TST_TS;

// PEPS message data content for LF Param
typedef struct
{
/*
  uint8_t norm_x;
  uint8_t norm_y;
  uint8_t norm_z;
  uint8_t ref_x;
  uint8_t ref_y;
  uint8_t ref_z;
  uint8_t cks;
*/
uint16_t norm_x;
uint16_t norm_y;
uint16_t norm_z;
uint16_t ref_x;
uint16_t ref_y;
uint16_t ref_z;
uint8_t cks;
} RX_MSG_PEPS_LF_PARAM_TS;

// PEPS message data content for SWID
typedef struct
{
  uint8_t cks;
} RX_MSG_PEPS_SWID_TS;

// PEPS message data content for DIAG
typedef struct
{
  uint32_t fid;
  uint8_t mode;
  uint8_t code[4];
  uint8_t cks;
} RX_MSG_PEPS_MODE_TS;

// PEPS message type
typedef struct
{
  uint8_t vid[CFG_PEPS_VID_LENGTH];
  uint8_t bat:1;
  uint8_t fidx:3;
  uint8_t cid:4;
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



//==============================================================================
// Public Types - RF Message
//==============================================================================
// Structure for TX message
/*typedef union
{
  MSG_RKE_TS rke;
  TX_MSG_PEPS_TS peps;
} TX_MSG_TU;*/


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
#define PEPS_SIZE_TX_MSG_COMMON      (1 + CFG_PEPS_PREAMBLE_LENGTH)//1+4
#define PEPS_SIZE_TX_MSG_ID          (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_RD_ID_TS))
#define PEPS_SIZE_TX_MSG_PARAM       (PEPS_SIZE_TX_MSG_COMMON+(sizeof(TX_MSG_PEPS_PARAM_TS)-1))//Dont include checksum GeRu
#define PEPS_SIZE_TX_MSG_2WAY        (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_2WAY_TS))
#define PEPS_SIZE_TX_MSG_AUTH        (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_AUTH_TS))
#define PEPS_SIZE_TX_MSG_LF_TST      (PEPS_SIZE_TX_MSG_COMMON+sizeof(TX_MSG_PEPS_LF_TST_TS))
#define PEPS_SIZE_TX_MSG_LF_PARAM    (PEPS_SIZE_TX_MSG_COMMON+(sizeof(TX_MSG_PEPS_LF_PARAM_TS)-1))//GeRu
#define PEPS_SIZE_TX_MSG_SWID        (PEPS_SIZE_TX_MSG_COMMON+(sizeof(TX_MSG_PEPS_SWID_TS)-1))//GeRu
#define PEPS_SIZE_TX_MSG_MODE        (PEPS_SIZE_TX_MSG_COMMON+(sizeof(TX_MSG_PEPS_MODE_TS)-1))//GeRu

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
#define MSG_TX_DATA  (*(TX_MSG_TU*)(g_MsgTXbuffer.aub_data))

//==============================================================================
// Public Variables Declarations
//==============================================================================
extern RFMSG_FRAME_TS g_MsgRXbuffer;
extern RFMSG_FRAME_TS g_MsgTXbuffer;


//==============================================================================
// Public functions Declarations
//==============================================================================




#endif /*APP_MSG_H_*/



