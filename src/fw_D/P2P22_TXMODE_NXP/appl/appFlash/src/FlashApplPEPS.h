/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2_Gen2_LF/appl/appFlash/src/FlashApplPEPS.h $
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

/** \file FlashApplPEPS.h
*/

#ifndef FLASHAPPLPEPS_H
#define FLASHAPPLPEPS_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/globals/src/globals.h"
#include "../../../firmware/spi/src/ata5700_command_set_flash.h"
#include "../../../firmware/system/src/system_flash.h"
#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/aes/src/aes.h"
#include "../../../firmware/eep/src/eep_flash.h"
#include "FlashApplMsg.h"



/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define LF_IDFRAMELENGTH        24 

// Wake-up ID 0 
#define CFG_LF_WUP0_LENGTH      24      // size of Wake-up ID (in bits)
#define CFG_LF_WUP0_IN_EEPROM           // Wake-up ID stored in EEprom
#define CFG_LF_WUP0_Adr         (uint16_t)(g_sEepFlashApp_RKEPEPS.aub_vid[0])

// Wake-up ID 1
#define CFG_LF_WUP1_LENGTH      24    // size of Wake-up ID (in bits)
#define CFG_LF_WUP1_Adr         (uint16_t)(g_sEepFlashApp_RKEPEPS.aub_vid[1])
#define CFG_LF_WUP1             0xFFFFFFFF

////////////////////////////////////////////////////////
// RX buffer configuration
////////////////////////////////////////////////////////
// RX buffer size (default 32 bytes)
//#define CFG_LF_BUFFER_SIZE  (24)  // max PEPS size command expected is 22
#define CFG_LF_BUFFER_SIZE  (32)  // max PEPS size command expected is 22

// list of PEPS LF/RF command ID
#define PEPS_CID_RD_ID        0
#define PEPS_CID_RD_PARAM     1
#define PEPS_CID_WR_PARAM     2
#define PEPS_CID_2WAY         4
#define PEPS_CID_UNI_AUTH     5
#define PEPS_CID_BI_AUTH      6
#define PEPS_CID_UNI_AUTH_SINGLE  7
#define PEPS_CID_BI_AUTH_SINGLE   8
#define PEPS_CID_LF_TST      10
#define PEPS_CID_LF_PARAM    11
#define PEPS_CID_SWID        14
#define PEPS_CID_MODE        15


// Number of Fobs managed in system (max 8)
#ifdef CFG_APP_2WAYRF
#define PEPS_NB_FOB_MAX        8
#else
#define PEPS_NB_FOB_MAX        4
#endif

// number of channels max 
#define PEPS_NB_CHANNEL_MAX    3

#define SLOT_COMMON     0x01
#define SLOT_SINGLES    0x02

#define INTERFRAME      3.0

#define RX_CHAN_LF0     0x08
#define RX_CHAN_LF1     0x10
#define RX_CHAN_LF_MSK  0x18

#define RX_WUID0    0x40
#define RX_WUID1    0x80
#define RX_CHAN_RF_MSK  0x07
#define RX_CHAN_LF_MSK  0x18
#define RX_WUID_MSK     0xC0


#define BM_CID          0xF0
#define BM_FIDX         0x0F
#define NO_SIGNDET      0
#define SIGNDET         1
#define LFRSSI_INT      1
#define LFRSSI_EXT      0

typedef enum 
{
  DIAG_OFF = 0,
  DIAG_OEM = 1,
  DIAG_AS = 2,
} DIAG_MODE_TE;


#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
/*stopSimExtraction*/
#endif

#endif /* FLASH_APP_PEPS_H */