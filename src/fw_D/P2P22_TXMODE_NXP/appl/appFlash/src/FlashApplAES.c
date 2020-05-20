/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/GR_inwork/appl/appFlash/src/FlashIrq.c $
  $LastChangedRevision: 328482 $
  $LastChangedDate: 2015-07-22 13:17:23 -0600 (Wed, 22 Jul 2015) $
  $LastChangedBy: grueter $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 5.51.0
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

/** \file FlashApplAES.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/

#include <string.h>
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set.h"
#include "../../../firmware/stdc/src/ioATA5700.h"
#include "../../../firmware/aes/src/aes.h"
#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"
#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/timer5/src/timer5_flash.h"
#include "../../../firmware/globals/src/globals.h"
#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"
#include "../../../firmware/extif/src/extif_flash.h"
#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"
#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"
#include "FlashApplVars.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

#define ATA_DEFAULT_CONFIG_VAL            (AES_CONFIG_BM_CRYPTO_MODULE_RESET)

#define DATABLOCK_4_AES_MAX_LEN_BITS      0x80U   /* maximum length of data block to encrypt is 128 bits */
#define DATABLOCK_4_AES_MAX_LEN_BYTES     0x10U   /* maximum length of data block to encrypt is 16 bytes */
#define DATABLOCK_4_AES_PAD_INFO          0x80U   /* padding information */
#define DATABLOCK_4_AES_PAD_INFO_OFFSET   0x09U   /* offset to the beginning of padding information */
#define DATABLOCK_4_AES_PAD_INFO_LENGTH   0x01U   /* length of padding information in bytes */
#define DATABLOCK_4_AES_PAD_ZERO_OFFSET   (DATABLOCK_4_AES_PAD_OFFSET + DATABLOCK_4_AES_PAD_INFO_LENGTH)    /* offset to the beginning with zero(s) */
#define DATABLOCK_4_AES_PAD_ZERO_LENGTH   (DATABLOCK_4_AES_MAX_LEN_BYTES - DATABLOCK_4_AES_PAD_ZERO_OFFSET) /* length of padding info */
#define DATABLOCK_4_AES_PAD_OFFSET        0x09U   /* offset to the beginning of byte padding within data block to encrypt including padding information byte */
#define DATABLOCK_4_AES_PAD_LENGTH        (DATABLOCK_4_AES_MAX_LEN_BYTES - DATABLOCK_4_AES_PAD_OFFSET)      /* number of padding bytes within data block to encrypt */
/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/

extern sFlashApplState gFlashApplState;
extern sFlashApplVars gFlashApplVars;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*--------------------------------------------------------------------------- */
/*brief <b>ToDo AES function*/
            
/*    The function contains ---                                               */
 
/*----------------------------------------------------------------------------*/
void ATA_FlashAppMsgEncrypt(uint8_t* aesMsg, uint8_t msgLength, uint8_t bSecretKeyGroup, uint8_t bKeyId)
{
  
  if (msgLength > DATABLOCK_4_AES_MAX_LEN_BYTES) {
    // Error invalid length
    return;
  }
  
  ATA_aesSetConfig_C(bSecretKeyGroup, ATA_DEFAULT_CONFIG_VAL , bKeyId);
  
  memcpy(g_sAesComponentData.bDataBuffer, aesMsg, msgLength);
     
  if (msgLength < DATABLOCK_4_AES_MAX_LEN_BYTES) {
    // Pad with Zeroes
    ATA_globalsInitSramSpace_C( &g_sAesComponentData.bDataBuffer[msgLength], DATABLOCK_4_AES_MAX_LEN_BYTES - msgLength);
  }
  
  //g_sAesComponentData.bConfig &= ~AES_CONFIG_BM_CRYPTO_MODULE_RESET;
  //g_sAesComponentData.bConfig |= AES_CONFIG_BM_XOR_STATE_MEMORY;
     
  ATA_aesEncryptData_C();
  
  if( !(g_sAesComponentData.bFlags & AES_FLAGS_BM_ERROR_FLAG) )
  {
    memcpy(aesMsg, g_sAesComponentData.bDataBuffer, msgLength);
  }
}
/*----------------------------------------------------------------------------*/
