//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Trunk/appl/appFlash_simTest/src/rfrcc/src/rfrcc_flash.h $
  $LastChangedRevision: 266862 $
  $LastChangedDate: 2014-06-11 23:41:26 -0600 (Wed, 11 Jun 2014) $
  $LastChangedBy: ajost $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.30.1
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
/** \file rfrcc_flash.h
*/
//lint -restore

#ifndef RFRCC_FLASH_H
#define RFRCC_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/aes/src/aes.h"
#include "../../../firmware/eep/src/eep.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/* defines for RollingCodeSequenceCounter message */
#define RFRCC_SERIAL_ID_LENGTH                  0x04U   /* length of transmitter unique serial ID */
#define RFRCC_ROLL_COUNT_LENGTH                 0x04U   /* length of rolling code sequence counter */
#define RFRCC_COMMAND_ID_LENGTH                 0x01U   /* length of command ID */
#define RFRCC_MAC_LENGTH                        0x04U   /* length of message authentication code */
//#define RFRCC_STUFF_BYTES_LENGTH               0x03U   /* length of stuffing bytes (16 byte message format) */

#define RFRCC_MSG_LENGTH                        (RFRCC_SERIAL_ID_LENGTH + RFRCC_ROLL_COUNT_LENGTH + \
                                                 RFRCC_COMMAND_ID_LENGTH + RFRCC_MAC_LENGTH)
/*#define RFRCC_MSG_LENGTH                       (RFRCC_SERIAL_ID_LENGTH + RFRCC_ROLL_COUNT_LENGTH + \
                                                 RFRCC_COMMAND_ID_LENGTH + RFRCC_MAC_LENGTH + RFRCC_STUFF_BYTES_LENGTH) */
#define RFRCC_MSG_LENGTH_EXCL_MAC               (RFRCC_SERIAL_ID_LENGTH + RFRCC_ROLL_COUNT_LENGTH + \
                                                 RFRCC_COMMAND_ID_LENGTH)

#define RFRCC_MSG_SERIAL_ID_OFFSET              0x00U   /* offset to the beginning of transmitter unique serial ID within message */
#define RFRCC_MSG_ROLL_COUNT_OFFSET             0x04U   /* offset to the beginning of rolling code sequence counter within message */
#define RFRCC_MSG_COMMAND_ID_OFFSET             0x08U   /* offset to the beginning of command ID within message */
#define RFRCC_MSG_MAC_OFFSET                    0x09U   /* offset to the beginning of message authentication code within message */
//#define RFRCC_MSG_3_STUFF_BYTES_OFFSET         0x0DU   /* offset to the beginning of stuffing bytes within message (16 byte message format) */

/* defines for data block to encrypt */
#define RFRCC_DATABLOCK_4_AES_MAX_LEN_BITS      0x80U   /* maximum length of data block to encrypt is 128 bits */
#define RFRCC_DATABLOCK_4_AES_MAX_LEN_BYTES     0x10U   /* maximum length of data block to encrypt is 16 bytes */
//#define RFRCC_DATABLOCK_4_AES_PAD_INFO          0x80U   /* padding information */
#define RFRCC_DATABLOCK_4_AES_PAD_INFO          0x00U   /* padding information chnaged for LDL/Atmel app*/
#define RFRCC_DATABLOCK_4_AES_PAD_INFO_OFFSET   0x09U   /* offset to the beginning of padding information */
#define RFRCC_DATABLOCK_4_AES_PAD_INFO_LENGTH   0x01U   /* length of padding information in bytes */
#define RFRCC_DATABLOCK_4_AES_PAD_ZERO_OFFSET   (RFRCC_DATABLOCK_4_AES_PAD_OFFSET + RFRCC_DATABLOCK_4_AES_PAD_INFO_LENGTH)    /* offset to the beginning with zero(s) */
#define RFRCC_DATABLOCK_4_AES_PAD_ZERO_LENGTH   (RFRCC_DATABLOCK_4_AES_MAX_LEN_BYTES - RFRCC_DATABLOCK_4_AES_PAD_ZERO_OFFSET) /* length of padding info */
#define RFRCC_DATABLOCK_4_AES_PAD_OFFSET        0x09U   /* offset to the beginning of byte padding within data block to encrypt including padding information byte */
#define RFRCC_DATABLOCK_4_AES_PAD_LENGTH        (RFRCC_DATABLOCK_4_AES_MAX_LEN_BYTES - RFRCC_DATABLOCK_4_AES_PAD_OFFSET)      /* number of padding bytes within data block to encrypt */

/* defines for sRfrccComponentData.bFlags */
#define RFRCC_FLAGS_RESET                       0x00U
#define RFRCC_FLAGS_ERROR_FLAG                  BIT_7
#define RFRCC_FLAGS_BM_ERROR_FLAG               BITMASK(RFRCC_FLAGS_ERROR_FLAG)

/* defines for sRfrccComponentData.bStatus */
#define RFRCC_STATUS_RESET                          0x00U
#define RFRCC_STATUS_UPDATE_EEPROM_READY_FLAG       BIT_2
#define RFRCC_STATUS_BM_UPDATE_EEPROM_READY_FLAG    BITMASK(RFRCC_STATUS_UPDATE_EEPROM_READY_FLAG)
#define RFRCC_STATUS_MSG_READY_FLAG                 BIT_1
#define RFRCC_STATUS_BM_MSG_READY_FLAG              BITMASK(RFRCC_STATUS_MSG_READY_FLAG)
#define RFRCC_STATUS_SUBKEY_READY_FLAG              BIT_0
#define RFRCC_STATUS_BM_SUBKEY_READY_FLAG           BITMASK(RFRCC_STATUS_SUBKEY_READY_FLAG)

/* length of rolling code sequence counter within EEPROM */
#define EEP_RFRCC_SIZE                          0x04U

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
typedef struct
{
    /** \brief <b>bFlags</b>
        is used to signalize events outside to the RFRCC module occurred during
        the module operation.
        \li Bit 7:      RFRCC module error flag
                        0: no error occurred
                        1: error occurred, see debug.errorCode for closer description
        \li Bit 6..0:   rfu
    */
    uint8_t bFlags;

    /** \brief <b>bStatus</b>
        is used to signalize events internal to the RFRCC module which occurred
        during the module operation.
        \li Bit 7..3:   rfu
        \li Bit 2:      Rolling Code Counter update flag
                        0: no changes in RCC EEPROM value
                        1: RCC value has been incremented by 1 and stored to EEPROM
        \li Bit 1:      Message ready flag
                        0: message generation in progress
                        1: message generation has been finished
        \li Bit 0:      Subkey generation flag
                        0: no subkey generated
                        1: subkey generation done
    */
    uint8_t bStatus;

    /** \brief <b>bRollCodeMsgBuffer</b>
        is used to store the message to transmit including rolling code sequence counter.
     */
    uint8_t bRollCodeMsgBuffer[RFRCC_MSG_LENGTH];

}sRfrccComponentData;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
extern sRfrccComponentData g_sRfrccComponentData;

extern VOIDFUNC ATA_rfrccGenRollCntMsg_C(uint16_t wEepRfrccAddress, \
                                         uint8_t bSecretKeyGroup, \
                                         uint8_t bUserCmd, \
                                         uint8_t bKeyId);

#elif defined __IAR_SYSTEMS_ASM__

#endif

#endif /* RFRCC_FLASH_H */