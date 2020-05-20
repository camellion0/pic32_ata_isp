//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/aes/src/aes.h $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
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

/** \file firmware/aes/src/aes.h
*/

//lint -restore

#ifndef AES_H
#define AES_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"
#include "../../eep/src/eep.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/* defines for sAesComponentData.bFlags */
#define AES_FLAGS_RESET                         (uint8_t)0x00
#define AES_FLAGS_ERROR_FLAG                    BIT_7
#define AES_FLAGS_BM_ERROR_FLAG                 BITMASK(AES_FLAGS_ERROR_FLAG)
#define AES_FLAGS_READY_FLAG                    BIT_6
#define AES_FLAGS_BM_READY_FLAG                 BITMASK(AES_FLAGS_READY_FLAG)

/* defines for sAesComponentData.bStatus */
#define AES_STATUS_RESET                        (uint8_t)0x00
#define AES_STATUS_LOCK_FLAG                    BIT_7
#define AES_STATUS_BM_LOCK_FLAG                 BITMASK(AES_STATUS_LOCK_FLAG)
#define AES_STATUS_ENCR_WITHIN_DECR_FLAG        BIT_0
#define AES_STATUS_BM_ENCR_WITHIN_DECR_FLAG     BITMASK(AES_STATUS_ENCR_WITHIN_DECR_FLAG)


/* defines for sAesComponentData.bConfig */
#define AES_CONFIG_RESET                        (uint8_t)0x00
#define AES_COMMON_KEY_MASK						0xC0U
#define AES_USECOMMON_KEY1						0x40U
#define AES_USECOMMON_KEY2						0x80U
#define AES_USECOMMON_KEY3						0xC0U
#define AES_CONFIG_CRYPTO_MODULE_RESET          BIT_5
#define AES_CONFIG_BM_CRYPTO_MODULE_RESET       BITMASK(AES_CONFIG_CRYPTO_MODULE_RESET)
#define AES_CONFIG_XOR_STATE_MEMORY             BIT_4
#define AES_CONFIG_BM_XOR_STATE_MEMORY          BITMASK(AES_CONFIG_XOR_STATE_MEMORY)

#define AES_CRYPTO_DIRECTION		            BIT_3
#define AES_BM_CRYPTO_DIRECTION         		BITMASK(AES_CRYPTO_DIRECTION)

#define AES_CONFIG_USE_INTERRUPT                BIT_2
#define AES_CONFIG_BM_USE_INTERRUPT             BITMASK(AES_CONFIG_USE_INTERRUPT)
#define AES_STATUS_PAD_DIRECTION_LSB            BIT_1
#define AES_STATUS_BM_PAD_DIRECTION_LSB         BITMASK(AES_STATUS_PAD_DIRECTION_LSB)
#define AES_CONFIG_SECRET_KEY_SELECTION         BIT_0
#define AES_CONFIG_BM_SECRET_KEY_SELECTION      BITMASK(AES_CONFIG_SECRET_KEY_SELECTION)

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/

/*----------------------------------------------------------------------------- */
/** \brief <b>eAesSecretKeySelection</b>
    defines the Secret Key IDs used to select the Secret Key to be used in the
    AES encryption/decryption phase.
 */
/*----------------------------------------------------------------------------- */
typedef enum
{
    AES_USE_SECRET_KEY_A = 0,
    AES_USE_SECRET_KEY_B = 1u,
    AES_USE_COMMON_KEY_1 = 0x40u,
    AES_USE_COMMON_KEY_2 = 0x80u,
    AES_USE_COMMON_KEY_3 = 0xC0u,
} eAesSecretKeySelection;


/*----------------------------------------------------------------------------- */
/** \brief <b>sAesComponentData</b>
    contains the parameters necessary to perform AES encryption, decryption
    and error reporting.
 */
/*----------------------------------------------------------------------------- */
typedef struct
{
    /** \brief <b>bFlags</b>
        is used to signalize events outside to the AES module occurred
        during AES encryption/decryption.

        \li Bit 7:      AES module error
                        0: no errors occurred during AES encryption/decryption
                        1: errors occurred during AES encryption/decryption,
                           see ::g_sDebug .bErrorCode for closer description
        \li Bit 6:      AES encryption/decryption ready
                        0: AES encryption/decryption not started or in progress
                        1: AES encryption/decryption successfully finished
        \li Bit 5..0:   rfu
     */
    uint8_t bFlags;

    /** \brief <b>bStatus</b>
        contains the current AES status, including the AES locked/unlock state,
        the secret key selection and the error code indication.

        \li Bit 7..1:   rfu
        \li Bit 0:      Encryption within decryption flag
                        0: no data encryption within a decryption request
                        1: data encryption within a decryption request
    */
    uint8_t bStatus;

    /** \brief <b>bConfig</b>
        contains configuration for the AES control register and specific settings
        for encryption/decryption.

        \li Bit 7..6:   CommonKey usage is overriding against the Key Group A and Key Group B
        				00 - No CommonKey
        				01 - CommonKey1
        				10 - CommonKey2
        				11 - CommonKey3
        \li Bit 5:      Reset AES crypto module, corresponds to AESCR.AESRES
                        0: do not reset the AES module
                        1: reset the AES module
        \li Bit 4:      Data load XOR to state memory, corresponds to AESCR.AESXOR
                        0: disable XOR data load to the State memory
                        1: enable XOR data load to the State memory
        \li Bit 3:      AES crypto direction
        				0: encrypt
        				1: decrypt
        \li Bit 2:      AES encryption/decryption completion interrupt usage, corresponds to AESCR.AESIM
                        0: wait completion without usage of AES interrupt
                        1: AES interrupt is used
        \li Bit 1:      Direction of padding in the incomplete byte of the source data
                        0: MSB aligned
                        1: LSB aligned
        \li Bit 0:      Secret key selection
                        0: Secret Key A
                        1: Secret Key B
     */
    uint8_t bConfig;

    /** \brief <b>bKeyId</b>
        contains configuration for the AES control register and specific settings
        for encryption/decryption.

        \li Bit 7..4:   AES Key - Group B index
        \li Bit 3..0:   AES Key - Group A index
     */
    uint8_t bKeyId;


    /** \brief <b>bDataBuffer</b>
        contains the data to be encrypted (AES encryption mode) or the data to
        be decrypted (AES decryption mode).
     */
    uint8_t bDataBuffer[EEP_SECRET_KEY_LENGTH];

    /** \brief <b>bDataLength</b>
        contains the length of the input data in bits.
     */
    uint8_t bDataLength;

}sAesComponentData;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (Variables)                                          */
/*===========================================================================*/

extern __root __no_init sAesComponentData g_sAesComponentData;

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (Functions)                                          */
/*===========================================================================*/
extern VOIDFUNC ATA_aesInit_C(void);
extern VOIDFUNC ATA_aesSetConfig_C(uint8_t uSecretKey, uint8_t uUserConfig, uint8_t bKeyId);
extern VOIDFUNC ATA_aesTriggerKeyDma_C(uint16_t wAddr);
extern VOIDFUNC ATA_aesKeyLoad_C(uint8_t bConfig , uint8_t bKeyId);
extern VOIDFUNC ATA_aesEncryptData_C(void);
extern VOIDFUNC ATA_aesDecryptData_C(void);
extern VOIDFUNC ATA_aesLoadData_C(unsigned char volatile *pDestReg, uint8_t *pSourceBuf);
extern VOIDFUNC ATA_aesApplyPaddingScheme_C(uint8_t uDataSize);

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
/*stopSimExtraction*/
#endif

#endif /* AES_H */
