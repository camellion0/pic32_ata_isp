//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Trunk/appl/appFlash_simTest/src/rfrcc/src/rfrcc_flash.c $
  $LastChangedRevision: 277341 $
  $LastChangedDate: 2014-09-03 05:07:52 -0600 (Wed, 03 Sep 2014) $
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
/** \file rfrcc_flash.c
*/
//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "rfrcc_flash.h"
#include "../../../firmware/globals/src/globals.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define ATA_DEFAULT_CONFIG_VAL       0x20

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>g_sRfrccComponentData</b>
    contains the RFRCC module related flags and message buffer.
*/
#pragma location = ".sram_FlashModule_Rfrcc"
__no_init sRfrccComponentData g_sRfrccComponentData;

//extern uint8_t g_EepFlashApp_FOBindx;
extern uint8_t g_EepFlashApp_USRID[4];

/*===========================================================================*/
/*  LOCAL PROTOTYPES (Functions)                                             */
/*===========================================================================*/
static VOIDFUNC ATA_rfrccGenSubKey_C(void);

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfrccGenRollCntMsg_C</b>
    This function transmits the rolling code counter message consisting of the
    transmitter's unique ID, sequential counter, command byte and the Message
    Authentication Code (MAC).

\param[in,out]  ::g_sRfrccComponentData     Global RFRCC component data
\param[in,out]  ::g_sAesComponentData       Global AES component data
\param[out]     ::g_sDebug                  Global Debug component data
\param[in]      bSecretKeyGroup             Group of the secret key with which
                                            to encrypt the data. Values are
                                            defined in eAesSecretKeySelection
                                            in aes.h
\param[in]      bUserCmd                    Command byte in the Rolling Code
                                            Counter message

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_rfrccGenRollCntMsg_C.png

\internal
\li 010: Initialize component variables.
         Note: The RFRCC component error flag is set on module entry and
               cleared only, if the incremented rolling code counter value
               has been successfully written to EEPROM.

\li 020: Read the 4-byte transmitter unique serial ID from EEPROM and store
         it to message buffer. Note that high byte is located at lowest
         address hence to reduce subsequent byte swapping effort, destination
         address already set to highest byte address.

\li 021: Adapt the 4-byte transmitter unique serial ID in message buffer
         according to low byte first order.

\li 030: Read the 4-byte rolling code sequence counter value from EEPROM and
         store it to message buffer in low byte first order.

\li 040: IF the EEPROM read operations are successful,
         THEN

\li 050:   Pass AES configuration settings and the selected secret key to
           the AES module.

\li 060:   Generate the subkey.

\li 070:   IF the subkey has been successfully generated,
           THEN
\li 080:     Store the 1-byte command ID to message buffer.

\li 090:     Copy 9 bytes of message buffer, consisting of the serial ID,
             rolling code counter and user command, to AES data buffer.

\li 100:     Add padding info byte (0x80) to AES data buffer and pad bytes
             11-15 with 0's.

\li 110:     During subkey generation new data were written to AES state memory
             used to generate the Message Authentication Code (MAC). Due to this,
             adapt AES configuration settings in order to prevent reset of
             crypto module and enable the AES hardware XOR functionality.

\li 120:     Generate the Message Authentication Code (MAC).

\li 130:     IF the MAC has been successfully generated,
             THEN

\li 140:       Copy upper 4 bytes of MAC to message buffer in low byte first order.
               Note: AES data buffer is MSB aligned, i.e. highest byte on lowest address.

\li 150:       Signalize readiness of rolling code counter message generation.

\li 160:       Increment the rolling code counter by 1 and write it into EEPROM.

\li 170:       IF EEPROM write access errors occurred,
               THEN

\li 180:         Set ::g_sDebug .bErrorCode to error code for EEPROM
                 write access error.
               ELSE

\li 190:         Wait till the write process is completed.
                 Signalize successful update of RCC value in EEPROM and clear
                 the RFRCC component error flag.
               ENDIF
             ELSE

\li 200:       Set ::g_sDebug .bErrorCode to error code for MAC generation error.
             ENDIF
           ELSE

\li 210:     Set ::g_sDebug .bErrorCode to error code for subkey generation
              error.
           ENDIF
         ELSE

\li 220:   Set ::g_sDebug .bErrorCode to error code for EEPROM read error.
         ENDIF

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-1096,Primus2P-1097,Primus2P-1099,Primus2P-1933}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfrccGenRollCntMsg_C(uint16_t wEepRfrccAddress, uint8_t bSecretKeyGroup, uint8_t bUserCmd, uint8_t bKeyId)
{
    eEepErrorCode sEepErrCode;

    ATA_SET_FUNCTION_TRACE_POINT_C( ATA_rfrccGenRollCntMsg_C, bSecretKeyGroup );

    /* LLR-Ref: 010 */
    g_sRfrccComponentData.bFlags  = RFRCC_FLAGS_BM_ERROR_FLAG;
    g_sRfrccComponentData.bStatus = RFRCC_STATUS_RESET;

    /* LLR-Ref: 020 */
    /* sEepErrCode  = ATA_eepReadBytes_C((uint8_t *)&g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_SERIAL_ID_OFFSET+3U], \
                                      (uint16_t)&g_sAtmelEEPromSection.eepUID[0], \
                                       EEP_XROW_UID_SIZE);
    */
    sEepErrCode  = ATA_eepReadBytes_C((uint8_t *)&g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_SERIAL_ID_OFFSET+3U], \
                                      (uint16_t)&g_EepFlashApp_USRID[0], \
                                       EEP_XROW_UID_SIZE);
    /* LLR-Ref: 021 */
    g_sRfrccComponentData.bRollCodeMsgBuffer[0U] = g_sRfrccComponentData.bRollCodeMsgBuffer[6U];
    g_sRfrccComponentData.bRollCodeMsgBuffer[1U] = g_sRfrccComponentData.bRollCodeMsgBuffer[5U];
    g_sRfrccComponentData.bRollCodeMsgBuffer[2U] = g_sRfrccComponentData.bRollCodeMsgBuffer[4U];
    
    uint8_t tempbuff;
    tempbuff = g_sRfrccComponentData.bRollCodeMsgBuffer[0U];     
    g_sRfrccComponentData.bRollCodeMsgBuffer[0U] = g_sRfrccComponentData.bRollCodeMsgBuffer[3U];
    g_sRfrccComponentData.bRollCodeMsgBuffer[3U] = tempbuff;
    
    tempbuff = g_sRfrccComponentData.bRollCodeMsgBuffer[1U];    
    g_sRfrccComponentData.bRollCodeMsgBuffer[1U] = g_sRfrccComponentData.bRollCodeMsgBuffer[2U];
    g_sRfrccComponentData.bRollCodeMsgBuffer[2U] = tempbuff;
    

    /* LLR-Ref: 030 */
    sEepErrCode |= ATA_eepReadBytes_C( (uint8_t *)&g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_ROLL_COUNT_OFFSET], \
                                       wEepRfrccAddress, \
                                       EEP_RFRCC_SIZE);
 

    /* LLR-Ref: 040 */
    if( sEepErrCode == EEC_NO_ERROR )
    {
        uint32_t wRccVal;

        /* LLR-Ref: 050 */
        ATA_aesSetConfig_C( bSecretKeyGroup, ATA_DEFAULT_CONFIG_VAL , bKeyId );

        /* LLR-Ref: 060 */
        ATA_rfrccGenSubKey_C();

        /* LLR-Ref: 070 */
        if( g_sRfrccComponentData.bStatus & RFRCC_STATUS_BM_SUBKEY_READY_FLAG )
        {
            /* LLR-Ref: 080 */
            g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_COMMAND_ID_OFFSET] = bUserCmd;

            /* LLR-Ref: 090 */
            for( uint8_t i = 0; i < RFRCC_MSG_LENGTH_EXCL_MAC; i++ )
            {
                g_sAesComponentData.bDataBuffer[i] = g_sRfrccComponentData.bRollCodeMsgBuffer[i];
            }

            /* LLR-Ref: 100 */
            /*Set to 0x00 for LDL Atnel app*/
            g_sAesComponentData.bDataBuffer[RFRCC_DATABLOCK_4_AES_PAD_INFO_OFFSET] = RFRCC_DATABLOCK_4_AES_PAD_INFO;
            ATA_globalsInitSramSpace_C( &g_sAesComponentData.bDataBuffer[RFRCC_DATABLOCK_4_AES_PAD_ZERO_OFFSET], \
                                        RFRCC_DATABLOCK_4_AES_PAD_ZERO_LENGTH );

            /* LLR-Ref: 110 */
            g_sAesComponentData.bConfig &= ~AES_CONFIG_BM_CRYPTO_MODULE_RESET;
         //   g_sAesComponentData.bConfig |= AES_CONFIG_BM_XOR_STATE_MEMORY;

            /* LLR-Ref: 120 */
            ATA_aesEncryptData_C();

            /* LLR-Ref: 130 */
            if( !(g_sAesComponentData.bFlags & AES_FLAGS_BM_ERROR_FLAG) )
            {
             //  g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+0] = g_sAesComponentData.bDataBuffer[3];
             //   g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+1] = g_sAesComponentData.bDataBuffer[2];
             //   g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+2] = g_sAesComponentData.bDataBuffer[1];
             //   g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+3] = g_sAesComponentData.bDataBuffer[0];
               
             /* Reverse order for MAC byte ins Atmel P2P demo */
                g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+0] = g_sAesComponentData.bDataBuffer[0];
                g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+1] = g_sAesComponentData.bDataBuffer[1];
                g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+2] = g_sAesComponentData.bDataBuffer[2];
                g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_MAC_OFFSET+3] = g_sAesComponentData.bDataBuffer[3];

                /* LLR-Ref: 150 */
                g_sRfrccComponentData.bStatus |= RFRCC_STATUS_BM_MSG_READY_FLAG;

                /* LLR-Ref: 160 */
                wRccVal  = (uint32_t)g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_ROLL_COUNT_OFFSET+3]<<24;
                wRccVal |= (uint32_t)g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_ROLL_COUNT_OFFSET+2]<<16;
                wRccVal |= (uint32_t)g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_ROLL_COUNT_OFFSET+1]<<8;
                wRccVal |= (uint32_t)g_sRfrccComponentData.bRollCodeMsgBuffer[RFRCC_MSG_ROLL_COUNT_OFFSET+0];
                wRccVal++;

                sEepErrCode = ATA_eepWriteBytes_C((uint8_t *)&wRccVal, wEepRfrccAddress, EEP_RFRCC_SIZE );

                /* LLR-Ref: 170 */
                if( sEepErrCode )
                {
                    /* LLR-Ref: 180 */
                    g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFRCC_EEP_WRITE_ACCESS_ERROR;
                }
                else
                {
                    /* LLR-Ref: 190 */
                    while(EECR & BM_NVMBSY)
                    {}

                    g_sRfrccComponentData.bStatus |= RFRCC_STATUS_BM_UPDATE_EEPROM_READY_FLAG;
                    g_sRfrccComponentData.bFlags  &= ~RFRCC_FLAGS_BM_ERROR_FLAG;
                }
            }
            else
            {
                /* LLR-Ref: 200 */
                g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFRCC_MAC_ERROR;
            }
        }
        else
        {
            /* LLR-Ref: 210 */
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFRCC_SUBKEY_ERROR;
        }
    }
    else
    {
        /* LLR-Ref: 220 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFRCC_EEP_READ_ERROR;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfrccGenSubKey_C</b>
    This function computes the subkey using AES-CMAC algorithm (OMAC).
    Note: Different from original CMAC algorithm the function computes only
          one final subkey. The result of subkey K1 computation will not be
          stored explicitely it is overwritten during second subkey
          computation. Finally K1 equals K2 subkey.

       +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       +            Original AES-CMAC subkey Algorithm                     +
       +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       +                                                                   +
       +   Input    : K  (128-bit key)                                     +
       +   Output   : K1 (128-bit first subkey)                            +
       +              K2 (128-bit second subkey)                           +
       +-------------------------------------------------------------------+
       +                                                                   +
       +   Constants: const_Zero is 0x00000000000000000000000000000000     +
       +              const_Rb   is 0x00000000000000000000000000000087     +
       +   Variables: L          for output of AES-128 applied to 0^128    +
       +                                                                   +
       +   Step 1.  L          := AES-128(K, const_Zero);                  +
       +   Step 2.  if MSB(L) is equal to 0                                +
       +            then    K1 := L << 1;                                  +
       +            else    K1 := (L << 1) XOR const_Rb;                   +
       +   Step 3.  if MSB(K1) is equal to 0                               +
       +            then    K2 := K1 << 1;                                 +
       +            else    K2 := (K1 << 1) XOR const_Rb;                  +
       +   Step 4.  return K1, K2;                                         +
       +                                                                   +
       +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

       +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       +               ATMEL AES-CMAC subkey Algorithm                     +
       +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       +                                                                   +
       +   Input    : K  (128-bit secret key)                              +
       +   Output   : K1 (128-bit subkey)                                  +
       +-------------------------------------------------------------------+
       +                                                                   +
       +   Constants: const_Zero is 0x00000000000000000000000000000000     +
       +              const_Rb   is 0x00000000000000000000000000000087     +
       +   Variables: L          for output of AES-128 applied to 0^128    +
       +                                                                   +
       +   Step 1.  L          := AES-128(K, const_Zero);                  +
       +   Step 2.  if MSB(L) is equal to 0                                +
       +            then    K1 := L << 1;                                  +
       +            else    K1 := (L << 1) XOR const_Rb;                   +
       +   Step 3.  if MSB(K1) is equal to 0                               +
       +            then    K1 := K1 << 1;                                 +
       +            else    K1 := (K1 << 1) XOR const_Rb;                  +
       +   Step 4.  return K1;                                             +
       +                                                                   +
       +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


\param[in,out]  ::g_sAesComponentData       Global AES component data

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_rfrccGenSubKey_C.png

\internal
\li 010:  Setting up a 16 bytes 'const_Zero' data input block and associated
          data length to 128 (16 bytes = 128 bits).\n\n
          Note: Array ::g_sAesComponentData .bDataBuffer is loaded into the
          AESDR during encryption.

\li 020:  Generate the initial subkey L with the 'const_Zero' input block and
          one of the two secret keys from EEPROM by calling the AES encryption
          function ::ATA_aesEncryptData_C.
          The encrypted data L is stored in ::g_sAesComponentData .bDataBuffer.

\li 030:  IF the subkey has been successfully generated,
          THEN

\li 040:    Generating the intermediate subkey K1.
            Store MSB information of L for later analysis and perform a
            multiplication by 2 on the encrypted data L. Analyze MSB of encrypted
            data L. If set, XOR K1 with 'const_Rb'.
            Note: AES data buffer is MSB aligned, i.e. highest byte on lowest address.

\li 050:    Generating the final subkey.
            Store MSB information of K1 for later analysis and perform a
            multiplication by 2 on intermediate subkey K1. Analyze MSB of encrypted
            data K1. If set, XOR K1 with 'const_Rb'.

\li 070:    Load generated subkey directly to AES state memory since hardware
            XOR functionality is used to generate message authentication code (MAC).

\li 080:    Signalize readiness of subkey generation.
          ENDIF

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-1096}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_rfrccGenSubKey_C(void)
{
    uint8_t *pDataPtr;
    uint8_t bBlockLen = RFRCC_DATABLOCK_4_AES_MAX_LEN_BYTES-1;
    uint8_t bOverflow;

    ATA_SET_FUNCTION_TRACE_POINT_C( ATA_rfrccGenSubKey_C, 0x00U );

    /* LLR-Ref: 010 */
    ATA_globalsInitSramSpace_C(g_sAesComponentData.bDataBuffer, EEP_SECRET_KEY_LENGTH);
    g_sAesComponentData.bDataLength = RFRCC_DATABLOCK_4_AES_MAX_LEN_BITS;

    /* LLR-Ref: 020 */
    ATA_aesEncryptData_C();

    /* LLR-Ref: 030 */
    if( !(g_sAesComponentData.bFlags & AES_FLAGS_BM_ERROR_FLAG) )
    {
        /* LLR-Ref: 040 */
        pDataPtr  = g_sAesComponentData.bDataBuffer;
        bOverflow = pDataPtr[0] & BIT_MASK_7;

        do{
            /* multiply first byte by 2 and OR MSB of succeeding byte */
            pDataPtr[0] = (pDataPtr[0] << 1) | (pDataPtr[1] >> 7);
            pDataPtr++;
            bBlockLen--;
        }while( bBlockLen != 0 );

        pDataPtr[0] <<= 1;  // multiply last byte by 2
        if( bOverflow )
        {
            pDataPtr[0] ^= 0x87;
        }

        /* LLR-Ref: 050 */
        pDataPtr  = g_sAesComponentData.bDataBuffer;
        bOverflow = pDataPtr[0] & BIT_MASK_7;
        bBlockLen = RFRCC_DATABLOCK_4_AES_MAX_LEN_BYTES-1;

        do{
            /* multiply first byte by 2 and OR MSB of succeeding byte */
            pDataPtr[0] = (pDataPtr[0] << 1) | (pDataPtr[1] >> 7);
            pDataPtr++;
            bBlockLen--;
        }while( bBlockLen != 0 );

        pDataPtr[0] <<= 1;  // multiply last byte by 2
        if( bOverflow )
        {
            pDataPtr[0] ^= 0x87;
        }

        /* LLR-Ref: 060 */
        PRR0 &= ~BM_PRCU;
        ATA_aesLoadData_C( &AESDR, &g_sAesComponentData.bDataBuffer[0] );
        PRR0 |= BM_PRCU;

        /* LLR-Ref: 070 */
        g_sRfrccComponentData.bStatus |= RFRCC_STATUS_BM_SUBKEY_READY_FLAG;
    }
}

