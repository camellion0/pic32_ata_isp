//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/aes/src/aes.c $
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
/** \file firmware/aes/src/aes.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/** \brief <b>Module AES</b>
    This Modules handels everything dealing with AES coding and encoding. 
    The available AES keys are also handled via this module.        

\internal      
\Traceability{Primus2P-808, Primus2P-809, Primus2P-810, Primus2P-811,\
              Primus2P-1354, Primus2P-1355, Primus2P-1357, Primus2P-1358,\
              Primus2P-3161, Primus2P-3190, Primus2P-3195, Primus2P-3212}
\endinternal
\n
*/    
/*---------------------------------------------------------------------------*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "aes.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define MAX_BYTE_VAL    255U

/*===========================================================================*/
/*  GLOBALS                                                                  */
/*===========================================================================*/

/** \brief <b>g_sAesComponentData</b>
    contains the parameters necessary to perform AES encryption/decryption
    and error reporting.
*/
#pragma location = ".sramAesComponentDataSection"
__root __no_init sAesComponentData g_sAesComponentData;

/** \brief <b>m_bAesLastSubKey</b>
    holds the last subkey read out of AESKR.
*/
#pragma location = ".sramAesComponentDataSection"
static __no_init uint8_t m_bAesLastSubKey[16];


/*===========================================================================*/
/*  LOCAL PROTOTYPES (Functions)                                             */
/*===========================================================================*/

/**/
static VOIDFUNC ATA_aesGetResultSynch_C(void);

/**/
static VOIDFUNC ATA_aesGetData_C(uint8_t *pDestBuf, unsigned char volatile *pSourceReg);

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesInit_C</b>
    initializes the AES component data with default values and clears the
    temporary buffer storing the last subkey.

    Variables Usage:
    \li [out] ::g_sAesComponentData   Global AES component data
    \li [out] ::m_bAesLastSubKey      Module global AES last subkey data

    \image html ATA_aesInit_C.png

    \internal
    \li 005: Update HW Trace Unit with function information.

    \li 010: Set the AES component input data buffer to zero. Clear the sub-key
             buffer to 0.

    \li 020: Clear the element bFlags of the structure ::g_sAesComponentData to 0.

    \li 030: Set Secret Key One as the default secret key to be used,
             error code indication to no error, and module locked status to unlocked.

    \li 040: Set AES configuration to the default value 0.

    \li 050: Initialize the length of the source data to 0.

    \Derived{Yes}

    \Rationale{The global AES data needs to be se to default values before
               usage, since no SRAM initialization takes place during reset}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesInit_C (void)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_aesInit_C, 0x00U);

    /* LLR-Ref: 010 */
    ATA_globalsInitSramSpace_C(g_sAesComponentData.bDataBuffer, EEP_SECRET_KEY_LENGTH);
    ATA_globalsInitSramSpace_C(m_bAesLastSubKey, EEP_SECRET_KEY_LENGTH);

    /* LLR-Ref: 020 */
    g_sAesComponentData.bFlags = AES_FLAGS_RESET;

    /* LLR-Ref: 030 */
    g_sAesComponentData.bStatus = AES_STATUS_RESET;

    /* LLR-Ref: 040 */
    g_sAesComponentData.bConfig = AES_CONFIG_RESET;

    /* LLR-Ref: 050 */
    g_sAesComponentData.bDataLength = 0;

    /* LLR-Ref: 060 */
    g_sAesComponentData.bKeyId = 0x00U;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesSetConfig_C</b>
    sets the secret key and the configuration settings chosen by the application.

    \param[in]  uSecretKey  The secret key selected
    \param[in]  uUserConfig The configuration settings requested.
    \param[in]  bKeyId      AES key group IDs
    
    Variables Usage:
    \li [out] ::g_sAesComponentData   Global AES component data
    
    \image html ATA_aesSetConfig_C.png

    \internal
    \li 005: Update HW Trace Unit with function information.

    \li 010: Store the given configuration requested to
              ::g_sAesComponentData .bConfig without modifying 
              the secret key selection bits.

    \li 020: Copy the selected secret key information to
              ::g_sAesComponentData .bConfig[0].

    \li 030: Setup the AES key group IDs to be used for the AES.

    \Derived{Yes}

    \Rationale{A convenient function to initialize the AES component configuration}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesSetConfig_C(uint8_t uSecretKey, uint8_t uUserConfig, uint8_t bKeyId)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_aesSetConfig_C, uUserConfig);

    /* LLR-Ref: 010 */
    g_sAesComponentData.bConfig = uUserConfig & ~(AES_COMMON_KEY_MASK|AES_CONFIG_BM_SECRET_KEY_SELECTION) ;

    /* LLR-Ref: 020 */    
    g_sAesComponentData.bConfig |= (uSecretKey & (AES_COMMON_KEY_MASK|AES_CONFIG_BM_SECRET_KEY_SELECTION));
    
    /* LLR-Ref: 030 */
    g_sAesComponentData.bKeyId = bKeyId;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesTriggerKeyDma_C</b>
    shall store the EEProm data starting at the given EEProm address to the 
    AES key memory. The AES block needs to be powered up already via PRR0.PRCU.

    \param[in]  wAddr   Starting address to transfered 
                        16 bytes via DMA from EEProm to 
                        the AES block
    
    Variables Usage:
    \li [in,out]  ::g_sAesComponentData       Global AES component data
    \li [out]     ::g_sDebug                  Global Debug component data

    \internal
    \li 010: Setup EEProm address registers for AES key to be loaded
    \li 020: Trigger the DMA transfer of the AES key located in EEPROM
    \li 030: Wait for the DMA transfer to be carried out
    \li 040: IF an EEPROM correction error occured during the AES DMA transfer,
             THEN
               Set the AES error flag within AES component data 
                ::g_sAesComponentData and the global error status ::g_sDebug
               Clear the EEPROM correction error in register EECR2 to have a well-
                defined EEPROM interface status
    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3161, Primus2P-3190}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesTriggerKeyDma_C(uint16_t wAddr)
/*---------------------------------------------------------------------------*/
{
    /* LLR-Ref: 010 */
    EEARH = (uint8_t)(wAddr>>8);
    EEARL = (uint8_t)wAddr;
    
    /* LLR-Ref: 020 */
    AESCR |= BM_AESLKM;
    
    /* LLR-Ref: 030 */
    while((AESCR & BM_AESLKM)!= 0x00U);

    /* LLR-Ref: 040 */
    if(EECR2 & BM_E2FF)
    {
        g_sAesComponentData.bFlags |= AES_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_AES_INVALID_KEY_ERROR;
        EECR2 |= BM_E2FF;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesKeyLoad_C</b>
    Function stores the key from the given EEPROM address to the AES key memory.
    AES block needs to be powered up already via PRR0.PRCU.

    \param[in] bKeyId  Index of used AES key (A/B as well 
                       as COMMON keys)
                       In case of an secret key it is a 
                       direct copy of the bKeyId variable.
                       In case of the common key the 
                       lower to bits will indicate the 
                       index of the used common AES key.
    \param[in] bConfig Encode/decode -> Key or subKey 
                       loading (bit 3)
                       KeyA / KeyB selection (bit 0)
                       COM Key selection (bit 7/6)

    Variables Usage:
    \li [in,out]  ::g_sAesComponentData       Global AES component data
    \li [out]     ::g_sDebug                  Global Debug component data
    \li [in]      ::g_sCustomerEEPromSection  Global customer EEPROM section

    \internal
    \li 010: Get base address of the desired AES key (COMMON, A or B) intended to 
             be used in the following cyphering task.
             For the COMMON keys, either COMMON key 1, 2 or 3 is used, depending
             on the given parameter ::bConfig.
             Regarding Secret Key A, respectively B, the key index of the
             key group (A or B) will be used to get the base address of the 
             selected key, with the last subkey of the corresponding key being 
             stored after the original AES key.

             Note: (Not valid anymore)
                EEPROM error checking is not required after retrieving the Secret
                Key address, since the EEPROM error indication will not be 
                overwritten with a different error indication. The whole function
                raises the same error condition, in case EEPROM data could not be
                read.

             IF an AES decryption sequence is to be carried out,
             THEN
    \li 020:   IF no AESKR register is available,
               THEN
                 Load the last subkey of the corresponding AES key from EEPROM via
                 DMA by calling function "ATA_aesTriggerKeyDma_C" with the
                 computed (16 bytes offset) subkey address.
               ELSE
                 
               ENDIF
             ELSE
               Load the the AES key from EEPROM via DMA by calling function 
               "ATA_aesTriggerKeyDma_C" with the computed (16 bytes offset)
               subkey address.
             ENDIF

    \li 020: Store the AES key base address to the EEProm address register
             Flow depends on AESKR presence or not. In case not, the subKey will already
             reside inside the EEProm.
    \li 030: Start the key transfer via DMA from EEProm to AES IP

    TBD

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-809, Primus2P-1354, Primus2P-3161, Primus2P-3190}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesKeyLoad_C(uint8_t bConfig , uint8_t bKeyId)
/*---------------------------------------------------------------------------*/
{
    uint8_t ary[2];
    uint16_t tmp;
    uint8_t i;

    /* LLR-Ref: 010 */
    if(bConfig&AES_COMMON_KEY_MASK)
    {
        i = (bConfig&AES_COMMON_KEY_MASK)>>6U;
        i--;
        
        ATA_eepReadBytes_C(&ary[0],(((uint16_t)&g_sCustomerEEPromSection.eepComKeyAddr1_l)+(i<<1)),2U);
    }
    else
    {
        if(g_sAesComponentData.bConfig&AES_CONFIG_BM_SECRET_KEY_SELECTION)
        {
            ATA_eepReadBytes_C(&ary[0],(((uint16_t)&g_sCustomerEEPromSection.eepSecKeyAddrB)+(bKeyId>>3U)),2U);
        }
        else
        {
            ATA_eepReadBytes_C(&ary[0],(((uint16_t)&g_sCustomerEEPromSection.eepSecKeyAddrA)+(bKeyId<<1U)),2U);
        }
    }
    
    /**/
    tmp = ((ary[1]<<8U|ary[0]));
    
    if(bConfig&AES_BM_CRYPTO_DIRECTION)
    {
        /* LLR-Ref: 020 */
        if(!(ATA_eepFuseRead_C(BM_GET_FUSE_SECF)&HAVE_NO_AESKR))
        {
            tmp+=16U;
            ATA_aesTriggerKeyDma_C(tmp);
        }
        else
        {
            /* LLR-Ref: 070 */                   
            AESCR = g_sAesComponentData.bConfig & AES_CONFIG_BM_CRYPTO_MODULE_RESET;
            g_sAesComponentData.bStatus |= AES_STATUS_BM_ENCR_WITHIN_DECR_FLAG;            
            ATA_aesTriggerKeyDma_C(tmp);
            
            /* Trigger AES operation to generate the needed subKey make sure IRQs are disabled */
            AESCR = g_sAesComponentData.bConfig & AES_CONFIG_BM_XOR_STATE_MEMORY;
            
            /* LLR-Ref: 200, back up copy for original data! */ 
            for(i=0;i<16;i++)
            {
                m_bAesLastSubKey[i] = g_sAesComponentData.bDataBuffer[i];
            }
    
            /* LLR-Ref: 200 */            
            AESCR |= BM_AESE;
            
            /* LLR-Ref: 210 */
            ATA_aesGetResultSynch_C();
            
            /* LLR-Ref: 200, restore original data! */ 
            for(i=0;i<16;i++)
            {
                g_sAesComponentData.bDataBuffer[i] = m_bAesLastSubKey[i];
            }                        
                
            for(i=0;i<16;i++)
            {
                m_bAesLastSubKey[i] = AESKR;
            }
            
            /* Store back subKey */
            for(i=0;i<16;i++)
            {
                AESKR = m_bAesLastSubKey[i];
            }
            
            /* LLR-Ref: 045 */
            g_sAesComponentData.bStatus &= ~AES_STATUS_BM_ENCR_WITHIN_DECR_FLAG;
        }
    }
    else
    {
        /* LLR-Ref: 020 */
        ATA_aesTriggerKeyDma_C(tmp);
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesEncryptData_C</b>
    Encrypts the given data with either the Secret Key 1 or Secret Key 2
    stored in the chip's EEPROM. The completion of an AES phase is indicated
    via either polling (or) the AES Crypto Unit Interrupt depending on the
    user configuration, and its result is made available in the AES State Memory.
    DMA Burst Read Mode will be used for this function.

    Variables Usage:
    \li [in,out]  ::g_sAesComponentData  Global AES component data

    \internal
    \li 005: Update HW Trace Unit with function information.\n

    TBD

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-810, Primus2P-811, Primus2P-3195}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesEncryptData_C (void)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_aesEncryptData_C, g_sAesComponentData.bConfig);
    
    /* signalize encryption in progress */
    g_sAesComponentData.bFlags = AES_FLAGS_RESET;
    
    /* Enable clock for AES */
    PRR0 &= ~BM_PRCU;
    
    /* LLR-Ref: 050 */
    AESCR = g_sAesComponentData.bConfig & AES_CONFIG_BM_CRYPTO_MODULE_RESET;
    
    /* LLR-Ref: 070 */
    ATA_aesKeyLoad_C(g_sAesComponentData.bConfig&(~AES_BM_CRYPTO_DIRECTION) , g_sAesComponentData.bKeyId);

    /* LLR-Ref: 170 */
    if( !(g_sAesComponentData.bFlags & AES_FLAGS_BM_ERROR_FLAG) )
    {
        /* There is no error in loading the secret key */
        AESCR = g_sAesComponentData.bConfig & (AES_CONFIG_BM_XOR_STATE_MEMORY | AES_CONFIG_BM_USE_INTERRUPT);

        /* Pad the source data and copy the padded bytes into state memory,
           only if encryption was requested by the user. If user requested
           decryption, state memory contents can be directly encrypted. */
        if( g_sAesComponentData.bDataLength < 128 )
        {
            /* LLR-Ref: 180 */
            ATA_aesApplyPaddingScheme_C(g_sAesComponentData.bDataLength);
        }
        /* LLR-Ref: 190 */
        ATA_aesLoadData_C(&AESDR, &g_sAesComponentData.bDataBuffer[0]);

        /* LLR-Ref: 200 */
        AESCR |= BM_AESE;

        if( !(g_sAesComponentData.bConfig & AES_CONFIG_BM_USE_INTERRUPT) )
        {
            /* LLR-Ref: 210 */
            ATA_aesGetResultSynch_C();
            PRR0 |= BM_PRCU;
        }
    }
    else
    {
        /* LLR-Ref: 220 */
        PRR0 |= BM_PRCU;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesDecryptData_C</b>
    Decrypts the given data with either the Secret Key 1 or Secret Key 2
    stored in the chip's EEPROM. The completion of an AES phase is indicated
    via either polling (or) the AES Crypto Unit Interrupt depending on the
    user configuration, and its result is made available in the AES State Memory.
    DMA Burst Read Mode will be used for this function.

    Variables Usage:
    \li [in,out]  ::g_sAesComponentData   Global AES component data
    \li [out]     ::m_bAesLastSubKey      Module global AES last subkey data

    \internal
    \li 005: Update HW Trace Unit with function information.\n
             IF the AES module is currently not in use, THEN

    TBD

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-808,Primus2P-809,Primus2P-1354,Primus2P-1358}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesDecryptData_C(void)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_aesDecryptData_C, g_sAesComponentData.bConfig);

    /* signalize encryption in progress */
    g_sAesComponentData.bFlags = AES_FLAGS_RESET;
    
    /* Enable clock for AES */
    PRR0 &= ~BM_PRCU;
    
    /* LLR-Ref: 050 */
    AESCR = g_sAesComponentData.bConfig & AES_CONFIG_BM_CRYPTO_MODULE_RESET;
    
    /* LLR-Ref: 070 */
	ATA_aesKeyLoad_C(g_sAesComponentData.bConfig|(AES_BM_CRYPTO_DIRECTION) , g_sAesComponentData.bKeyId);

    /* LLR-Ref: 170 */
    if( !(g_sAesComponentData.bFlags & AES_FLAGS_BM_ERROR_FLAG) )
    {
        /* There is no error in loading the secret key */
        AESCR = g_sAesComponentData.bConfig & (AES_CONFIG_BM_XOR_STATE_MEMORY | AES_CONFIG_BM_USE_INTERRUPT);

        /* LLR-Ref: 190 */
        ATA_aesLoadData_C(&AESDR, &g_sAesComponentData.bDataBuffer[0]);

        /* LLR-Ref: 200 */
        AESCR |= (BM_AESE|BM_AESD);

        if( !(g_sAesComponentData.bConfig & AES_CONFIG_BM_USE_INTERRUPT) )
        {
            /* LLR-Ref: 210 */
            ATA_aesGetResultSynch_C();
            
            PRR0 |= BM_PRCU;
        }
    }
    else
    {
        /* LLR-Ref: 220 */
        PRR0 |= BM_PRCU;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesGetResultSynch_C</b>
    gets the result of encryption / decryption from the state buffer.

    Variables Usage:
    \li [out] ::g_sAesComponentData   Global AES component data
    \li [out] ::g_sDebug              Global Debug component data

    \internal
    \li 010: In the synchronous mode, the program waits for the completion of the
             running encryption/decryption, which is indicated by setting of
             the AESRF bit in register AESSR (or) the occurence of an error which is
             indicated by setting of the AESERF bit in register AESSR.

    \li 035: Set bit 7 of the element bFlags of AES component data to 1 and the debug
             error code byte to DEBUG_ERROR_CODE_AES_RUN_ERROR.

    \li 040: If the AESRF is set, indicate that the encrypted/decrypted result is
             available by setting bit 1 in the status byte of the component data.
             Copy the content of AESDR into the data buffer (code optimized, take
             ~20us longer than 16 individual steps).\n
             IF the application requested an encryption and the encryption is successful
              OR
             the application requested a decryption AND result of the overall
             decryption is successful
    \li 060:   Set bit 1 in the status byte of AES component data to indicate that
               encryption/decryption is successfully completed.
               (This flag should not be set if the encryption done within a
                decryption is error-free, since it is only an intermediary result)\n
             ENDIF

    \Derived{Yes}

    \Rationale{Convenience function to retrieve the encrypted/decrypted data
               from the AES block}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_aesGetResultSynch_C(void)
{
    /* LLR-Ref: 010 */
    while( (AESSR & (BM_AESERF|BM_AESRF)) == 0U )
    {}

    if( AESSR & BM_AESERF )
    {
        /* LLR-Ref: 035 */
        g_sAesComponentData.bFlags |= AES_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_AES_RUN_ERROR;
    }
    else
    {
        /* Since control has exited the while loop, the AESRF is set this point*/
        /* LLR-Ref: 040 */
        ATA_aesGetData_C(&g_sAesComponentData.bDataBuffer[0], &AESDR);
        

        /* LLR-Ref: 060 */
        if(!(g_sAesComponentData.bStatus & AES_STATUS_BM_ENCR_WITHIN_DECR_FLAG)){
            g_sAesComponentData.bFlags |= AES_FLAGS_BM_READY_FLAG;
        }
    }

    /* clear flags */
    AESSR = BM_AESERF|BM_AESRF;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesLoadData_C</b>
    This function loads a 16 source byte buffer to the destination register.

    \param[out] pDestReg    Pointer to destination register
    \param[in]  pSourceBuf  Pointer to source data buffer

    \internal
    \li 010: Copy exactly 16 bytes from the given source buffer to the destination
             register (which is realized as a port, i.e. internally the register
             address represents multiple values).

    \Derived{Yes}

    \Rationale{Convenience function to easily copy data back and forth
               between registers, realized as ports and data buffers}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesLoadData_C(unsigned char volatile *pDestReg, uint8_t *pSourceBuf)
{
    /* LLR-Ref: 010 */
    for(uint8_t i = 0; i < 16; i++)
    {
        *pDestReg = pSourceBuf[i];
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesGetData_C</b>
    This function loads 16 bytes from source register to the destination bufffer.

    \param[out] pDestBuf    Pointer to destination buffer
    \param[in]  pSourceReg  Pointer to source register

    \image html ATA_aesGetData_C.png

    \internal
    \li 010: Copy exactly 16 bytes from the given source register (which is realized
             as a port, i.e. internally the register address represents multiple
             values) to the destination buffer.

    \Derived{Yes}

    \Rationale{Convenience function to easily copy data back and forth
               between registers, realized as ports and data buffers}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_aesGetData_C(uint8_t *pDestBuf, unsigned char volatile *pSourceReg)
{
    /* LLR-Ref: 010 */
    for(uint8_t i = 0; i < 16; i++)
    {
        pDestBuf[i] = *pSourceReg;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesApplyPaddingScheme_C</b>
    pads the raw data with a 0 immediately after the last data
    bit, and 0s for the remainder of the 128 bits. Depending on the user
    configuration, the incomplete byte of the source data must be padded
    LSB aligned (or) MSB aligned. If the source data length is a multiple
    of 8, the LSB- (or) MSB-aligned padding must be carried out in the next
    byte of the source data.
    NOTE: The numbering of all bit and byte positions starts from 0.

    \param[in] bDataSize    Size in bits of the source data
                            to be encrypted (or) decrypted
    
    Variables Usage:
    \li [in,out]  ::g_sAesComponentData   Global AES component data

    \image html ATA_aesApplyPaddingScheme_C.png

    \internal
    \li 010: Determine the number of whole bytes in the raw (source) data. The
             padding with '0' will be entered into the data at this byte position.

    \li 020: Determine the bit position at which '0' is to be padded : this is the
             number of incomplete bits in the last byte of the source data.

    \li 030: IF the number of source data bits is not a multiple of 8, do the
              padding depending on the user configuration how data are aligned in
              the incomplete byte.

    \li 040: Increase the index to the next padding byte.

    \li 050: Do the padding of the remaining bytes with '0'.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1355,Primus2P-1357}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_aesApplyPaddingScheme_C(uint8_t bDataSize)
{
    uint8_t bNumDataBytes;    /* number of complete bytes in the raw data */
    uint8_t bPadPos;          /* Bit position within the incomplete byte where
                              the first padding bit will be written */

    /* Update HW Trace Unit with function information */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_aesApplyPaddingScheme_C, g_sAesComponentData.bConfig);

    /* LLR-Ref: 010 */
    bNumDataBytes = (bDataSize >> BIT_3);

    /* LLR-Ref: 020 */
    bPadPos = (bDataSize % 8);

    /* LLR-Ref: 030 */
    if(bPadPos != BIT_0)
    {
        if(g_sAesComponentData.bConfig & AES_STATUS_BM_PAD_DIRECTION_LSB)
        { /* xxxx 1011 -> 0000 1011
                ^            ^      */
            g_sAesComponentData.bDataBuffer[bNumDataBytes] &= (MAX_BYTE_VAL >> (8U - bPadPos));
        }
        else
        { /* 1011 xxxx -> 1011 0000
                  ^            ^    */
            g_sAesComponentData.bDataBuffer[bNumDataBytes] &= (MAX_BYTE_VAL << (8U - bPadPos));
        }

        /* LLR-Ref: 040 */
        bNumDataBytes++;
    }

    /* LLR-Ref: 050 */
    if(bNumDataBytes < EEP_SECRET_KEY_LENGTH)
    {
        ATA_globalsInitSramSpace_C(&g_sAesComponentData.bDataBuffer[bNumDataBytes], (EEP_SECRET_KEY_LENGTH - bNumDataBytes));
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_aesComplete_ISR_C</b>
    handles the AES interrupt triggered in case an encryption, respectively
    decryption phase has been completed.

    Variables Usage:
    \li [in,out]  ::g_sAesComponentData   Global AES component data
    \li [out]     ::g_sDebug              Global Debug component data

    \image html ATA_aesComplete_ISR_C.png

    \internal
             IF the AESERF bit in AEESR register is set to 1, THEN
    \li 010: A run error occured during the last AES operation. Update bits 4:4 of
             the error status byte with AES_RUN_ERROR.

    \li 020: Set ::g_sDebug .bFlags[7] to DEBUG_ERROR_CODE_AES_RUN_ERROR.\n
           ELSE
    \li 040: The AES operation has been completed successfully. Call function
             ::ATA_aesGetData_C to copy the AESDR
             the elements of the data buffer of AES component data (code
             optimized, take ~20us longer than 16 individual steps).

    \li 050 Set ::g_sAesComponentData .bFlags[6] to denote that an encryption/decr.
             user request is completed successfully.\
           ENDIF

    \li 060: Clear the AES Ready and Error bit in AESSR register by writing a
              logical 1 to it.

    \li 060: Disable clock input for AES block.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-809, Primus2P-811}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
//lint -esym(765, ATA_aesComplete_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_aesComplete_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_aesComplete_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_aesComplete_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=AES_vect*/
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_aesComplete_ISR_C(void)
{
    /* Update HW Trace Unit with function information */
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_aesComplete_ISR_C, g_sAesComponentData.bConfig);

    if(AESSR & BM_AESERF)
    {
        /* LLR-Ref: 020 */
        g_sAesComponentData.bFlags |= AES_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_AES_RUN_ERROR;
    }
    else
    {
        /* LLR-Ref: 040 */
        ATA_aesGetData_C(&g_sAesComponentData.bDataBuffer[0], &AESDR);

        /* LLR-Ref: 050 */
        g_sAesComponentData.bFlags |= AES_FLAGS_BM_READY_FLAG;
    }

    /* LLR-Ref: 060 */
    AESSR = BM_AESERF|BM_AESRF;

    /* LLR-Ref: 070 */
    PRR0 |= BM_PRCU;
}
