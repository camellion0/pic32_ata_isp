//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/tp/src/tp_flash.c $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.3.18.0
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
/** \file tp_flash.c
*/

//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "tp_flash.h"
#include "../../eep/src/eep.h"
#include "../../init/src/init_flash.h"
#include "../../system/src/system_flash.h"
#include "../../globals/src/globals_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/

/** \brief <b>tpRxTx</b>
    contains the Transponder configuration
*/
#pragma location = ".sram_FlashApp_TpRxTx"
__root __no_init sTpRxTxConfig g_sTpRxTx;

/** \brief <b>g_bLfRxEmTransponderModeCommand_flash</b>
    contains which EM Transponder Mode command has been received
*/
#pragma location = ".sram_FlashApp_TpRxTx"
__root __no_init static uint8_t g_bTpEmTransponderModeCommand_flash;

/** \brief <b>g_bTpEmTransponderStateIndex_flash</b>
    contains the EM Mode SW state
*/
#pragma location = ".sram_FlashApp_TpRxTx"
__root __no_init uint8_t g_bTpEmTransponderStateIndex_flash;

/** \brief <b>g_fTpEmTransponderModeErrorFlag_flash</b>
    indicates any EM Transponder Mode error
*/
#pragma location = ".sram_FlashApp_TpRxTx"
__root __no_init static uint8_t g_fTpEmTransponderModeErrorFlag_flash;


/** \brief <b>g_fTpEmTransponderModeLongCommand_flash</b>
    indicates the reception of a long EM Transponder Mode command, e.g.
    Authentication
*/
#pragma location = ".sram_FlashApp_TpRxTx"
__root __no_init static uint8_t g_fTpEmTransponderModeLongCommand_flash;



/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/**/
VOIDFUNC ATA_tpEmModeInit_flash_C(void)
{
    /* Reset long command indication */
    g_fTpEmTransponderModeLongCommand_flash = FALSE;
    g_fTpEmTransponderModeErrorFlag_flash = 0x00U;

    /* disable ID0 and ID1 */
    PHID0L = 0x00U;
    PHID1L = 0x00U;

    /* Set ID frame length to 3 bit to be able to set the parity bit length
       when BCR interrupt is executed. */
    PHIDFR = 0x03U;

    /* data frame length 4 bit */
    PHDFR  = 0x04U;

    /* telegram bit length 4 bit */
    PHTBLR = 0x04U;

    /* LLR-Ref: 050 */
    PHTCR &= ~BM_CSM;
    PHTCR |= BM_FRFIFO;
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)(BM_LDFSCKS | BM_LDFSCSW));

    /* LLR-Ref: 070 */
    LDFC = 0x00U|BM_LDFMSB;  /* Set fill level configuration to 0 */
    LDFFL = BM_LDFCLR;        /* clear RdPtr, WrPtr, fill-level, status and bit length */
    PHFR = 0x07;            /* clear data frame, bit length and CRC error flags */

    /* LLR-Ref: 080 */
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x00U);

    /* LLR-Ref: 090 */
    TPIMR &= ~BM_TPFTIM;                /* disable transponder field timeout interrupt (TPTOERR_vect) */
    PHIMR = (BM_PHIDFIM | BM_PHTBLIM);  /* disable data frame interrupt (LFBCR_vect)
                                           enable bit count reached interrupt ()
                                           enable ID Frame interrupt
                                        */
    LDFIM  = 0x00U;                      /* disable FIFO interrupts */


    /* LLR-Ref: 120 */
    TPCR3 |= (BM_TPRD | BM_TPTLIW);
}


/* function will be executed in an interrupt context */
VOIDFUNC ATA_tpEmModeCommandReconfiguration_flash_C(void)
{
    /* Retrieve the received command ID, switch FIFO clock */
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)(BM_LDFSCKS | BM_LDFSCSW));
    
    /* Store EM command ID */
    g_bTpEmTransponderModeCommand_flash = LDFD;
    g_bTpEmTransponderModeCommand_flash >>= 5;
    
    /* If command ID is set to ID mode */
    switch (g_bTpEmTransponderModeCommand_flash)
    {
        case EM_CMD_ID_MODE:
          break;

        case EM_CMD_AUTHENTICATION:
          /* data frame and telegram bit length to 36 bit */
          PHDFR  = 0x24U;
          PHTBLR = 0x24U;
          break;

        default:
          break;
    }

    /**/
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x00U);
}


/* function will be executed in an interrupt context */
VOIDFUNC ATA_tpEmModeSingleTelProc_flash_C(void)
{
    /* Variable to hold the function's return value. */
    eEepErrorCode eepErrorCode = EEC_NO_ERROR;
    uint8_t tempDataBuffer[EEP_XROW_UID_SIZE];
    uint8_t bParityBitValue;

    /* Retrieve the received command ID, switch FIFO clock */
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)(BM_LDFSCKS | BM_LDFSCSW));

    /* If command ID is set to ID mode */
    switch (g_bTpEmTransponderModeCommand_flash)
    {
        case EM_CMD_ID_MODE:

            /* Store EM command ID parity bit */
            bParityBitValue = LDFD;
            bParityBitValue >>= 7;
            
            if (bParityBitValue == EM_CMD_ID_MODE_PARITY)
            {
                /* Disable Transponder reception, since a response is required */
                TPCR3 &= ~ (BM_TPRD | BM_TPTLIW);

                /* LLR-Ref: 000 */
                PHDFR  = 0x32U;              /* data frame length 50 bit */
                PHTBLR = 0x32U;              /* telegram bit length 50 bit */

                /* LLR-Ref: 000 */
                LDFC = (BM_LDFMSB);  /* Set fill level configuration to 0 */
                LDFFL = 0x80U;       /* clear RdPtr, WrPtr, fill-level, status and bit length */
                LDFIM = 0x00U;       /* disable interrupts */

                /* Write data to PH FIFO: First header 12 Bit 1's and 4 Bit 0's */
                LDFD = 0xFFU;
                LDFD = 0xF0U;

                /* Set Manchester encoding and bit length */
                TPECMR = 0xFFU;
                TPECR1 = 0x32U;

                /* Fill EEPROM UID of 32 bits */
                eepErrorCode = ATA_eepReadBytes_C(&tempDataBuffer[0], (uint16_t)&g_sAtmelEEPromSection.eepUID[0], EEP_XROW_UID_SIZE);

                /* UID is stored MSB-wise in V2.0, thus this code is correct */
                for (uint8_t i = 0; i < EEP_XROW_UID_SIZE; i++)
                {
                    if (eepErrorCode == EEC_NO_ERROR)
                    {
                        LDFD = tempDataBuffer[i];
                    }
                    else
                    {
                        LDFD = 0x55U;
                    }
                }

                /* Add two trailing bits to have a none byte-aligned response. */
                LDFD = 0xC0U;

                /* Enable Transponder trasmit sequence */
                TPCR3 |= BM_TPTD;
            }
            break;

        case EM_CMD_AUTHENTICATION:

            /* Only the first part of the AUTHENTICATION message has been
               received */
            if ( g_fTpEmTransponderModeLongCommand_flash == FALSE )
            {
                g_fTpEmTransponderModeLongCommand_flash = TRUE;

                /* Store AUTHENTICATION command ID parity bit */
                bParityBitValue = LDFD;
                bParityBitValue >>= 7;

                if (bParityBitValue != EM_CMD_AUTHENTICATION_PARITY)
                {
                    g_fTpEmTransponderModeErrorFlag_flash = 0x01U;
                }
            }
            else
            {
                /* Send ACK if command ID and parity do match, i.e. no error is flagged */
                if ( g_fTpEmTransponderModeErrorFlag_flash == 0x00U )
                {
                    /* Disable Transponder reception, since a response is required */
                    TPCR3 &= ~ (BM_TPRD | BM_TPTLIW);

                    /* Send NACK, since parity bit is wrong */
                    PHDFR  = 0x0AU;              /* data frame length 10 bit */
                    PHTBLR = 0x0AU;              /* telegram bit length 10 bit */

                    LDFC = (BM_LDFMSB);  /* Set fill level configuration to 0 */
                    LDFFL = 0x80U;       /* clear RdPtr, WrPtr, fill-level, status and bit length */
                    LDFIM = 0x00U;       /* disable interrupts */

                    /* Write data to PH FIFO: 10 Bit ACK 0b1011 1011 10 */
                    LDFD = 0xBBU;
                    LDFD = 0x80U;

                    /* Set NRZ encoding and bit length */
                    TPECMR = 0xAAU;
                    TPECR1 = 0x0AU;

                    /* Enable Transponder trasmit sequence */
                    TPCR3 |= BM_TPTD;
                }
                else
                {
                    /* Disable Transponder reception, since a response is required */
                    TPCR3 &= ~ (BM_TPRD | BM_TPTLIW);

                    /* Send ACK, since parity bit is wrong */
                    PHDFR  = 0x0AU;              /* data frame length 10 bit */
                    PHTBLR = 0x0AU;              /* telegram bit length 10 bit */

                    LDFC = (BM_LDFMSB);  /* Set fill level configuration to 0 */
                    LDFFL = 0x80U;       /* clear RdPtr, WrPtr, fill-level, status and bit length */
                    LDFIM = 0x00U;       /* disable interrupts */

                    /* Write data to PH FIFO: 10 Bit NACK 0b1011 1011 01 */
                    LDFD = 0xBBU;
                    LDFD = 0x40U;

                    /* Set NRZ encoding and bit length */
                    TPECMR = 0xAAU;
                    TPECR1 = 0x0AU;

                    /* Enable Transponder trasmit sequence */
                    TPCR3 |= BM_TPTD;
                }
            }
            break;

        default:

          break;
    }

    /**/
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x00U);
}



/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_tpTimeoutError_flash_ISR_C</b>
    shall handle Transponder errors.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_tpTimeoutError_flash_ISR_C.png
    \image rtf ATA_tpTimeoutError_flash_ISR_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=TPTOERR_vect
__interrupt VOIDFUNC ATA_tpTimeoutError_flash_ISR_C(void)
{
    /* Check if a transponder bit error has been detected */
    if ( (TPFR & BM_TPBERF) != 0x00U )
    {
        g_sTpRxTx.bTpFlags |= BM_LFRXCONFIG_TP_FLAGS_TPBERF;

        /* Do event pin handling */
        if ( g_sEventHandling_flash.bTpRxTx & BM_LFRXCONFIG_TP_FLAGS_TPBERF )
        {
            ATA_systemSetEventPin_flash_ASM();
        }

        /* Clear flag */
        TPFR |= BM_TPBERF;
    }

    /* Check if a No field timeout has been detected */
    if ( (TPFR & BM_TPNFTF) != 0x00U )
    {
        g_sTpRxTx.bTpFlags |= BM_LFRXCONFIG_TP_FLAGS_TPNFTF;

        /* Do event pin handling */
        if ( g_sEventHandling_flash.bTpRxTx & BM_LFRXCONFIG_TP_FLAGS_TPNFTF )
        {
            ATA_systemSetEventPin_flash_ASM();
        }

        /* Clear flag */
        TPFR |= BM_TPNFTF;
    }

    /* Check if a field timeout has been detected */
    if ( (TPFR & BM_TPFTF) != 0x00U )
    {
        g_sTpRxTx.bTpFlags |= BM_LFRXCONFIG_TP_FLAGS_TPFTF;

        /* Do event pin handling */
        if ( g_sEventHandling_flash.bTpRxTx & BM_LFRXCONFIG_TP_FLAGS_TPFTF )
        {
            ATA_systemSetEventPin_flash_ASM();
        }

        /* Clear flag */
        TPFR |= BM_TPFTF;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_tpRxTxInit_flash_C</b>
    shall initialize the Transponder component data.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_tpRxTxInit_flash_C.png
    \image rtf ATA_tpRxTxInit_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_tpRxTxInit_flash_C(void)
{
    g_sTpRxTx.bTpFlags = 0x00U;
    g_sTpRxTx.bStatus = 0x00U;
    g_sTpRxTx.bConfig = 0x00U;

    /* EM Transponder Mode */
    g_bTpEmTransponderModeCommand_flash = 0x00U;
    g_bTpEmTransponderStateIndex_flash = EM_MODE_STATE_INIT;
    g_fTpEmTransponderModeErrorFlag_flash = 0x00U;
}