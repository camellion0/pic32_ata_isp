//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrx/src/lfrx_flash.c $
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
/** \file lfrx_flash.c
*/
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "lfrx_flash.h"
#include "../../eep/src/eep.h"
#include "../../init/src/init_flash.h"
#include "../../system/src/system_flash.h"
#include "../../spi/src/ata5700_command_set_flash.h"
#include "../../lfrx/src/lfrx_immo_flash.h"
#include "../../tp/src/tp_flash.h"
#include "../../lfrssi/src/lfrssi_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/

/** \brief <b>g_sLfRx_flash</b>
    contains the 3D LF Receiver configuration
*/
#pragma location = ".sram_FlashApp_LfRx"
__root __no_init s3dLfRxConfig g_sLfRx_flash;

/** \brief <b>g_sLfRxIdShadowValues_flash</b>
    contains the shadow register values to be applied during an ID match
    interrupt
*/
#pragma location = ".sram_FlashApp_LfRx"
__root __no_init sPhShadowRegisters g_sLfRxIdShadowValues_flash;

/** \brief <b>g_sLfRxIdShadowValues_flash</b>
    contains the shadow register values to be applied during an EOT interrupt
*/
#pragma location = ".sram_FlashApp_LfRx"
__root __no_init sPhShadowRegisters g_sLfRxEotShadowValues_flash[ATA_SYSVER_MAX_LF_CONT_EOT_TELEGRAM_DEFS];

/** \brief <b>g_bLfRxEotShadowIndex_flash</b>
    contains the index of the shadow register values to be applied during an
    EOT interrupt
*/
#pragma location = ".sram_FlashApp_LfRx"
__root __no_init uint8_t            g_bLfRxEotShadowIndex_flash;

/** \brief <b>g_uTpConfig</b>
    structure to copy the TP configuration from EEPOM
*/
__no_init uConfiguration g_uTpConfig;

/** \brief <b>g_bRxBufferIndex</b>
    index for receive buffer(g_bRxBuffer) used during LF reception
*/
__no_init uint8_t g_bRxBufferIndex;

/** \brief <b>g_bTpStatusByte</b>
    status byte of the transponder
*/
__no_init uint8_t g_bTpStatusByte;

/** \brief <b>g_bBitsToReceive</b>
    indication of how many bits are expected to be received
*/
__no_init uint8_t g_bBitsToReceive;

/** \brief <b>g_bReceiveState</b>
    indication of receive state
*/
__no_init uint8_t g_bReceiveState;

/* transmission buffer to hold the data bytes to be transmitted */
__no_init uint8_t g_bTxBuffer[36];

/* receive buffer to hold the data bytes Received */
__no_init uint8_t g_bRxBuffer[36];

/* variable to hold response delay reached flag */
uint8_t g_bResponseDelayReached = FALSE;

/*  */
uint8_t g_bBytesToTransmit = 0;

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfId0MatchInterrupt_ISR_flash_C</b>
    shall handle the LF ID0 interrupt triggered when ID0 has been found within
    the received data stream.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfId0MatchInterrupt_ISR_flash_C.png
    \image rtf ATA_lfId0MatchInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFID0INT_vect
__interrupt VOIDFUNC ATA_lfRxId0MatchInterrupt_ISR_flash_C(void)
{
    /* Update 3D LF Received status */
    g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_PHID0F;
    
    
    /* Do event pin handling */
    if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHID0F )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
    
    ID0_Wake=0x01;
    
    LDFCKSW |= (1<<LDFSCSW); 
    while ((LDFCKSW & (1<<LDFSCKS)) ==0);          
    
    LFFR = 0x0F;//Clear the flags 
    LFIMR |= (1<<LFEOIM);
    LDFCKSW &= ~(1<<LDFSCSW); 
    PORTD &= ~(1<<1);   // Set PD3 high (LED1)

    
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxId1MatchInterrupt_ISR_flash_C</b>
    shall handle the LF ID1 interrupt triggered when ID1 has been found within
    the received data stream.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxId1MatchInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxId1MatchInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFID1INT_vect
__interrupt VOIDFUNC ATA_lfRxId1MatchInterrupt_ISR_flash_C(void)
{
    /* Update 3D LF Received status */
    g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_PHID1F;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHID1F )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
    ID1_Wake=0x01;
    
    LDFCKSW |= (1<<LDFSCSW); 
    while ((LDFCKSW & (1<<LDFSCKS)) ==0);          
    
    LFFR = 0x0F;//Clear the flags 
    LFIMR |= (1<<LFEOIM);
    LDFCKSW &= ~(1<<LDFSCSW); 
    
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxIdFrameEndInterrupt_ISR_flash_C</b>
    shall handle the LF Frame End interrupt triggered when ID1 has been found within
    the received data stream.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxIdFrameEndInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxIdFrameEndInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFFEINT_vect
__interrupt VOIDFUNC ATA_lfRxIdFrameEndInterrupt_ISR_flash_C(void)
{
   
  
  /* Update 3D LF Received status */
    g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_PHIDFF;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHIDFF )
    {
        ATA_systemSetEventPin_flash_ASM();
    }

    /* Handle EM Transponder Rx/Tx */
    if (extReq.tpEmModeConfig)
    {
        /* Disable ID Frame interrupt */
        PHIMR &= ~BM_PHIDFIM;
        PHIDFR = 0x00;          /* disable ID frame length */

        /**/
        PHTCR |= BM_CSM;

        /* Set global variable to EM mode reconfiguration */
        g_bTpEmTransponderStateIndex_flash = EM_MODE_STATE_RECONFIG;

    }
    else /* Normal LF/Transponder processing */
    {
        if ( (PHTCR & BM_CSM) != 0x00 )
        {
            /* Apply shadow register values for ID match */
            PHCRPH = g_sLfRxIdShadowValues_flash.bPhcrph;
            PHCRPL = g_sLfRxIdShadowValues_flash.bPhcrpl;
            PHCSTH = g_sLfRxIdShadowValues_flash.bPhcsth;
            PHCSTL = g_sLfRxIdShadowValues_flash.bPhcstl;
            PHIDFR = g_sLfRxIdShadowValues_flash.bPhidfr;
            PHDFR  = g_sLfRxIdShadowValues_flash.bPhdfr;
            PHTBLR = g_sLfRxIdShadowValues_flash.bPhtblr;
            PHCRCR = g_sLfRxIdShadowValues_flash.bPhcrcr;

            if ((PRR1 & BM_PRLFTP) == 0x00)
            {
                TPECMR = g_sLfRxIdShadowValues_flash.bTpecmr;
                TPECR1 = g_sLfRxIdShadowValues_flash.bTpecr1;
                TPECR2 = g_sLfRxIdShadowValues_flash.bTpecr2;
                TPECR3 = g_sLfRxIdShadowValues_flash.bTpecr3;
                TPECR4 = g_sLfRxIdShadowValues_flash.bTpecr4;
            }

            /* Only modify CSM bit */
            if ( (g_sLfRxIdShadowValues_flash.bPhtcr & BM_CSM) == 0x00U )
            {
                PHTCR &= ~BM_CSM;
            }
            else
            {
                PHTCR |= BM_CSM;
            }
          }
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxEotInterrupt_ISR_flash_C</b>
    shall handle the LF end of telegram interrupt.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxEotInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxEotInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFEOT_vect
__interrupt VOIDFUNC ATA_lfRxEotInterrupt_ISR_flash_C(void)
{
        DDRD |= (1<<1);
        PORTD |= (1<<1);   // Set PD3 high (LED1)

  /* Check if EOT has been detected */
    if ( (LFFR & BM_LFEOF) != 0x00U )
    {
        g_sLfRx_flash.bLfFlags |= BM_LFRXCONFIG_LF_FLAGS_LFEOF;

        /* Do event pin handling */
        if ( g_sEventHandling_flash.bLf3dRx & BM_LFRXCONFIG_LF_FLAGS_LFEOF )
        {
            ATA_systemSetEventPin_flash_ASM();
        }

        /* Clear flag */
        LFFR |= BM_LFEOF;
    }

    /* Check if a timeout has been detected */
    if ( (LFFR & BM_LFTOF) != 0x00U )
    {
        g_sLfRx_flash.bLfFlags |= BM_LFRXCONFIG_LF_FLAGS_LFTOF;

        /* Do event pin handling */
        if ( g_sEventHandling_flash.bLf3dRx & BM_LFRXCONFIG_LF_FLAGS_LFTOF )
        {
            ATA_systemSetEventPin_flash_ASM();
        }

        /* Clear flag */
        LFFR |= BM_LFTOF;
    }

    /* Check if a CRC error has been detected */
    if ( (PHFR & BM_CRCEF) != 0x00U )
    {
        g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_CRCEF;

        /* Do event pin handling */
        if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_CRCEF )
        {
            ATA_systemSetEventPin_flash_ASM();
        }

        /* Clear flag */
        PHFR |= BM_CRCEF;
    }
 
    LF_DecErrFlag = 0x01; 
    LFFR = 0x0F;//Clear the flags 
    LFIMR =0;
    

}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxBitCountReachedInterrupt_ISR_flash_C</b>
    shall handle the Data Frame End interrupt and the Telegram Bit Length
    reached interrupt.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxDataFrameEndInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxDataFrameEndInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFBCR_vect
__interrupt VOIDFUNC ATA_lfRxBitCountReachedInterrupt_ISR_flash_C(void)
{
if ((PORTC & 0x01)==0x01) PORTC &= ~(1<<PORTC0);
    else PORTC |= (1<<PORTC0);       
  if( extReq.lfrxRftxConfig & BIT_MASK_0 )
    {
        ATA_lfRxBitCntReachedImmo_flash_C();
    }
    else if ( extReq.tpEmModeConfig == 0x01 )
    {

        /* Transmission finished, go back to init state */
        if ((TPCR3 & BM_TPTD) != 0x00U)
        {
            g_bTpEmTransponderStateIndex_flash = EM_MODE_STATE_INIT;

            /* Stop transmission. */
            TPCR3 &= ~BM_TPTD;
        }
        /* Reception finished */
        else if ((TPCR3 & BM_TPRD) != 0x00U)
        {
            /* Set global variable to EM mode processing */
            g_bTpEmTransponderStateIndex_flash = EM_MODE_STATE_PROCESSING;
        }

        /* Clear Telegram Bit length reached flag, since this interrupt is
           enabled. PHDFF interrupt flag is not used for EM mode. */
        PHFR |= BM_PHTBLF;
    }
    else
    {
        /* Check if Data Frame End has been detected */
        if ( (PHFR & BM_PHDFF) != 0x00U )
        {
            g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_PHDFF;

            /* Do event pin handling */
            if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHDFF )
            {
                ATA_systemSetEventPin_flash_ASM();
            }

            /* Clear flag */
            PHFR |= BM_PHDFF;
        }

        /* Check if Telegram Bit Length reached has been detected */
        if ( (PHFR & BM_PHTBLF) != 0x00U )
        {
            g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_PHTBLF;

            /* Do event pin handling */
            if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHTBLF )
            {
                ATA_systemSetEventPin_flash_ASM();
            }

            /* Clear flag */
            PHFR |= BM_PHTBLF;

            /* Set next shadow configuration for next EOT. */
            if (g_bLfRxEotShadowIndex_flash >= ATA_SYSVER_MAX_LF_CONT_EOT_TELEGRAM_DEFS)
            {
                g_bLfRxEotShadowIndex_flash = ATA_SYSVER_MAX_LF_CONT_EOT_TELEGRAM_DEFS - 0x01U;
            }

            /**/
            if ( (PHTCR & BM_CSM) != 0x00 )
            {
                PHCRPH = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhcrph;
                PHCRPL = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhcrpl;
                PHCSTH = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhcsth;
                PHCSTL = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhcstl;

                /* Load first shadow register values which DO NOT require a clock switch */
                PHIDFR = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhidfr;
                PHDFR  = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhdfr;
                PHTBLR = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhtblr;
                PHCRCR = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhcrcr;

                if ((PRR1 & BM_PRLFTP) == 0x00)
                {
                    TPECMR = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bTpecmr;
                    TPECR1 = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bTpecr1;
                    TPECR2 = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bTpecr2;
                    TPECR3 = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bTpecr3;
                    TPECR4 = g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bTpecr4;
                }

                /* Only modify CSM bit */
                if ( (g_sLfRxEotShadowValues_flash[g_bLfRxEotShadowIndex_flash].bPhtcr) == 0x00U )
                {
                    PHTCR &= ~BM_CSM;
                }
                else
                {
                    PHTCR |= BM_CSM;
                }

                /**/
                g_bLfRxEotShadowIndex_flash++;
            }
        }

        /* Check if a CRC error has been detected */
        if ( (PHFR & BM_CRCEF) != 0x00U )
        {
            g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_CRCEF;

            /* Do event pin handling */
            if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_CRCEF )
            {
                ATA_systemSetEventPin_flash_ASM();
            }

            /* Clear flag */
            PHFR |= BM_CRCEF;
        }
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxLDFIMfoErrorInterrupt_ISR_flash_C</b>
    shall handle an overflow/underflow error of the PH FIFO.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxLDFIMfoErrorInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxLDFIMfoErrorInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LDFOUE_vect
__interrupt VOIDFUNC ATA_lfRxLDFIMfoErrorInterrupt_ISR_flash_C(void)
{
    /* Check if EOT has been detected */
    g_sLfRx_flash.bLDFFLags1 |= BM_LFRXCONFIG_PH_FLAGS_1_OUFLF;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bPhRxTx1 & BM_LFRXCONFIG_PH_FLAGS_1_OUFLF )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxLDFIMfoFillLevelReachedInterrupt_ISR_flash_C</b>
    shall handle the PH FIFO fill level reached interrupt.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxLDFIMfoFillLevelReachedInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxLDFIMfoFillLevelReachedInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LDFFLR_vect
__interrupt VOIDFUNC ATA_lfRxLDFIMfoFillLevelReachedInterrupt_ISR_flash_C(void)
{
    /* Check if the fill level of the PH FIFO has been detected */
    g_sLfRx_flash.bLDFFLags1 |= BM_LFRXCONFIG_PH_FLAGS_1_FLRF;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bPhRxTx1 & BM_LFRXCONFIG_PH_FLAGS_1_FLRF )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxPhTimerCompareInterrupt_ISR_flash_C</b>
    shall handle the detection of the PH Timer compare interrupt.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxPhTimerCompareInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxPhTimerCompareInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFTCOR_vect
__interrupt VOIDFUNC ATA_lfRxPhTimerCompareInterrupt_ISR_flash_C(void)
{
    __disable_interrupt();
    /*if( extReq.lfrxRftxConfig )
    //if( g_sLfRssiConfig_flash.bFlags & LFRSSICONFIG_FLAGS_BM_MEASUREMENT_ENABLE_FLAG )
    {
        // disable LF timer
        if( !(extReq.lfrxRftxConfig & BIT_MASK_2) )
        //if( g_sLfRssiConfig_flash.bStatus & LFRSSICONFIG_STATUS_BM_MEASUREMENT_SERIAL_FLAG )
        {
            LTCMR &= ~BM_LTENA;
        }
    }
    else
    {

        LTCMR &= ~BM_LTENA;
    }*/
   
    g_bResponseDelayReached = TRUE;
    __enable_interrupt();

    g_sLfRx_flash.bLDFFLags0 |= BM_LFRXCONFIG_PH_FLAGS_0_PHCOF;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHCOF )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxSyncDetectedInterrupt_ISR_flash_C</b>
    shall handle the detection of the synchronizaion pattern.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxSyncDetectedInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxSyncDetectedInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFPBD_vect
__interrupt VOIDFUNC ATA_lfRxSyncDetectedInterrupt_ISR_flash_C(void)
{
    g_sLfRx_flash.bLfFlags |= BM_LFRXCONFIG_LF_FLAGS_LFSYDF;
    
    /* Do event pin handling */
    if ( g_sEventHandling_flash.bLf3dRx & BM_LFRXCONFIG_LF_FLAGS_LFSYDF )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxDecoderErrorInterrupt_ISR_flash_C</b>
    shall handle the detection of a decoder error.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxDecoderErrorInterrupt_ISR_flash_C.png
    \image rtf ATA_lfRxDecoderErrorInterrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=LFDE_vect
__interrupt VOIDFUNC ATA_lfRxDecoderErrorInterrupt_ISR_flash_C(void)
{ 
  g_sLfRx_flash.bLfFlags |= BM_LFRXCONFIG_LF_FLAGS_LFDEF;
    
    /* Do event pin handling */
    if ( g_sEventHandling_flash.bLf3dRx & BM_LFRXCONFIG_LF_FLAGS_LFDEF )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
    

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxInit_flash_C</b>
    shall initialize the Flash LF Receiver component data.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_lfRxInit_flash_C.png
    \image rtf ATA_lfRxInit_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRxInit_flash_C(void)
{
    /* LF Receiver and Protocol Handler */
    g_sLfRx_flash.bLDFFLags0 = 0x00U;
    g_sLfRx_flash.bLDFFLags1 = 0x00U;
    g_sLfRx_flash.bLfFlags = 0x00U;
    g_sLfRx_flash.bStatus = 0x00U;
    g_sLfRx_flash.bConfig = 0x00U;
}