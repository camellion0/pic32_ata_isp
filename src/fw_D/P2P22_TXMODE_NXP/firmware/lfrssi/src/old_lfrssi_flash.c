/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrssi/src/old_lfrssi_flash.c $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.12.1
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
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "lfrssi_flash.h"
#include "lfrssi.h"
//#include "../../../firmware/spi/src/ata5700_command_set.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
#pragma location = ".sram_FlashApp_LfRssi"
__no_init sLfRssiFlowCtrl g_sLfRssiFlowCtrl_flash;
#pragma location = ".sram_FlashApp_LfRssi"
__no_init sLfRssiConfig   g_sLfRssiConfig_flash;
#pragma location = ".sram_FlashApp_LfRssi"
__root __no_init sLfRssiData     g_sLfRssiData_flash;

#pragma location = ".sram_FlashApp_LfRssi"
__no_init lfRssiFlowStateMachineFuncLut_t *g_pLfRssiMeasStateMachineSerial_flash;

lfRssiFlowStateMachineFuncLut_t g_sLfRssiMeasStateMachineLutSerial_flash[] = {
    ATA_lfRssiMeasOpen_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasPrepSendResults_flash_C,
    ATA_lfRssiMeasSendResult_flash_C,
    ATA_lfRssiMeasClose_flash_C
};

#pragma location = ".sram_FlashApp_LfRssi"
__no_init lfRssiFlowStateMachineFuncLut_t *g_pLfRssiMeasStateMachineParallel_flash;

lfRssiFlowStateMachineFuncLut_t g_sLfRssiMeasStateMachineLutParallel_flash[] = {
    ATA_lfRssiMeasOpen_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasStartInt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasPrepSendResults_flash_C,
    ATA_lfRssiMeasSendResult_flash_C,
    ATA_lfRssiMeasClose_flash_C
};

uint8_t m_bDataByteCnt;             // RSSI result counter holds the number of bytes to transmit
uint8_t m_bLocDataBuff_FillLevel;   // only used for test purposes

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*----------------------------------------------------------------------------- */
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasInit_flash_C</b>

    \param none

    \return none

    \image html ATA_lfRssiMeasInit_flash_C.png
    \image rtf ATA_lfRssiMeasInit_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiMeasInit_flash_C(void)
{
 
/*  ///WAS  
  //g_sLfRssiConfig_flash.bFlags = LFRSSICONFIG_FLAGS_RESET;   // LF RSSI measurement disabled
    g_sLfRssiConfig_flash.bFlags &= (1<<LFRSSICONFIG_FLAGS_BM_MEASUREMENT_ENABLE_FLAG);//LF RSSI Enable GR
    g_sLfRssiConfig_flash.bStatus = LFRSSICONFIG_STATUS_RESET;
    g_pLfRssiMeasStateMachineSerial_flash   = g_sLfRssiMeasStateMachineLutSerial_flash;
    g_pLfRssiMeasStateMachineParallel_flash = g_sLfRssiMeasStateMachineLutParallel_flash;
  *///End was
  
    g_sLfRssiConfig_flash.bFlags = LFRSSICONFIG_FLAGS_RESET;   // LF RSSI measurement disabled
    g_sLfRssiConfig_flash.bStatus = LFRSSICONFIG_STATUS_RESET;
    g_pLfRssiMeasStateMachineSerial_flash   = g_sLfRssiMeasStateMachineLutSerial_flash;
    g_pLfRssiMeasStateMachineParallel_flash = g_sLfRssiMeasStateMachineLutParallel_flash;
  
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasStart_flash_C</b>

    \param none

    \return none

    \image html ATA_lfRssiMeasStart_flash_C.png
    \image rtf ATA_lfRssiMeasStart_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiMeasConfig_flash_C(uint8_t bType)
{
    if( bType & SERIAL_LFRSSI_MEASUREMENT )
    {
        g_sLfRssiConfig_flash.bStatus = LFRSSICONFIG_STATUS_BM_MEASUREMENT_SERIAL_FLAG; // LF RSSI measurement after sending TX response
        g_sLfRssiConfig_flash.bLfrscr = 0x14;   // enable interrupt and averaging result over 16 RSSI measurements
        g_sLfRssiConfig_flash.bLfrsmr = 0x00;   // 256us, 9bit, 5uA, channel settings?
        g_sLfRssiFlowCtrl_flash.fpLut = g_pLfRssiMeasStateMachineSerial_flash;
        m_bLocDataBuff_FillLevel      = 0;
    }
    else
    {
        g_sLfRssiConfig_flash.bStatus = LFRSSICONFIG_STATUS_RESET;      // LF RSSI measurement in parallel with sending TX response
        g_sLfRssiConfig_flash.bLfrscr = 0x12;   // enable interrupt and averaging result over 4 RSSI measurements
        g_sLfRssiConfig_flash.bLfrsmr = 0x00;   // 256us, 9bit, 5uA, channel settings?
        g_sLfRssiFlowCtrl_flash.fpLut = g_pLfRssiMeasStateMachineParallel_flash;
        m_bLocDataBuff_FillLevel      = 4;
    }

    g_sLfRssiFlowCtrl_flash.bIndex = 0;
    g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex = 0;

    m_bDataByteCnt = 1;
    g_sLfRssiConfig_flash.bFlags = LFRSSICONFIG_FLAGS_BM_MEASUREMENT_ENABLE_FLAG;   // enable LF RSSI measurement
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasProcessing_flash_C</b>
    triggers the active sw state of currently running sw state machine. This
    function should be placed in main loop context or should be run periodically
    to ensure RFTX functionality

    010:    set GPIOR0.1 to indicate begin of state machine activity for debug purpose
    020:    decode and execute currently active sw state
    030:    reset GPIOR0.1 to indicate end of state machine activity for debug purpose

    \param none

    \return none

    \image html ATA_lfRssiMeasProcessing_flash_C.png
    \image rtf ATA_lfRssiMeasProcessing_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiMeasProcessing_flash_C(void)
{
    /* LLR-Ref: 010 */
    //GPIOR0 |= BIT_MASK_1;   /* set GPIOR0.1 to indicate begin of state machine activity */

    /* LLR-Ref: 020 */
    lfRssiFlowStateMachineFunc_t fpFunc = *(lfRssiFlowStateMachineFunc_t)g_sLfRssiFlowCtrl_flash.fpLut[g_sLfRssiFlowCtrl_flash.bIndex];
    fpFunc();

    /* LLR-Ref: 030 */
    //GPIOR0 &= (uint8_t)~BIT_MASK_1;   /* reset GPIOR0.1 to indicate end of state machine activity */
}



/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasOpen_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasOpen_flash_C.png
    \image rtf ATA_lfRssiMeasOpen_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasOpen_flash_C(void)
{
    /* TODO AJ: using flags instead states? */
    /* NOTE AJ: index changed from 0 to 1 by state 0 itself, so state 1 has to be invoked at least one time */
    /* do the LF RSSI initialization in dependency of certain TX state machine state */
    if( ( g_sRfTxFlowCtrl.bIndex == RFTX_BUF_STATE_WAIT_AVCC ) ||
        ( g_sRfTxFlowCtrl.bIndex == RFTX_BUF_STATE_WAIT_XTO ) ||
        ( g_sRfTxFlowCtrl.bIndex == RFTX_BUF_STATE_WAIT_FILLLEVEL ) )
    {
        /* TODO AJ: compare last state and current state, if unequal, set flag
                    and store current state */
        if( g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex != g_sRfTxFlowCtrl.bIndex )
        {
            g_sLfRssiConfig_flash.bStatus |= LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;
            g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex = g_sRfTxFlowCtrl.bIndex;
        }
        else
        {
            /* TODO AJ: invoke open routine only if both states matching */

            if( g_sLfRssiConfig_flash.bStatus & LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG )
            {
                ATA_lfRssiMeasOpen_C();
                g_sLfRssiFlowCtrl_flash.bIndex++;
                g_sLfRssiConfig_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;

                /* TODO AJ: implement error handling */
            }
        }
    }
    else
    {
        g_sLfRssiConfig_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;
    }

}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasStartInt_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasStartInt_flash_C.png
    \image rtf ATA_lfRssiMeasStartInt_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasStartInt_flash_C(void)
{
    /* TODO AJ: using flags instead states? */
    /* NOTE AJ: index changed from 0 to 1 by state 0 itself, so state 1 has to be invoked at least one time */
    /* start the external LF RSSI measurement after PH timer has expired */
    if( g_bResponseDelayReached && !g_bBytesToTransmit )
    {
        g_bResponseDelayReached = FALSE;
        ATA_lfRssiMeasStartInt_C( g_sLfRssiConfig_flash.bLfrscr, g_sLfRssiConfig_flash.bLfrsmr|0x07, 0x00 );

        /* reconfigure PH timer with 3ms (serial measurement), 4ms (parallel measurement) */
        PHTCOR  = (g_sLfRssiConfig_flash.bStatus & BIT_MASK_0) ? 91 : 123;
        PHTCMR |= BM_PHTE;

        g_sLfRssiFlowCtrl_flash.bIndex++;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasStartExt_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasStartExt_flash_C.png
    \image rtf ATA_lfRssiMeasStartExt_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasStartExt_flash_C(void)
{
    /* TODO AJ: using flags instead states? */
    /* NOTE AJ: index changed from 0 to 1 by state 0 itself, so state 1 has to be invoked at least one time */
    /* start the external LF RSSI measurement after PH timer has expired */
    if( g_bResponseDelayReached && !g_bBytesToTransmit )
    {
        g_bResponseDelayReached = FALSE;
        ATA_lfRssiMeasStartExt_C( g_sLfRssiConfig_flash.bLfrscr, g_sLfRssiConfig_flash.bLfrsmr&0xF8 );

        /* reconfigure PH timer with 3ms (serial measurement), 4ms (parallel measurement) */
        PHTCOR  = (g_sLfRssiConfig_flash.bStatus & BIT_MASK_0) ? 91 : 123;
        PHTCMR |= BM_PHTE;

        g_bBytesToTransmit       = m_bLocDataBuff_FillLevel;
        m_bLocDataBuff_FillLevel = 0;

        g_sLfRssiFlowCtrl_flash.bIndex++;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasWaitReady_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasWaitReady_flash_C.png
    \image rtf ATA_lfRssiMeasWaitReady_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasWaitReady_flash_C(void)
{
    /* wait until LF RSSI measurement signalizes readiness */
    if( g_sLfRssi.bStatus & LFRSSI_STATUS_BM_DATA_AVAILABLE_FLAG )
    {
        /* TODO AJ: possibility to start a follow-up measurement before data conversion? */

        /* check (possible) channel timeout signalization */
        ATA_lfRssiMeasCheckTimeOut_C( &g_sLfRssi.wRawLfRssi[0], &g_sLfRssi.bChanTimeOutMask, APPL_LOWER_THRESHOLD_RAW_RSSI_VALUE, APPL_UPPER_THRESHOLD_RAW_RSSI_VALUE );
        /* convert measured data and add to transmit buffer */
        ATA_lfRssiMeasLfRssi2IndVoltage_C( &g_sLfRssi.wRawLfRssi[0], (uint16_t*) &g_bTxBuffer[m_bDataByteCnt] );

        /* increment number of data bytes to send and state machine index */
        m_bDataByteCnt += 2;

        /* clear flag due to data have been processed */
        g_sLfRssi.bStatus &= ~LFRSSI_STATUS_BM_DATA_AVAILABLE_FLAG;

        g_sLfRssiFlowCtrl_flash.bIndex++;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasPrepSendResults_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasPrepSendResults_flash_C.png
    \image rtf ATA_lfRssiMeasPrepSendResults_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasPrepSendResults_flash_C(void)
{
    /* wait until PH timer has expired */
    if( g_bResponseDelayReached )
    {
        g_bResponseDelayReached = FALSE;

        /* reconfigure PH timer with 2ms due to sending LF RSSI results */
        PHTCOR  = 58;       /* 62 * 32ms = 1984ms */
        PHTCMR |= BM_PHTE;

        /* keep configurations? */
        //uint8_t bTxFifoStartFillLevel = g_sRfTxCurrentService.sPath.bTxSetPath[0] & 0x3F;
        //uint8_t bPreambleFifoStartFillLevel = g_sRfTxCurrentService.sPath.bTxSetPath[1] & 0x1F;

        /* copy data to DFIFO */
        //DFC = BM_DFDRA; /* enable AVR write to FIFO */
        //ATA_rfTxFillDFifo_C(bufferIdx, &g_bTxBuffer[0]);

        /* after transmitting LF RSSI results, TX statemachine ends in IDLE mode */
        g_sRfTx.bConfig &= ~BM_RFTXCONFIG_BCONFIG_STAY_TX;

        g_sLfRssiFlowCtrl_flash.bIndex++;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasSendResult_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasSendResult_flash_C.png
    \image rtf ATA_lfRssiMeasSendResult_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasSendResult_flash_C(void)
{
    /* wait until PH timer has expired */
    if( g_bResponseDelayReached )
    {
        //g_bResponseDelayReached = FALSE;

        /* stop and reset running PH timer */
        PHTCMR |= BM_PHRES;
        g_bBytesToTransmit = m_bDataByteCnt;

        /* copy data to DFIFO */
        //DFC = BM_DFDRA; /* enable AVR write to FIFO */
        //ATA_rfTxFillDFifo_C( , &g_bTxBuffer[0] );

        g_sLfRssiFlowCtrl_flash.bIndex++;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasClose_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasClose_flash_C.png
    \image rtf ATA_lfRssiMeasClose_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiMeasClose_flash_C(void)
{
    ATA_lfRssiMeasClose_C();
    g_sLfRssiConfig_flash.bFlags   = LFRSSICONFIG_FLAGS_RESET;
    g_sLfRssiConfig_flash.bStatus  = LFRSSICONFIG_STATUS_RESET;
    g_sLfRssiFlowCtrl_flash.bIndex = 0;
    g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex = 0;
}


