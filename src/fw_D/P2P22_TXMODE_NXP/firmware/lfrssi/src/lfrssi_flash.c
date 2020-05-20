//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrssi/src/lfrssi_flash.c $
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
/** \file lfrssi_flash.c
*/

//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "lfrssi_flash.h"
#include "lfrssi.h"
#include "../../../firmware/timer3/src/timer3.h"
#include "../../../firmware/timer4/src/timer4.h"

#include "../../globals/src/globals.h"
#include "../../eep/src/eep.h"
#include "../../eep/src/eep_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
#pragma location = ".sram_FlashApp_LfRssi"
__no_init sLfRssiAppFlowCtrl g_sLfRssiFlowCtrl_flash;

#pragma location = ".sram_FlashApp_LfRssi"
__no_init sLfRssiAppCtrl  g_sLfRssiCtrl_flash;

#pragma location = ".sram_FlashApp_LfRssi"
__no_init sLfRssiAppRegConfig g_sLfRssiRegConfig_flash;

#pragma location = ".sram_FlashApp_LfRssi"
__root __no_init sLfRssiAppCustConfig g_sLfRssiCustConfig_flash;

#pragma location = ".sram_FlashApp_LfRssi_BMW"
__root __no_init sLfRssiAppResult g_sLfRssiBmwResult_flash[3];

#pragma location = ".sram_FlashApp_LfRssi_BMW"
__no_init lfRssiFlowStateMachineFuncLut_t *g_pLfRssiMeasStateMachineSerial_flash;

lfRssiFlowStateMachineFuncLut_t g_sLfRssiMeasStateMachineLutSerial_flash[] = {
    ATA_lfRssiMeasOpen_flash_C,
    ATA_lfRssiMeasEnableLfReceiver_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasExecCalc_flash_C,
    ATA_lfRssiMeasPrepSendResults_flash_C,
    ATA_lfRssiMeasSendResult_flash_C,
    ATA_lfRssiMeasClose_flash_C
};

#pragma location = ".sram_FlashApp_LfRssi_BMW"
__no_init lfRssiFlowStateMachineFuncLut_t *g_pLfRssiMeasStateMachineParallel_flash;

lfRssiFlowStateMachineFuncLut_t g_sLfRssiMeasStateMachineLutParallel_flash[] = {
    ATA_lfRssiMeasOpen_flash_C,
    ATA_lfRssiMeasEnableLfReceiver_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasStartExt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasStartInt_flash_C,
    ATA_lfRssiMeasWaitReady_flash_C,
    ATA_lfRssiMeasExecCalc_flash_C,
    ATA_lfRssiMeasPrepSendResults_flash_C,
    ATA_lfRssiMeasSendResult_flash_C,
    ATA_lfRssiMeasClose_flash_C
};


//#pragma location = ".sram_FlashApp_LfRssi_HFM"
//__root __no_init sLfRssiAppResult g_sLfRssiHfmResult_flash[9];
//#pragma location = ".sram_FlashApp_LfRssi_HFM"
//__no_init uint16_t g_wLfRssiMeasSamples[48];
//#pragma location = ".sram_FlashApp_LfRssi_HFM"
//__no_init lfRssiFlowStateMachineFuncLut_t *g_pLfRssiHfmStateMachine_flash;

lfRssiFlowStateMachineFuncLut_t g_sLfRssiHfmStateMachineLut_flash[] = {
    ATA_lfRssiMeasOpenHfm_flash_C,
    ATA_lfRssiMeasEnableLfReceiver_flash_C,
    ATA_lfRssiMeasWaitReadyHfm_flash_C,
    ATA_lfRssiMeasExecCalc_flash_C,
    ATA_lfRssiMeasClose_flash_C
};

uint8_t           m_bDataByteCnt;               // RSSI result counter holds the number of bytes to transmit
uint8_t           m_bLocDataBuff_FillLevel;     // only used for test purposes
uint8_t           m_bNumMeasCnt;                // measurement counter value
uint8_t           m_bPayloadOffsetTxBuf;        // offset value used to store the measurement result data (payload) in TX buffer
uint16_t          m_wBref;
sLfRssiAppResult *m_pResultExt;                 // pointer to structure holding the external measurement results
sLfRssiAppResult *m_pResultInt;                 // pointer to structure holding the internal measurement results
sLfRssiAppResult *m_pCurrResult;                // pointer to structure to store the current measurement results

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
    uint8_t tmpBuf[16];

    g_sLfRssiCtrl_flash.bFlags  = LFRSSICONFIG_FLAGS_RESET;
    g_sLfRssiCtrl_flash.bStatus = LFRSSICONFIG_STATUS_RESET;

    // read LF RSSI End of Line and LF RSSI SRC calibration EEPROM settings
    if( ATA_eepReadBytes_C( &tmpBuf[0], (uint16_t)&g_sEepFlashAppLfRssiEndOfLineCalibrationSettings_flash, sizeof(sEepFlashAppLfRssiEndOfLineCalibrationSettings) + 1U ) == EEC_NO_ERROR )
    {
        // assign pointers to related state machines
        g_pLfRssiMeasStateMachineSerial_flash   = g_sLfRssiMeasStateMachineLutSerial_flash;
        g_pLfRssiMeasStateMachineParallel_flash = g_sLfRssiMeasStateMachineLutParallel_flash;
        //g_pLfRssiHfmStateMachine_flash          = g_sLfRssiHfmStateMachineLut_flash;

        m_bNumMeasCnt         = 0;
        m_bPayloadOffsetTxBuf = 0; // no payload offset

        // build Bref value for final result calculations
        m_wBref = (uint16_t)((tmpBuf[14]<<SHIFT_LOW_TO_HIGH_BYTE | tmpBuf[13]));

        // load EEPROM settings to registers, pass the desired SRC calibration value
        ATA_lfRssiSetEepromConfig_C( tmpBuf[15] );

        // calculate the LF channel calibration values
        ATA_lfRssiCalcChanCalibVal_C( tmpBuf[0], (uint16_t*)&tmpBuf[1], (uint16_t*)&tmpBuf[7] );
    }
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
        LTCMR &= ~BM_LTCM;  // disable LF timer continuous mode

        g_sLfRssiCtrl_flash.bStatus   = LFRSSICONFIG_STATUS_BM_MEASUREMENT_SERIAL_FLAG; // LF RSSI measurement after sending TX response
        g_sLfRssiFlowCtrl_flash.fpLut = g_pLfRssiMeasStateMachineSerial_flash;
        m_bLocDataBuff_FillLevel      = 0;
    }
    else
    {
        LTEMR  = 0x00;                                                  // disable LF timer events
        LTCOR  = (uint8_t)(g_sLfRssiCustConfig_flash.wCycleTime>>8);    // set next compare match value
        LTCMR &= ~BM_LTSM;                                              // disable start LF timer on event

        g_sLfRssiCtrl_flash.bStatus   = LFRSSICONFIG_STATUS_RESET;      // LF RSSI measurement in parallel with sending TX response
        g_sLfRssiFlowCtrl_flash.fpLut = g_pLfRssiMeasStateMachineParallel_flash;
        m_bLocDataBuff_FillLevel      = 4;
    }

    m_pResultExt = &g_sLfRssiBmwResult_flash[0];    // set pointer to structure holding the first external measurement results
    m_pResultInt = &g_sLfRssiBmwResult_flash[2];    // set pointer to structure holding the internal measurement results

    g_sLfRssiCustConfig_flash.bNumMeas = 3; // 2x external + 1x internal LF RSSI measurement, NOTE: only dummy internal measurement for serial measurement test
    g_sLfRssiCustConfig_flash.bOptions = 2; // internal LF RSSI measurement takes place at the end of the measurement sequence

    g_sLfRssiFlowCtrl_flash.bIndex = 0;
    g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex = 0;

    m_bDataByteCnt        = 1; // at least one byte to send back via RF TX
    m_bPayloadOffsetTxBuf = 1; // one byte offset, used as response identifier
    g_sLfRssiCtrl_flash.bFlags = LFRSSICONFIG_FLAGS_BM_MEASUREMENT_ENABLE_FLAG;   // enable LF RSSI measurement
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
    /* NOTE AJ: index changed from 0 to 1 by state 0 itself, so state 1 has to be invoked at least one time */
    /* do the LF RSSI initialization in dependency of certain TX state machine state */
    if( ( g_sRfTxFlowCtrl.bIndex == RFTX_BUF_STATE_WAIT_AVCC ) ||
        ( g_sRfTxFlowCtrl.bIndex == RFTX_BUF_STATE_WAIT_XTO ) ||
        ( g_sRfTxFlowCtrl.bIndex == RFTX_BUF_STATE_WAIT_FILLLEVEL ) )
    {
        /* compare last state and current state, if unequal, set flag and store current state */
        if( g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex != g_sRfTxFlowCtrl.bIndex )
        {
            g_sLfRssiCtrl_flash.bStatus |= LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;
            g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex = g_sRfTxFlowCtrl.bIndex;
        }
        else
        {
            /* invoke open routine only if both states matching */
            if( g_sLfRssiCtrl_flash.bStatus & LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG )
            {
                ATA_lfRssiOpen_C();
                g_sLfRssiFlowCtrl_flash.bIndex++;
                g_sLfRssiCtrl_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;

                /* TODO AJ: implement error handling */
            }
        }
    }
    else
    {
        g_sLfRssiCtrl_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;
    }

}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasEnableLfReceiver_flash_C</b>
    enables the LF receiver and increment the state number.

    \param none

    \return none

    \image html ATA_lfRssiMeasEnableLfReceiver_flash_C.png
    \image rtf ATA_lfRssiMeasEnableLfReceiver_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasEnableLfReceiver_flash_C(void)
{
    LFCR1 |= BM_LFRE;
    g_sLfRssiFlowCtrl_flash.bIndex++;
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
    /* NOTE AJ: index changed from 0 to 1 by state 0 itself, so state 1 has to be invoked at least one time */
    /* start the internal LF RSSI measurement after LF timer has expired */
    if( g_bResponseDelayReached && !g_bBytesToTransmit )
    {
        g_bResponseDelayReached = FALSE;

        /* close Timer3 to synchronize TX modulator and Timer3 clocks when re-opening by TX state machine */
        ATA_timer3Close_C();

        ATA_lfRssiMeasStart_C( &g_sLfRssiRegConfig_flash, 1U, 0U );

        /* reconfigure LF timer, 3ms (serial measurement), 4ms (parallel measurement) */
        LTCOR = (uint8_t)g_sLfRssiCustConfig_flash.wCycleTime;
        /* NOTE AJ: LF timer still running as configured */

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
    /* NOTE AJ: index changed from 0 to 1 by state 0 itself, so state 1 has to be invoked at least one time */
    /* start the external LF RSSI measurement after LF timer has expired */
    if( g_bResponseDelayReached && !g_bBytesToTransmit )
    {
        g_bResponseDelayReached = FALSE;

        /* close Timer3 to synchronize TX modulator and Timer3 clocks when re-opening by TX state machine */
        ATA_timer3Close_C();

        ATA_lfRssiMeasStart_C( &g_sLfRssiRegConfig_flash, 0U, 0U );

        /* no LF timer reconfiguration needed, still running, re-use of counter value */
        /* NOTE AJ: LF timer still running as configured */

        g_bBytesToTransmit       = m_bLocDataBuff_FillLevel;
        m_bLocDataBuff_FillLevel = 0;

        /* increment number of data bytes to send and state machine index */
        m_bDataByteCnt += 2;

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
    if( g_sLfRssi.bStatus & LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG )
    {
        /* gather measurement results */
        ATA_lfRssiGetAverageResult_C((uint8_t*)&g_sLfRssiBmwResult_flash[m_bNumMeasCnt].wRawLfRssi[0], (uint8_t*)0x0000);

        /* increment number of measurements */
        m_bNumMeasCnt++;

        /* clear flag due to data have been processed */
        g_sLfRssi.bStatus &= ~LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG;

        /* reconfigure LF timer in case serial measurement is taking place */
        if( (g_sLfRssiCtrl_flash.bStatus & LFRSSICONFIG_STATUS_BM_MEASUREMENT_SERIAL_FLAG) && (m_bNumMeasCnt == 2) )
        {
            LTCOR = (uint8_t)g_sLfRssiCustConfig_flash.wCycleTime;       /* 62 * 32ms = 1984ms */
        }
        /* NOTE AJ: LF timer still running as configured */

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
    /* wait until LF timer has expired */
    if( g_bResponseDelayReached )
    {
        g_bResponseDelayReached = FALSE;

        /* reconfigure LF timer, 2ms due to sending LF RSSI results */
        //LTCOR  = (uint8_t)g_sLfRssiCustConfig_flash.wCycleTime;       /* 62 * 32ms = 1984ms */
        /* NOTE AJ: LF timer still running as configured */

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
    /* wait until LF timer has expired */
    if( g_bResponseDelayReached )
    {
        /* disable running LF timer */
        LTCMR &= ~BM_LTENA;
        g_bBytesToTransmit = m_bDataByteCnt;

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
    ATA_lfRssiClose_C();

    g_sLfRssiCtrl_flash.bFlags     = LFRSSICONFIG_FLAGS_RESET;
    g_sLfRssiCtrl_flash.bStatus    = LFRSSICONFIG_STATUS_RESET;
    g_sLfRssiFlowCtrl_flash.bIndex = 0;
    g_sLfRssiFlowCtrl_flash.bLastRfTxStateIndex = 0;
}




/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasConfigHfm_flash_C</b>

    \param none

    \return none

    \image html ATA_lfRssiMeasConfigHfm_flash_C.png
    \image rtf ATA_lfRssiMeasConfigHfm_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiMeasConfigHfm_flash_C(uint8_t bType)
{
    timerIRQHandler t4IrqHandler = (timerIRQHandler)ATA_lfRssiMeasStartExtHfm_flash_C;

    g_sLfRssiCtrl_flash.bFlags  = LFRSSICONFIG_FLAGS_RESET;   // LF RSSI measurement disabled
    g_sLfRssiCtrl_flash.bStatus = LFRSSICONFIG_STATUS_RESET;

    // check if XTO is running, if not activate AVCC and XTO
    if( !(SUPCR & BM_AVEN) || !(FEEN1 & BM_XTOEN) )
    {
        ATA_globalsActivateXTO_C(); // activate XTO, used as input clock source (CLKT) for Timer 4
    }

    //g_sLfRssiFlowCtrl_flash.fpLut = g_pLfRssiHfmStateMachine_flash;
    //g_sLfRssiFlowCtrl_flash.fpLut = g_sLfRssiHfmStateMachineLut_flash;

    // 8 HFM pulses a 2,8ms, 0,2ms idle between pulses
    if( (bType & 0x03) == 0x01)
    {
    }
    // 8 HFM pulses a 1ms, 1ms idle between pulses
    else if((bType & 0x03) == 0x02)
    {
    }
    // 6 HFM pulses a 600us, 400us idle between pulses
    else if( (bType & 0x03) == 0x03)
    {
    }
    else
    {}

    // check for internal LF RSSI measurement first
    if( (g_sLfRssiCustConfig_flash.bOptions & 0x03) == 0x01 )
    {
        // replace function pointer
        t4IrqHandler = (timerIRQHandler)ATA_lfRssiMeasStartIntHfm_flash_C;
    }

    sTimerAsyn16BitParams sTimer4Params = {
        (BM_T4ENA|BM_T4CRM),                                // T4CR
        (BM_T4CS0),                                         // T4MRA  -> CLKT as input clock source
        (BM_T4ICS1| BM_T4CE0),                              // T4MRB
        (uint8_t)g_sLfRssiCustConfig_flash.wCycleTime,      // T4CORL
        (uint8_t)(g_sLfRssiCustConfig_flash.wCycleTime>>8), // T4CORH
        (BM_T4CIM),                                         // T4IMR
        (timerIRQHandler)0x0000,                            // g_sTimer4.fpOvfIsr
        t4IrqHandler,                                       // g_sTimer4.fpCompIsr
        (timerIRQHandler)0x0000                             // g_sTimer4.fpCapIsr
    };

    m_bNumMeasCnt = 0;   //

    /* LLR-Ref: 040 */
    if(  ATA_timer4Open_C(&sTimer4Params) == OK )
    {
        // possibility to open here LF RSSI module and enable LF receiver (LFRE) in next state???
        // ATA_lfRssiOpen_C();

        g_sLfRssiFlowCtrl_flash.bIndex = 0;

        g_sLfRssiCtrl_flash.bFlags = LFRSSICONFIG_FLAGS_BM_MEASUREMENT_ENABLE_FLAG;   // enable LF RSSI measurement

        // check if internal LF RSSI measurement is part of the measurement sequence
        if( g_sLfRssiCustConfig_flash.bOptions & 0x03 )
        {
            g_sLfRssiCtrl_flash.bStatus |= LFRSSICONFIG_STATUS_BM_INTERNAL_MEASUREMENT_FLAG;
        }
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasOpenHfm_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasOpenHfm_flash_C.png
    \image rtf ATA_lfRssiMeasOpenHfm_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasOpenHfm_flash_C(void)
{
    //if( do the SRC calibration manually if desired )
    {
    //ATA_calibInit_C();
    //ATA_calibStartCalibration_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC);
    }


    if( g_sLfRssi.bFlags & LFRSSI_FLAGS_BM_ERROR_FLAG )
    {
        /* TODO AJ: implement error handling */
    }
    else
    {
        ATA_lfRssiOpen_C();

        g_sLfRssiFlowCtrl_flash.bIndex++;
        g_sLfRssiCtrl_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG;

        // set trigger signal for test case
        g_sLfRssiCtrl_flash.bFlags |= 0x02;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasStartIntHfm_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasStartIntHfm_flash_C.png
    \image rtf ATA_lfRssiMeasStartIntHfm_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasStartIntHfm_flash_C(void)
{
    /* start an internal LF RSSI measurement on occurrence of Timer4 compare match interrupt */
    ATA_lfRssiMeasStart_C( &g_sLfRssiRegConfig_flash, 1U, 0U );

    //m_pCurrResult = &g_sLfRssiHfmResult_flash[8];

    // close Timer4 in case last measurement was started
    if( m_bNumMeasCnt == (g_sLfRssiCustConfig_flash.bNumMeas-1) )
    {
        ATA_timer4Close_C();
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasStartExtHfm_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasStartExtHfm_flash_C.png
    \image rtf ATA_lfRssiMeasStartExtHfm_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasStartExtHfm_flash_C(void)
{
    uint8_t idx = m_bNumMeasCnt;

    /* start an external LF RSSI measurement on occurrence of Timer4 compare match interrupt */
    ATA_lfRssiMeasStart_C( &g_sLfRssiRegConfig_flash, 0U, 0U );

    // check if internal LF RSSI measurement has been already done
    if( (g_sLfRssiCustConfig_flash.bOptions & 0x03 )
        &&
        !(g_sLfRssiCtrl_flash.bStatus & LFRSSICONFIG_STATUS_BM_INTERNAL_MEASUREMENT_FLAG) )
    {
        idx--;
    }

    //m_pCurrResult = &g_sLfRssiHfmResult_flash[idx];

    if( m_bNumMeasCnt == (g_sLfRssiCustConfig_flash.bNumMeas-1) )
    {
        ATA_timer4Close_C();
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasWaitReadyHfm_flash_C</b>
    .

    \param none

    \return none

    \image html ATA_lfRssiMeasWaitReadyHfm_flash_C.png
    \image rtf ATA_lfRssiMeasWaitReadyHfm_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasWaitReadyHfm_flash_C(void)
{
    /* wait until LF RSSI measurement signalizes readiness */
    if( g_sLfRssi.bStatus & LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG )
    {
        /* gather measurement results */
        //ATA_lfRssiGetAverageResult_C((uint8_t*)&g_sLfRssiHfmResult_flash[m_bNumMeasCnt].wRawLfRssi[0], &g_sLfRssiHfmResult_flash[m_bNumMeasCnt].bSignDetect[0]);
        ATA_lfRssiGetAverageResult_C((uint8_t*)&m_pCurrResult->wRawLfRssi[0], &m_pCurrResult->bSignDetect[0]);
        //ATA_lfRssiGetSamplesResult_C((uint8_t*)&g_wLfRssiMeasSamples[0], 16U, 0U);

        // increment number of measurements
        m_bNumMeasCnt++;

        // check for internal LF RSSI measurement
        if( g_sLfRssiCtrl_flash.bStatus & LFRSSICONFIG_STATUS_BM_INTERNAL_MEASUREMENT_FLAG )
        {
            // check if first measurement was an internal LF RSSI measurement
            if( (g_sLfRssiCustConfig_flash.bOptions & 0x03) == 1 )
            {
                g_sTimer4.fpCompIsr = (timerIRQHandler)ATA_lfRssiMeasStartExtHfm_flash_C;
                g_sLfRssiCtrl_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_INTERNAL_MEASUREMENT_FLAG;
            }
            // check if next/last measurement is an internal LF RSSI measurement
            else if( (m_bNumMeasCnt+1) == g_sLfRssiCustConfig_flash.bNumMeas )
            {
                g_sTimer4.fpCompIsr = (timerIRQHandler)ATA_lfRssiMeasStartIntHfm_flash_C;
                g_sLfRssiCtrl_flash.bStatus &= ~LFRSSICONFIG_STATUS_BM_INTERNAL_MEASUREMENT_FLAG;
            }
            else{}
        }

        // clear flag due to data have been copied
        g_sLfRssi.bStatus &= ~LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG;

        if( m_bNumMeasCnt >= g_sLfRssiCustConfig_flash.bNumMeas )
        {
            // reset to first structure holding the external measurement results
            //m_pResultExt = &g_sLfRssiHfmResult_flash[0];
           // m_pResultInt = &g_sLfRssiHfmResult_flash[8];

            g_sLfRssiFlowCtrl_flash.bIndex++;
        }
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasExecCalc_flash_C</b>
    executes all required calculations.

    \param none

    \return none

    \image html ATA_lfRssiMeasExecCalc_flash_C.png
    \image rtf ATA_lfRssiMeasExecCalc_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiMeasExecCalc_flash_C(void)
{
    uint8_t  i, j;
    uint8_t  numCalcs = g_sLfRssiCustConfig_flash.bNumMeas & 0x0F;
    uint16_t val;

    /* do calculations only if internal measurement data are available */
    if( g_sLfRssiCustConfig_flash.bOptions & 0x03 )
    {
        numCalcs--;

        for( i=0, j=0; i<numCalcs; i++, j+=2 )
        {
            // calculate the channel correction values
            ATA_lfRssiCalcChanCorr_C( (uint8_t*)&m_pResultExt->wRawLfRssi[0], (uint8_t*)&m_pResultInt->wRawLfRssi[0], (uint8_t*)&m_pResultExt->wCorrLfRssi[0] );

            // calculate the 3D vector
            ATA_lfRssiCalc3dVec_C( (uint8_t*)&m_pResultExt->wCorrLfRssi[0], (uint8_t*)&m_pResultExt->w3dVecVal );

            // calculate the linear voltage representation
            ATA_lfRssiCalcLog2Lin_C( (uint8_t*)&m_pResultExt->w3dVecVal, (uint8_t*)&m_pResultExt->wLinearVal );

            // calculate the Bappl value
            val = ATA_lfRssiCalcBappl_C( m_wBref, m_pResultExt->wLinearVal );

            // store Bappl results to local buffer
            g_bTxBuffer[m_bPayloadOffsetTxBuf+j]   = val;
            g_bTxBuffer[m_bPayloadOffsetTxBuf+j+1] = val>>8;

            m_pResultExt++;
        }
    }

    g_sLfRssiFlowCtrl_flash.bIndex++;
}

