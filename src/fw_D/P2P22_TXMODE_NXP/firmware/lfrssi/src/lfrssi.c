//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrssi/src/lfrssi.c $
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
//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "lfrssi.h"
#include "../../globals/src/globals.h"
#include "../../eep/src/eep.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define LF_RSSI_NUM_EEPROM_DATA    12U

#define LF_RSSI_12Q4_EXPONENT_MIN   -9
#define LF_RSSI_12Q4_EXPONENT_SIZE  4
#define LF_RSSI_12Q4_VALUE_SIZE     12

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>g_sLfRssi</b>
    contains the LF RSSI component data.
*/
//lint -esym(9003, g_sLfRssi) FlSc (10.06.2014)
/* disable lint note 9003 - could define variable 'g_sLfRssi' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
#pragma location = ".lfRssi"
__no_init sLfRssi g_sLfRssi;


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiInit_C</b>
    initializes the LF RSSI component data with default values.

    Variable Usage;:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data

    \image html ATA_lfRssiInit_C.png

    \internal
    \li 010: Initialize component variables with default values.

    \Derived{Yes}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRssiInit_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiInit_C, 0x00U);

    /* LLR-Ref: 010 */
    g_sLfRssi.bFlags  = LFRSSI_FLAGS_RESET;
    g_sLfRssi.bStatus = LFRSSI_STATUS_RESET;

    g_sLfRssi.bOutOfRangeMask = 0U;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiOpen_C</b>
    is used to power-up the LF RSSI module, LF transponder and LF receiver.
    It disables the digital part of the LF receiver and selects the LF RSSI
    measurement mode.
    Note: LF channels for LF RSSI measurement has to be already enabled.

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data

    \image html ATA_lfRssiOpen_C.png

    \internal
    \li 010:  Disable interrupts globally due to changes on PRR1, transponder
              and LF receiver settings.

    \li 020:  Save PRR1 register setting due to configuration changes when using
              the LF RSSI module. Content will be restored when leaving the module.
              No need to save PRR0 register affected bit only used within module.

    \li 030:  Enable power to the RSSI module and LF receiver (the analog
              RSSI block is also powered on). Enable power to the LF transponder
              to access TPCR2 register.

    \li 040:  Save TPCR2 register setting before disabling the transponder mode to
              prevent system reset in case a present LF field will be turn off.

    \li 050:  Save LFCR1 register setting and disable LF receiver for mode switching.\n
              Note: Between disable and enable the LF receiver a waiting time of
                    at least 3 SRC clock cycles is required.

    \li 060:  Save SRCCAL register setting.
              Note: Reading SRCCAL register possible without enabling the clock (PRLFPH).

    \li 070:  Enable interrupts globally.

    \li 080:  Reset component variables except error flag.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2000}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiOpen_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiOpen_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    _CLI;

    /* LLR-Ref: 020 */
    g_sLfRssi.bPrr1 = PRR1;

    /* LLR-Ref: 030 */
    /* no power reduction bits to enable the digital LF RSSI register block, clocked by cp2io
       Note: It is assumed that LF receiver has been already enabled. But if not, it is enabled here. */
    PRR0 &= ~BM_PRLFRS;
    PRR1 &= ~(BM_PRLFTP | BM_PRLFR);

    /* LLR-Ref: 040 */
    g_sLfRssi.bTpcr2 = TPCR2;

    /* LLR-Ref: 050 */
    g_sLfRssi.bLfcr1 = LFCR1;
    LFCR1 |= BM_LFFM1;
    LFCR1 &= ~BM_LFRE;

    /* LLR-Ref: 060 */
    g_sLfRssi.bSrcCal = SRCCAL;

    /* LLR-Ref: 070 */
    _SEI;

    /* LLR-Ref: 080 */
    g_sLfRssi.bFlags &= LFRSSI_FLAGS_BM_ERROR_FLAG;
    g_sLfRssi.bStatus = LFRSSI_STATUS_RESET;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiClose_C</b>
    is used to power-down the LF RSSI module and the LF receiver including the
    analog RSSI blocks. It also restores temporarily saved register settings.

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data

    \image html ATA_lfRssiClose_C.png

    \internal
    \li 010:  Disable interrupts globally to restore register settings.

    \li 010:  Enable power to the LF protocol handler to restore SRCCAL value.

    \li 030:  Restore saved register settings.
              Note: The LF receiver is disabled when leaving LF RSSI module.

    \li 040:  Disable power to the LF RSSI module.

    \li 050:  Enable interrupts globally.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2001}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiClose_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiClose_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    _CLI;

    /* LLR-Ref: 020 */
    ATA_POWERON_C(PRR1, PRLFPH);

    /* LLR-Ref: 030 */
    SRCCAL = g_sLfRssi.bSrcCal;
    LFCR1  = g_sLfRssi.bLfcr1 & ~(BM_LFRE | BM_LFFM1);
    TPCR2  = g_sLfRssi.bTpcr2 & ~BM_TPD;

    /* no power reduction bits to disable the digital RSSI register block, clocked by cp2io
       disabling power of the LF receiver disables power of the analog RSSI block */
    PRR1  = g_sLfRssi.bPrr1;

    /* LLR-Ref: 040 */
    PRR0 |= BM_PRLFRS;

    /* LLR-Ref: 050 */
    _SEI;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiSetEepromConfig_C</b>
    is used to initialize LF RSSI registers with associated EEPROM configuration.

    \param[in]      bEepRssiSrcCalVal   Contains the SRC calibration setting used during LF RSSI measurement.

    Variable Usage:
    \li [out] ::g_sLfRssi   Global LF RSSI component data
    \li [out] ::g_sDebug    Global Debug component data

    \image html ATA_lfRssiSetEepromConfig_C.png

    \internal
    \li 010:  Enable power to the RSSI module before accessing the registers.

    \li 020:  Wait until any previous EEPROM access has been finished.

    \li 030:  Reset the digital LF RSSI block.

    \li 040:  Setup base address to read from EEPROM first.

    \li 050:  Enable EEPROM burst read mode.

    \li 060:  Signalize writing of the LF RSSI calibration configuration.

    \li 070:  Copy settings from EEPROM: 10 bytes for range correction,
              1 byte pre-divider switch and 1 byte nameless.

    \li 080:  Configure LF RSSI with the passed SRC calibration value.

    \li 090:  Clear bit to signalize end of LF RSSI register calibration
              sequence.

    \li 100:  ON occurrence of uncorrectable errors during EEPROM reads,
              THEN

    \li 110:    Set component global error flag and the global debug error
                variable with error code for EEPROM read error.
              ENDIF

    \li 120:  ON occurrence of access violation errors during EEPROM reads,
              THEN

    \li 130:    Set component global error flag and the global debug error
                variable with error code for EEPROM access error.
              ENDIF

    \li 140:  Clear Burst Read Mode flag and implicitly clear both EEPROM
              error flags.

    \li 150:  Disable power to the LF RSSI module within protected section.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2002,Primus2P-2478}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiSetEepromConfig_C(uint8_t bEepRssiSrcCalVal)  // registers in use: R16 registers unused: R17, R18, R19, R20, R21, R22, R23
{
    uint8_t i;
    uint8_t prr0;

    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiSetEepromConfig_C, 0x00);

    /* LLR-Ref: 010 */
    _CLI;
    prr0 = PRR0;
    PRR0 &= ~BM_PRLFRS;
    _SEI;

    /* LLR-Ref: 020 */
    while(EECR & BM_NVMBSY){}

    /* LLR-Ref: 030 */
    RSCR |= BM_RSRES;

    /* LLR-Ref: 040 */
    /*lint -e923 GeWi (23nov2011)*/
    /* disable lint error 923 - Cast between pointer type and an integral type.
     * Misra Rule 11.3 says: Casting between a pointer and an integer type should be avoided where possible,
     * but may be unavoidable when addressing memory mapped registers or other hardware specific features.
     */
    EEARH = (uint8_t)(((uint16_t)&g_sAtmelEEPromSection.eepRssiRangeCorrectionValues[0]) >> 8U);
    EEARL = (uint8_t)(((uint16_t)&g_sAtmelEEPromSection.eepRssiRangeCorrectionValues[0]) >> 0U);
    /*lint -restore */

    /* LLR-Ref: 050 */
    EECR2 |= BM_EEBRE;

    /* LLR-Ref: 060 */
    //RSMS2R = 0;       // already done by reset
    RSMS1R |= BM_RSSCAL;

    /* LLR-Ref: 070 */
    for( i = 1; i < LF_RSSI_NUM_EEPROM_DATA; i++ )
    {
        RSCALIB = EEDR;
        RSMS2R = i;
        while( !(RSSR & BM_RSSVLD) ){}
    }

    /* LLR-Ref: 080 */
    RSCALIB = bEepRssiSrcCalVal;
    RSMS2R  = 0x0C;
    while( !(RSSR & BM_RSSVLD) ){}

    /* LLR-Ref: 090 */
    RSMS1R &= ~BM_RSSCAL;

    /* LLR-Ref: 100 */
    if( EECR2 & BM_E2FF )
    {
        /* LLR-Ref: 110 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_EEPROM_READ_ERROR;
        EECR2 |= BM_E2FF;
    }

    /* LLR-Ref: 120 */
    if(EECR2 & BM_E2AVF)
    {
        /* LLR-Ref: 130 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_EEPROM_ACCESS_ERROR;
        EECR2 |= BM_E2AVF;
    }

    /* LLR-Ref: 140 */
    EECR2 &= ~BM_EEBRE;

    /* LLR-Ref: 150 */
    _CLI;
    PRR0 = prr0;
    _SEI;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasStart_C</b>
    is used to start the external LF RSSI measurement.

    \param[in]      pRegConfig  Contains the configuration for the registers RSCR, RSMS1R, RSMS2R, RSDLYR and RSSRCR
    \param[in]      bMode       Contains internal or external LF RSSI measurement request
    \param[in]      bSign       Contains sign detection request, only applicable in conjunction with external LF RSSI measurement

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data
    \li [out]    ::g_sDebug     Global Debug component data

    \image html ATA_lfRssiMeasStart_C.png

    \internal
    \li 010:  Reset component flags except the error flag. All previous information
              are cleared each time a new LF RSSI measurement is started.

    \li 020:  IF an LF RSSI module operation is still in progress,
              THEN

    \li 030:    Set component global error flag and the global debug error
                variable with error code for operation still active error.
              ELSEIF

    \li 040:  IF the analog LF RSSI part has not finished its initialization,
              THEN

    \li 050:    Set component global error flag and the global debug error
                variable with error code for RSSI analog part not ready error.
              ELSEIF

    \li 060:  IF no LF channel is activated to participate the measurement,
              THEN

    \li 070:    Set component global error flag and the global debug error
                variable with error code for invalid measurement channels error.
              ELSE

    \li 080:    Clear bits RSSCAL and RSINTM of input parameter used to configure
                register RSMS1R.

    \li 090:    IF an internal LF RSSI measurement shall be started,
                THEN

    \li 100:      Set bit RSINTM in register RSMS1R and signalize internal
                  LF RSSI measurement by setting internal status flag.
                ENDIF

    \li 110:    Clear bit RSOFM of input parameter used to configure
                register RSCR.

    \li 120:    IF an external LF RSSI measurement with sign detection
                shall be started,
                THEN

    \li 130:      Enable sign detection by setting bit SDEN in register RSCR and
                  enable all LF channels to participate the LF RSSI measurement
                  regardless RSCHxE bit settings of input parameter.

    \li 140:      Signalize sign detection LF RSSI measurement by setting internal
                  status flag.
                ENDIF

    \li 150:    IF there is a mismatch in the LF receiver channels participating
                the LF RSSI measurement and its enable configuration,
                THEN

    \li 160:      Signalize the mismatch by setting component global warning flag.
                ENDIF

    \li 170:    Configure RSMS2R, RSDLYR and RSSRCR register with input parameters.

    \li 180:    Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 190:    Reset the transponder watchdog timer.

    \li 200:    Disable interrupts globally before start the measurement.

    \li 210:    Signalize LF RSSI operation is in progress.

    \li 220:    Trigger start of LF RSSI measurement by setting bit RSOS in
                register RSCR.

    \li 230:    Enable interrupts globally.
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2004}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasStart_C(void *pRegConfig, uint8_t bMode, uint8_t bSign)  // registers in use: R17:R16, R18, R19 registers unused:  R20, R21, R22, R23
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiMeasStart_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    uint8_t tmpReg;
    sLfRssiRegConfig *psLfRssiRegConfig = (sLfRssiRegConfig*)pRegConfig;

    /* LLR-Ref: 010 */
    g_sLfRssi.bFlags &= LFRSSI_FLAGS_BM_ERROR_FLAG;
    g_sLfRssi.bStatus = LFRSSI_STATUS_RESET;

    tmpReg = psLfRssiRegConfig->bRsms1r & 0x07;     // mask RSSI channel enable bits

    /* LLR-Ref: 020 */
    if( RSCR & BM_RSOS )
    {
        /* LLR-Ref: 030 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE;
    }
    /* LLR-Ref: 040 */
    else if( !(RSSR & BM_RSRDY) )
    {
        /* LLR-Ref: 050 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_ANALOGPART_NOTREADY;
    }
    /* LLR-Ref: 060 */
    else if( !tmpReg )
    {
        /* LLR-Ref: 070 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_INVALID_MEASUREMENT_CHANNELS;
    }
    else
    {
        /* LLR-Ref: 080 */
        RSMS1R = psLfRssiRegConfig->bRsms1r & ~(BM_RSSCAL | BM_RSINTM);

        /* LLR-Ref: 090 */
        if( bMode )
        {
            /* LLR-Ref: 100 */
            RSMS1R            |= BM_RSINTM;
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_INTERNAL_MEASUREMENT_FLAG;
        }

        /* LLR-Ref: 110 */
        RSCR = psLfRssiRegConfig->bRscr & BM_RSOFM;

        /* LLR-Ref: 120 */
        if( !bMode && bSign )
        {
            /* LLR-Ref: 130 */
            RSCR   |= BM_RSSDEN;
            RSMS1R |= (BM_RSCH3E | BM_RSCH2E | BM_RSCH1E);

            /* LLR-Ref: 140 */
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_SIGNDETECT_ENA_FLAG;
            tmpReg = 0x07;
        }

        /* LLR-Ref: 150 */
        if( (LFCR0 & tmpReg) != tmpReg )    // check desired measurement channels vs. its enable configuration, if mismatching set warning flag
        {
            /* LLR-Ref: 160 */
            g_sLfRssi.bFlags |= LFRSSI_FLAGS_BM_CHANNEL_ENABLE_MISMATCH_FLAG;
        }

        /* LLR-Ref: 170 */
        RSMS2R = psLfRssiRegConfig->bRsms2r;    // check influence of bits RSSADR for measurement
        RSDLYR = psLfRssiRegConfig->bRsdlyr;    // may be load from EEPROM???
        RSSRCR = psLfRssiRegConfig->bRssrcr;    // obsolete for internal measurement

        /* LLR-Ref: 180 */
        RSFR  = BM_RSOFF;                       // all other flags are read-only

        /* LLR-Ref: 190 */
        TPCR2 |= BM_TPD;                        // reset transponder watchdog

        /* LLR-Ref: 200 */
        _CLI;

        /* LLR-Ref: 210 */
        g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;

        /* LLR-Ref: 220 */
        RSCR |= BM_RSOS;                        // start measurement

        /* LLR-Ref: 230 */
        _SEI;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiMeasStop_C</b>
    is used to signalize to end an active LF RSSI operation (mainly internal or
    external measurement). In case no operation is in progress, no unintended
    behaviour occurs. Measurement end request flag is cleared without side
    effects.

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data

    \image html ATA_lfRssiMeasStop_C.png

    \internal
    \li 010:  Disable interrupts globally to prevent .

    \li 020:  IF an LF RSSI operation is still in progress,
              THEN

    \li 030:    Set bit to finish the current operation and set internal flag
                signalizing that operation has been aborted.
              ENDIF

    \li 040:  Enable interrupts globally.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiMeasStop_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiMeasStop_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    _CLI;

    /* LLR-Ref: 020 */
    if( RSCR & BM_RSOS )
    {
        /* LLR-Ref: 030 */
        RSCR |= BM_RSEOR;
        g_sLfRssi.bFlags |= LFRSSI_FLAGS_BM_MEASUREMENT_ABORTED_FLAG;
    }

    /* LLR-Ref: 040 */
    _SEI;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiGetAverageResult_C</b>
    is used to read out the LF RSSI measurement average result for each LF channel
    and to store it to SRAM location. Furthermore, the results of a sign detection
    is also read and stored to SRAM location.

    \param[out]     pResMeas        Base address to data structure to store 3 x 16 bit LF RSSI averaging results
    \param[out]     pResSignDetect  Base address to data structure to store 4 x 8 bit LF RSSI sign detection results

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data
    \li [out]    ::g_sDebug     Global Debug component data

    \image html ATA_lfRssiGetAverageResult_C.png

    \internal
    \li 010:  Signalize LF RSSI operation is in progress.

    \li 020:  IF valid pointer to store the averaging results is used,
              THEN

    \li 030:    Clear bit RSSSV in register RSMS1R to be sure to read the LF channel
                average results from result registers RSRES1..3.

    \li 040:    Wait as long as the result registers are not containing the
                average results.

    \li 050:    Read the LF channel average result registers and store content
                to SRAM location.
              ENDIF

    \li 060:  IF valid pointer to store the sign detection results is used,
              THEN

    \li 070:    Read sign detection result registers and store content to
                SRAM location.
              ENDIF

    \li 080:  Signalize LF RSSI operation has been finished/is not in progress.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2503}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiGetAverageResult_C(uint8_t *pResMeas, uint8_t *pResSignDetect)  // registers in use: R17:R16, R19:R18 registers unused: R20, R21, R22, R23
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiGetAverageResult_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;

    /* LLR-Ref: 020 */
    if( pResMeas )
    {
        /* LLR-Ref: 030 */
        RSMS1R &= ~(BM_RSSCAL | BM_RSSSV);

        /* LLR-Ref: 040 */
        // in case RSSVLD never occurs, transponder watchdog expires and should do a software reset
        while( !(RSSR & BM_RSSVLD) ){}

        /* LLR-Ref: 050 */
        *(pResMeas+LFRSSI_BYTEOFFSET_RESULT_RSC1L) = RSRES1L;
        *(pResMeas+LFRSSI_BYTEOFFSET_RESULT_RSC1H) = RSRES1H;
        *(pResMeas+LFRSSI_BYTEOFFSET_RESULT_RSC2L) = RSRES2L;
        *(pResMeas+LFRSSI_BYTEOFFSET_RESULT_RSC2H) = RSRES2H;
        *(pResMeas+LFRSSI_BYTEOFFSET_RESULT_RSC3L) = RSRES3L;
        *(pResMeas+LFRSSI_BYTEOFFSET_RESULT_RSC3H) = RSRES3H;
    }

    /* LLR-Ref: 060 */
    if( pResSignDetect )
    {
        /* LLR-Ref: 070 */
        // content of registers only valid, if sign detection has been enabled
        // for external measurement
        *(pResSignDetect+LFRSSI_BYTEOFFSET_RESULT_SD12RR) = SD12RR;
        *(pResSignDetect+LFRSSI_BYTEOFFSET_RESULT_SD13RR) = SD13RR;
        *(pResSignDetect+LFRSSI_BYTEOFFSET_RESULT_SD23RR) = SD23RR;
        *(pResSignDetect+LFRSSI_BYTEOFFSET_RESULT_SD360R) = SD360R;
    }

    /* LLR-Ref: 080 */
    g_sLfRssi.bStatus &= ~LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiGetSamplesResult_C</b>
    is used to read out a certain number of LF RSSI sample values for
    each LF channel and to store it to SRAM location.

    \param[out]     pResMeas    Base address to data structure to store 3 x 16 bit LF RSSI sample value(s)
    \param[in]      bNum        Number of LF RSSI sample values to read
    \param[in]      bIndex      Index to read the LF RSSI sample value(s) firstly

    Variable Usage:
    \li [in,out]  ::g_sLfRssi   Global LF RSSI component data
    \li [out]     ::g_sDebug    Global Debug component data

    \image html ATA_lfRssiGetSamplesResult_C.png

    \internal
    \li 010:  IF valid pointer to store the sample value(s) is used,
              THEN

    \li 020:    Set bit RSSSV in register RSMS1R to be sure to read
                the LF channel sample results from result registers RSRES1..3.

    \li 030:    IF the desired number of samples to read exceeds the maximum
                number without index wrap around,
                THEN

    \li 040:      Limit number to 16 samples to read.
                ENDIF

    \li 050:    Signalize LF RSSI operation is in progress.

    \li 060:    IF the number of samples to read is not reached,
                THEN

    \li 070:      Set read index to register RSMS2R.

    \li 080:      Wait as long as the result registers are not containing the
                  sample values.

    \li 090:      Read the LF channel result registers and store sample value
                  to SRAM location.
                ENDIF

    \li 100:    Signalize LF RSSI operation has been finished/is not in progress.
              ELSE

    \li 110:    Set component global error flag and the global debug error
                variable with error code for invalid output pointers are used error.
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2503}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiGetSamplesResult_C(uint8_t *pResMeas, uint8_t bNum, uint8_t bIndex)  // registers in use: R17:R16, R18, R19 registers unused: R20, R21, R22, R23
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiGetSamplesResult_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    if( pResMeas )
    {
        /* LLR-Ref: 020 */
        RSMS1R &= ~BM_RSSCAL;
        RSMS1R |= BM_RSSSV;

        /* LLR-Ref: 030 */
        if( bNum > 16 )
        {
            /* LLR-Ref: 040 */
            bNum = 16;
        }

        /* LLR-Ref: 050 */
        g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;

        /* LLR-Ref: 060 */
        for( ; bNum > 0; bNum--, bIndex++ )
        {
            /* LLR-Ref: 070 */
            RSMS2R = bIndex & 0x0F; // possibility to apply OR operation to preserve bits RSAVGS[3..0],
                                    // RSAVGS[3..0] are overwritten when start a new measurement

            /* LLR-Ref: 080 */
            // in case RSSVLD never occurs, transponder watchdog expires and should do a software reset
            while( !(RSSR & BM_RSSVLD) ){}

            /* LLR-Ref: 090 */
            *(pResMeas++) = RSRES1L;
            *(pResMeas++) = RSRES1H;
            *(pResMeas++) = RSRES2L;
            *(pResMeas++) = RSRES2H;
            *(pResMeas++) = RSRES3L;
            *(pResMeas++) = RSRES3H;
        }

        /* LLR-Ref: 100 */
        g_sLfRssi.bStatus &= ~LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;
    }
    else
    {
        /* LLR-Ref: 110 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED;
    }

}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiCalcChanCalibVal_C</b>
    is used to calculate the calibration values for each LF channel.

    \param[in]      bMargin     MARGIN value for the calculation
    \param[in]      pRefVal     Pointer to structure containing 3 x 16 bit RSSIref values
    \param[in]      pNormVal    Pointer to structure containing 3 x 16 bit RSSInorm values

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data
    \li [out]    ::g_sDebug     Global Debug component data

    \image html ATA_lfRssiCalcChanCalibVal_C.png

    \internal
    \li 010:  IF valid input pointer is used,
              THEN

    \li 020:    Shift MARGIN value to fit 9q3 data format.

    \li 030:    Calculate calibration value for LF channel 1 and store it to
                component variable.

    \li 040:    Intermediate calculation.

    \li 050:    Calculate calibration value for LF channel 2 and store it to
                component variable.

    \li 060:    Calculate calibration value for LF channel 3 and store it to
                component variable.
              ELSE

    \li 070:    Set component global error flag and the global debug error
                variable with error code for invalid input pointers are used error.
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiCalcChanCalibVal_C(uint8_t bMargin, uint16_t *pRefVal, uint16_t *pNormVal)  // registers in use: R16, R19:R18, R21:R20, registers unused: R17, R22, R23
{
    uint16_t result, resultTmp;
    uint16_t wMargin;

    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiCalcChanCalibVal_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    if( pRefVal && pNormVal )
    {
        /* LLR-Ref: 020 */
        // shift MARGIN value to fit 9q3 data format
        wMargin = (uint16_t)(bMargin<<3);

        /* LLR-Ref: 030 */
        // RSSIref_x + MARGIN
        result = pRefVal[LFRSSI_INTOFFSET_REF_VAL_CHAN1] + wMargin;
        g_sLfRssi.bChanCalibVal[0] = result;
        g_sLfRssi.bChanCalibVal[1] = result>>8;

        /* LLR-Ref: 040 */
        // MARGIN + RSSInorm_x
        result    = wMargin + pNormVal[LFRSSI_INTOFFSET_NORM_VAL_CHAN1];
        resultTmp = result;

        /* LLR-Ref: 050 */
        // RSSIref_y + MARGIN + RSSInorm_x - RSSInorm_y
        result += pRefVal[LFRSSI_INTOFFSET_REF_VAL_CHAN2];
        result -= pNormVal[LFRSSI_INTOFFSET_NORM_VAL_CHAN2];
        g_sLfRssi.bChanCalibVal[2] = result;
        g_sLfRssi.bChanCalibVal[3] = result>>8;

        /* LLR-Ref: 060 */
        // RSSIref_z + MARGIN + RSSInorm_x - RSSInorm_z
        resultTmp += pRefVal[LFRSSI_INTOFFSET_REF_VAL_CHAN3];
        resultTmp -= pNormVal[LFRSSI_INTOFFSET_NORM_VAL_CHAN3];
        g_sLfRssi.bChanCalibVal[4] = resultTmp;
        g_sLfRssi.bChanCalibVal[5] = resultTmp>>8;
    }
    else
    {
        /* LLR-Ref: 070 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiCalcChanCorr_C</b>
    is used to calculate the channel correction value for the LF channels.

    \param[in]      pExtData    Pointer to structure containing 3 x 16 bit (raw) external LF RSSI values
    \param[in]      pIntData    Pointer to structure containing 3 x 16 bit (raw) internal LF RSSI values
    \param[out]     pResult     Pointer to 3 x 16 bit data structure to store the corrected LF RSSI channel values

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data
    \li [out]    ::g_sDebug     Global Debug component data

    \image html ATA_lfRssiCalcChanCorr_C.png

    \internal
    \li 010:  IF valid input and output pointers are used,
              THEN

    \li 020:    Disable interrupts globally to do an atomic flag check.

    \li 030:    IF no LF RSSI operation is in progress,
                THEN

    \li 040:      Signalize LF RSSI operation is in progress and re-enable
                  interrupts globally.

    \li 050:      Disable usage of RSSI operation finished interrupt and set
                  "calculation correction" mode to register RSCR.

    \li 060:      IF not all LF channel correction values has been calculated,
                  THEN

    \li 070:        Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 080:        Fill calculation registers with external, internal and
                    calibration value for the currently selected LF channel.

    \li 090:        Trigger start of correction calculation by setting bit RSOS in
                    register RSCR.

    \li 100:        Wait as long as the calculation has not finished.

    \li 110:        Read channel correction value and store it to SRAM location.
                  ENDIF

    \li 120:      Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 130:      Update component flag and status variables:\n
                  Signalize channel correction calculation has finished and the
                  associated values are available in SRAM. Additionally signalize
                  that all previous measurement and calculation data are no longer
                  available by result registers.
                ELSE

    \li 140:      Enable interrupts globally.

    \li 150:      Set component global error flag and the global debug error
                  variable with error code for operation still active error.
                ENDIF
              ELSE

    \li 160:    Set component global error flag and the global debug error
                variable with error code for invalid pointers are used error.
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiCalcChanCorr_C(uint8_t *pExtData, uint8_t *pIntData, uint8_t *pResult)  // registers in use: R17:R16, R19:R18, R21:R20 unused: R22, R23
{
    uint8_t bChanOffset;

    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiCalcChanCorr_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    if( pExtData && pIntData && pResult )
    {
        /* LLR-Ref: 020 */
        _CLI;

        /* LLR-Ref: 030 */
        if( !(RSCR & BM_RSOS) )
        {
            /* LLR-Ref: 040 */
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;
            _SEI;

            /* LLR-Ref: 050 */
            RSMS1R &= ~BM_RSSCAL;
            RSCR   &= ~(BM_RSMODE1 | BM_RSOFM | BM_RSEOR);
            RSCR   |= BM_RSMODE0;

            /* LLR-Ref: 060 */
            for( bChanOffset = 0; bChanOffset < 6; bChanOffset += 2 )
            {
                /* LLR-Ref: 070 */
                RSFR = BM_RSOFF;

                /* LLR-Ref: 080 */
                RSRES1L = *(pExtData + bChanOffset + LFRSSI_LOW_BYTE);
                RSRES1H = *(pExtData + bChanOffset + LFRSSI_HIGH_BYTE);
                RSRES2L = *(pIntData + bChanOffset + LFRSSI_LOW_BYTE);
                RSRES2H = *(pIntData + bChanOffset + LFRSSI_HIGH_BYTE);
                RSRES3L = g_sLfRssi.bChanCalibVal[bChanOffset + LFRSSI_LOW_BYTE];
                RSRES3H = g_sLfRssi.bChanCalibVal[bChanOffset + LFRSSI_HIGH_BYTE];

                /* LLR-Ref: 090 */
                _CLI;
                RSCR |= BM_RSOS;    // start correction calculation
                _SEI;

                /* LLR-Ref: 100 */
                while( !(RSFR & BM_RSOFF) ){}

                /* LLR-Ref: 110 */
                *(pResult + bChanOffset + LFRSSI_LOW_BYTE)  = RSRES4L;
                *(pResult + bChanOffset + LFRSSI_HIGH_BYTE) = RSRES4H;
            }

            /* LLR-Ref: 120 */
            RSFR = BM_RSOFF;

            /* LLR-Ref: 130 */
            g_sLfRssi.bStatus &= ~(LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_3DVEC_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_LINEAR_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG);
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_CHANCORR_DATA_AVAILABLE_FLAG;
        }
        else
        {
            /* LLR-Ref: 140 */
            _SEI;

            /* LLR-Ref: 150 */
            g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE;
        }
    }
    else
    {
        /* LLR-Ref: 160 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiCalc3dVec_C</b>
    is used to calculate the length of the logarithmic LF RSSI 3D vector.

    \param[in]      pData       Pointer to structure containing 3 x 16 bit channel correction values
    \param[out]     pResult     Pointer to 16 bit location to store the calculated length of the 3D vector

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data
    \li [out]    ::g_sDebug     Global Debug component data

    \image html ATA_lfRssiCalc3dVec_C.png

    \internal
    \li 010:  IF valid input and output pointers are used,
              THEN

    \li 020:    Disable interrupts globally to do an atomic flag check.

    \li 030:    IF no LF RSSI operation is in progress,
                THEN

    \li 040:      Enable interrupts globally.

    \li 050:      Disable usage of RSSI operation finished interrupt and set
                  "calculate vector" mode to register RSCR.

    \li 060:      Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 070:      Fill calculation registers with channel correction values.

    \li 080:      Signalize LF RSSI operation is in progress and trigger start
                  of length calculation by setting bit RSOS in register RSCR.

    \li 090:      Wait as long as the calculation has not finished.

    \li 100:      Read 3D vector lenght value and store it to SRAM location.

    \li 110:      Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 120:      Update component flag and status variables:\n
                  Signalize 3d vector length calculation has finished and the
                  associated value is available in SRAM. Additionally signalize
                  that all previous measurement and calculation data are no longer
                  available by result registers.
                ELSE

    \li 130:      Enable interrupts globally.

    \li 140:      Set component global error flag and the global debug error
                  variable with error code for operation still active error.
                ENDIF
              ELSE

    \li 150:    Set component global error flag and the global debug error
                variable with error code for invalid pointers are used error.
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiCalc3dVec_C(uint8_t *pData, uint8_t *pResult)    // registers in use: R17:R16, R19:R18 unused: R21, R20, R22, R23
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiCalc3dVec_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    if( pData && pResult )
    {
        /* LLR-Ref: 020 */
        _CLI;

        /* LLR-Ref: 030 */
        if( !(RSCR & BM_RSOS) )
        {
            /* LLR-Ref: 040 */
            _SEI;

            /* LLR-Ref: 050 */
            RSMS1R &= ~BM_RSSCAL;
            RSCR   &= ~(BM_RSMODE0 | BM_RSOFM | BM_RSEOR);
            RSCR   |= BM_RSMODE1;

            /* LLR-Ref: 060 */
            RSFR = BM_RSOFF;

            /* LLR-Ref: 070 */
            RSRES1L = *(pData + LFRSSI_BYTEOFFSET_RESULT_RSC1L);
            RSRES1H = *(pData + LFRSSI_BYTEOFFSET_RESULT_RSC1H);
            RSRES2L = *(pData + LFRSSI_BYTEOFFSET_RESULT_RSC2L);
            RSRES2H = *(pData + LFRSSI_BYTEOFFSET_RESULT_RSC2H);
            RSRES3L = *(pData + LFRSSI_BYTEOFFSET_RESULT_RSC3L);
            RSRES3H = *(pData + LFRSSI_BYTEOFFSET_RESULT_RSC3H);

            /* LLR-Ref: 080 */
            _CLI;
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;

            RSCR |= BM_RSOS;    // start 3D vector length calculation
            _SEI;

            /* LLR-Ref: 090 */
            while( !(RSFR & BM_RSOFF) ){}

            /* LLR-Ref: 100 */
            *(pResult + LFRSSI_LOW_BYTE)  = RSRES4L;
            *(pResult + LFRSSI_HIGH_BYTE) = RSRES4H;

            /* LLR-Ref: 110 */
            RSFR = BM_RSOFF;

            /* LLR-Ref: 120 */
            g_sLfRssi.bStatus &= ~(LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_CHANCORR_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_LINEAR_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG);
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_3DVEC_DATA_AVAILABLE_FLAG;
        }
        else
        {
            /* LLR-Ref: 130 */
            _SEI;

            /* LLR-Ref: 140 */
            g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE;
        }
    }
    else
    {
        /* LLR-Ref: 150 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED;
    }
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_lfRssiCalcLog2Lin_C</b>
    is used to calculate the linear encoded RSSI value from a logarithmic
    (3D vector) length value.

    \param[in]      p3dVec      Pointer to structure containing the 16 bit logarithmic length value
    \param[out]     pResult     Pointer to 16 bit location to store the calculated linear encoded RSSI value

    Variable Usage:
    \li [in,out] ::g_sLfRssi    Global LF RSSI component data
    \li [out]    ::g_sDebug     Global Debug component data

    \image html ATA_lfRssiCalcLog2Lin_C.png

    \internal
    \li 010:  IF valid input and output pointers are used,
              THEN

    \li 020:    Disable interrupts globally to do an atomic flag check.

    \li 030:    IF no LF RSSI operation is in progress,
                THEN

    \li 040:      Enable interrupts globally.

    \li 050:      Disable usage of RSSI operation finished interrupt and set
                  "calculate linear value" mode to register RSCR.

    \li 060:      Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 070:      Fill calculation registers with logarithmic length value.

    \li 080:      Signalize LF RSSI operation is in progress and trigger start
                  of logarithmic to linear calculation by setting bit RSOS in
                  register RSCR.

    \li 090:      Wait as long as the calculation has not finished.

    \li 100:      Read linear encoded RSSI value and store it to SRAM location.

    \li 110:      Clear LF RSSI operation finished flag (RSOFF) in register RSFR.

    \li 120:      Update component flag and status variables:\n
                  Signalize logarithmic to linear calculation has finished and the
                  associated value is available in SRAM. Additionally signalize
                  that all previous measurement and calculation data are no longer
                  available by result registers.
                ELSE

    \li 130:      Enable interrupts globally.

    \li 140:      Set component global error flag and the global debug error
                  variable with error code for operation still active error.
                ENDIF
              ELSE

    \li 150:    Set component global error flag and the global debug error
                variable with error code for invalid pointers are used error.
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_lfRssiCalcLog2Lin_C(uint8_t *p3dVec, uint8_t *pResult)     // registers in use: R17:R16, R19:R18 unused: R21, R20, R22, R23
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_lfRssiCalcLog2Lin_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    /* LLR-Ref: 010 */
    if( p3dVec && pResult )
    {
        /* LLR-Ref: 020 */
        _CLI;

        /* LLR-Ref: 030 */
        if( !(RSCR & BM_RSOS) )
        {
            /* LLR-Ref: 040 */
            _SEI;

            /* LLR-Ref: 050 */
            RSMS1R &= ~BM_RSSCAL;
            RSCR   &= ~(BM_RSOFM | BM_RSEOR);
            RSCR   |= (BM_RSMODE1 | BM_RSMODE0);

            /* LLR-Ref: 060 */
            RSFR |= BM_RSOFF;

            /* LLR-Ref: 070 */
            RSRES4L = *(p3dVec + LFRSSI_LOW_BYTE);
            RSRES4H = *(p3dVec + LFRSSI_HIGH_BYTE);

            /* LLR-Ref: 080 */
            _CLI;
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG;

            RSCR |= BM_RSOS;    // start logarithmic to linear calculation
            _SEI;

            /* LLR-Ref: 090 */
            while( !(RSFR & BM_RSOFF) ){}

            /* LLR-Ref: 100 */
            *(pResult + LFRSSI_LOW_BYTE)  = RSRES4L;
            *(pResult + LFRSSI_HIGH_BYTE) = RSRES4H;

            /* LLR-Ref: 110 */
            RSFR = BM_RSOFF;

            /* LLR-Ref: 120 */
            g_sLfRssi.bStatus &= ~(LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_CHANCORR_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_3DVEC_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG);
            g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_LINEAR_DATA_AVAILABLE_FLAG;
        }
        else
        {
            /* LLR-Ref: 130 */
            _SEI;

            /* LLR-Ref: 140 */
            g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE;
        }
    }
    else
    {
        /* LLR-Ref: 150 */
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiCalcBappl_C</b>
    calculates flux density for given RSSI value.

    \param[in]  wBref           Multiplier in 12q4 format
    \param[in]  wRssiValue      Mulitplicand in 12q4 format

    \return     Product in Atmel 12q4 floating point format

    \image html ATA_lfRssiCalcBappl_C.png

    \internal
    \li 010:  Calculate product of wBref and wRssiValue.

    \li 020:  Normalize and return result 12q4 format.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT16FUNC ATA_lfRssiCalcBappl_C(uint16_t wBref, uint16_t wRssiValue)
{
    uint16_t wMultiplicand  = wBref >> 4;
    uint16_t wMultiplicator = wRssiValue >> 4;

    /* LLR-Ref: 010 */
    uint32_t dwProduct = ATA_globalsMulU16U16_ASM(wMultiplicand, wMultiplicator);
    int8_t   bExponent = (2*(wBref & 0x000F) - 9) + (2*(wRssiValue & 0x000F) - 9);

    /* LLR-Ref: 020 */
    return ATA_lfRssiNormalize12q4_C(dwProduct, bExponent);
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiNormalize12q4_C</b>
    normalizes dwValue and bExponent to Atmels 12q4 format.

    \param[in]  dwValue     Value for Atmel 12q4 floating point format
    \param[in]  bExponent   Exponent for Atmel 12q4 floating point format

    \return     Normalized value in Atmel 12q4 floating point format

    \image html ATA_lfRssiNormalize12q4_C.png

    \internal
    \li 010:  Ensure odd exponent to avoid rounding error (/2) of exponent
              calculation.

    \li 020:  IF value to normalize is different than 0,
              THEN

    \li 030:    IF value is greater or equal to 12bit,
                THEN

    \li 040:      Shift values to the right and increment exponent until minimum
                  exponent value has reached or value is normalized.
                ELSE

    \li 050:      IF exponent is greater than minimum exponent value,
                  THEN

    \li 060:        Shift values to the left until minimum exponent value has not
                    reached and value is normalized.
                  ELSE

    \li 070:        IF the exponent value is less than minimum exponent value,
                    THEN

    \li 080:          Shift values to the right until exponent reaches -9.
                    ENDIF
                  ENDIF
                ENDIF

    \li 090:    Encode value and exponent in Atmel 12q4 format
              ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{N/A}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT16FUNC ATA_lfRssiNormalize12q4_C(uint32_t dwValue, int8_t bExponent)
{
    uint16_t wResult = 0;

    /* LLR-Ref: 010 */
    if( (bExponent & 0x01) == 0 )
    {
        dwValue  <<= 1;
        bExponent -= 1;
    }

    /* LLR-Ref: 020 */
    if( dwValue != 0 )
    {
        /* LLR-Ref: 030 */
        if( dwValue >= (1<<LF_RSSI_12Q4_VALUE_SIZE) )
        {
            /* LLR-Ref: 040 */
            while( dwValue >= (1<<LF_RSSI_12Q4_VALUE_SIZE) || bExponent < LF_RSSI_12Q4_EXPONENT_MIN )
            {
                dwValue >>= 2;
                bExponent += 2;
            }
        }
        else
        {
            /* LLR-Ref: 050 */
            if( bExponent > LF_RSSI_12Q4_EXPONENT_MIN )
            {
                /* LLR-Ref: 060 */
                while ( (dwValue < (1<<(LF_RSSI_12Q4_VALUE_SIZE-2))) && (bExponent > LF_RSSI_12Q4_EXPONENT_MIN+1) )
                {
                    dwValue  <<= 2;
                    bExponent -= 2;
                }
            }
            else
            {
                /* LLR-Ref: 070 */
                while( bExponent < LF_RSSI_12Q4_EXPONENT_MIN )
                {
                    /* LLR-Ref: 080 */
                    dwValue >>= 2;
                    bExponent += 2;
                }
            }
        }

        /* LLR-Ref: 090 */
        wResult = (dwValue) << LF_RSSI_12Q4_EXPONENT_SIZE | ((bExponent + 9)>>1);
    }
    return wResult;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRssiMeasReady_ISR_C</b>
    interrupt function is used to store the measured LF RSSI values from
    LF RSSI data registers to internal array and signals availability of data
    and signals also the end of current measurement. Furthermore it masks the
    participated measurement channels with its associated timeout flags. In
    case channel timeout(s) is(are) detected, the error flag inside the status
    variable is set. The associated channel timeout mask is stored to
    component variable.

    Variable Usage:
    \li [in,out] ::g_sLfRssi Global LF RSSI component data

    \image html ATA_lfRssiMeasReady_ISR_C.png

    \internal
    \li 010:  Disable occurrences of further LF RSSI operation finished interrupts
              by clearing associated interrupt mask bit.

    \li 020:  Create channel out of range mask. LF Channels which signals out of
              range are AND masked with its digital enable/disable bit settings.
              The resulting mask is stored to component variable.

    \li 030:  IF the measurement participating LF channel(s) signalizes out of range,
              THEN

    \li 040:    Set internal status flag indicating field strenght value outside
                the accessible range.
                Set component global error flag and the global debug error
                variable with error code for field strenght value outside the
                accessible range error.
              ENDIF

    \li 050:  Update component flag and status variables:\n
              Signalize LF RSSI measurement has finished and raw RSSI data are
              available in the result register. Additionally signalize
              that all previous calculation data are no longer available by the
              result registers.

    \Derived{No}

    \Rationale{N/A}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
/* #pragma vector=LFRSCO_vect */
//lint -esym(714, ATA_lfRssiMeasReady_ISR_C) FlSc (10.06.2014)
//lint -esym(765, ATA_lfRssiMeasReady_ISR_C) FlSc (10.06.2014)
/* disable lint rule 714 - symbol 'ATA_lfRssiMeasReady_ISR_C' not referenced
 * interrupt assignment to Interrupt Vector Table is done by Flash application
 *
 * disable lint rule 765 - external symbol 'ATA_lfRssiMeasReady_ISR_C' could be made static
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_lfRssiMeasReady_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_lfRssiMeasReady_ISR_C, g_sLfRssi.bFlags | g_sLfRssi.bStatus);

    uint8_t tmpReg, mask;

    /* LLR-Ref: 010 */
    RSCR &= ~(BM_RSOFM | BM_RSOS);

    /* LLR-Ref: 020 */
    mask    = RSMS1R & (BM_RSCH3E | BM_RSCH2E | BM_RSCH1E); // result range is 0x00 - 0x07
    tmpReg  = RSFR & mask;                                  // result range is 0x00 - 0x07
    tmpReg |= (RSFR & (mask<<5U));                          // result range is 0x00 - 0xE7

    g_sLfRssi.bOutOfRangeMask = tmpReg;                     // result is max. 0xE7 absolutely

    /* LLR-Ref: 030 */
    if( tmpReg )
    {
        /* LLR-Ref: 040 */
        g_sLfRssi.bStatus  |= LFRSSI_STATUS_BM_CHANNELS_OUTOFRANGE_FLAG;
        g_sLfRssi.bFlags   |= LFRSSI_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRSSI_CHANNELS_OUTOFRANGE;
    }

    /* LLR-Ref: 050 */
    g_sLfRssi.bStatus &= ~(LFRSSI_STATUS_BM_CHANCORR_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_3DVEC_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_LINEAR_DATA_AVAILABLE_FLAG | LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG);
    g_sLfRssi.bStatus |= LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG;
    g_sLfRssi.bFlags  |= LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG;
}





