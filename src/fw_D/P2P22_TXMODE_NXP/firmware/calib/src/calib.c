//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/calib/src/calib.c $
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
/** \file calib.c
*/
//lint -restore
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "calib.h"

#include "..\..\globals\src\globals.h"
#include "..\..\eep\src\eep.h"

#include "..\..\timer2\src\timer2.h"
#include "..\..\timer3\src\timer3.h"
#include "..\..\timer4\src\timer4.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>g_sCalibConfig</b>
    contains the configuration and status information for module CALIB.
*/
#pragma location = ".calib"
__root __no_init sCalibConfig g_sCalibConfig;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

static VOIDFUNC ATA_calibSrcMeasurement_C(void);
static VOIDFUNC ATA_calibSetSrcCalRegister_C(void);

static VOIDFUNC ATA_calibFrcMeasurement_C(void);
static VOIDFUNC ATA_calibSetFrcCalRegister_C(void);


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibInit_C</b>
    initializes the CALIB module. This function should be executed before
    starting CALIB.

    Variable Usage:
    \li [in,out] ::g_sCalibConfig Global Calibration component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_calibInit_C.png

    \internal
    \li 005: Set HW trace point for this function

    \li 010: Initialize module global ::g_sCalibConfig

    \li 017: Get address for SRC calibration configuration located in the
             User EEPROM section by evaluating the indirect address defined in the
             Customer EEPROM section.

    \li 018: IF after the EEPROM read access to retrieve the requested data, an
              uncorrectable EEPROM error was detected, indicated by
              the return value of function "ATA_eepReadBytes_C",
             THEN
               Set an EEPROM read error indication in ::g_sCalibConfig .bFlags and
               set ::g_sDebug .bErrorCode to the Calib module EEPROM read error
             ENDIF

    \li 020: Initialize SRC variables with EEPROM content from g_sEepCalibSrcCalibrationConfig

    \li 025: IF after the EEPROM read access to retrieve the requested data, an
              uncorrectable EEPROM error was detected, indicated by
              the return value of function "ATA_eepReadBytes_C",
             THEN
               Set an EEPROM read error indication in ::g_sCalibConfig .bFlags and
               set ::g_sDebug .bErrorCode to the Calib module EEPROM read error
             ENDIF

    \li 030: Initialize FRC variables with EEPROM content from g_sEepCalibFrcCalibrationConfig
             and get address for FRC calibration configuration located in the
             User EEPROM section

    \li 040: IF after the EEPROM read access to retrieve the requested data, an
              uncorrectable EEPROM error was detected, indicated by
              the return value of function "ATA_eepReadBytes_C",
             THEN
               Set an EEPROM read error indication in ::g_sCalibConfig .bFlags and
               set ::g_sDebug .bErrorCode to the Calib module EEPROM read error
             ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-875,Primus2P-1979,Primus2P-1980,Primus2P-1981,\
                  Primus2P-2444}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibInit_C(void)
{
    eEepErrorCode sEepErrCode;

    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibInit_C, 0x00U);

    /* LLR-Ref: 010 */
    g_sCalibConfig.bFlags     = 0x00U;
    g_sCalibConfig.bStatus    = 0x00U;
    g_sCalibConfig.bConfig    = 0x00U;

    /* LLR-Ref: 017 */
    uint8_t ary[2];
    sEepErrCode = ATA_eepReadBytes_C(&ary[0],(uint16_t)&g_sCustomerEEPromSection.eepCalibConfPtr_l,2U);
    uint16_t wEepCalibAddr = (uint16_t)((ary[1]<<SHIFT_LOW_TO_HIGH_BYTE|ary[0]));

    /* LLR-Ref: 018 */
    if(sEepErrCode != EEC_NO_ERROR)
    {
        ATA_SETBITMASK_C(g_sCalibConfig.bFlags,BM_CALIB_CONFIG_FLAGS_ERROR)
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR;
    }

    /* LLR-Ref: 020 */
    g_sCalibConfig.wSrcResult = 0x0000U;
    uint8_t *pDestData = (uint8_t*)(&g_sCalibConfig.sSrcCalibrationConfig);

    sEepErrCode = ATA_eepReadBytes_C(pDestData, wEepCalibAddr, sizeof(g_sCalibConfig.sSrcCalibrationConfig));

    /* LLR-Ref: 025 */
    if(sEepErrCode != EEC_NO_ERROR)
    {
        ATA_SETBITMASK_C(g_sCalibConfig.bFlags,BM_CALIB_CONFIG_FLAGS_ERROR)
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR;
    }

    /* LLR-Ref: 030 */
    g_sCalibConfig.wFrcResult = 0x0000U;
    wEepCalibAddr += (uint16_t)sizeof(g_sCalibConfig.sSrcCalibrationConfig);
    pDestData = (uint8_t *)(&g_sCalibConfig.sFrcCalibrationConfig);

    sEepErrCode = ATA_eepReadBytes_C(pDestData, wEepCalibAddr, sizeof(g_sCalibConfig.sFrcCalibrationConfig));

    /* LLR-Ref: 040 */
    if(sEepErrCode != EEC_NO_ERROR)
    {
        ATA_SETBITMASK_C(g_sCalibConfig.bFlags,BM_CALIB_CONFIG_FLAGS_ERROR)
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibClose_C</b>
    closes the CALIB module by disabling CALIB specific parts

    \image html ATA_calibClose_C.png

    \internal
    \li 005: Set HW trace point for this function

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-875,Primus2P-1979,Primus2P-1980,Primus2P-1981}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibClose_C(void)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibClose_C, 0x00U);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibStartCalibration_C</b>
    starts a SRC/FRC calibration as configured in parameter "bConfig".

    \param[in]      bConfig             Calibration configuration value
    
    Variable Usage:
    \li [in,out] ::g_sCalibConfig Global Calibration component data

    \image html ATA_calibStartCalibration_C.png

    \internal
    \li 005: Set HW trace point for this function
    \li 010: Copy function argument "bConfig" to ::g_sCalibConfig .bConfig
    \li 020: Activate XTO by callign function ::ATA_globalsActivateXTO_C if SRC/FRC
          calibration with XTO_CLK is selected in ::g_sCalibConfig .bConfig[0].
          Activation can be skipped, if XTO is already available (FESR.XRDY=1)
    \li 030: Start SRC calibration by calling function ::ATA_calibStartSrcCalibration_C,
          if configured in ::g_sCalibConfig .bConfig[6]
    \li 040: Start FRC calibration by calling function ::ATA_calibStartFrcCalibration_C,
          if configured in ::g_sCalibConfig .bConfig[7]
    \li 050: Perform function [SHUTDOWN_RF_FRONTEND] to deactivate the XTO if
          configured in ::g_sCalibConfig .bConfig[1]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-875,Primus2P-1979,Primus2P-1980,Primus2P-1981}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibStartCalibration_C(uint8_t bConfig)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibStartCalibration_C, bConfig);

    /* LLR-Ref: 010 */
    g_sCalibConfig.bConfig = bConfig;
    g_sCalibConfig.bStatus = 0x00U;

    /* LLR-Ref: 020 */
    if ((g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK) == 0) {
        ATA_globalsActivateXTO_C();
    }

    /* LLR-Ref: 030 */
    if (g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC) {
        ATA_calibStartSrcCalibration_C();
    }

    /* LLR-Ref: 040 */
    if (g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC) {
        ATA_calibStartFrcCalibration_C();
    }

    /* LLR-Ref: 050 */
    if ((g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK) == 0) {
        if ((g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_XTO) == 0) {
            ATA_CLEARBIT_C(FEEN1, XTOEN);
            ATA_CLEARBIT_C(SUPCR, AVEN);
        }
    }

}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibStartSrcCalibration_C</b>
    starts a SRC calibration

    Variable Usage:
    \li [in,out] ::g_sCalibConfig Global Calibration component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_calibStartSrcCalibration_C.png

    \internal
    \li 005: Set HW trace point for this function
    \li 010: Load default configuration for Timer2
               input clock of Timer2:      CLKSRC
               prescaler value of Timer2:  2^0 = 1
    \li 020: Load default configuration for Timer3
               input clock of Timer3:      CLKXTO4
               prescaler value of Timer3:  2^0 = 1
    \li 030: IF calibration shall be done with external clock input, clock of
              Timer3 shall be changed to CLKTEI
    \li 040: Open Timer2 by calling function ::ATA_timer2Open_C and Timer3 by
              calling function ::ATA_timer3Open_C for SRC calibration with
              configuration of timer2Params and timer3params respectively.\n
                IF Timer2 and Timer3 are available for SRC calibration
    \li 050:      Turn on power for LF Protocol Handler to enable SRCCAL setting
    \li 060:      Measure the SRC oscillator using function
                   ::ATA_calibSrcMeasurement_C
                  Apply the result to SRCCAl register using function
                   ::ATA_calibSetSrcCalRegister_C
    \li 070:      Turn off power from LF-Reveiver via setting PRR1.PRLFR
    \li 080:      Release Timer2 and Timer3 by calling function ::ATA_timer2Close_C
                   and ::ATA_timer3Close_C
                ELSE (Timer2 or Timer3 are not available for SRC calibration)
    \li 090:      Set error in ::g_sCalibConfig .bFlags[7] and
                  set error code in ::g_sDebug .bErrorCode and ::g_sDebug .bSsmErrorCode
    \li 095: Set HW trace point for this function again in order to be able to
              perform timing measurements for SRC calibration

    Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1980,Primus2P-1981}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibStartSrcCalibration_C(void)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibStartSrcCalibration_C, 0x00U);

    /* LLR-Ref: 010 */
    sTimerAsyn8BitParams  sTimer2Params = {
        (BM_T2ENA | BM_T2CTM),      // T2CR
        0x00U,                      // T2MR
        0x01U,                      // T2COR
        0x00U,                      // T2IMR
        (timerIRQHandler)0x0000,    // g_sTimer2.ovfIsr
        (timerIRQHandler)0x0000     // g_sTimer2.compIsr
        };

    /* LLR-Ref: 020 */
    sTimerAsyn16BitParams sTimer3Params = {
        (BM_T3ENA | BM_T3CPRM),     // T3CR
        BM_T3CS1,                   // T3MRA
        (BM_T3CE0 | BM_T3CNC),      // T3MRB
        0x00U,                      // T3CORL
        0x00U,                      // T3CORH
        0x00U,                      // T3IMR
        (timerIRQHandler)0x0000,    // g_sTimer3.ovfIsr
        (timerIRQHandler)0x0000,    // g_sTimer3.compIsr
        (timerIRQHandler)0x0000     // g_sTimer3.capIsr
        };

    /* LLR-Ref: 030 */
    if (g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK) {
        sTimer3Params.modeA = (BM_T3CS1 | BM_T3CS0);
    }
    /* LLR-Ref: 040 */
    if (   (ATA_timer2Open_C(&sTimer2Params) == OK)
        && (ATA_timer3Open_C(&sTimer3Params) == OK)
       ){
        /* LLR-Ref: 050 */
        uint8_t bPrr1Config = PRR1;
        ATA_POWERON_C(PRR1, PRLFPH);

        /* LLR-Ref: 060 */
        ATA_calibSrcMeasurement_C();
        ATA_calibSetSrcCalRegister_C();

        /* LLR-Ref: 070 */
        PRR1 = bPrr1Config;

        /* LLR-Ref: 080 */
        ATA_timer2Close_C();
        ATA_timer3Close_C();
    }else {
        /* LLR-Ref: 090 */
        ATA_SETBITMASK_C(g_sCalibConfig.bFlags,BM_CALIB_CONFIG_FLAGS_ERROR)
        g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED;
        g_sDebug.bSsmErrorCode = 0x00U;
    }

    /* LLR-Ref: 095 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibStartSrcCalibration_C+1, 0x00U);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibSrcMeasurement_C</b>
    measures the number of XTO clock cycles in on SRC clock period and stores
    the result in g_sCalibConfig.srcResult

    Variable Usage:
    \li[out] ::g_sCalibConfig    Global Calibration component data

    \image html ATA_calibSrcMeasurement_C.png

    \internal
    \li 010: Reset and Restart Timer2
             The T2RES bit can be written to logic one to reset the prescaler and counter.
             This is only allowed if the timer is stopped (T2ENA=0).
             The T2RES bit is automatically cleared one cycle after the write.
    \li 020: Wait for 1st capture event by polling T2IFR.T2COF flag
             This step is needed to reset T3CNTL/H register for the SRC Measurement
             Resetting is done in Timer3 IP via T3CR.T3CPRM setting in ::ATA_timer3Open_C
    \li 030: Reset Timer2 Compare Flag for SRC Measurement
    \li 040: Set next capture event after 4 SRC cycles to use Timer3 input Capture Noise Canceller.
             the noise canceller is activated, the input from the input capture pin is filtered.
             The filter function requires four successive equal valued samples of the input
             capture pin to change its output. The input capture is therefore delayed by four
             counter clock (CL3) cycles when the noise canceller is enabled.
    \li 050: Wait for next capture event
    \li 055: Due to slow SRC clock compared to AVR system clock, extra cycles
             are required in order for registers T3ICRH/T3ICRL to be updated
             correctly.
    \li 060: Store Timer3 input capture value to ::g_sCalibConfig .wSrcResult

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1980,Primus2P-1981}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibSrcMeasurement_C(void)
{
    /* LLR-Ref: 010 */
    ATA_CLEARBITMASK_C(T2CR,BM_T2ENA)
    while(T2CR & BM_T2ENA);

    ATA_SETBITMASK_C(T2CR,BM_T2RES)
    ATA_SETBITMASK_C(T2CR,BM_T2ENA)

    /* LLR-Ref: 020 */
    while(!(T2IFR & BM_T2COF));

    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(T2IFR,BM_T2COF)

    /* LLR-Ref: 040 */
    T2COR = 0x05U;

    /* LLR-Ref: 050 */
    while(!(T2IFR & BM_T2COF));

    /* LLR-Ref: 055 */
    ATA_globalsWaitNus_ASM(1U);

    /* LLR-Ref: 060 */
    g_sCalibConfig.wSrcResult  = (T3ICRH << SHIFT_LOW_TO_HIGH_BYTE);
    g_sCalibConfig.wSrcResult |=  T3ICRL;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibSetSrcCalRegister_C</b>
    applies the SRC measurement result ::g_sCalibConfig .srcResult to HW
    register SRCCAL.

    Variable Usage:
    \li [in]  ::g_sCalibConfig  Global Calibration component data

    \image html ATA_calibSetSrcCalRegister_C.png

    \internal
    \li 010: Calculate difference between measured xto clocks
              ::g_sCalibConfig .wSrcResult and reference value ::g_sCalibConfig
              .srcCalibrationConfig.xtoCyclesPerMeasurement
    \li 020: Apply ::g_sCalibConfig .srcCalibrationConfig.gradient to the difference
              and add current SRCCAL setting
    \li 030: Limit result to 127 to ensure a linear SRC calibration step
    \li 040: Apply result to SRCCAL

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1980,Primus2P-1981}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibSetSrcCalRegister_C(void)
{
    /* LLR-Ref: 010 */
    int8_t  bDiffCycles = g_sCalibConfig.wSrcResult - g_sCalibConfig.sSrcCalibrationConfig.wXtoCyclesPerMeasurement;
    /* LLR-Ref: 020 */
    int8_t  bSrcOffset  = (int8_t)(ATA_globalsMulS8U8_ASM(bDiffCycles, g_sCalibConfig.sSrcCalibrationConfig.bGradient) >> 6U);

    uint8_t bSrcValue = SRCCAL + bSrcOffset;
    /* LLR-Ref: 030 */
    bSrcValue  = (bSrcValue > 0x7F) ? 0x7F:bSrcValue;

    /* LLR-Ref: 040 */
    SRCCAL = bSrcValue;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibStartFrcCalibration_C</b>
    starts FRC calibration

    Variable Usage:
    \li [in,out] ::g_sCalibConfig Global Calibration component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_calibStartFrcCalibration_C.png

    \internal
    \li 005: Set HW trace point for this function
    \li 010: Load default configuration for Timer3
               input clock of Timer3:      CLKXTO
               prescaler value of Timer3:  2^0 = 1
    \li 020: Load default confiugration for Timer4
               input clock of Timer4:      CLKFRC
               prescaler value of Timer4:  2^0 = 1
    \li 030: IF calibration shall be done with external clock input, clock of
              Timer3 shall be changed to CLKTEI
    \li 040: Open Timer3 by calling function ::ATA_timer3Open_C and Timer4 by
              calling function ::ATA_timer3Open_C for FRC calibration with
              configuration of timer3Params and timer4params respectively.
             IF Timer3 and Timer4 are available for FRC calibration
    \li 050:   Measure the FRC oscillator by calling function ::ATA_calibFrcMeasurement_C
               Apply the result to FRCCAl register by calling function
               ::ATA_calibSetFrcCalRegister_C
    \li 060:   Release Timer3 and Timer4 by calling function ::ATA_timer3Close_C
                and ::ATA_timer4Close_C
             ELSE (Timer3 or Timer4 is not available for FRC calibration)
    \li 070:   Set error in ::g_sCalibConfig .bFlags[7] and
                set error code in variable ::g_sDebug .bErrorCode and ::g_sDebug
                .bSsmErrorCode
    \li 095: Set HW trace point for this function again in order to be able to
              perform timing measurements for FRC calibration

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-875,Primus2P-1979}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibStartFrcCalibration_C(void)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibStartFrcCalibration_C, 0x00U);

    /* LLR-Ref: 010 */
    sTimerAsyn16BitParams sTimer3Params = {
        (BM_T3ENA|BM_T3CTM|BM_T3CRM),   // T3CR
        (BM_T3CS1),                     // T3MRA
        0x00U,                          // T3MRB
        0x80U,                          // T3CORL
        0x00U,                          // T3CORH
        0x00U,                          // T3IMR
        (timerIRQHandler)0x0000,        // g_sTimer3.fpOvfIsr
        (timerIRQHandler)0x0000,        // g_sTimer3.fpCompIsr
        (timerIRQHandler)0x0000         // g_sTimer3.fpCapIsr
        };

    /* LLR-Ref: 020 */
    sTimerAsyn16BitParams sTimer4Params = {
        (BM_T4ENA | BM_T4CPRM),     // T4CR
        (BM_T4CS0 | BM_T4CS1),      // T4MRA
        (BM_T4ICS1| BM_T4CE0),      // T4MRB
        0x00U,                      // T4CORL
        0x00U,                      // T4CORH
        0x00U,                      // T4IMR
        (timerIRQHandler)0x0000,    // g_sTimer4.fpOvfIsr
        (timerIRQHandler)0x0000,    // g_sTimer4.fpCompIsr
        (timerIRQHandler)0x0000     // g_sTimer4.fpCapIsr
        };

    /* LLR-Ref: 030 */
    if (g_sCalibConfig.bConfig & BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK) {
        sTimer3Params.modeA = (BM_T3CS1 | BM_T3CS0);
    }

    /* LLR-Ref: 040 */
     if (   (ATA_timer3Open_C(&sTimer3Params) == OK)
         && (ATA_timer4Open_C(&sTimer4Params) == OK)
        ){
        /* LLR-Ref: 050 */
        ATA_calibFrcMeasurement_C();
        ATA_calibSetFrcCalRegister_C();

        ATA_calibFrcMeasurement_C();
        ATA_calibSetFrcCalRegister_C();

        /* LLR-Ref: 060 */
        ATA_timer3Close_C();
        ATA_timer4Close_C();

    }else {
        /* LLR-Ref: 070 */
        ATA_SETBITMASK_C(g_sCalibConfig.bFlags,BM_CALIB_CONFIG_FLAGS_ERROR)
        g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED;
        g_sDebug.bSsmErrorCode = 0x00U;
    }

    /* LLR-Ref: 075 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_calibStartFrcCalibration_C+1, 0x00U);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibFrcMeasurement_C</b>
    measures the number of XTO/4 clock cycles in during a period of 200 FRC cycles
    (31.45µs @6.36MHz). The number of XTO/4 cycles for the measurement period
    is stored in eepFrcCalibration.xtoCyclesPerMeasurement for reference.

    Variable Usage:
    \li [in,out]  ::g_sCalibConfig  Global Calibration component data 

    \image html ATA_calibFrcMeasurement_C.png

    \internal
    \li 010: Reset and Restart Timer3
             The T3RES bit can be written to logic one to reset the prescaler and counter.
             This is only allowed if the timer is stopped (T3ENA=0).
             The T3RES bit is automatically cleared one cycle after the write.
    \li 020: Wait for 1st capture event by polling T3IFR.T3COF flag
             This step is needed to reset T3CNTL/H register for the FRC Measurement
             Resetting is done in Timer4 IP via T4CR.T4CPRM setting in
             ::ATA_timer4Open_C
    \li 030: Reset Timer3 Compare Flag for FRC Measurement
    \li 040: Set next capture event to ::g_sCalibConfig
              .sFrcCalibrationConfig.bT3CORL/H
    \li 050: Wait for next capture event
    \li 060: Store Timer4 input capture value to ::g_sCalibConfig .wFrcResult

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-875,Primus2P-1979}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibFrcMeasurement_C(void)
{
    /* LLR-Ref: 010 */
    ATA_CLEARBITMASK_C(T3CR,BM_T3ENA)

    while(T3CR & BM_T3ENA);
    ATA_SETBITMASK_C(T3CR,BM_T3RES)
    ATA_SETBITMASK_C(T3CR,BM_T3ENA)

    /* LLR-Ref: 020 */
    while(!(T3IFR & BM_T3COF));

    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(T3IFR,BM_T3COF)

    /* LLR-Ref: 040 */
    T3CORL = g_sCalibConfig.sFrcCalibrationConfig.bT3CORL;
    T3CORH = g_sCalibConfig.sFrcCalibrationConfig.bT3CORH;

    /* LLR-Ref: 050 */
    while(!(T3IFR & BM_T3COF));

    /* LLR-Ref: 060 */
    g_sCalibConfig.wFrcResult  = (T4ICRH << SHIFT_LOW_TO_HIGH_BYTE);
    g_sCalibConfig.wFrcResult |=  T4ICRL;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_calibSetFrcCalRegister_C</b>
    applies the FRC measurement result g_sCalibConfig.frcResult to HW register FRCCAL

    Variable Usage:
    \li [in] ::g_sCalibConfig Global Calibration component data

    \image html ATA_calibSetFrcCalRegister_C.png

    \internal
    \li 010: Only low byte of ::g_sCalibConfig .frcResult is used for calibration.
             High byte can be ignored as range is 200 +/- 10% --> 180 - 220
    \li 020: Add 3 for negative values to avoid rounding errors
    \li 030: Divide result by for and limit result to a range of 45 ... 55
    \li 040: Apply result to FRCCAL

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-875,Primus2P-1979}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_calibSetFrcCalRegister_C(void)
{
    /* LLR-Ref: 010 */
    uint8_t bFrcRes = (uint8_t)g_sCalibConfig.wFrcResult;

    /* LLR-Ref: 020 */
    if (bFrcRes < 200U) {
        bFrcRes += 3U;
    }

    /* LLR-Ref: 030 */
    bFrcRes >>= 2U;
    bFrcRes = (bFrcRes < 45U) ? 45U:bFrcRes;
    bFrcRes = (bFrcRes > 55U) ? 55U:bFrcRes;

    /* LLR-Ref: 040 */
    int8_t bResult = 50 - bFrcRes;
    bResult += FRCCAL & (BM_FRCCAL4|BM_FRCCAL3|BM_FRCCAL2|BM_FRCCAL1|BM_FRCCAL0);
    bResult  = (bResult < 0x00) ? 0x00:bResult;
    bResult  = (bResult > 0x1F) ? 0x1F:bResult;
    FRCCAL  = bResult;
}
