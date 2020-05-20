/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/moduleTest/src/module_test_calib.c $
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
/** \file module_test.c
*/
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "module_test.h"
#include "../../calib/src/calib.h"
#include "../../system/src/system_flash.h"
#include "../../timer2/src/timer2.h"
#include "../../timer3/src/timer3.h"
#include "../../timer4/src/timer4.h"


/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_moduleTestCalibNoCal_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibFrcCalInt_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibSrcCalInt_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibFrcCalExt_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibSrcCalExt_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibTest_flash_C(uint8_t bConfig);

static VOIDFUNC ATA_moduleTestCalibFrcCalTimer3Locked_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibFrcCalTimer4Locked_flash_C(void);

static VOIDFUNC ATA_moduleTestCalibSrcCalTimer2Locked_flash_C(void);
static VOIDFUNC ATA_moduleTestCalibSrcCalTimer3Locked_flash_C(void);

static VOIDFUNC ATA_moduleTestCalibInitFail_flash_C(void);

/** \brief <b>m_fpModuleTestCalib</b>
    is a look up table for calib module test. The sub test are addressed via
    index variable g_sModuleTest_flash.bSubId
*/
static moduleTestFuncLut m_fpModuleTestCalib_flash[] = {
    ATA_moduleTestCalibNoCal_flash_C,
    ATA_moduleTestCalibFrcCalInt_flash_C,
    ATA_moduleTestCalibSrcCalInt_flash_C,
    ATA_moduleTestCalibFrcCalExt_flash_C,
    ATA_moduleTestCalibSrcCalExt_flash_C,
    ATA_moduleTestCalibFrcCalTimer3Locked_flash_C,
    ATA_moduleTestCalibFrcCalTimer4Locked_flash_C,
    ATA_moduleTestCalibSrcCalTimer2Locked_flash_C,
    ATA_moduleTestCalibSrcCalTimer3Locked_flash_C,
    ATA_moduleTestCalibInitFail_flash_C, // SRC EEPROM fail
    ATA_moduleTestCalibInitFail_flash_C  // FRC EEPROM fail
};

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalib_flash_C</b>
    module test for SRC and FRC calibration. calibration configuration is
    defined via variable moduleTest.subId as follows
    moduleTest.subId=0: no calibration
    moduleTest.subId=1: FRC calibration with internal XTO/4
    moduleTest.subId=2: SRC calibration with internal XTO/4
    moduleTest.subId=3: FRC calibration with external XTO/8
    moduleTest.subId=4: SRC calibration with internal XTO/8
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalib_flash_C(void)
{
    moduleTestFunc fpFunc = m_fpModuleTestCalib_flash[g_sModuleTest_flash.bSubId];
    if (fpFunc != 0x0000U) {
        fpFunc();
    } else {}
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalibNoCal_flash_C</b>
    starts module calib with parameter bConfig -> no calibration
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibNoCal_flash_C(void)
{
    ATA_moduleTestCalibTest_flash_C(0x00U);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalibFrcCalInt_flash_C</b>
    starts module calib with parameter bConfig -> FRC calibration with internal XTO/4
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibFrcCalInt_flash_C(void)
{
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalibFrcCalInt_flash_C</b>
    starts module calib with parameter bConfig -> SRC calibration with internal XTO/4
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibSrcCalInt_flash_C(void)
{
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalibFrcCalInt_flash_C</b>
    starts module calib with parameter bConfig -> FRC calibration with external XTO/8
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibFrcCalExt_flash_C(void)
{
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC | BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalibFrcCalInt_flash_C</b>
    starts module calib with parameter bConfig -> SRC calibration with internal XTO/8
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibSrcCalExt_flash_C(void)
{
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC | BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestCalibTest_flash_C</b>
    common module test routine for SRC and FRC calibration. Calibration configuration is
    defined via variable bConfig
    010:    initialize calib module via function ATA_calibInit_C
    020:    start calibration with selected configuration via function ATA_calibStartCalibration_C
    030:    close calib module
    040:    disable module test by resetting moduleTest.id and moduleTest.subId respectively
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibTest_flash_C(uint8_t bConfig)
{
    /* LLR-Ref: 010 */
    ATA_calibInit_C();
    /* LLR-Ref: 020 */
    ATA_calibStartCalibration_C(bConfig);

    /* LLR-Ref: 030 */
    ATA_calibClose_C();

    /* LLR-Ref: 040 */
    g_sModuleTest_flash.bId = 0U;
    g_sModuleTest_flash.bSubId = 0U;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibFrcCalTimer3Locked_flash_C(void)
{
    sTimerAsyn16BitParams sTimer3Params = {
        0x00U,                      // T3CR
        0x00U,                      // T3MRA
        0x00U,                      // T3MRB
        0x00U,                      // T3CORL
        0x00U,                      // T3CORH
        0x00U,                      // T3IMR
        (timerIRQHandler)0x0000,    // g_sTimer3.ovfIsr
        (timerIRQHandler)0x0000,    // g_sTimer3.compIsr
        (timerIRQHandler)0x0000     // g_sTimer3.capIsr
        };
        
    ATA_timer3Open_C(&sTimer3Params);
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC);

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       ){
        ATA_globalsInitDebug_C();
    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    ATA_timer3Close_C(); 
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibFrcCalTimer4Locked_flash_C(void)
{
    sTimerAsyn16BitParams sTimer4Params = {
        0x00U,                      // T4CR
        0x00U,                      // T4MRA
        0x00U,                      // T4MRB
        0x00U,                      // T4CORL
        0x00U,                      // T4CORH
        0x00U,                      // T4IMR
        (timerIRQHandler)0x0000,    // g_sTimer4.fpOvfIsr
        (timerIRQHandler)0x0000,    // g_sTimer4.fpCompIsr
        (timerIRQHandler)0x0000     // g_sTimer4.fpCapIsr
        };
    ATA_timer4Open_C(&sTimer4Params);
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC);

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       ){
        ATA_globalsInitDebug_C();
    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    ATA_timer4Close_C();
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibSrcCalTimer2Locked_flash_C(void)
{
    sTimerAsyn8BitParams  sTimer2Params = {
        0x00U,      // T2CR
        0x00U,      // T2MR
        0x00U,      // T2COR
        0x00U,      // T2IMR
        (timerIRQHandler)0x0000,    // g_sTimer2.ovfIsr
        (timerIRQHandler)0x0000     // g_sTimer2.compIsr
        };
        
    ATA_timer2Open_C(&sTimer2Params);
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC);

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       ){
        ATA_globalsInitDebug_C();
    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    ATA_timer2Close_C();    
    
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibSrcCalTimer3Locked_flash_C(void)
{
    sTimerAsyn16BitParams sTimer3Params = {
        0x00U,                      // T3CR
        0x00U,                      // T3MRA
        0x00U,                      // T3MRB
        0x00U,                      // T3CORL
        0x00U,                      // T3CORH
        0x00U,                      // T3IMR
        (timerIRQHandler)0x0000,    // g_sTimer3.ovfIsr
        (timerIRQHandler)0x0000,    // g_sTimer3.compIsr
        (timerIRQHandler)0x0000     // g_sTimer3.capIsr
        };
        
    ATA_timer3Open_C(&sTimer3Params);
    ATA_moduleTestCalibTest_flash_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC);

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       ){
        ATA_globalsInitDebug_C();
    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    ATA_timer3Close_C(); 
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestCalibInitFail_flash_C(void)
{
    ATA_calibInit_C();

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       )
    {
        ATA_globalsInitDebug_C();
    }
    else
    {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
}

