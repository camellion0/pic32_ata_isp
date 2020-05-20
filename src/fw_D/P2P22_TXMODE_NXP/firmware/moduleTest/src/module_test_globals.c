/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/moduleTest/src/module_test_globals.c $
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
/** \file module_test_rftx.c
*/
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "module_test.h"
#include "../../system/src/system_flash.h"

static VOIDFUNC ATA_moduleTestGlobalsDisableVoltageMonitor_flash_C(void);
static VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorFrc_flash_C(void);
static VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorXto_flash_C(void);
static VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorMrc_flash_C(void);
static VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorSrc_flash_C(void);
static VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorWoInt_flash_C(void);
static VOIDFUNC ATA_moduleTestGlobalsVoltageMonitor_flash_C(uint8_t bVmcr);
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/** \brief <b>m_fpModuleTestGlobals_flash</b>
    is a look up table for Globals module test. The sub test are addressed via
    index variable g_sModuleTest_flash.bSubId
    0x00:   tbd
    0x01:   ATA_moduleTestGlobalsDisableVoltageMonitor_flash_C
    0x02:   ATA_moduleTestGlobalsEnableVoltageMonitorFrc_flash_C
    0x03:   ATA_moduleTestGlobalsEnableVoltageMonitorXto_flash_C
    0x04:   ATA_moduleTestGlobalsEnableVoltageMonitorMrc_flash_C
    0x05:   ATA_moduleTestGlobalsEnableVoltageMonitorSrc_flash_C
    0x06:   ATA_moduleTestGlobalsEnableVoltageMonitorWoInt_flash_C
*/
static moduleTestFuncLut m_fpModuleTestGlobals_flash[] = {
    0x0000U,
    ATA_moduleTestGlobalsDisableVoltageMonitor_flash_C,     // 0x01
    ATA_moduleTestGlobalsEnableVoltageMonitorFrc_flash_C,   // 0x02
    ATA_moduleTestGlobalsEnableVoltageMonitorXto_flash_C,   // 0x03
    ATA_moduleTestGlobalsEnableVoltageMonitorMrc_flash_C,   // 0x04
    ATA_moduleTestGlobalsEnableVoltageMonitorSrc_flash_C,   // 0x05
    ATA_moduleTestGlobalsEnableVoltageMonitorWoInt_flash_C, // 0x06
};

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobals_flash_C</b>
    is the entry point for globals module test. 
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobals_flash_C(void)
{
    moduleTestFunc fpFunc = m_fpModuleTestGlobals_flash[g_sModuleTest_flash.bSubId];
    if (fpFunc != 0x0000U) {
        fpFunc();
    } else {}
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsDisableVoltageMonitor_flash_C</b>
    checks disabling of the Voltage Monitor using ATA_globalsSetVoltageMonitor_C.
    If Voltage Monitor is powered an error is signalled.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsDisableVoltageMonitor_flash_C(void)
{
    ATA_globalsSetVoltageMonitor_C(0x00);
    if ((PRR0 & BM_PRVM) == 0){
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsEnableVoltageMonitorFrc_flash_C</b>
    checks enabling of the Voltage Monitor using ATA_globalsSetVoltageMonitor_C
    with avr core clock FRC.
    If Voltage Monitor is not powered and VMCR register setting is incorrect,
    an error is signalled.
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorFrc_flash_C(void)
{
    ATA_globalsClkSwitchFrc_C();
    ATA_moduleTestGlobalsVoltageMonitor_flash_C(0x15);
    ATA_globalsSetVoltageMonitor_C(0x00);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsEnableVoltageMonitorXto_flash_C</b>
    checks enabling of the Voltage Monitor using ATA_globalsSetVoltageMonitor_C
    with avr core clock XTO/4.
    If Voltage Monitor is not powered and VMCR register setting is incorrect,
    an error is signalled.    
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorXto_flash_C(void)
{
    ATA_globalsClockSwitchXtoWithMvccEnable_C(0x07);
    ATA_moduleTestGlobalsVoltageMonitor_flash_C(0x15);
    ATA_globalsSetVoltageMonitor_C(0x00);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsEnableVoltageMonitorMrc_flash_C</b>
    checks enabling of the Voltage Monitor using ATA_globalsSetVoltageMonitor_C
    with avr core clock MRC.
    If Voltage Monitor is not powered and VMCR register setting is incorrect,
    an error is signalled.
    
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorMrc_flash_C(void)
{
    ATA_globalsClkSwitchMrc_C();
    ATA_moduleTestGlobalsVoltageMonitor_flash_C(0x15);
    ATA_globalsSetVoltageMonitor_C(0x00);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsEnableVoltageMonitorSrc_flash_C</b>
    checks enabling of the Voltage Monitor using ATA_globalsSetVoltageMonitor_C
    with avr core clock SRC.
    If Voltage Monitor is not powered and VMCR register setting is incorrect,
    an error is signalled.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorSrc_flash_C(void)
{
    ATA_globalsClkSwitchSrc_C();
    ATA_moduleTestGlobalsVoltageMonitor_flash_C(0x15);
    ATA_globalsSetVoltageMonitor_C(0x00);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsEnableVoltageMonitorWoInt_flash_C</b>
    checks enabling of the Voltage Monitor using ATA_globalsSetVoltageMonitor_C
    with avr core clock FRC and voltage monitor interrupt disabled.
    If Voltage Monitor is not powered and VMCR register setting is incorrect,
    an error is signalled.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsEnableVoltageMonitorWoInt_flash_C(void)
{
    ATA_globalsClkSwitchFrc_C();
    ATA_moduleTestGlobalsVoltageMonitor_flash_C(0x05);
    ATA_globalsSetVoltageMonitor_C(0x00);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestGlobalsVoltageMonitor_flash_C</b>
    starts and checks the configuration of the Voltage Monitor using 
    ATA_globalsSetVoltageMonitor_C.
    If Voltage Monitor is not powered and VMCR register setting is incorrect,
    an error is signalled.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestGlobalsVoltageMonitor_flash_C(uint8_t bVmcr)
{
    ATA_globalsSetVoltageMonitor_C(bVmcr);
    if (   (PRR0 & BM_PRVM) 
        || (VMCR != bVmcr)
       ){
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
}