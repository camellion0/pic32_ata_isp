/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/moduleTest/src/module_test_rftx.c $
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

#include "../../spi/src/spi.h"
#include "../../system/src/system_flash.h"
#include "../../spi/src/ata5831_command_set.h"
#include "../../rftx/src/rftx.h"
#include "../../rftx/src/rftx_vco.h"
#include "../../rftx/src/rftx_ant.h"

#include "../../timer2/src/timer2.h"


static VOIDFUNC ATA_moduleTestRfTxInit_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxTxBufSvc1_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxTxBufSvc2_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxTxBufSvc3_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxVcoTuning1_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxVcoTuning2_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxVcoTuning3_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxAntTuning1_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxAntTuning2_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxAntTuning3_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxSsmWatchdog_flash_C(void);
static VOIDFUNC ATA_moduleTestRfTxClose_flash_C(void);

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/** \brief <b>moduleTestRfTx</b>
    is a look up table for RfTx module test. The sub test are addressed via
    index variable g_sModuleTest_flash.bSubId
    0x00:   init module RfTx
    0x01:   tbd
    0x02:   tbd
    0x..:   close module RfTx

*/
static moduleTestFuncLut m_fpModuleTestRfTx_flash[] = {
    0x0000U,
    ATA_moduleTestRfTxInit_flash_C,         // 0x01
    ATA_moduleTestRfTxTxBufSvc1_flash_C,    // 0x02    
    ATA_moduleTestRfTxTxBufSvc2_flash_C,    // 0x03
    ATA_moduleTestRfTxTxBufSvc3_flash_C,    // 0x04
    ATA_moduleTestRfTxVcoTuning1_flash_C,   // 0x05
    ATA_moduleTestRfTxVcoTuning2_flash_C,   // 0x06
    ATA_moduleTestRfTxVcoTuning3_flash_C,   //
    ATA_moduleTestRfTxAntTuning1_flash_C,   // 0x07
    ATA_moduleTestRfTxAntTuning2_flash_C,   // 0x08
    ATA_moduleTestRfTxAntTuning3_flash_C,   // 
    ATA_moduleTestRfTxSsmWatchdog_flash_C,  // 
    ATA_moduleTestRfTxClose_flash_C         // 0x09

};

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
#pragma location = ".sram_FlashApp_ModuleTest"
__root __no_init static sRfTxServiceChannelConfig m_bModuleTestRfTxServiceConfig_flash;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTx_flash_C</b>
    is the entry point for RfTx module test. 
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTx_flash_C(void)
{
    moduleTestFunc fpFunc = m_fpModuleTestRfTx_flash[g_sModuleTest_flash.bSubId];
    if (fpFunc != 0x0000U) {
        fpFunc();
    } else {}
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxInit_flash_C</b>
    checks the initialization of rfTx Module using function ATA_rfTxInit_C.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxInit_flash_C(void)
{
    ATA_rfTxInit_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxTxBufSvc1_flash_C</b>
    checks TXMode(buffered) with service configuration from EEPROM variable 
    g_sEepRfTxServiceConfig1. 
    Filling DFIFO and SFIFO is done via function ATA_rfTxFillDFifo_C and 
    ATA_rfTxFillSFifo_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxTxBufSvc1_flash_C(void)
{
    uint8_t result = ATA_rfTxGetAntennaTuningResult_C() + 1;

    g_bModuleTestBuffer_flash[0] = 0x18;
    g_bModuleTestBuffer_flash[1] = 0x55;
    g_bModuleTestBuffer_flash[2] = 0x55;
    g_bModuleTestBuffer_flash[3] = 0x54;
    ATA_rfTxFillSFifo_C(0x04U, g_bModuleTestBuffer_flash);
    
    g_bModuleTestBuffer_flash[0] = 0x01;
    g_bModuleTestBuffer_flash[1] = 0x23;
    g_bModuleTestBuffer_flash[2] = 0x45;
    g_bModuleTestBuffer_flash[3] = 0x67;
    ATA_rfTxFillDFifo_C(0x04U, g_bModuleTestBuffer_flash);
    
    // start rfTx with eeprom service g_sEepRfTxServiceConfig1
    ATA_rfTxStartTx_C(0xC8, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);

    if (result == ATA_rfTxGetAntennaTuningResult_C()){
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    };
    
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxTxBufSvc2_flash_C</b>
    checks TXMode(buffered) with service configuration from EEPROM variable 
    g_sEepRfTxServiceConfig2. 
    Filling DFIFO and SFIFO is done via function ATA_rfTxFillDFifo_C and 
    ATA_rfTxFillSFifo_C.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxTxBufSvc2_flash_C(void)
{
    g_bModuleTestBuffer_flash[0] = 0x18;
    g_bModuleTestBuffer_flash[1] = 0x55;
    g_bModuleTestBuffer_flash[2] = 0x55;
    g_bModuleTestBuffer_flash[3] = 0x54;
    ATA_rfTxFillSFifo_C(0x04U, g_bModuleTestBuffer_flash);
    
    g_bModuleTestBuffer_flash[0] = 0x01;
    g_bModuleTestBuffer_flash[1] = 0x23;
    g_bModuleTestBuffer_flash[2] = 0x45;
    g_bModuleTestBuffer_flash[3] = 0x67;
    ATA_rfTxFillDFifo_C(0x04U, g_bModuleTestBuffer_flash);
    
    // start rfTx with eeprom service g_sEepRfTxServiceConfig1
    ATA_rfTxStartTx_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig2);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxTxBufSvc3_flash_C</b>
    checks TXMode(buffered) with service configuration from SRAM  variable 
    m_bModuleTestRfTxServiceConfig. 
    Filling DFIFO and SFIFO is done via function ATA_rfTxFillDFifo_C and 
    ATA_rfTxFillSFifo_C.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxTxBufSvc3_flash_C(void)
{
    if (ATA_eepReadMultipleBytes_C((uint16_t)&g_sEepRfTxServiceConfig1,sizeof(sRfTxServiceChannelConfig),(uint8_t *)&m_bModuleTestRfTxServiceConfig_flash) == EEC_NO_ERROR) {

        g_bModuleTestBuffer_flash[0] = 0x18;
        g_bModuleTestBuffer_flash[1] = 0x55;
        g_bModuleTestBuffer_flash[2] = 0x55;
        g_bModuleTestBuffer_flash[3] = 0x54;
        ATA_rfTxFillSFifo_C(0x04U, g_bModuleTestBuffer_flash);
        
        g_bModuleTestBuffer_flash[0] = 0x01;
        g_bModuleTestBuffer_flash[1] = 0x23;
        g_bModuleTestBuffer_flash[2] = 0x45;
        g_bModuleTestBuffer_flash[3] = 0x67;
        ATA_rfTxFillDFifo_C(0x04U, g_bModuleTestBuffer_flash);
        
        // start rfTx with sram service g_sEepRfTxServiceConfig1
        ATA_rfTxStartTx_C(0x40, (uint8_t *) &m_bModuleTestRfTxServiceConfig_flash);
        do {
            ATA_rfTxProcessing_C();
        }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);

    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxVcoTuning1_flash_C</b>
    checks VCO tuning with service configuration from EEPROM variable 
    g_sEepRfTxServiceConfig1. 
    VCO tuning result is checked with function ATA_rfTxGetVcoTuningResult_C
    If V
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxVcoTuning1_flash_C(void)
{
    uint8_t result = ATA_rfTxGetVcoTuningResult_C() + 1;
    ATA_rfTxStartVco_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
    
    if (result == ATA_rfTxGetVcoTuningResult_C()){
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
        
    };
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxVcoTuning2_flash_C</b>
    checks stopping/canceling of VCO tuning with function ATA_rfTxStopVco_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxVcoTuning2_flash_C(void)
{
    uint8_t result = ATA_rfTxGetVcoTuningResult_C();

    ATA_rfTxStartVco_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
        if (g_sRfTxFlowCtrl.bIndex == RFTX_VCO_STATE_WAIT_SSMRDY) {
            ATA_rfTxStopVco_C();
        }
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxVcoTuning3_flash_C</b>
    checks the error handling of VCO tuning SSM
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxVcoTuning3_flash_C(void)
{
    ATA_rfTxStartVco_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
    
    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_RFTX_WAIT4VCOSSMRDY_TIMEOUT)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       ){
        ATA_globalsInitDebug_C();
    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxAntTuning1_flash_C</b>
    checks antenna tuning with service configuration from EEPROM variable 
    g_sEepRfTxServiceConfig1. 
    Antenna tuning result is checked with function ATA_rfTxGetAntennaTuningResult_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxAntTuning1_flash_C(void)
{
    uint8_t result = ATA_rfTxGetAntennaTuningResult_C() + 1;
    ATA_rfTxStartAnt_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);

    if (result == ATA_rfTxGetAntennaTuningResult_C()){
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    };
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxAntTuning2_flash_C</b>
    checks antenna tuning with service configuration from EEPROM variable 
    g_sEepRfTxServiceConfig2. 
    Antenna tuning result is checked with function ATA_rfTxGetAntennaTuningResult_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxAntTuning2_flash_C(void)
{
    ATA_rfTxStartAnt_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
        if (g_sRfTxFlowCtrl.bIndex == RFTX_ANT_STATE_WAIT_SSMRDY) {
            ATA_rfTxStopAnt_C();
        }
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxAntTuning3_flash_C</b>
    checks the error handling of antenna tuning SSM
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxAntTuning3_flash_C(void)
{
    ATA_rfTxStartAnt_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_RFTX_WAIT4ANTSSMRDY_TIMEOUT)
        && (g_sDebug.bSsmErrorCode == 0x00U)
       ){
        ATA_globalsInitDebug_C();
    } else {
        g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestRfTxSsmWatchdog_flash_C</b>
    checks the error handling SSM watchdog (timer2)
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxSsmWatchdog_flash_C(void)
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
    
    ATA_rfTxStartAnt_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
    do {
        ATA_rfTxProcessing_C();
    }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);

    if (   (g_sDebug.bErrorCode    == DEBUG_ERROR_CODE_RFTX_STARTSSM_TIMER_LOCKED)
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
/** \brief <b>ATA_moduleTestRfTxClose_flash_C</b>
    checks closing of rfTx Module using function ATA_rfTxClose_C.
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestRfTxClose_flash_C(void)
{
    ATA_rfTxClose_C();
}
