/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/moduleTest/src/module_test.c $
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

#include "../../spi/src/spi.h"
#include "../../spi/src/ata5831_command_set.h"


#include "../../iic/src/iic.h"

#include "../../timer1/src/timer1.h"
#include "../../timer2/src/timer2.h"
#include "../../timer3/src/timer3.h"
#include "../../timer4/src/timer4.h"
#include "../../timer5/src/timer5.h"
#include "../../eep/src/eep_flash.h"
#include "../../rfrcc/src/rfrcc.h"
#include "../../lfrssi/src/lfrssi.h"
#include "../../lfrssi/src/lfrssi_flash.h"
#include "../../calib/src/calib.h"
#include "../../lfrx/src/lfrx.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

#define IIC_SLAVE_ADDR  0x2A
#define IIC_BUFFER_SIZE 16

#define ATA_RFRCC_USER_CMND  0x44U
#define ATA_RFRCC_SELEC_KEY  0

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
__root __no_init sModuleTest g_sModuleTest_flash;
__root __no_init uint8_t g_bModuleTestBuffer_flash[MODULE_TEST_BUFFER_SIZE];

/* Data structures to be set by the dedicated I2Ccomponent test cases. */
__root __no_init  uint8_t g_bModuleTestIicBuffer1_flash[IIC_BUFFER_SIZE];
__root __no_init  uint8_t g_bModuleTestIicBuffer2_flash[IIC_BUFFER_SIZE];

/* Data structures to be set by the dedicated EEPROM component test cases. */
__root __no_init static sEepModuleTest g_sEepModuleTest_flash;

/* Data structures to be set by the dedicated Timer component test cases. */
__root __no_init uint8_t g_bTimerModuleTestStatus_flash;
__root __no_init static uint8_t g_bTimerModuleReturnValue_flash;
__root __no_init static sTimerAsyn8BitParams g_sTimer8BitParams_flash;
__root __no_init static sTimerAsyn16BitParams g_sTimer16BitParams_flash;
__root __no_init static sTimerSyn16BitParams g_sTimer16BitSynParams_flash;

/* Data structures to be set by the dedicated LF RSSI component test cases. */
__no_init sModuleTestLfRssi static g_sModuleTestLfRssi_flash;

/* Data structures to be set by the dedicated Watchdog component test case. */
__root __no_init static  uint8_t g_bModuleTestWdtcr_flash;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/* Watchdog module test function */
static VOIDFUNC ATA_moduleTestWatchdog_flash_C(void);

/* EEP module test function */
static VOIDFUNC ATA_moduleTestEep_flash_C(void);

/* AES module test function */
static VOIDFUNC ATA_moduleTestAes_flash_C(void);

/* IIC module test function */
static VOIDFUNC ATA_moduleTestIicIRQ_flash_C(void);
static VOIDFUNC ATA_moduleTestIicPOL_flash_C(void);

/* Timer module test functions */
static VOIDFUNC ATA_moduleTestTimer1Handling_flash_C(void);
static VOIDFUNC ATA_moduleTestTimer2Handling_flash_C(void);
static VOIDFUNC ATA_moduleTestTimer3Handling_flash_C(void);
static VOIDFUNC ATA_moduleTestTimer4Handling_flash_C(void);
static VOIDFUNC ATA_moduleTestTimer5Handling_flash_C(void);
static VOIDFUNC ATA_moduleTestTimerMultipleOpen_flash_C(void);

/* Clock module test functions */
static VOIDFUNC ATA_moduleTestClkMrcFrcHandling_flash_C(void);
static VOIDFUNC ATA_moduleTestClkExtClockHandling_flash_C(void);

/* Rolling code module test functions */
static VOIDFUNC ATA_moduleTestRfrcc_flash_C(void);

/* LF RSSI measurement module test functions */
static VOIDFUNC ATA_moduleTestLfRssiHandling_flash_C(void);

/* LF RSSI measurement module test functions */
static VOIDFUNC ATA_moduleTestLfTx_flash_C(void);

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestInit_flash_C</b>
    shall initialize the data structures used in package moduleTest.

    \Derived no

    \Rationale none

    \Traceability Primus2P-xxx

    \StackUsage SU_XXX bytes

    \image html ATA_moduleTestInit_flash_C.png
    \image rtf ATA_moduleTestInit_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestInit_flash_C(void)
{
    g_sModuleTest_flash.bId = 0U;
    g_sModuleTest_flash.bSubId = 0U;
    g_bTimerModuleTestStatus_flash = 0U;
    g_bTimerModuleReturnValue_flash = 0U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSoftware_flash_C</b>
    shall manage the module tests for each package.

    \Derived no

    \Rationale none

    \Traceability Primus2P-xxx

    \StackUsage SU_XXX bytes

    \image html ATA_moduleTestSoftware_flash_C.png
    \image rtf ATA_moduleTestSoftware_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSoftware_flash_C()
{
    switch(g_sModuleTest_flash.bId)
    {
        case MODULE_TEST_ID_SPI_WITH_INTERRUPTS:      /* MODULE SPI (normal mode)   */
        case MODULE_TEST_ID_SPI_NO_INTERRUPTS:        /* MODULE SPI (checksum mode) */
            ATA_moduleTestSpi_flash_C();
            break;

        case MODULE_TEST_ID_IIC_WITH_INTERRUPTS:
            ATA_moduleTestIicIRQ_flash_C();
            break;
        case MODULE_TEST_ID_IIC_NO_INTERRUPTS:
            ATA_moduleTestIicPOL_flash_C();
            break;


        case MODULE_TEST_ID_WDT:           /* Module Watchdog */
            ATA_moduleTestWatchdog_flash_C();
            break;

        case MODULE_TEST_ID_EEP:         /* MODULE EEP */
            ATA_moduleTestEep_flash_C();
            break;

        case MODULE_TEST_ID_TIMER1:      /* MODULE Timer1 */
            ATA_moduleTestTimer1Handling_flash_C();
            break;

        case MODULE_TEST_ID_TIMER2:      /* MODULE Timer2 */
            ATA_moduleTestTimer2Handling_flash_C();
            break;

        case MODULE_TEST_ID_TIMER3:      /* MODULE Timer3 */
            ATA_moduleTestTimer3Handling_flash_C();
            break;

        case MODULE_TEST_ID_TIMER4:      /* MODULE Timer4 */
            ATA_moduleTestTimer4Handling_flash_C();
            break;

        case MODULE_TEST_ID_TIMER5:      /* MODULE Timer5 */
            ATA_moduleTestTimer5Handling_flash_C();
            break;

        case MODULE_TEST_ID_TIMER_FAIL:  /* MODULE Timer Multiple Opening */
            ATA_moduleTestTimerMultipleOpen_flash_C();
            break;

        case MODULE_TEST_ID_AES:         /* MODULE AES */
            ATA_moduleTestAes_flash_C();
            break;

        case MODULE_TEST_ID_CLK_MRC_FRC:    /* MODULE CLOCK MRC_FRC */
            ATA_moduleTestClkMrcFrcHandling_flash_C();
            break;

        case MODULE_TEST_ID_ROLL_CODE_CTR:
            ATA_moduleTestRfrcc_flash_C();
          break;

        case MODULE_TEST_ID_CLK_EXT_CLOCK:
            ATA_moduleTestClkExtClockHandling_flash_C();
          break;

        case MODULE_TEST_ID_LF_RSSI:        /* MODULE LF RSSI Measurement */
            ATA_moduleTestLfRssiHandling_flash_C();
            break;

        case MODULE_TEST_ID_CALIB:          /* SRC and FRC Calibration */
            ATA_moduleTestCalib_flash_C();
            break;

        case MODULE_TEST_ID_SPI_BASIC_FUNCTIONS: /* low level functions of module SPI */
            ATA_moduleTestSpiBasicFunctions_flash_C();
            break;
        case MODULE_TEST_ID_RFTX:   /* MODULE RF TX module tests */
            ATA_moduleTestRfTx_flash_C();
            break;
        case MODULE_TEST_ID_LFTX:   /* MODULE LF TX module tests */
            ATA_moduleTestLfTx_flash_C();
            break;
        case MODULE_TEST_ID_GLOBALS:
            ATA_moduleTestGlobals_flash_C();
        case MODULE_TEST_DISABLED:      /* no module test selected */
        default:
            _NOP;
            break;
    }

    g_sModuleTest_flash.bId = 0U;
}


VOIDFUNC ATA_moduleTestIicPOL_flash_C(void){


  }


VOIDFUNC ATA_moduleTestIicIRQ_flash_C(void){
    uint8_t tmp;
  switch(g_sModuleTest_flash.bSubId)
  {

        case MODULE_TEST_IIC_FILL_BUFFER1:
          for(tmp=0;tmp<IIC_BUFFER_SIZE;tmp++){
            g_bModuleTestIicBuffer1_flash[tmp] = tmp;
          }
    break;

        case MODULE_TEST_IIC_FILL_BUFFER2:
          for(tmp=0;tmp<IIC_BUFFER_SIZE;tmp++){
            g_bModuleTestIicBuffer2_flash[tmp] = tmp;
          }
    break;
        case MODULE_TEST_IIC_CLR_BUFFER1:
          for(tmp=0;tmp<IIC_BUFFER_SIZE;tmp++){
            g_bModuleTestIicBuffer1_flash[tmp] = 0x00U;
          }
      break;
        case MODULE_TEST_IIC_CLR_BUFFER2:
          for(tmp=0;tmp<IIC_BUFFER_SIZE;tmp++){
            g_bModuleTestIicBuffer2_flash[tmp] = 0x00U;
  }
          break;

        case MODULE_TEST_IIC_SET_BUFFER1:
          ATA_iicSetBuffer_C(&g_bModuleTestIicBuffer1_flash[0],IIC_BUFFER_SIZE,&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_SET_BUFFER2:
          ATA_iicSetBuffer_C(&g_bModuleTestIicBuffer2_flash[0],IIC_BUFFER_SIZE,&iicCtrlBlock);
          break;

        case MODULE_TEST_IIC_GET_BUFFER1:
          ATA_iicGetBuffer_C(&g_bModuleTestIicBuffer1_flash[0],IIC_BUFFER_SIZE,&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_GET_BUFFER2:
          ATA_iicGetBuffer_C(&g_bModuleTestIicBuffer2_flash[0],IIC_BUFFER_SIZE,&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_SET_DATA_0:
          ATA_iicSetData_C(0x00U,0xAA,&iicCtrlBlock,0x0FU);
          break;
        case MODULE_TEST_IIC_SET_DATA_1:
          ATA_iicSetData_C(0x01U,0xBB,&iicCtrlBlock,0x0FU);
          break;
        case MODULE_TEST_IIC_SET_DATA_15:
          ATA_iicSetData_C(0x0FU,0xCC,&iicCtrlBlock,0x0FU);
          break;
        case MODULE_TEST_IIC_GET_DATA_0:
          ATA_iicGetData_C(0x00U,&iicCtrlBlock,&g_bModuleTestIicBuffer2_flash[0]);
          break;
        case MODULE_TEST_IIC_GET_DATA_1:
          ATA_iicGetData_C(0x01U,&iicCtrlBlock,&g_bModuleTestIicBuffer2_flash[0]);
          break;
        case MODULE_TEST_IIC_GET_DATA_15:
          ATA_iicGetData_C(0x0FU,&iicCtrlBlock,&g_bModuleTestIicBuffer2_flash[0]);
    break;
        case MODULE_TEST_IIC_SET_DATA_OUTOFBOUNDS:
          ATA_iicSetData_C(IIC_BUFFER_SIZE,0xAA,&iicCtrlBlock,0x0FU);
    break;
        case MODULE_TEST_IIC_GET_DATA_OUTOFBOUNDS:
          ATA_iicGetData_C(IIC_BUFFER_SIZE,&iicCtrlBlock,&g_bModuleTestIicBuffer2_flash[0]);
    break;
        case MODULE_TEST_IIC_SET_BUFFER_OUTOFBOUNDS:
          ATA_iicSetBuffer_C(&g_bModuleTestIicBuffer2_flash[0],(IIC_BUFFER_SIZE+1),&iicCtrlBlock);
      break;
        case MODULE_TEST_IIC_GET_BUFFER_OUTOFBOUNDS:
          ATA_iicGetBuffer_C(&g_bModuleTestIicBuffer1_flash[0],(IIC_BUFFER_SIZE+1),&iicCtrlBlock);
      break;
        case MODULE_TEST_IIC_RESET:
          ATA_iicReset_C(&iicCtrlBlock);
    break;
        case MODULE_TEST_IIC_HW_SETUP:
          ATA_iicHwSetup_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U);
    break;
        case MODULE_TEST_IIC_BUFFER_SETUP1:
          ATA_iicBufferSetup_C(IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_BUFFER_SETUP2:
          ATA_iicBufferSetup_C(IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer2_flash[0],&iicCtrlBlock);
    break;
        case MODULE_TEST_IIC_SET_BUFFER_ENABLEFLAG:
          iicCtrlBlock.iicConfig |= BM_PTRVALID;
    break;
        case MODULE_TEST_IIC_OPEN:
          ATA_iicOpen_C(&iicCtrlBlock);
      break;
        case MODULE_TEST_IIC_USE_CONFIG1:
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          break;
        case MODULE_TEST_IIC_USE_CONFIG2:
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer2_flash[0],0xC0U);
    break;
        case MODULE_TEST_IIC_USE_CONFIG3:
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0BU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          break;
        case MODULE_TEST_IIC_USE_CONFIG4:
          ATA_iicUse_C(&iicCtrlBlock,0x01U,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
    break;
        case MODULE_TEST_IIC_SRAM_RESET:
          iicCtrlBlock.iicAddr = 0x00U;
          iicCtrlBlock.iicAddrMsk = 0x00U;
          iicCtrlBlock.iicBaudRate = 0x00U;
          iicCtrlBlock.iicPrescaler = 0x00U;
          iicCtrlBlock.iicConfig = 0x00U;
          iicCtrlBlock.iicStatus = 0x00U;
          iicCtrlBlock.iicByteCount = 0x00U;
          iicCtrlBlock.iicDataTx = 0x00U;
          iicCtrlBlock.iicBufLen = 0x00U;
          iicCtrlBlock.iicBuffer = (uint8_t *)0x0000;
          break;
        case MODULE_TEST_IIC_TRIGGER_MASTERTX:
          *iicCtrlBlock.iicBuffer = 0xF0U;//address is 1111000 , Master-TX
          iicCtrlBlock.iicDataTx = 0x10U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_ERROR_RESET1:
        case MODULE_TEST_IIC_ERROR_RESET2:
        case MODULE_TEST_IIC_ERROR_RESET3:
        case MODULE_TEST_IIC_ERROR_RESET4:
        case MODULE_TEST_IIC_ERROR_RESET5:
        case MODULE_TEST_IIC_ERROR_RESET6:
        case MODULE_TEST_IIC_ERROR_RESET7:
          iicCtrlBlock.iicStatus &= ~(BM_CONFERRORIIC|BM_IICTRXERROR);
    break;
        case MODULE_TEST_IIC_DO_MASTER_TX:
          ATA_iicReset_C(&iicCtrlBlock);
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          for(tmp=0;tmp<IIC_BUFFER_SIZE;tmp++){
            g_bModuleTestIicBuffer1_flash[tmp] = tmp;
  }
          *iicCtrlBlock.iicBuffer = 0xF0U;//address is 1111000 , Master-TX
          iicCtrlBlock.iicDataTx = 0x10U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_TRIGGER_MASTERTX_NODATA:
          *iicCtrlBlock.iicBuffer = 0xF0U;//address is 1111000 , Master-TX
          iicCtrlBlock.iicDataTx = 0x00U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_RETRIGGER_MASTERTX:
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
    break;
        case MODULE_TEST_IIC_TRIGGER_MASTERRX_NODATA:
          *iicCtrlBlock.iicBuffer = 0xF1U;//address is 1111000 , Master-RX
          iicCtrlBlock.iicDataTx = 0x00U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
    break;
        case MODULE_TEST_IIC_TRIGGER_MASTERRX:
          *iicCtrlBlock.iicBuffer = 0xF1U;//address is 1111000 , Master-RX
          iicCtrlBlock.iicDataTx = 0x02U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
    break;
        case MODULE_TEST_IIC_DO_MASTER_RX:
          ATA_iicReset_C(&iicCtrlBlock);
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          for(tmp=0;tmp<IIC_BUFFER_SIZE;tmp++){
            g_bModuleTestIicBuffer1_flash[tmp] = 0x00U;
  }
          *iicCtrlBlock.iicBuffer = 0xF1U;//address is 1111000 , Master-RX
          iicCtrlBlock.iicDataTx = 0x08U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_RETRIGGER_MASTERRX:
          *iicCtrlBlock.iicBuffer = 0xF1U;//address is 1111000 , Master-RX
          iicCtrlBlock.iicDataTx = 0x08U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_CLR_ERROR_ENA_EA:
          ATA_iicClrError_C(&iicCtrlBlock);
          ATA_iicClrRxDataPend_C(&iicCtrlBlock);
          break;
        case MODULE_TEST_IIC_SETDATATX_LEN:
          ATA_iicUse_C(&iicCtrlBlock,0xF0U,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          iicCtrlBlock.iicDataTx = 0x08U;
    break;
        case MODULE_TEST_IIC_CLRDATATX_LEN:
          ATA_iicUse_C(&iicCtrlBlock,0xF0U,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          iicCtrlBlock.iicDataTx = 0x00U;
    break;
    case MODULE_TEST_IIC_TRIGGER_MASTERRX_BADLEN:
          ATA_iicReset_C(&iicCtrlBlock);
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          *iicCtrlBlock.iicBuffer = 0xF1U;//address is 1111000 , Master-RX
          iicCtrlBlock.iicDataTx = 0x01U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
    break;
    case MODULE_TEST_IIC_DO_MASTER_TX8:
          ATA_iicReset_C(&iicCtrlBlock);
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0xC0U);
          for(tmp=0;tmp<0x08U;tmp++){
            g_bModuleTestIicBuffer1_flash[tmp] = 0x55;
          }
          *iicCtrlBlock.iicBuffer = 0xF0U;//address is 1111000 , Master-TX
          iicCtrlBlock.iicDataTx = 0x08U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
    break;
    case MODULE_TEST_IIC_DO_MASTER_TX_POLL:
          ATA_iicReset_C(&iicCtrlBlock);
          ATA_iicUse_C(&iicCtrlBlock,IIC_SLAVE_ADDR,0x00U,0x0AU,0x00U,IIC_BUFFER_SIZE,&g_bModuleTestIicBuffer1_flash[0],0x80U);
          for(tmp=0;tmp<0x08;tmp++){
            g_bModuleTestIicBuffer1_flash[tmp] = 0xAA;
          }
          *iicCtrlBlock.iicBuffer = 0xF0U;//address is 1111000 , Master-TX
          iicCtrlBlock.iicDataTx = 0x08U;
          ATA_iicMasterTrigger_C(&iicCtrlBlock);
    break;
    case MODULE_TEST_IIC_POLL_I2CR:
        ATA_iicServeIsr_C();
    break;

    default:
      break;
  }
}



VOIDFUNC ATA_moduleTestWatchdog_flash_C(void) /* Primus2P - 1319 */
{
    switch (g_sModuleTest_flash.bSubId)
    {
        case ATA_WATCHDOG_ENABLE:
          /* Enable watchdog with the given settings. */
          ATA_globalsWdtEnable_C(g_bModuleTestWdtcr_flash);
          while(1);
          break;

        case ATA_WATCHDOG_DISABLE:
          ATA_globalsWdtDisable_C();
          break;
          
        /* Test POWER_DOWN sleep mode */
        case ATA_SLEEP_ENABLE:
          ATA_globalsSleep_C(0x00U);
          break;
    }
}

/* EEPROM test function */
VOIDFUNC ATA_moduleTestEep_flash_C(void)
{
    /* Get Access Right Map section, which defines the access rights for the given EEPROM address */
    uint8_t tempEepSection = (uint8_t)(g_sEepModuleTest_flash.eepAddress >> 6);

    switch (g_sModuleTest_flash.bSubId)
    {
        /* Clear EEPROM module test handler parameter. */
        case ATA_EEP_CLEAR_MODULE_HANDLER_PARAMETERS:
          g_sEepModuleTest_flash.eepErrorCode = EEC_NOT_USED;
          g_sEepModuleTest_flash.eepAddress = 0x0000U;
          g_sEepModuleTest_flash.eepData[0] = 0x00U;
          g_sEepModuleTest_flash.eepData[1] = 0x00U;
          g_sEepModuleTest_flash.eepData[2] = 0x00U;
          g_sEepModuleTest_flash.eepData[3] = 0x00U;
          g_sEepModuleTest_flash.eepData[4] = 0x00U;
          g_sEepModuleTest_flash.eepData[5] = 0x00U;
          g_sEepModuleTest_flash.eepData[6] = 0x00U;
          g_sEepModuleTest_flash.eepData[7] = 0x00U;
          g_sEepModuleTest_flash.eepData[8] = 0x00U;
          g_sEepModuleTest_flash.eepData[9] = 0x00U;
          g_sEepModuleTest_flash.eepData[10] = 0x00U;
          g_sEepModuleTest_flash.eepData[11] = 0x00U;
          g_sEepModuleTest_flash.eepData[12] = 0x00U;
          g_sEepModuleTest_flash.eepData[13] = 0x00U;
          g_sEepModuleTest_flash.eepData[14] = 0x00U;
          g_sEepModuleTest_flash.eepData[15] = 0x00U;
          g_sEepModuleTest_flash.eepDataLength = 0x00U;
          g_sEepModuleTest_flash.eepAccessRightValue = 0x00U;
          g_sEepModuleTest_flash.eepAccessRightMask = 0x00U;
          break;

        /* Single Byte Write Access */
        case ATA_EEP_SINGLE_BYTE_WRITE_ACCESS:
          g_sEepModuleTest_flash.eepErrorCode = ATA_eepWriteByte_C(g_sEepModuleTest_flash.eepAddress, g_sEepModuleTest_flash.eepData[0]);

          /* Wait until EEPROM write operation is finished */
          while(EECR & BM_NVMBSY){}
          break;

        /* Single Byte Read Access */
        case ATA_EEP_SINGLE_BYTE_READ_ACCESS:
          g_sEepModuleTest_flash.eepErrorCode = ATA_eepReadByte_C(g_sEepModuleTest_flash.eepAddress, &g_sEepModuleTest_flash.eepData[0]);
          break;

        /* Multi Byte Write Access */
        case ATA_EEP_MULTI_BYTE_WRITE_ACCESS:
          g_sEepModuleTest_flash.eepErrorCode = ATA_eepWriteMultipleBytes_C(g_sEepModuleTest_flash.eepAddress, g_sEepModuleTest_flash.eepDataLength, &g_sEepModuleTest_flash.eepData[0]);

          /* Wait until EEPROM write operation is finished */
          while(EECR & BM_NVMBSY){}
          break;

        /* Multi Byte Write Access */
        case ATA_EEP_MULTI_BYTE_READ_ACCESS:
          g_sEepModuleTest_flash.eepErrorCode = ATA_eepReadMultipleBytes_C(g_sEepModuleTest_flash.eepAddress, g_sEepModuleTest_flash.eepDataLength, &g_sEepModuleTest_flash.eepData[0]);
          break;

        /* Multi Byte Write Access */
        case ATA_EEP_CHANGE_ACCESS_RIGHTS:
          g_sEepModuleTest_flash.eepErrorCode = ATA_eepChangeAccessRights_C(tempEepSection, g_sEepModuleTest_flash.eepAccessRightValue, g_sEepModuleTest_flash.eepAccessRightMask);

          /* Wait until EEPROM write operation is finished */
          while(EECR & BM_NVMBSY){}
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}


/* AES test function */
VOIDFUNC ATA_moduleTestAes_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Initialize AES component data. */
        case ATA_AES_INIT_SECR_KEY1:
          ATA_aesSetConfig_C(AES_USE_SECRET_KEY_ONE, 0u);
          break;

        case ATA_AES_INIT_SECR_KEY2:
          ATA_aesSetConfig_C(AES_USE_SECRET_KEY_TWO, 0u);
          break;

        /* AES encryption */
        case ATA_AES_ENCRYPTION:
            ATA_aesEncryptData_C();
            break;

        /* AES decryption */
        case ATA_AES_DECRYPTION:
            ATA_aesDecryptData_C();
            break;

        /* AES run error during encryption with interrupt mode configured */
        case ATA_AES_RUN_ERROR_ENCRYPTION_INT_MODE:
            ATA_aesEncryptData_C();
            while ((AESCR & BM_AESE)== 0)
            {;}
            break;

        /* AES run error during encryption with polling mode configured */
        case ATA_AES_RUN_ERROR_ENCRYPTION_POLL_MODE:
            ATA_aesEncryptData_C();
            break;

         /* AES run error during decryption with interrupt mode configured.
            Reading from register AESCR already creates an AES error ?!*/
        case ATA_AES_RUN_ERROR_DECRYPTION_INT_MODE:
            ATA_aesDecryptData_C();
            while ((AESCR & BM_AESE)== 0)
            {;}
            break;

         /* AES run error during decryption with polling mode configured */
        case ATA_AES_RUN_ERROR_DECRYPTION_POLL_MODE:
            ATA_aesDecryptData_C();
            break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}


/* Timer1 handling */
VOIDFUNC ATA_moduleTestTimer1Handling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Timer1 Initialization */
        case ATA_TIMER_INIT:
          ATA_timer1Init_C();
          break;

        /* Timer1 open in C code */
        case ATA_TIMER_OPEN_C:
          g_bTimerModuleReturnValue_flash = ATA_timer1Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Timer1 close in C code */
        case ATA_TIMER_CLOSE_C:
          ATA_timer1Close_C();
          break;

        /* Timer1 close in Assembler code */
        case ATA_TIMER_CLOSE_ASM:
          ATA_timer1Close_ASM();
          break;

        /* Timer1 Compare Match Interrupt Handling */
        case ATA_TIMER_COM_INTHANDLER_ASM:
          ATA_timer1Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer1Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Timer1 Counter Overflow Interrupt Handling */
        case ATA_TIMER_OVF_INTHANDLER_ASM:
          ATA_timer1Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer1Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}

/* Timer2 handling */
VOIDFUNC ATA_moduleTestTimer2Handling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Timer2 Initialization */
        case ATA_TIMER_INIT:
          ATA_timer2Init_C();
          break;

        /* Timer2 open in C code */
        case ATA_TIMER_OPEN_C:
          g_bTimerModuleReturnValue_flash = ATA_timer2Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Timer2 close in C code */
        case ATA_TIMER_CLOSE_C:
          ATA_timer2Close_C();
          break;

        /* Timer2 close in Assembler code */
        case ATA_TIMER_CLOSE_ASM:
          ATA_timer2Close_ASM();
          break;

        /* Timer2 Compare Match Interrupt Handling */
        case ATA_TIMER_COM_INTHANDLER_ASM:
          ATA_timer2Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer2Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Timer2 Counter Overflow Interrupt Handling */
        case ATA_TIMER_OVF_INTHANDLER_ASM:
          ATA_timer2Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer2Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}

/* Timer3 handling */
VOIDFUNC ATA_moduleTestTimer3Handling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Timer3 Initialization */
        case ATA_TIMER_INIT:
          ATA_timer3Init_C();
          break;

        /* Timer3 open in C code */
        case ATA_TIMER_OPEN_C:
          g_bTimerModuleReturnValue_flash = ATA_timer3Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer3 close in C code */
        case ATA_TIMER_CLOSE_C:
          ATA_timer3Close_C();
          break;

        /* Timer3 close in Assembler code */
        case ATA_TIMER_CLOSE_ASM:
          ATA_timer3Close_ASM();
          break;

        /* Timer3 Compare Match Interrupt Handling */
        case ATA_TIMER_COM_INTHANDLER_ASM:
          ATA_timer3Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer3Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer3 Counter Overflow Interrupt Handling */
        case ATA_TIMER_OVF_INTHANDLER_ASM:
          ATA_timer3Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer3Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer3 Capture Interrupt Handling */
        case ATA_TIMER_CAP_INTHANDLER_ASM:
          ATA_timer3Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer3Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}

/* Timer4 handling */
VOIDFUNC ATA_moduleTestTimer4Handling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Timer4 Initialization */
        case ATA_TIMER_INIT:
          ATA_timer4Init_C();
          break;

        /* Timer4 open in C code */
        case ATA_TIMER_OPEN_C:
          g_bTimerModuleReturnValue_flash = ATA_timer4Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer4 close in C code */
        case ATA_TIMER_CLOSE_C:
          ATA_timer4Close_C();
          break;

        /* Timer4 close in Assembler code */
        case ATA_TIMER_CLOSE_ASM:
          ATA_timer4Close_ASM();
          break;

        /* Timer4 Compare Match Interrupt Handling */
        case ATA_TIMER_COM_INTHANDLER_ASM:
          ATA_timer4Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer4Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer3 Counter Overflow Interrupt Handling */
        case ATA_TIMER_OVF_INTHANDLER_ASM:
          ATA_timer4Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer4Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer4 Capture Interrupt Handling */
        case ATA_TIMER_CAP_INTHANDLER_ASM:
          ATA_timer4Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer4Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}

/* Timer5 handling */
VOIDFUNC ATA_moduleTestTimer5Handling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Timer5 Initialization */
        case ATA_TIMER_INIT:
          ATA_timer5Init_C();
          break;

        /* Timer5 open in C code */
        case ATA_TIMER_OPEN_C:
          g_bTimerModuleReturnValue_flash = ATA_timer5Open_C(&g_sTimer16BitSynParams_flash);
          break;

        /* Timer5 close in C code */
        case ATA_TIMER_CLOSE_C:
          ATA_timer5Close_C();
          break;

        /* Timer5 close in Assembler code */
        case ATA_TIMER_CLOSE_ASM:
          ATA_timer5Close_ASM();
          break;

        /* Timer5 Compare Match Interrupt Handling */
        case ATA_TIMER_COM_INTHANDLER_ASM:
          ATA_timer5Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer5Open_C(&g_sTimer16BitSynParams_flash);
          break;

        /* Timer5 Counter Overflow Interrupt Handling */
        case ATA_TIMER_OVF_INTHANDLER_ASM:
          ATA_timer5Init_C();
          g_bTimerModuleReturnValue_flash = ATA_timer5Open_C(&g_sTimer16BitSynParams_flash);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}

/* Timer Multiple Opening */
VOIDFUNC ATA_moduleTestTimerMultipleOpen_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Timer1 Initialization */
        case ATA_TIMER_INIT:
          ATA_timer1Init_C();
          break;

        /* Timer 1 open in C code */
        case ATA_TIMER_1_OPEN_FAIL_C:
          g_bTimerModuleReturnValue_flash = ATA_timer1Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Timer 2 open in C code */
        case ATA_TIMER_2_OPEN_FAIL_C:
          g_bTimerModuleReturnValue_flash = ATA_timer2Open_C(&g_sTimer8BitParams_flash);
          break;

        /* Timer 3 open in C code */
        case ATA_TIMER_3_OPEN_FAIL_C:
          g_bTimerModuleReturnValue_flash = ATA_timer3Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer 4 open in C code */
        case ATA_TIMER_4_OPEN_FAIL_C:
          g_bTimerModuleReturnValue_flash = ATA_timer4Open_C(&g_sTimer16BitParams_flash);
          break;

        /* Timer 5 open in C code */
        case ATA_TIMER_5_OPEN_FAIL_C:
          g_bTimerModuleReturnValue_flash = ATA_timer5Open_C(&g_sTimer16BitSynParams_flash);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}

/* Clock MRC_FRC Handling */
VOIDFUNC ATA_moduleTestClkMrcFrcHandling_flash_C(void)  /* Primus2P - 1565 */
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Switch clock MRC on */
        case ATA_CLOCK_MRC_ON:
          ATA_globalsClkSwitchMrc_C();
          break;

        /* Switch clock FRC on */
        case ATA_CLOCK_FRC_ON:
          ATA_globalsClkSwitchFrc_C();
          break;

        /* Switch clock SRC on */
        case ATA_CLOCK_SRC_ON:
          ATA_globalsClkSwitchSrc_C();
          break;
          
        /* Disable VMEM and DVCC */
        case ATA_CLOCK_DISABLE_VMEM_DVCC:
          ATA_globalsSwitchMvccRegulator_C(FALSE);
          break;

        /* Enable VMEM and DVCC */
        case ATA_CLOCK_ENABLE_VMEM_DVCC:
          ATA_globalsSwitchMvccRegulator_C(TRUE);
          break;

        /* Switch clock XTO on */
        case ATA_CLOCK_XTO_ON:
          ATA_globalsClockSwitchXtoWithMvccEnable_C(0x07U);
          break;
        
        /* XTO off (XTO must not be set as system clock */
        case ATA_CLOCK_XTO_OFF:
          ATA_globalsDeActivateXTO_C();
          break;
          
        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}


/* External Clock Handling */
VOIDFUNC ATA_moduleTestClkExtClockHandling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Switch clock MRC on */
        case ATA_CLOCK_MRC_ON:
          ATA_globalsClkSwitchMrc_C();
          break;

        /* Switch clock FRC on */
        case ATA_CLOCK_FRC_ON:
          ATA_globalsClkSwitchFrc_C();
          break;

        /* Disable VMEM and DVCC */
        case ATA_CLOCK_DISABLE_VMEM_DVCC:
          ATA_globalsSwitchMvccRegulator_C(FALSE);
          break;

        /* Enable VMEM and DVCC */
        case ATA_CLOCK_ENABLE_VMEM_DVCC:
          ATA_globalsSwitchMvccRegulator_C(TRUE);
          break;

        /* Select external clock greater than 1 MHZ as system clock */
        case ATA_CLOCK_EXT_HIGH:
          ATA_globalsClkSwitchExt_C(TRUE);
          break;

        /* Select external clock smaller than 1 MHZ as system clock */
        case ATA_CLOCK_EXT_LOW:
          ATA_globalsClkSwitchExt_C(FALSE);
          break;

        /* Wrong sub id loaded by test case. */
        default:
           _NOP;
    }
}


VOIDFUNC ATA_moduleTestRfrcc_flash_C(void)
{
  switch (g_sModuleTest_flash.bSubId)
  {
    case ATA_RFRCC_TRANSMIT_RCC:
      ATA_rfrccGenRollCntMsg_C(ATA_RFRCC_SELEC_KEY, ATA_RFRCC_USER_CMND);
    break;

   default:
     _NOP;
  }
}
/* LF RSSI measurement Handling */
VOIDFUNC ATA_moduleTestLfRssiHandling_flash_C(void)
{
    switch (g_sModuleTest_flash.bSubId)
    {
        /* Power-up the LF RSSI block */
        case ATA_LFRSSI_OPEN:
            ATA_lfRssiMeasOpen_C();
            break;
        /* Initialize the working RSSI registers */
        case ATA_LFRSSI_INIT:
            ATA_lfRssiMeasInit_C();
            break;
        /* Start internal RSSI measurement */
        case ATA_LFRSSI_START_INTERN:
            ATA_calibInit_C();
            g_sLfRssi.bChanTimeOutMask = 0;
            ATA_lfRssiMeasStartInt_C(g_sModuleTestLfRssi_flash.lfrscr, g_sModuleTestLfRssi_flash.lfrsmr, g_sModuleTestLfRssi_flash.calib);
            break;
        /* Start external RSSI measurement */
        case ATA_LFRSSI_START_EXTERN:
            g_sLfRssi.bChanTimeOutMask = 0;
            ATA_lfRssiMeasStartExt_C(g_sModuleTestLfRssi_flash.lfrscr, g_sModuleTestLfRssi_flash.lfrsmr);
            break;
        /* Stop the RSSI measurement */
        case ATA_LFRSSI_STOP:
            ATA_lfRssiMeasStop_C();
            break;
        /* Power-down the LF RSSI block */
        case ATA_LFRSSI_CLOSE:
            ATA_lfRssiMeasClose_C();
            break;
        /* Compensates the raw LF RSSI values */
        case ATA_LFRSSI_COMPENSATE:
            //ATA_lfRssiMeasComp_C();
            break;
        /* Converts raw logarithmic LF RSSI values to corresponding induced voltage value */
        case ATA_LFRSSI_LOG2VOLTAGE:
            ATA_lfRssiMeasLfRssi2IndVoltage_C(&g_sLfRssiData_flash.wLfRssiCoilsComp[0], &g_sLfRssiData_flash.wLfRssiVal);
            break;
        /* Checks for channel timeout signalization during measurement */
        case ATA_LFRSSI_CHECKTIMEOUT:
            g_sLfRssi.bChanTimeOutMask &= 0x07;   // mask timeout value with enabled channels -> for test all channels are enabled
            ATA_lfRssiMeasCheckTimeOut_C(&g_sLfRssi.wRawLfRssi[0], &g_sLfRssi.bChanTimeOutMask, g_sModuleTestLfRssi_flash.thresLow, g_sModuleTestLfRssi_flash.thresHigh);
            break;
        /* Wrong sub id loaded by test case. */
        default:
            _NOP;
    }
}

/**/
VOIDFUNC ATA_moduleTestLfTx_flash_C(void)
{
    /* Only call function ATA_lfRxInit_C. It does not matter that the LF
       registers are already locked via register LFCPR. */
    switch (g_sModuleTest_flash.bSubId)
    {
    case MODULE_TEST_LFTX_SUBID_INIT:
      ATA_lfRxInit_C();
      break;

    default:
      break;
    }

}

