/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/moduleTest/src/module_test_spi.c $
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
/** \file module_test_spi.c
*/
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "module_test.h"

#include "../../spi/src/spi.h"
#include "../../system/src/system_flash.h"
#include "../../spi/src/ata5831_command_set.h"

static VOIDFUNC ATA_moduleTestSpiInitSpiMaster_flash_C(void);

static VOIDFUNC ATA_moduleTestSpiReadFillLevelRxFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadFillLevelTxFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadFillLevelRssiFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiGetEventBytes_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadRssiFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadRxFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiWriteRegisterSram_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadRegisterSram_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiWriteEeprom_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadEeprom_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiWriteTxFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiWriteTxPreambleFifo_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiSetSystemMode_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiCalibrateAndCheck_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiPatchSpi_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiGetRomVersion_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiGetFlashVersion_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiCustomCommand_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiSystemReset_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiTriggerEepSecureWrite_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiSetVoltageMonitor_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiOffCommand_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiReadTemperature_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiInitSramService_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiStartRssiMeasurement_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiGetRssiValue_flash_C(void);

static VOIDFUNC ATA_moduleTestSpiCloseSpiMaster_flash_C(void);

static UINT8FUNC ATA_moduleTestSpiCheckEventBytes_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiCheckWriteCommand_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiClearModuleTestBuffer_flash_C(void);
static VOIDFUNC ATA_moduleTestSpiNotInitialized_flash_C(void);

static VOIDFUNC ATA_moduleTestSpiNotInitializedCheckError_flash_C(void);
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/


/** \brief <b>moduleTestSpi</b>
    is a look up table for spi module test. The sub test are addressed via
    index variable g_sModuleTest_flash.bSubId
    0x00:   init module ATA5831
    0x01:   check command 'Read Fill Level RX FIFO'
    0x02:   check command 'Read Fill Level TX FIFO'
    0x03:   check command 'Read Fill Level RSSI FIFO'
    0x04:   check command 'Get Event Bytes'
    0x05:   check command 'Read RSSI FIFO'
    0x06:   check command 'Read RX FIFO'
    0x07:   check command 'Write Register/SRAM'
    0x08:   check command 'Read Register/SRAM'
    0x09:   check command 'Write EEPROM'
    0x0A:   check command 'Read EEPROM'
    0x0B:   check command 'Write TX FIFO'
    0x0C:   check command 'Write TX Preamble FIFO'
    0x0D:   check command 'Set System Mode'
    0x0E:   check command 'Calibrate And Check'
    0x0F:   check command 'Patch SPI'
    ... :   rfu
    0x12:   check command 'Get ROM Version'
    0x13:   check command 'Get FLASH Version'
    0x14:   check command 'Custom Command'
    0x15:   check command 'System Reset'
    0x16:   check command 'Trigger EEP Secure Write'
    0x17:   check command 'Set Voltage Monitor'
    0x18:   check command 'OFF Command'
    0x19:   check command 'Read Temperature'
    0x1A:   check command 'Init Sram service'
    0x1B:   check command 'Start RSSI Meas'
    0x1C:   check command 'Get RSSI Value'
    0x1D:   close module ATA5831

*/
static moduleTestFuncLut m_fpModuleTestSpi_flash[] = {
    ATA_moduleTestSpiInitSpiMaster_flash_C,
    ATA_moduleTestSpiReadFillLevelRxFifo_flash_C,       // ATA5831_ID_READ_FILL_LEVEL_RX_FIFO   (0x01U)
    ATA_moduleTestSpiReadFillLevelTxFifo_flash_C,       // ATA5831_ID_READ_FILL_LEVEL_TX_FIFO   (0x02U)
    ATA_moduleTestSpiReadFillLevelRssiFifo_flash_C,     // ATA5831_ID_READ_FILL_LEVEL_RSSI_FIFO (0x03U)
    ATA_moduleTestSpiGetEventBytes_flash_C,             // ATA5831_ID_GET_EVENT_BYTES           (0x04U)
    ATA_moduleTestSpiReadRssiFifo_flash_C,              // ATA5831_ID_READ_RSSI_FIFO            (0x05U)
    ATA_moduleTestSpiReadRxFifo_flash_C,                // ATA5831_ID_READ_RX_FIFO              (0x06U)
    ATA_moduleTestSpiWriteRegisterSram_flash_C,         // ATA5831_ID_WRITE_REGISTER_SRAM       (0x07U)
    ATA_moduleTestSpiReadRegisterSram_flash_C,          // ATA5831_ID_READ_REGISTER_SRAM        (0x08U)
    ATA_moduleTestSpiWriteEeprom_flash_C,               // ATA5831_ID_WRITE_EEPROM              (0x09U)
    ATA_moduleTestSpiReadEeprom_flash_C,                // ATA5831_ID_READ_EEPROM               (0x0AU)
    ATA_moduleTestSpiWriteTxFifo_flash_C,               // ATA5831_ID_WRITE_TX_FIFO             (0x0BU)
    ATA_moduleTestSpiWriteTxPreambleFifo_flash_C,       // ATA5831_ID_WRITE_TX_PREAMBLE_FIFO    (0x0CU)
    ATA_moduleTestSpiSetSystemMode_flash_C,             // ATA5831_ID_SET_SYSTEM_MODE           (0x0DU)
    ATA_moduleTestSpiCalibrateAndCheck_flash_C,         // ATA5831_ID_CALIBRATE_AND_CHECK       (0x0EU)
    ATA_moduleTestSpiPatchSpi_flash_C,                  // ATA5831_ID_PATCH_SPI                 (0x0FU)
    0x0000U,                                            //                                      (0x10U)
    0x0000U,                                            //                                      (0x11U)
    ATA_moduleTestSpiGetRomVersion_flash_C,             // ATA5831_ID_GET_ROM_VERSION           (0x12U)
    ATA_moduleTestSpiGetFlashVersion_flash_C,           // ATA5831_ID_GET_FLASH_VERSION         (0x13U)
    ATA_moduleTestSpiCustomCommand_flash_C,             // ATA5831_ID_CUSTOM_COMMAND            (0x14U)
    ATA_moduleTestSpiSystemReset_flash_C,               // ATA5831_ID_SYSTEM_RESET              (0x15U)
    ATA_moduleTestSpiTriggerEepSecureWrite_flash_C,     // ATA5831_ID_TRIGGER_EEP_SECURE_WRITE  (0x16U)
    ATA_moduleTestSpiSetVoltageMonitor_flash_C,         // ATA5831_ID_SET_VOLTAGE_MONITOR       (0x17U)
    ATA_moduleTestSpiOffCommand_flash_C,                // ATA5831_ID_OFF_COMMAND               (0x18U)
    ATA_moduleTestSpiReadTemperature_flash_C,           // ATA5831_ID_READ_TEMPERATURE          (0x19U)
    ATA_moduleTestSpiInitSramService_flash_C,           // ATA5831_ID_INIT_SRAM_SERVICE         (0x1AU)
    ATA_moduleTestSpiStartRssiMeasurement_flash_C,      // ATA5831_ID_START_RSSI_MEAS           (0x1BU)
    ATA_moduleTestSpiGetRssiValue_flash_C,              // ATA5831_ID_GET_RSSI_VALUE            (0x1CU)
    ATA_moduleTestSpiCloseSpiMaster_flash_C,            // (0x1D)
    ATA_moduleTestSpiNotInitialized_flash_C             // (0x1E)

};

#define BASIC_SPI_FUNCTION_TELEGRAM_LENGTH  5U
/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpi_C</b>
    is the entry point for spi module test. The spi module test checks all
    spi commands in normal mode and checksum mode. Switching between the spi
    command modes is done via g_sModuleTest_flash.bId
    - g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_WITH_INTERRUPTS: normal mode
    - g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_NO_INTERRUPTS: checksum mode
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpi_flash_C(void)
{
    moduleTestFunc fpFunc = m_fpModuleTestSpi_flash[g_sModuleTest_flash.bSubId];
    if (fpFunc != 0x0000U) {
        fpFunc();
    } else {}
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiCheckEventBytes_C</b>
    checks the content of the event bytes. Simulation returns g_sModuleTest_flash.bId in
    byte[0] and g_sModuleTest_flash.bSubId in byte[1]. This values are stored in
    g_sAta5831.bEventsSystem and g_sAta5831.bEventsEvent respectively.

    \param  result  If the event check doesn't match,
                    the function returns false otherwise true
 */
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_moduleTestSpiCheckEventBytes_flash_C(void)
{
    uint8_t fRetVal = TRUE;
    if (    (g_sAta5831.bEventsSystem != g_sModuleTest_flash.bId   )
         || (g_sAta5831.bEventsEvent  != g_sModuleTest_flash.bSubId)
        )
    {
        fRetVal = FALSE;
    }
    g_sAta5831.bEventsSystem = 0;
    g_sAta5831.bEventsEvent  = 0;
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiCheckWriteCommand_C</b>
    is used to check the event bytes content in spi write commands. If event
    check fails, an error code is set in g_sDebug.bErrorCode and g_sDebug.bSsmErrorCode.
    Furthermore a system error is signalled in ata5700.events_system
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiCheckWriteCommand_flash_C(void)
{
    if (ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE) {
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiClearModuleTestBuffer_C</b>
    clears the g_bModuleTestBuffer_flash after each sub module test
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiClearModuleTestBuffer_flash_C(void)
{
    ATA_globalsInitSramSpace_C(g_bModuleTestBuffer_flash, sizeof(g_bModuleTestBuffer_flash));
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiInitSpiMaster</b>
    does the initialization of module ATA5831 for
    - normal mode       (g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_WITH_INTERRUPTS)
    - checksum mode     (g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_NO_INTERRUPTS)
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiInitSpiMaster_flash_C(void)
{
    ATA_spiClose_C();
    if (g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_WITH_INTERRUPTS) {
        ATA_5831Init_C(BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG, 150U, 150U);
    } else {
        ATA_5831Init_C(0x00U, 150U, 150U);

    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiCloseSpiMaster</b>
    closes module ATA5831 after module test has finished (last entry of
    spi module test look up table).
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiCloseSpiMaster_flash_C(void)
{
    ATA_5831Close_C();
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadFillLevelRxFifo_C</b>
    checks the ATA5831 SPI command 'Read Fill Level Rx FIFO' as follows
    010: send command 'Read Fill Level Rx FIFO' to the slave
    020: wait for NSS rising edge (spi command finished)
    030: check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
         check if fill level is set to 1
    040: in case of wrong return data set error code in variable debug
    050: clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadFillLevelRxFifo_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831ReadFillLevelRxFifo_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));

    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadFillLevelTxFifo_C</b>
    checks the ATA5831 SPI command 'Read Fill Level Tx FIFO' as follows
    010: send command 'Read Fill Level Tx FIFO' to the slave
    020: wait for NSS rising edge (spi command finished)
    030: check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
         check if fill level is set to 1
    040: in case of wrong return data set error code in variable debug
    050: clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadFillLevelTxFifo_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831ReadFillLevelTxFifo_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadFillLevelRssiFifo_C</b>
    checks the ATA5831 SPI command 'Read Fill Level RSSI FIFO' as follows
    010: send command 'Read Fill Level RSSI FIFO' to the slave
    020: wait for NSS rising edge (spi command finished)
    030: check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
         check if fill level is set to 1
    040: in case of wrong return data set error code in variable debug
    050: clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadFillLevelRssiFifo_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831ReadFillLevelRssiFifo_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiGetEventBytes_C</b>
    checks the ATA5831 SPI command 'Get Event Bytes' as follows
    010: send command 'Get Event Bytes' to the slave
    020: wait for NSS rising edge (spi command finished)
    030: check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
         check content of g_sAta5831.bEventsPower (0x01) and g_sAta5831.bEventsConfig(0x02)
    040: in case of wrong return data set error code in variable debug
    050: clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiGetEventBytes_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831GetEventBytes_C();
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_sAta5831.bEventsPower  != 0x01U
         || g_sAta5831.bEventsConfig != 0x02U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadRssiFifo_C</b>
    checks the ATA5831 SPI command 'Read RSSI FIFO' as follows
    010: execute the following test steps MODULE_TEST_BUFFER_SIZE times
    020:    send command 'Read RSSI FIFO' to the slave
    030:    wait for NSS rising edge (spi command finished)
    040:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_sAta5831.bEventsPower(0x01) and g_sAta5831.bEventsConfig(0x02)
    050:    in case of wrong return data set error code in variable debug
    060:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadRssiFifo_flash_C(void)
{
    uint8_t bLoopCnt_flash = (g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_WITH_INTERRUPTS) ? MODULE_TEST_BUFFER_SIZE : 16;
    /* LLR-Ref: 010 */
    for(uint8_t i = 0; i < bLoopCnt_flash; i++) {
        /* LLR-Ref: 020 */
        ATA_5831ReadRssiFifo_C(i+1, g_bModuleTestBuffer_flash);
        /* LLR-Ref: 030 */
        while(!(PINB & BM_SPI_NSS_PIN));
        /* LLR-Ref: 040 */
        if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
             || g_bModuleTestBuffer_flash[i] != 0x02U + i
            )
        {
            /* LLR-Ref: 050 */
            g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
            g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
            ATA_systemSetSystemError_flash_ASM();
        }
        /* LLR-Ref: 060 */
        ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadRxFifo_C</b>
    checks the ATA5831 SPI command 'Read RX FIFO' as follows
    010: execute the following test steps MODULE_TEST_BUFFER_SIZE times
    020:    send command 'Read RX FIFO' to the slave
    030:    wait for NSS rising edge (spi command finished)
    040:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    050:    in case of wrong return data set error code in variable debug
    060:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadRxFifo_flash_C(void)
{
    uint8_t bLoopCnt_flash = (g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_WITH_INTERRUPTS) ? MODULE_TEST_BUFFER_SIZE : 16;
    /* LLR-Ref: 010 */
    for(uint8_t i = 0; i < bLoopCnt_flash; i++) {
        /* LLR-Ref: 020 */
        ATA_5831ReadRxFifo_C(i+1, g_bModuleTestBuffer_flash);
        /* LLR-Ref: 030 */
        while(!(PINB & BM_SPI_NSS_PIN));
        /* LLR-Ref: 040 */
        if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
             || g_bModuleTestBuffer_flash[i] != 0x02U + i
            )
        {
            /* LLR-Ref: 050 */
            g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
            g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
            ATA_systemSetSystemError_flash_ASM();
        }
        /* LLR-Ref: 060 */
        ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiWriteRegisterSram_C</b>
    checks the ATA5831 SPI command 'Write Register/SRAM' as follows
    010:    initialize g_bModuleTestBuffer_flash
    020:    send command 'Write Register/SRAM' to the slave
    030:    wait for NSS rising edge (spi command finished)
    040:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiWriteRegisterSram_flash_C(void)
{
    /* LLR-Ref: 010 */
    for(uint8_t i = 0; i < MODULE_TEST_BUFFER_SIZE; i++) {
        g_bModuleTestBuffer_flash[i] = i<<4 | i;
    }
    /* LLR-Ref: 020 */
    ATA_5831WriteSramRegister_C(0x200, 10, g_bModuleTestBuffer_flash);
    /* LLR-Ref: 030 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 040 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadRegisterSram_C</b>
    checks the ATA5831 SPI command 'Read Register/SRAM' as follows
    010: execute the following test steps MODULE_TEST_BUFFER_SIZE times
    020:    send command 'Read Register/SRAM' to the slave
    030:    wait for NSS rising edge (spi command finished)
    040:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    050:    in case of wrong return data set error code in variable debug
    060:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadRegisterSram_flash_C(void)
{
    uint8_t bLoopCnt_flash = (g_sModuleTest_flash.bId == MODULE_TEST_ID_SPI_WITH_INTERRUPTS) ? MODULE_TEST_BUFFER_SIZE : 16;
    /* LLR-Ref: 010 */
    for(uint8_t i = 0; i < bLoopCnt_flash; i++) {
        /* LLR-Ref: 020 */
        ATA_5831ReadSramRegister_C(0x200, i+1, g_bModuleTestBuffer_flash);
        /* LLR-Ref: 030 */
        while(!(PINB & BM_SPI_NSS_PIN));
        if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
             || g_bModuleTestBuffer_flash[i]   != 0x04U + i
            )
        {
            /* LLR-Ref: 040 */
            g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
            g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
            ATA_systemSetSystemError_flash_ASM();
        }
        /* LLR-Ref: 050 */
        ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiWriteEeprom_C</b>
    checks the ATA5831 SPI command 'Write EEPROM' as follows
    010:    send command 'Write EEPROM' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiWriteEeprom_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831WriteEeprom_C(0x1234, 0xAAU);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadEeprom_C</b>
    checks the ATA5831 SPI command 'Read EEPROM' as follows
    010:    send command 'Read EEPROM' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    040:    in case of wrong return data set error code in variable debug
    050:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadEeprom_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831ReadEeprom_C(0x1234, g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x03U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiWriteTxFifo_C</b>
    checks the ATA5831 SPI command 'Write TX FIFO' as follows
    010:    send command 'Write TX FIFO' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiWriteTxFifo_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831WriteTxFifo_C(4U, g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiWriteTxPreambleFifo_C</b>
    checks the ATA5831 SPI command 'Write TX Preamble FIFO' as follows
    010:    send command 'Write TX Preamble FIFO' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiWriteTxPreambleFifo_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831WriteTxPreambleFifo_C(7U, g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiSetSystemMode_C</b>
    checks the ATA5831 SPI command 'Set System Mode' as follows
    010:    send command 'Set System Mode' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiSetSystemMode_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831SetSystemMode_C(0x11U, 0x22U);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiCalibrateAndCheck_C</b>
    checks the ATA5831 SPI command 'Calibrate and Check' as follows
    010:    send command 'Calibrate and Check' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiCalibrateAndCheck_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831CalibrateAndCheck_C(0x33U, 0x44U);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiPatchSpi_C</b>
    checks the ATA5831 SPI command 'Patch SPI' as follows
    010:    send command 'Patch SPI' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiPatchSpi_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831PatchSpi_C(0x55U);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiGetRomVersion_C</b>
    checks the ATA5831 SPI command 'Get ROM Version' as follows
    010:    send command 'Get ROM Version' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    040:    in case of wrong return data set error code in variable debug
    050:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiGetRomVersion_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831GetRomVersion_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiGetFlashVersion_C</b>
    checks the ATA5831 SPI command 'Get FLASH Version' as follows
    010:    send command 'Get FLASH Version' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    040:    in case of wrong return data set error code in variable debug
    050:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiGetFlashVersion_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831GetFlashVersion_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
         || g_bModuleTestBuffer_flash[1] != 0x02U
         || g_bModuleTestBuffer_flash[2] != 0x03U
         || g_bModuleTestBuffer_flash[3] != 0x04U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiCustomCommand_C</b>
    checks the ATA5831 SPI command 'Custom Command' as follows
    010:    send command 'Custom Command' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiCustomCommand_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831CustomerConfigurableCommand_C();
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiSystemReset_C</b>
    checks the ATA5831 SPI command 'System Reset' as follows
    010:    send command 'System Reset' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiSystemReset_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831SystemReset_C();
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiTriggerEepSecureWrite_C</b>
    checks the ATA5831 SPI command 'Trigger EEPROM Secure Write' as follows
    010:    send command 'Trigger EEPROM Secure Write' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiTriggerEepSecureWrite_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831TriggerEepromSecureWrite_C();
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiSetVoltageMonitor_C</b>
    checks the ATA5831 SPI command 'Set Voltage Monitor' as follows
    010:    send command 'Set Voltage Monitor' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiSetVoltageMonitor_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831SetVoltageMonitor_C(0x66U);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiOffCommand_C</b>
    checks the ATA5831 SPI command 'OFF Command' as follows
    010:    send command 'OFF Command' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiOffCommand_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831OffCommand_C();
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiReadTemperature_C</b>
    checks the ATA5831 SPI command 'Read Temperature' as follows
    010:    send command 'Read Temperature' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    040:    in case of wrong return data set error code in variable debug
    050:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiReadTemperature_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831ReadTemperatureValue_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
         || g_bModuleTestBuffer_flash[1] != 0x02U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiInitSramService_C</b>
    checks the ATA5831 SPI command 'Init SRAM Service' as follows
    010:    send command 'Init SRAM Service' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiInitSramService_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831InitSramService_C(0x77U, 0x88U);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiStartRssiMeasurement_C</b>
    checks the ATA5831 SPI command 'Start RSSI Measurement' as follows
    010:    send command 'Start RSSI Measurement' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check write command via function ATA_moduleTestSpiCheckWriteCommand_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiStartRssiMeasurement_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831StartRssiMeasurement_C(0x99U);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    ATA_moduleTestSpiCheckWriteCommand_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiGetRssiValue_C</b>
    checks the ATA5831 SPI command 'Get RSSI Value' as follows
    010:    send command 'Get RSSI Value' to the slave
    020:    wait for NSS rising edge (spi command finished)
    030:    check event bytes via function ATA_moduleTestSpiCheckEventBytes_C
            check content of g_bModuleTestBuffer_flash
    040:    in case of wrong return data set error code in variable debug
    050:    clear g_bModuleTestBuffer_flash via function ATA_moduleTestSpiClearModuleTestBuffer_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiGetRssiValue_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831GetRssiValue_C(g_bModuleTestBuffer_flash);
    /* LLR-Ref: 020 */
    while(!(PINB & BM_SPI_NSS_PIN));
    /* LLR-Ref: 030 */
    if (    ATA_moduleTestSpiCheckEventBytes_flash_C() == FALSE
         || g_bModuleTestBuffer_flash[0] != 0x01U
         || g_bModuleTestBuffer_flash[1] != 0x02U
        )
    {
        /* LLR-Ref: 040 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
    /* LLR-Ref: 050 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiBasicFunctions_flash_C</b>
    checks the SPI low level function of module spi
        - ATA_spiOpen_C
        - ATA_spiClose_C
        - ATA_spiDataExchange_C
        - ATA_spiTransfer_C
        - ATA_spiSelect_C
        - ATA_spiDeselect_C
    010:    initialize TxData with counter value 0..4
    020:    close and reopen SPI using ATA_spiOpen_C
    030:    check g_sModuleTest_flash.bSubId
    040:    IF g_sModuleTest_flash.bSubId is set to MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_TRANSFER
                transfer data via command ATA_spiDataExchange_C
    050:    ELSE IF g_sModuleTest_flash.bSubId is set to MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_EXCHANGE
                transfer data via commands ATA_spiSelect_C, ATA_spiDataExchange_C and ATA_spiDeselect_C
    060:    ELSE IF g_sModuleTest_flash.bSubId is set to MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_TRANSFER_FAIL
                close SPI and try to transfer data via command ATA_spiTransfer_C and check return value for FAIL.
                check error code for DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED
    070:    ELSE IF g_sModuleTest_flash.bSubId is set to MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_OPEN_FAIL
                try to open SPI via command ATA_spiOpen_C and check return value for FAIL
                check error code for DEBUG_ERROR_CODE_SPI_ALREADY_OPENED
    080:    ELSE do nothing due to a invalid g_sModuleTest_flash.bSubId
    090:    check RxData for value 0..4 for following g_sModuleTest_flash.bSubId
                - MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_TRANSFER
                - MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_EXCHANGE
    100:    close SPI using ATA_spiClose_C
    110:    clear g_bModuleTestBuffer_flash
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiBasicFunctions_flash_C(void)
{
    uint8_t *pTxData = &g_bModuleTestBuffer_flash[0];
    uint8_t *pRxData = &g_bModuleTestBuffer_flash[BASIC_SPI_FUNCTION_TELEGRAM_LENGTH];

    /* LLR-Ref: 010 */
    for(int i = 0; i < BASIC_SPI_FUNCTION_TELEGRAM_LENGTH; i++) {
        g_bModuleTestBuffer_flash[i] = i;
    }
    /* LLR-Ref: 020 */
    ATA_spiClose_C();
    ATA_spiOpen_C((BM_SPE | BM_MSTR | BM_SPR0),0U);

    /* LLR-Ref: 030 */
    if (g_sModuleTest_flash.bSubId == MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_TRANSFER) {
        /* LLR-Ref: 040 */
        ATA_spiTransfer_C(pTxData, pRxData, BASIC_SPI_FUNCTION_TELEGRAM_LENGTH);
    } else if (g_sModuleTest_flash.bSubId == MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_EXCHANGE) {
        /* LLR-Ref: 050 */
        ATA_spiSelect_C();
        for(int i = 0; i < BASIC_SPI_FUNCTION_TELEGRAM_LENGTH; i++) {
            *pRxData++ = ATA_spiDataExchange_C(*pTxData++);
        }
        ATA_spiDeselect_C();
    } else if (g_sModuleTest_flash.bSubId == MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_TRANSFER_FAIL) {
        /* LLR-Ref: 060 */
        ATA_spiClose_C();
        if (   (ATA_spiTransfer_C(pTxData, pRxData, BASIC_SPI_FUNCTION_TELEGRAM_LENGTH) == FAIL)
            && (g_sDebug.bErrorCode == DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED)
            ) {
            ATA_globalsInitDebug_C();
        } else {
            g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
            g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
            ATA_systemSetSystemError_flash_ASM();
        }

    } else if (g_sModuleTest_flash.bSubId == MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_OPEN_FAIL) {
        /* LLR-Ref: 070 */
        if (   (ATA_spiOpen_C((BM_SPE | BM_MSTR | BM_SPR0),0U) == FAIL)
            && (g_sDebug.bErrorCode == DEBUG_ERROR_CODE_SPI_ALREADY_OPENED)
           ){
            ATA_globalsInitDebug_C();
        } else {
            g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
            g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
            ATA_systemSetSystemError_flash_ASM();
        }

    } else {
        /* LLR-Ref: 080 */
    }

    /* LLR-Ref: 090 */
    if (   (g_sModuleTest_flash.bSubId == MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_TRANSFER)
        || (g_sModuleTest_flash.bSubId == MODULE_TEST_SPI_BASIC_FUNCTIONS_SUBID_EXCHANGE)
    ) {

        for(int i = 0; i < BASIC_SPI_FUNCTION_TELEGRAM_LENGTH; i++) {
            if (g_bModuleTestBuffer_flash[BASIC_SPI_FUNCTION_TELEGRAM_LENGTH + i] != i) {
                g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
                g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
                ATA_systemSetSystemError_flash_ASM();
                break;
            } else {
            }
        }
    }
    /* LLR-Ref: 100 */
    ATA_spiClose_C();
    /* LLR-Ref: 110 */
    ATA_moduleTestSpiClearModuleTestBuffer_flash_C();

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiNotInitializedCheckError_flash_C</b>
    checks for g_sDebug.bErrorCode == DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED
    010:    wait for NSS rising edge (spi command finished)
    020:    checks for g_sDebug.bErrorCode == DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED
            if error code is set clear variable g_sDebug via function ATA_globalsInitDebug_C
    030:    otherwise set system error and error code
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiNotInitializedCheckError_flash_C(void)
{
    /* LLR-Ref: 010 */
    while(!(PINB & BM_SPI_NSS_PIN));
    if (g_sDebug.bErrorCode == DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED){
        /* LLR-Ref: 020 */
        ATA_globalsInitDebug_C();
    } else {
        /* LLR-Ref: 030 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_SPI_MODULE_TEST;
        g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
        ATA_systemSetSystemError_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_moduleTestSpiNotInitialized_flash_C</b>
    checks the error handling of ATA5831 command set if SPI is not initialized
    010: check error handling of function ATA_5831ReadFillLevelRxFifo_C
    020: check error handling of function ATA_5831ReadFillLevelTxFifo_C
    030: check error handling of function ATA_5831ReadFillLevelRssiFifo_C
    040: check error handling of function ATA_5831GetEventBytes_C
    050: check error handling of function ATA_5831ReadRssiFifo_C
    060: check error handling of function ATA_5831ReadRxFifo_C
    070: check error handling of function ATA_5831WriteSramRegister_C
    080: check error handling of function ATA_5831ReadSramRegister_C
    090: check error handling of function ATA_5831WriteEeprom_C
    100: check error handling of function ATA_5831ReadEeprom_C
    110: check error handling of function ATA_5831WriteTxFifo_C
    120: check error handling of function ATA_5831WriteTxPreambleFifo_C
    130: check error handling of function ATA_5831SetSystemMode_C
    140: check error handling of function ATA_5831CalibrateAndCheck_C
    150: check error handling of function ATA_5831PatchSpi_C
    160: check error handling of function ATA_5831GetRomVersion_C
    170: check error handling of function ATA_5831GetFlashVersion_C
    180: check error handling of function ATA_5831CustomerConfigurableCommand_C
    190: check error handling of function ATA_5831SystemReset_C
    200: check error handling of function ATA_5831TriggerEepromSecureWrite_C
    210: check error handling of function ATA_5831SetVoltageMonitor_C
    220: check error handling of function ATA_5831OffCommand_C
    230: check error handling of function ATA_5831ReadTemperatureValue_C
    240: check error handling of function ATA_5831InitSramService_C
    250: check error handling of function ATA_5831StartRssiMeasurement_C
    260: check error handling of function ATA_5831GetRssiValue_C
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_moduleTestSpiNotInitialized_flash_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831ReadFillLevelRxFifo_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 020 */
    ATA_5831ReadFillLevelTxFifo_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 030 */
    ATA_5831ReadFillLevelRssiFifo_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 040 */
    ATA_5831GetEventBytes_C();
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 050 */
    ATA_5831ReadRssiFifo_C(1U, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 060 */
    ATA_5831ReadRxFifo_C(1U, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 070 */
    ATA_5831WriteSramRegister_C(0x200, 10, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 080 */
    ATA_5831ReadSramRegister_C(0x200, 1U, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 090 */
    ATA_5831WriteEeprom_C(0x1234, 0xAAU);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 100 */
    ATA_5831ReadEeprom_C(0x1234, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 110 */
    ATA_5831WriteTxFifo_C(4U, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 120 */
    ATA_5831WriteTxPreambleFifo_C(7U, g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 130 */
    ATA_5831SetSystemMode_C(0x11U, 0x22U);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 140 */
    ATA_5831CalibrateAndCheck_C(0x33U, 0x44U);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 150 */
    ATA_5831PatchSpi_C(0x55U);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 160 */
    ATA_5831GetRomVersion_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 170 */
    ATA_5831GetFlashVersion_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 180 */
    ATA_5831CustomerConfigurableCommand_C();
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 190 */
    ATA_5831SystemReset_C();
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 200 */
    ATA_5831TriggerEepromSecureWrite_C();
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 210 */
    ATA_5831SetVoltageMonitor_C(0x66U);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 220 */
    ATA_5831OffCommand_C();
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 230 */
    ATA_5831ReadTemperatureValue_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 240 */
    ATA_5831InitSramService_C(0x77U, 0x88U);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 250 */
    ATA_5831StartRssiMeasurement_C(0x99U);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

    /* LLR-Ref: 260 */
    ATA_5831GetRssiValue_C(g_bModuleTestBuffer_flash);
    ATA_moduleTestSpiNotInitializedCheckError_flash_C();

}

