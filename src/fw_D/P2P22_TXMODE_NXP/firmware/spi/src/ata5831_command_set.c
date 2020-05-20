//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/spi/src/ata5831_command_set.c $
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
/** \file ata5831_command_set.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "ata5831_command_set.h"
#include "spi.h"
#include "../../timer1/src/timer1.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/** \brief <b>g_sAta5831</b>
    contains the ATA5831 specific SPI settings.
*/
#pragma location = ".ata5831"
__no_init sAta5831CommandSet g_sAta5831;

/** \brief <b>m_bAta5831Buffer</b>
    contains the ATA5831 specific SPI header information.
*/
#pragma location = ".ata5831"
static __no_init uint8_t m_bAta5831Buffer[3];

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

static UINT8FUNC ATA_5831SpiDataExchangeWithChecksum_C(uint8_t bData);
static VOIDFUNC ATA_5831DoCheckSumProcessing_C(void);

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831GetRssiValue_C</b>
    shall read the current Rssi average and peak values from the ATA5831 via
    SPI interface.

    \param[out]     pData           ATA5831 response data

    Variable Usage:
    \li [in] ::g_sSpiConfig Global SPI component data
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_5831GetRssiValue_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 .bConfig[0]
                goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and return

    \li 040: Start SPI command by setting NSS to low via function ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function ::ATA_5831SpiDataExchangeWithChecksum_C
             read back the event bytes via SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-984,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831GetRssiValue_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831GetRssiValue_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {   /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_GET_RSSI_VALUE;
            g_sAta5831.bTelegramLength = 4U;
            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        }
        else {                  /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_GET_RSSI_VALUE | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
        g_sDebug.bSsmErrorCode = 0x00U;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831StartRssiMeasurement_C</b>
    shall start a Rssi measurement on the ATA5831 via SPI interface.

    \param[in] bConfig Service/Channel configuration for RSSI Measurement

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data 
    
    \image html ATA_5831StartRssiMeasurement_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 .bConfig[0]
                goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and return
    \li 040: Start the SPI command by setting NSS to low via function ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function ::ATA_5831SpiDataExchangeWithChecksum_C
             read back the event bytes via SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-983,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831StartRssiMeasurement_C(svcChConfig_t bConfig)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831StartRssiMeasurement_C, g_sAta5831.bConfig);

    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bConfig;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            /* set command.type to read for correct reading of events */
            /* ---Primus2P-2917--- */
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            /* ---Primus2P-2917--- */
            g_sAta5831.bId             = ATA5831_ID_START_RSSI_MEAS;
            g_sAta5831.bTelegramLength = 2U;
            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        }
        else {                  /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_START_RSSI_MEAS | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bConfig);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831InitSramService_C</b>
    shall initialize a SRAM service with EEPROM content on ATA5831

    \param[in]      bSramSvc            SRAM service number
    \param[in]      bEepromSvc          EEPROM service number

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831InitSramService_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 .bConfig[0]
                goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and return
    \li 040: Start the SPI command by setting NSS to low via function ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function ::ATA_5831SpiDataExchangeWithChecksum_C
             read back the event bytes via SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-982,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831InitSramService_C(uint8_t bSramSvc, uint8_t bEepromSvc)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831InitSramService_C, g_sAta5831.bConfig);

    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bSramSvc;
            m_bAta5831Buffer[1]        = bEepromSvc;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_INIT_SRAM_SERVICE;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        }
        else {                  /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_INIT_SRAM_SERVICE | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bSramSvc);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bEepromSvc);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadTemperatureValue_C</b>
    shall read the currently available temperature value on ATA5831

    \param[out]     pData           Temperature value reference
    
    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831ReadTemperatureValue_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 .bConfig[0]
                goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and return
    \li 040: Start the SPI command by setting NSS to low via function ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function ::ATA_5831SpiDataExchangeWithChecksum_C
             read back the event bytes via SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-981,Primus2P-989}

    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadTemperatureValue_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadTemperatureValue_C, g_sAta5831.bConfig);

    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_READ_TEMPERATURE;
            g_sAta5831.bTelegramLength = 4U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        }
        else {                  /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_TEMPERATURE | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831OffCommand_C</b>
    shall switch the ATA5831 in OFF mode via SPI interface

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831OffCommand_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 .bConfig[0]
                goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and return
    \li 040: Start the SPI command by setting NSS to low via function ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function ::ATA_5831SpiDataExchangeWithChecksum_C
             read back the event bytes via SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-980,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831OffCommand_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831OffCommand_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = 0x00U;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            /* set command.type to read for correct reading of events */
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_OFF_COMMAND;
            g_sAta5831.bTelegramLength = 2U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        }
        else {                  /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_OFF_COMMAND | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831SetVoltageMonitor_C</b>
    shall configure the voltage monitor of ATA5831 via SPI interface

    \param[in]      bValue              Voltage Monitor configuration

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831SetVoltageMonitor_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-979,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831SetVoltageMonitor_C(uint8_t bValue)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831SetVoltageMonitor_C, g_sAta5831.bConfig);

    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bValue;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            /* set patch spi command.type to read for correct reading of events */
            /* ---Primus2P-2916--- */
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_SET_VOLTAGE_MONITOR;
            /* ---Primus2P-2916--- */
            g_sAta5831.bTelegramLength = 2U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_SET_VOLTAGE_MONITOR | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bValue);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831TriggerEepromSecureWrite_C</b>
    shall start a secure eeprom write mechanism on ATA5831 via SPI interface

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831TriggerEepromSecureWrite_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
            ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
            ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
            return
    \li 040: Start the SPI command by setting NSS to low via function 
            ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
            ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
            SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
            and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
            ::g_sDebug .bErrorCode
    
    \Derived{No}
    
    \Rationale{N/A}
    
    \Traceability{Primus2P-978,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831TriggerEepromSecureWrite_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831TriggerEepromSecureWrite_C, g_sAta5831.bConfig);

    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = 0xAAU;
            m_bAta5831Buffer[1]        = 0xCCU;
            m_bAta5831Buffer[2]        = 0xF0U;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_TRIGGER_EEP_SECURE_WRITE;
            g_sAta5831.bTelegramLength = 4U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_TRIGGER_EEP_SECURE_WRITE | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0xAAU);
                                       ATA_5831SpiDataExchangeWithChecksum_C(0xCCU);
                                       ATA_5831SpiDataExchangeWithChecksum_C(0xF0U);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831SystemReset_C</b>
    shall reset the ATA5831 via SPI interface

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831SystemReset_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-977,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831SystemReset_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831SystemReset_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = 0U;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            /* set patch spi command.type to read for correct reading of events */
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_SYSTEM_RESET;
            g_sAta5831.bTelegramLength = 2U;
            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;

            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);
            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_SYSTEM_RESET | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831CustomerConfigurableCommand_C</b>
    shall send the SPI Command "Customer Configurable Command" to ATA5831

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831CustomerConfigurableCommand_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-976,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831CustomerConfigurableCommand_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831CustomerConfigurableCommand_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = 0U;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            /* set patch spi command.type to read for correct reading of events */
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_CUSTOM_COMMAND;
            g_sAta5831.bTelegramLength = 2U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_CUSTOM_COMMAND | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831PatchSpi_C</b>
    shall send the SPI Command "Patch SPI" to ATA5831

    \param[in]      bValue              SPI command parameter

    Variable Usage>
    \li [in,out] ::m_bAta5831Buffer  Global ATA5831 SPI header information
    \li [out] ::g_sAta5831        Global ATA5831 component data
    \li [out] ::g_sDebug          Global Debug component data
    \li [in] ::g_sSpiConfig      Global SPI component data

    \image html ATA_5831PatchSpi_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-975,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831PatchSpi_C(uint8_t bValue)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831PatchSpi_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bValue;

            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            /* set command.type to read for correct reading of events */
            /* ---Primus2P-2915--- */
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            /* ---Primus2P-2915--- */
            g_sAta5831.bId             = ATA5831_ID_PATCH_SPI;
            g_sAta5831.bTelegramLength = 2U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_PATCH_SPI | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bValue);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831CalibrateAndCheck_C</b>
    shall trigger a tune and check action on ATA5831

    \param[in]      bTuneCheckConfig        Action to be triggered
    \param[in]      bServiceChannelConfig   Service/Channel configuration
    
    Variable Usage>
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in]  ::g_sSpiConfig Global SPI component data

    \image html ATA_5831CalibrateAndCheck_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-974,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831CalibrateAndCheck_C(
    tuneCheckConfig_t bTuneCheckConfig,
    svcChConfig_t bServiceChannelConfig
)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831CalibrateAndCheck_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bTuneCheckConfig;
            m_bAta5831Buffer[1]        = bServiceChannelConfig;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_CALIBRATE_AND_CHECK;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_CALIBRATE_AND_CHECK | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bTuneCheckConfig);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bServiceChannelConfig);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831SetSystemMode_C</b>
    shall trigger a system mode switching on ATA5831

    \param[in]      bSystemModeConfig       Action to be triggered
    \param[in]      bServiceChannelConfig   Service/Channel configuration

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data
    
    \image html ATA_5831SetSystemMode_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-973,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831SetSystemMode_C(
    sysModeConfig_t bSystemModeConfig,
    svcChConfig_t bServiceChannelConfig
)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831SetSystemMode_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bSystemModeConfig;
            m_bAta5831Buffer[1]        = bServiceChannelConfig;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;

            g_sAta5831.bId             = ATA5831_ID_SET_SYSTEM_MODE;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_SET_SYSTEM_MODE | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bSystemModeConfig);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bServiceChannelConfig);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831WriteTxPreambleFifo_C</b>
    shall send Preamble data to the ATA5831 via SPI interface.

    \param[in] pData Pointer to preamble data
    \param[in] bLen Preamble data length

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831WriteTxPreambleFifo_C.png

    \internal
    \li 005: IF SPI initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-972,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831WriteTxPreambleFifo_C(uint8_t bLen, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831WriteTxPreambleFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_LENGTH_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_WRITE_TX_PREAMBLE_FIFO;
            g_sAta5831.bLength         = bLen;
            g_sAta5831.bTelegramLength = g_sAta5831.bLength + 2U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_WRITE_TX_PREAMBLE_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bLen);
            for(uint8_t i = 0; i< bLen; i++) {
                 ATA_5831SpiDataExchangeWithChecksum_C(*pData++);
            }
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831WriteTxFifo_C</b>
    shall send Tx data to the ATA5831 via SPI interface.
    
    \param[in]      pData                   Pointer to Tx data
    \param[in]      bLen                    Tx data length

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831WriteTxFifo_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-971,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831WriteTxFifo_C(uint8_t bLen, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831WriteTxFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_LENGTH_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;

            g_sAta5831.bId             = ATA5831_ID_WRITE_TX_FIFO;
            g_sAta5831.bLength         = bLen;
            g_sAta5831.bTelegramLength = g_sAta5831.bLength + 2U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_WRITE_TX_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bLen);
            for(uint8_t i = 0; i< bLen; i++) {
                 ATA_5831SpiDataExchangeWithChecksum_C(*pData++);
            }
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadEeprom_C</b>
    shall read eeprom content from ATA5831 via SPI interface.

    \param[out]     pData                   Data read from ATA5831 EEPROM
    \param[in]      wAddress                ATA5831 EEPROM Address to be read from 

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831ReadEeprom_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-970,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadEeprom_C(uint16_t wAddress, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadEeprom_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_ADDRESS_INDICATOR
                                        | BM_SATA5831_CONFIG_DUMMY_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_READ_EEPROM;
            g_sAta5831.bLength         = 1U;
            /* ---Primus2P-2922--- */
            g_sAta5831.wAddress        = wAddress;
            /* ---Primus2P-2922--- */
            g_sAta5831.bTelegramLength = 5U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            uint8_t bAddressHigh = (uint8_t)(wAddress >> 8U);
            uint8_t bAddressLow  = (uint8_t)(wAddress & 0x00FFU);

            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_EEPROM | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bAddressHigh);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bAddressLow);
                                       ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            *pData                   = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831WriteEeprom_C</b>
    shall write data to ATA5831 eeprom via SPI interface.

    \param[in]      wAddress                ATA5831 EEPROM address to write to 
    \param[in]      bData                   Data to write to ATA5831 EEPROM

    Variable Usage:
    \li [in,out] ::m_bAta5831Buffer Global ATA5831 SPI header information
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831WriteEeprom_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-969,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831WriteEeprom_C(uint16_t wAddress, uint8_t bData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831WriteEeprom_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            m_bAta5831Buffer[0]        = bData;
            g_sAta5831.pDataPtr        = m_bAta5831Buffer;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_ADDRESS_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_WRITE_EEPROM;
            g_sAta5831.wAddress        = wAddress;
            g_sAta5831.bTelegramLength = 4U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            uint8_t bAddressHigh = (uint8_t)(wAddress >> 8U);
            uint8_t bAddressLow  = (uint8_t)(wAddress & 0x00FFU);

            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_WRITE_EEPROM | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bAddressHigh);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bAddressLow);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bData);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadSramRegister_C</b>
    shall read data from ATA5831 sram or register via SPI interface.

    \param[out]     pData                   SRAM/Register data read from ATA5831
    \param[in]      wAddress                ATA5831 SRAM/Register start address to be read from
    \param[in]      bLen                    Number of SRAM/Register bytes to read from ATA5831

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831ReadSramRegister_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-968,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadSramRegister_C(uint16_t wAddress, uint8_t bLen, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadSramRegister_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_ADDRESS_INDICATOR
                                        | BM_SATA5831_CONFIG_LENGTH_INDICATOR
                                        | BM_SATA5831_CONFIG_DUMMY_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_READ_REGISTER_SRAM;
            g_sAta5831.bLength         = bLen;
            /* ---Primus2P-2924--- */
            g_sAta5831.wAddress        = wAddress;
            /* ---Primus2P-2924--- */
            g_sAta5831.bTelegramLength = g_sAta5831.bLength + 5U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            uint8_t bAddressHigh = (uint8_t)(wAddress >> 8U);
            uint8_t bAddressLow  = (uint8_t)(wAddress & 0x00FFU);

            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_REGISTER_SRAM | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bLen);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bAddressHigh);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bAddressLow);
                                       ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            for(uint8_t i = 0; i< bLen; i++) {
                 *pData++ = ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            }
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831WriteSramRegister_C</b>
    shall write data to ATA5831 sram or register via SPI interface.

    \param[in] wAddress ATA5831 SRAM/Register start address to be written to
    \param[in] bLen Number of SRAM/Register bytes to write
    \param[in] pData SRAM/Register data to write

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831WriteSramRegister_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-967,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831WriteSramRegister_C(uint16_t wAddress, uint8_t bLen, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831WriteSramRegister_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_READ_WRITE_INDICATOR
                                        | BM_SATA5831_CONFIG_LENGTH_INDICATOR
                                        | BM_SATA5831_CONFIG_ADDRESS_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;

            g_sAta5831.bId             = ATA5831_ID_WRITE_REGISTER_SRAM;
            g_sAta5831.wAddress        = wAddress;
            g_sAta5831.bLength         = bLen;
            g_sAta5831.bTelegramLength = g_sAta5831.bLength + 4U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            uint8_t bAddressHigh = (uint8_t)(wAddress >> 8U);
            uint8_t bAddressLow  = (uint8_t)(wAddress & 0x00FFU);

            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_WRITE_REGISTER_SRAM | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bLen);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bAddressHigh);
                                       ATA_5831SpiDataExchangeWithChecksum_C(bAddressLow);

            for(uint8_t i = 0; i< bLen; i++) {
                 ATA_5831SpiDataExchangeWithChecksum_C(*pData++);
            }
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadRxFifo_C</b>
    shall read Rx data from the ATA5831 via SPI interface.

    \param[out] pData Pointer to where the Rx FIFO data shall be stored
    \param[in] bLen Number of bytes to be read from Rx FIFO

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831ReadRxFifo_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable 
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-966,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadRxFifo_C(uint8_t bLen, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadRxFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_LENGTH_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG
                                        | BM_SATA5831_CONFIG_DUMMY_INDICATOR;

            g_sAta5831.bId             = ATA5831_ID_READ_RX_FIFO;
            g_sAta5831.bLength         = bLen;
            g_sAta5831.bTelegramLength = g_sAta5831.bLength + 3U;
            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_RX_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bLen);
                                       ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            for(uint8_t i = 0; i< bLen; i++) {
                *pData++ = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            }
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadFillLevelRxFifo_C</b>
    shall read the fill level of DFIFO of ATA5831 via SPI interface.

    \param[out]     pData                   Pointer to where the Rx FIFO Fill Level shall be stored

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831ReadFillLevelRxFifo_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-961,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadFillLevelRxFifo_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadFillLevelRxFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;

            g_sAta5831.bId             = ATA5831_ID_READ_FILL_LEVEL_RX_FIFO;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem   = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_FILL_LEVEL_RX_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent    = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            *pData++                   = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadFillLevelTxFifo_C</b>
    shall read the fill level of DFIFO of ATA5831 via SPI interface.

    \param[out]     pData                   Pointer to where the Tx FIFO Fill Level shall be stored

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data
    
    \image html ATA_5831ReadFillLevelTxFifo_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return

    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-962,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadFillLevelTxFifo_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadFillLevelTxFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_READ_FILL_LEVEL_TX_FIFO;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem   = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_FILL_LEVEL_TX_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent    = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            *pData++                   = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadFillLevelRssiFifo_C</b>
    shall read the fill level of SFIFO of ATA5831 via SPI interface.

    \param[out]     pData                   Pointer to where the RSSI FIFO Fill Level shall be stored

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831ReadFillLevelRssiFifo_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-963,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadFillLevelRssiFifo_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadFillLevelRssiFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_READ_FILL_LEVEL_RSSI_FIFO;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem   = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_FILL_LEVEL_RSSI_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent    = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            *pData++                = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831GetEventBytes_C</b>
    shall read the event bytes from the ATA5831 via SPI interface.

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831GetEventBytes_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPi functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-964,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831GetEventBytes_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831GetEventBytes_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = &g_sAta5831.bEventsPower;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_GET_EVENT_BYTES;
            g_sAta5831.bTelegramLength = 4U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_GET_EVENT_BYTES | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            g_sAta5831.bEventsPower  = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            g_sAta5831.bEventsConfig = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831ReadRssiFifo_C</b>
    shall read Rssi data from the ATA5831 via SPI interface.

    \param[out]     pData                   Pointer to where RSSI/Preamble buffer data shall be stored
    \param[in]      bLen                    Number of bytes to read from the RSSI/Preamble buffer

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data
    
    \image html ATA_5831ReadRssiFifo_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-965,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831ReadRssiFifo_C(uint8_t bLen, uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831ReadRssiFifo_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         =  BM_SATA5831_CONFIG_LENGTH_INDICATOR
                                        | BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG
                                        | BM_SATA5831_CONFIG_DUMMY_INDICATOR;
            g_sAta5831.bId             = ATA5831_ID_READ_RSSI_FIFO;
            g_sAta5831.bLength         = bLen;
            g_sAta5831.bTelegramLength = g_sAta5831.bLength + 3U;
            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_READ_RSSI_FIFO | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(bLen);
                                    ATA_5831SpiDataExchangeWithChecksum_C(0x00U);
            for(uint8_t i = 0; i< bLen; i++) {
                *pData++ = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            }
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831GetRomVersion_C</b>
    shall read the rom version information from the ATA5831 via SPI interface.

    \param[out]     pData                   Pointer to where the ROM version of ATA5831 shall be stored

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data
    
    \image html ATA_5831GetRomVersion_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831 
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-959,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831GetRomVersion_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831GetRomVersion_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr        = pData;
            g_sAta5831.bConfig         = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId             = ATA5831_ID_GET_ROM_VERSION;
            g_sAta5831.bTelegramLength = 3U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_GET_ROM_VERSION | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00);
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831GetFlashVersion_C</b>
    shall read the flash version information from the ATA5831 via SPI interface.

    \param[out] pData Pointer to where the FLASH version of ATA5831 shall be stored

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data

    \image html ATA_5831GetFlashVersion_C.png

    \internal
    \li 005: IF SPI is initialized (check with flag ::g_sSpiConfig .bStatus[7])
                continue
             ELSE
                goto 090
    \li 010: IF SPI functionality with interrupt usage is selected in ::g_sAta5831
                .bConfig[0] goto 020
             ELSE
                goto 040
    \li 020: Configure data structure ::g_sAta5831
    \li 030: Start SPI communication by calling function ::ATA_5831Start_C and 
             return
    \li 040: Start the SPI command by setting NSS to low via function 
             ::ATA_spiSelect_C
    \li 050: Wait ::g_sAta5831 .bNssLowSckDelay us before starting data transmission
    \li 060: Write ID and read data bytes via SPI using function 
             ::ATA_5831SpiDataExchangeWithChecksum_C read back the event bytes via 
             SPI and store it in variable
                - ::g_sAta5831 .bEventsSystem
                - ::g_sAta5831 .bEventsEvent
    \li 070: Wait ::g_sAta5831 .bSckNssHighDelay us after last data transmission
    \li 080: Close SPI communication be setting NSS to high via ::ATA_spiDeselect_C 
             and return
    \li 090: Signal error code DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED via variable 
             ::g_sDebug .bErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-960,Primus2P-989}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831GetFlashVersion_C(uint8_t *pData)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831GetFlashVersion_C, g_sAta5831.bConfig);
    /* LLR-Ref: 005 */
    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS) {
        /* LLR-Ref: 010 */
        if (g_sAta5831.bConfig & BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG) {    /* with interupt usage */
            /* LLR-Ref: 020 */
            g_sAta5831.pDataPtr         = pData;
            g_sAta5831.bConfig          = BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG;
            g_sAta5831.bId              = ATA5831_ID_GET_FLASH_VERSION;
            g_sAta5831.bTelegramLength  = 6U;

            /* LLR-Ref: 030 */
            ATA_5831Start_C();
        } else {                /* spi checksum usage enabled */
            g_sAta5831.bChecksumMiso = g_sAta5831.bChecksumMosi = 0x00U;
            /* LLR-Ref: 040 */
            ATA_spiSelect_C();

            /* LLR-Ref: 050 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bNssLowSckDelay);

            /* LLR-Ref: 060 */
            g_sAta5831.bEventsSystem = ATA_5831SpiDataExchangeWithChecksum_C(ATA5831_ID_GET_FLASH_VERSION | BIT_MASK_7);
            g_sAta5831.bEventsEvent  = ATA_5831SpiDataExchangeWithChecksum_C(0x00);

            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00); /* rom version */
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00); /* flash version high byte */
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00); /* flash version low byte */
            *pData++                 = ATA_5831SpiDataExchangeWithChecksum_C(0x00); /* customer specific version */
            ATA_5831DoCheckSumProcessing_C();

            /* LLR-Ref: 070 */
            ATA_globalsWaitNus_ASM(g_sAta5831.bSckNssHighDelay);

            /* LLR-Ref: 080 */
            ATA_spiDeselect_C();
        }
    }
    else {
        /* LLR-Ref: 090 */
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831Init_C</b>
    shall shall initialize the variable g_sAta5831
    \param[in] bConfig ATA5831 component configuration 
    \param[in] bNssSck Defines the delay between falling edge of NSS and beginning of first SCK cycle in MRC cycles
    \param[in] bSckNss Defines the delay between last SCK cycle and rising edge of NSS in MRC cycles

    Variable Usage:
    \li [out] ::g_sAta5831 Global ATA5831 component data
   
    \image html ATA_5831Start_C.png

    \internal
    \li 010: Initialize variable ::g_sAta5831 according to the function parameters
    \li 020: Open SPI using function ::ATA_spiOpen_C

    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the ATA5831
               component handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831Init_C(uint8_t bConfig, uint8_t bNssSck, uint8_t bSckNss)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831Init_C, bConfig);
    /* LLR-Ref: 010*/
    g_sAta5831.bConfig          = bConfig;
    g_sAta5831.pDataPtr         = 0x0000U;
    g_sAta5831.bNssLowSckDelay  = bNssSck;
    g_sAta5831.bSckNssHighDelay = bSckNss;
    g_sAta5831.bEventsSystem    = 0x00U;
    g_sAta5831.bEventsEvent     = 0x00U;
    g_sAta5831.bEventsPower     = 0x00U;
    g_sAta5831.bEventsConfig    = 0x00U;
    g_sAta5831.bChecksumMiso    = 0x00U;
    g_sAta5831.bChecksumMosi    = 0x00U;
    
    /* LLR-Ref: 020*/
    ATA_spiOpen_C((BM_SPE | BM_MSTR | BM_SPR0),0U);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831Close_C</b>
    shall close the ATA5831 command set.

    \image html ATA_5831Close_C.png

    \internal
    \li 010: Close the SPI interface using ::ATA_spiClose_C

    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the ATA5831
               component handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831Close_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831Close_C, 0);
    /* LLR-Ref: 010 */
    ATA_spiClose_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831Start_C</b>
    shall start the spi communication with interrupt usage.

    Variable Usage:
    \li [in,out]  ::g_sAta5831            Global ATA5831 component data
    \li [out]     ::g_sSpiConfig          Global SPI component data

    \image html ATA_5831Start_C.png

        Note
        For the SPI Master for ATA5831 to work correctly with e.g. the MRC,
        the timer 1 clock needs to be reassigned to use the MRC as input clock,
        i.e. BM_T1CS1|BM_T1CS0.

    \internal
    \li 010: Initialize ::g_sSpiConfig .pAddress and ::g_sAta5831 .bRxLength for 
             interrupt routines
    \li 020: Set NSS low to indicate SPI communication
    \li 030: Clear SPI interrupt flags in SFFR Register
    \li 040: Enable Timer 1
                - Power up timer 1 by clearing PRR1.PRT1
                - Initialize T1COR register with content of ::g_sAta5831 
                  .bNssLowSckDelay
                - Reset timer 1 by setting T1CR.T1RES
                - Open timer 1 by using function ::ATA_timer1Open_C

    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the ATA5831
               component handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5831Start_C(void)
{
    /* Timer 1 configuration parameters */
    sTimerAsyn8BitParams sTimer1Params;
  
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_5831Start_C, 0);

    /* LLR-Ref: 010 */
    g_sSpiConfig.pAddress = g_sAta5831.pDataPtr;
    g_sAta5831.bRxLength  = g_sAta5831.bTelegramLength;
    
    /* LLR-Ref: 020 */
    PORTD &= (uint8_t)~BM_SPI_NSS_PIN;

    /* LLR-Ref: 030 */
    SFFR  |= BM_TFC | BM_RFC;
    SFIR   = 0x07U;

    /* LLR-Ref: 040 */
    sTimer1Params.ctrl = BM_T1ENA;
    sTimer1Params.mode = BM_T1CS0;
    sTimer1Params.comp = g_sAta5831.bNssLowSckDelay;
    sTimer1Params.irqMask = BM_T1CIM;
    sTimer1Params.ovfIsr = (timerIRQHandler)0x0000;
    sTimer1Params.compIsr = ATA_5831CommandTimer1Isr_ASM;
    
    ATA_timer1Open_C(&sTimer1Params);

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831SpiDataExchangeWithChecksum_C</b>
    sends a byte via SPI interface and returns the response.

    \param[in]      bData                   Data byte to be sent
    \return         Received data byte
    
    Variable Usage:
    \li [in,out] ::g_sAta5831 Global ATA5831 component data

    \image html ATA_5831SpiDataExchangeWithChecksum_C.png

    \internal

    \li 010: Write given byte "bData" to the SPI interface via regiser SPDR and add
             byte to future checksum byte ::g_sAta5831 .bChecksumMosi.

    \li 020: Wait until serial transfer of byte "bData" is completed. 

    \li 030: Read byte from SPI interface via register SPDR and add
             content to received data (until now) in ::g_sAta5831 .bChecksumMiso to 
             compute the SPI checksum

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2532}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static UINT8FUNC ATA_5831SpiDataExchangeWithChecksum_C(uint8_t bData)
{
    uint8_t bMisoData = 0U;
    
    /* LLR-Ref: 010 */
    SPDR = bData;
    g_sAta5831.bChecksumMosi += bData;
    
    /* LLR-Ref: 020 */
    while(!(SPSR & BM_SPIF));
    
    /* LLR-Ref: 030 */
    bMisoData = SPDR;
    g_sAta5831.bChecksumMiso += bMisoData;
    
    return bMisoData;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5831DoCheckSumProcessing_C</b>
    does the checksum processing/calculation. This routine sends the
    inverted checksum on MOSI line and reads the checksum on MISO line
    using function ::ATA_5831SpiDataExchangeWithChecksum_C. If checksum on
    MISO line is not equal to 0xFF an error is signalled by ::g_sDebug.

    Variable Usage:
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sAta5831 Global ATA5831 component data

    \image html ATA_5831DoCheckSumProcessing_C.png

    \internal

    \li 010: Compute SPI checksum for actual SPI data transfer by calling function
             ::ATA_5831SpiDataExchangeWithChecksum_C with the inverted value of
             ::g_sAta5831 .bChecksumMosi as parameter

    \li 020: IF the received SPI checksum value in ::g_sAta5831 .bChecksumMiso is
             not correct, i.e. 0xFFU, then set error condition 
             DEBUG_ERROR_CODE_ATA5831_SPI_CHECKSUM_ERROR in ::g_sDebug .bErrorCode
             and set ::g_sDebug .bSsmErrorCode to 0.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2533}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_5831DoCheckSumProcessing_C(void)
{
    /* LLR-Ref: 010 */
    ATA_5831SpiDataExchangeWithChecksum_C(~g_sAta5831.bChecksumMosi);
    
    /* LLR-Ref: 020 */
    if (g_sAta5831.bChecksumMiso != 0xFFU) {
        g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_ATA5831_SPI_CHECKSUM_ERROR;
        g_sDebug.bSsmErrorCode = 0x00U;
    }
}
