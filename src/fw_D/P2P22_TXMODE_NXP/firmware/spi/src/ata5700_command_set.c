/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/spi/src/ata5700_command_set.c $
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
/** \file ata5700_command_set.c
*/
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "ata5700_command_set.h"
#include "../../calib/src/calib.h"
#include "../../rftx/src/rftx.h"
#include "../../rftx/src/rftx_ant.h"
#include "../../rftx/src/rftx_vco.h"
#include "../../globals/src/globals.h"
#include "../../lfrx/src/lfrx_immo.h"
#include "../../tp/src/tp_flash.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/

/* Must be located at SRAM address 0x100 (global 0x300) */
#pragma location = ".sram_FlashApp_Ata5700CommandSet"
__no_init sAta5700 g_sAta5700_flash;


/** \brief <b>patchSpi</b>
    contains a function pointer for a patched SPI command.
*/
#pragma location = ".sram_FlashApp_Ata5700CommandSet"
__no_init patchSpiFuncPtr patchSpi;

/** \brief <b>extReq</b>
    contains information for controlling external commands/requests.
*/
#pragma location = ".sram_FlashApp_Ata5700CommandSet"
__no_init sExtReq extReq;

/** \brief <b>trxConf</b>
    contains the transceiver configuration information/setting.
*/
#pragma location = ".sram_FlashApp_Ata5700CommandSet"
__no_init sTrxConfig trxConf;

/** \brief <b>g_sRfRemoteKeylessEntryConf</b>
    contains the Remote Keyless Entry configuration
*/
#pragma location = ".sram_FlashApp_Ata5700CommandSet"
__no_init sRfRemoteKeylessEntryConf g_sRfRemoteKeylessEntryConf;


static VOIDFUNC  ATA_5700systemModeSwitching_flash_C(void);
static VOIDFUNC  ATA_5700rfTxCalibrationRequest_flash_C(void);
static UINT8FUNC ATA_5700convertTrxConf2rfTxConfig_flash_C(void);
static uint8_t* ATA_5700convertTrxConf2rfTxService_flash_C(void);

/**/
static VOIDFUNC ATA_triggerFrcCalibration_flash_C(void);

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5700InitCommandSet_C</b>
    does the initialization for ATA5700 SPI

    \Derived no

    \Rationale none

    \Traceability Primus2P-xxx

    \StackUsage SU_XXX bytes

    \image html ATA_5700InitCommandSet_C.png
    \image rtf ATA_5700InitCommandSet_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_5700InitCommandSet_flash_C()
{
    patchSpi = (patchSpiFuncPtr)0x0000;

    g_sAta5700_flash.events_system           = 0x00U;
    g_sAta5700_flash.events_wakeup           = 0x00U;
    g_sAta5700_flash.events_rf_flags_0       = 0x00U;
    g_sAta5700_flash.events_rf_flags_1       = 0x00U;
    g_sAta5700_flash.events_ph_flags_0       = 0x00U;
    g_sAta5700_flash.events_ph_flags_1       = 0x00U;
    g_sAta5700_flash.events_lf_flags         = 0x00U;
    g_sAta5700_flash.events_tp_flags         = 0x00U;
    g_sAta5700_flash.events_components       = 0x00U;
    g_sAta5700_flash.events_config           = 0x00U;
    g_sAta5700_flash.events_reset            = 0x00U;
    g_sAta5700_flash.events_pinChangePortB   = 0x00U;
    g_sAta5700_flash.events_pinChangePortD   = 0x00U;
    g_sAta5700_flash.status                  = 0x00U;

    extReq.tuneCheckConfig              = 0U;
    extReq.systemModeConfig             = 0U;
    extReq.serviceChannelConfig         = 0U;
    extReq.serviceInitConfig            = 0U;
    extReq.miscTrigger                  = 0U;
    extReq.lfrxRftxConfig               = 0U;
    extReq.tpEmModeConfig               = 0U;
    extReq.rfRemoteKeylessEntryConfig   = 0U;

    trxConf.systemModeConfig        = 0x00U;
    trxConf.serviceChannelConfig    = 0x00U;

    g_sRfRemoteKeylessEntryConf.bSecretKeyIdx   = 0x00U;
    g_sRfRemoteKeylessEntryConf.bCmdId          = 0x00U;
}

/*----------------------------------------------------------------------------- */
/**\brief  <b>ATA_5700CommandRequest_C</b>
 */
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_5700CommandRequest_flash_C(void)
{
    uint8_t bTempTpEmTransponderStateIndex_flash;

    // check for command request (internal or external)
    if(g_sAta5700_flash.status & BM_ATA5700_STATUS_UPDATE_FLAG)
    {
        ATA_5700systemModeSwitching_flash_C();
    }

    if (extReq.tuneCheckConfig)
    {
        ATA_5700rfTxCalibrationRequest_flash_C();
    }

    if (extReq.lfrxRftxConfig & BIT_MASK_0)
    {
        ATA_lfRxRfTxProcessImmo_flash_C();
    }

    if (extReq.tpEmModeConfig)
    {
        /* Protection, since used in interrupt */
        _CLI;
        bTempTpEmTransponderStateIndex_flash = g_bTpEmTransponderStateIndex_flash;
        _SEI;

        switch ( bTempTpEmTransponderStateIndex_flash )
        {
        case EM_MODE_STATE_INIT:
          ATA_tpEmModeInit_flash_C();
          _CLI;
          g_bTpEmTransponderStateIndex_flash = EM_MODE_WAIT_FOR_DATA;
          _SEI;
          break;

        case EM_MODE_STATE_RECONFIG:
          ATA_tpEmModeCommandReconfiguration_flash_C();
          _CLI;
          g_bTpEmTransponderStateIndex_flash = EM_MODE_WAIT_FOR_DATA;
          _SEI;
          break;

        case EM_MODE_STATE_PROCESSING:
          ATA_tpEmModeSingleTelProc_flash_C();
          _CLI;
          g_bTpEmTransponderStateIndex_flash = EM_MODE_WAIT_FOR_DATA;
          _SEI;
          break;

        default:
          break;
        }
    }

    /* Execute exactly one Remote Keyless Entry sequence */
    if( extReq.rfRemoteKeylessEntryConfig )
    {
//        ATA_rfrccGenRollCntMsg_C( g_sRfRemoteKeylessEntryConf.bSecretKeyIdx, g_sRfRemoteKeylessEntryConf.bCmdId );

//        if( !(g_sRfrccComponentData.bFlags & RFRCC_FLAGS_BM_ERROR_FLAG) )
//        {
//            // transmit generated message
//            ATA_rfTxStartTx_C( 0x48U, (uint8_t *) &g_sEepRfTxServiceConfig1 );
//            ATA_rfTxFillDFifo_C( RFRCC_MSG_LENGTH, &g_sRfrccComponentData.bRollCodeMsgBuffer[0] );
//        }
//        extReq.rfRemoteKeylessEntryConfig = 0x00U;
    }
}


/*----------------------------------------------------------------------------- */
/**\brief  <b>ATA_5700systemModeSwitching_flash_C</b>
 */
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_5700systemModeSwitching_flash_C(void)
{
    /* Request is being processed, clear status flag */
    g_sAta5700_flash.status &= (uint8_t)~BM_ATA5700_STATUS_UPDATE_FLAG;

    __disable_interrupt();
    trxConf.systemModeConfig     = extReq.systemModeConfig;
    extReq.systemModeConfig      = 0U;
    trxConf.serviceChannelConfig = (extReq.serviceChannelConfig & 0x3FU);
    extReq.serviceChannelConfig  = 0U;
    __enable_interrupt();

    uint8_t opm = trxConf.systemModeConfig & BM_SYS_MODE_CONFIG_OPM;

    if (opm == OPM_IDLE)
    {
        /* set module RFTX to IDLEMode */
        if (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE) {
            ATA_rfTxStop_C();
        }

        /* Check IdleModeSelector */
        if ( (trxConf.systemModeConfig & BM_SYS_MODE_CONFIG_IDLE_MODE_SELECTOR) >> 3 )
        {
            /* Select XTO/4 as system clock */
            ATA_globalsClkSwitchXTO_C(0x07U);
        }
        else
        {
            /* Clock switch to FRC */
            ATA_globalsClkSwitchFrcWithMvccEnable_C();
            /* disable XTO */
            ATA_globalsDeActivateXTO_C();

        }
    }
    else {

        /**/
        uint8_t config    = ATA_5700convertTrxConf2rfTxConfig_flash_C();
        uint8_t *pService = ATA_5700convertTrxConf2rfTxService_flash_C();

        /* set module RFTX to TXMode(buffered/transparent) */
        ATA_rfTxStartTx_C(config, pService);
    }
}

/*----------------------------------------------------------------------------- */
/**\brief  <b>ATA_5700rfTxCalibrationRequest_flash_C</b>
 */
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_5700rfTxCalibrationRequest_flash_C(void)
{
    __disable_interrupt();
    trxConf.tuneCheckConfig = extReq.tuneCheckConfig;
    extReq.tuneCheckConfig  = 0U;
    trxConf.serviceChannelConfig = (extReq.serviceChannelConfig & 0x3FU);
    extReq.serviceChannelConfig  = 0U;
    __enable_interrupt();

    trxConf.systemModeConfig = 0U;
    uint8_t config    = ATA_5700convertTrxConf2rfTxConfig_flash_C();
    uint8_t *pService = ATA_5700convertTrxConf2rfTxService_flash_C();

    if( trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_ANTENNA_TUNING )
    {
        // Antenna tuning
        ATA_rfTxStartAnt_C(config, pService);
    }
    else if (trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_SRC_CALIB)
    {
        // SRC calibration

    }
    else if (trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_FRC_CALIB)
    {
        // FRC calibration
        ATA_triggerFrcCalibration_flash_C();
    }
    else if (trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_VCO_CALIB)
    {
        // VCO tuning
        ATA_rfTxStartVco_C(config, pService);
    }
    else {}
}

/*----------------------------------------------------------------------------- */
/**\brief  <b>ATA_5700rfTxCalibrationRequest_C</b>
    used for conversion from sigmax syntax to primus2p syntax
 */
/*----------------------------------------------------------------------------- */
UINT8FUNC ATA_5700convertTrxConf2rfTxConfig_flash_C(void)
{
    uint8_t config = 0x00;

    // convert configuration from SIX to P2P
    if (trxConf.systemModeConfig & BM_SYS_MODE_CONFIG_ANTENNA_TUNING) {
        config |=  BM_RFTXCONFIG_BCONFIG_ANT_TUNING;
    }

    if (trxConf.systemModeConfig & BM_SYS_MODE_CONFIG_VCO_TUNING) {
        config |= BM_RFTXCONFIG_BCONFIG_VCO_TUNING;
    }

    if (trxConf.systemModeConfig & BM_SYS_MODE_CONFIG_TRANSPARENT_MODE) {
        config |=  BM_RFTXCONFIG_BCONFIG_TRANSPARENT_MODE;
    }

    if (trxConf.systemModeConfig & BM_SYS_MODE_CONFIG_IDLE_MODE_SELECTOR) {
        config |= BM_RFTXCONFIG_BCONFIG_SHUTDOWN_MODE;
    }

    config |= ((trxConf.serviceChannelConfig & BM_SVC_CH_CONFIG_CH ) >> 4U);

    config |= BM_RFTXCONFIG_BCONFIG_SVC_LOCATION;

    return config;

}

/*----------------------------------------------------------------------------- */
/**\brief  <b>ATA_5700convertTrxConf2rfTxService_C</b>
    used for conversion from sigmax syntax to primus2p syntax
 */
/*----------------------------------------------------------------------------- */
uint8_t* ATA_5700convertTrxConf2rfTxService_flash_C(void)
{
    uint8_t service = trxConf.serviceChannelConfig & BM_SVC_CH_CONFIG_SER;
    uint8_t *pService = (uint8_t *)&g_sEepRfTxServiceConfig1;
    if (service == 1) {
        pService = (uint8_t *)&g_sEepRfTxServiceConfig2;
    }
    else {
        pService = (uint8_t *)&g_sEepRfTxServiceConfig1;
    }
    return pService;
}


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_triggerFrcCalibration_C</b>
    triggers FRC calibration feature.

    \return none

    \image html ATA_triggerFrcCalibration_C.png
    \image rtf ATA_triggerFrcCalibration_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
static VOIDFUNC ATA_triggerFrcCalibration_flash_C(void)
{
    /* indicates a calibration process is ongoing */
    g_sAta5700_flash.status |= BM_ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG;

    /* Do FRC calibration */
    ATA_calibInit_C();

    if ((g_sCalibConfig.bFlags & BM_CALIB_CONFIG_FLAGS_ERROR) == 0x00U)
    {
        ATA_calibStartCalibration_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC);

        /* Shutdown XTO and AVCC and switch to FRC as default clock. */
        // ATA_shutDownFe_C();

        trxConf.tuneCheckConfig &= (uint8_t)~BM_TUNE_CHECK_CONFIG_FRC_CALIB;
        g_sAta5700_flash.status &= (uint8_t)~BM_ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG;

        g_sAta5700_flash.events_rf_flags_1 |= BM_ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY;
    }
}
