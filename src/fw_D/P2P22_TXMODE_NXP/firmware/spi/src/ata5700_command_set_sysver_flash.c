//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Branches/VB_PRIMUS2P_ROM_2.0/firmware/spi/src/ata5700_command_set_sysver_flash.c $
  $LastChangedRevision: 291252 $
  $LastChangedDate: 2014-12-16 15:38:03 +0100 (Di, 16 Dez 2014) $
  $LastChangedBy: florian.schweidler $
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
/** \file ata5700_command_set_sysver_flash.c
*/

//lint -restore

/*===========================================================================*/
/*  INCLUDES (Flash Application)                                             */
/*===========================================================================*/
#include "ata5700_command_set_sysver_flash.h"
#include "../../lfrx/src/lfrx_immo_flash.h"
#include "../../tp/src/tp_flash.h"
#include "../../init/src/init_flash.h"
#include "../../lfrx/src/lfrx_flash.h"
#include "../../system/src/system_flash.h"
#include "../../rftx/src/rftx_flash.h"
#include "../../lfrssi/src/lfrssi_flash.h"

/*===========================================================================*/
/*  INCLUDES (ROM firmware)                                                  */
/*===========================================================================*/
#include "../../calib/src/calib.h"
#include "../../rftx/src/rftx.h"
#include "../../rftx/src/rftx_ant.h"
#include "../../rftx/src/rftx_vco.h"
#include "../../globals/src/globals.h"
#include "../../lfrssi/src/lfrssi.h"

/*===========================================================================*/
/*  INCLUDES (Flash Module Deliverable)                                      */
/*===========================================================================*/
#include "../../../appl/appFlash_simTest/src/rfrcc/src/rfrcc_flash.h"
#include "../../../appl/appFlash_simTest/src/twi/src/twi_flash.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

#define     TWI_BUFFER_LEN      16U

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

/* No pragma */
__no_init sTwiConfiguration g_TwiConfiguration_flash;
__root __no_init  uint8_t g_TwiBuffer_flash[TWI_BUFFER_LEN];

static VOIDFUNC  ATA_5700systemModeSwitching_flash_C(void);
static VOIDFUNC  ATA_5700rfTxCalibrationRequest_flash_C(void);
static UINT8FUNC ATA_5700convertTrxConf2rfTxConfig_flash_C(void);
static uint8_t* ATA_5700convertTrxConf2rfTxService_flash_C(void);

/**/
static VOIDFUNC ATA_triggerFrcCalibration_flash_C(void);
static VOIDFUNC ATA_triggerSrcCalibration_flash_C(void);


/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_5700InitCommandSet_flash_C</b>
    does the initialization for ATA5700 SPI

    \Derived no

    \Rationale none

    \Traceability Primus2P-xxx

    \StackUsage SU_XXX bytes

    \image html ATA_5700InitCommandSet_flash_C.png
    \image rtf ATA_5700InitCommandSet_flash_C.png
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

    g_TwiConfiguration_flash.twiAddr = 0x00;
    g_TwiConfiguration_flash.twiAddrMsk = 0x00;
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

    if (extReq.lfrxRftxConfig & BIT_MASK_7)
    {
        ATA_lfTpInit_flash_ASM();

        /* Reset LF TP initialization trigger */
        extReq.lfrxRftxConfig &= ~BIT_MASK_7;
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
        ATA_rfrccGenRollCntMsg_C(0x06C0U, g_sRfRemoteKeylessEntryConf.bSecretKeyIdx, g_sRfRemoteKeylessEntryConf.bCmdId, 0x00U );

        if( !(g_sRfrccComponentData.bFlags & RFRCC_FLAGS_BM_ERROR_FLAG) )
        {
            /* Get service 0 by default */
            uint16_t eepService  = ATA_rfTxGetIndirectEEPromServiceConfigAddr_flash_C((uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l);

            /* transmit generated message */
            ATA_rfTxStartTx_C( 0x48U, (uint8_t*)eepService);
            ATA_rfTxFillDFifo_C( RFRCC_MSG_LENGTH, &g_sRfrccComponentData.bRollCodeMsgBuffer[0] );
        }
        extReq.rfRemoteKeylessEntryConfig = 0x00U;
    }

    if (extReq.serviceInitConfig) {

        __disable_interrupt();
        uint8_t serviceInitConfig = extReq.serviceInitConfig & (~BM_EXT_REQ_SERVICE_INIT_CONFIG_UPDATE_FLAG);
        extReq.serviceInitConfig = 0;
        __enable_interrupt();

        /* Get service 0 by default */
        uint16_t eepService  = ATA_rfTxGetIndirectEEPromServiceConfigAddr_flash_C((uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l);

        if (serviceInitConfig >> 1)
        {
            eepService  = ATA_rfTxGetIndirectEEPromServiceConfigAddr_flash_C((uint16_t)&g_sCustomerEEPromSection.eepRfTxSer1Ptr_l);
        }

        if (ATA_eepReadBytes_C((uint8_t *)&g_sSramRfTxServiceConfig, eepService,sizeof(sRfTxServiceChannelConfig)) != EEC_NO_ERROR) {
            // --GW-TODO-- g_sDebug.bErrorCode    = DEBUG_ERROR_MODULE_TEST + g_sModuleTest_flash.bId;
            // --GW-TODO-- g_sDebug.bSsmErrorCode = g_sModuleTest_flash.bSubId;
            ATA_systemSetSystemError_flash_ASM();
        }
    }

    if ( extReq.miscTrigger & BM_EXT_REQ_MISC_TRIGGER_ACTIVATE_TWI_SLAVE )
    {
         ATA_twiUse_C(&twiCtrlBlock,\
                      g_TwiConfiguration_flash.twiAddr,\
                      g_TwiConfiguration_flash.twiAddrMsk,\
                      0x00,\
                      0x00,\
                      TWI_BUFFER_LEN,\
                      &g_TwiBuffer_flash[0],\
                      0xC0U);

         /* Setup answer */
         // Warum geht es so nicht?!? Das ist eine Konstante im Flash, weiter unten ist es SRAM bzw. Stack
         // uint8_t tempBuf[4U] = {0xDE,0xAD,0xBE,0xEF};  
         uint8_t tempBuf[4U];
         tempBuf[0] = 0xDE;
         tempBuf[1] = 0xAD;
         tempBuf[2] = 0xBE;
         tempBuf[3] = 0xEF;
         ATA_twiSetBuffer_C(&twiCtrlBlock,&tempBuf[0],4U);

         extReq.miscTrigger &= ~BM_EXT_REQ_MISC_TRIGGER_ACTIVATE_TWI_SLAVE;
    }

    if( extReq.miscTrigger & BM_EXT_REQ_MISC_TRIGGER_START_RSSI )
    {
        if( g_sAta5700_flash.events_ph_flags_0 & BM_LFRXCONFIG_PH_FLAGS_0_PHID0F )
        {
            ATA_lfRssiMeasConfigHfm_flash_C( extReq.miscTrigger & BM_EXT_REQ_MISC_TRIGGER_START_RSSI );
            g_sAta5700_flash.events_ph_flags_0 &= ~BM_LFRXCONFIG_PH_FLAGS_0_PHID0F;
            extReq.miscTrigger &= ~BM_EXT_REQ_MISC_TRIGGER_START_RSSI;
        }
    }
}


/*----------------------------------------------------------------------------- */
/**\brief  <b>ATA_5700PerformEventHandling_flash_C</b>
 */
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_5700PerformEventHandling_flash_C(void)
{
    uint8_t setEventPin = 0;
    // Update ATA 5700 event bytes regarding RF Tx component and clear component event information.
    if (g_sRfTx.bFlags || g_sRfTx.bTuneFlags) {
        _CLI;
        if (   (g_sRfTx.bFlags & g_sEventHandling_flash.bRfTx0)
            || (g_sRfTx.bTuneFlags & g_sEventHandling_flash.bRfTx1))
        {
            setEventPin = 1;
        }

        g_sAta5700_flash.events_rf_flags_0 |= g_sRfTx.bFlags;
        g_sRfTx.bFlags = 0x00U;

        g_sAta5700_flash.events_rf_flags_1 |= g_sRfTx.bTuneFlags;
        g_sRfTx.bTuneFlags = 0x00U;

        g_sAta5700_flash.events_config = trxConf.serviceChannelConfig;
        _SEI;
    }

    // Update ATA 5700 event bytes regarding Protocol Handler (FIFO) component.
    if (g_sLfRx_flash.bLDFFLags0 || g_sLfRx_flash.bLDFFLags1) {
        _CLI;
        g_sAta5700_flash.events_ph_flags_0 |= g_sLfRx_flash.bLDFFLags0 ;
        g_sAta5700_flash.events_ph_flags_1 |= g_sLfRx_flash.bLDFFLags1;
        g_sLfRx_flash.bLDFFLags0 = 0x00U;
        g_sLfRx_flash.bLDFFLags1 = 0x00U;
        _SEI;
    }

    // Update ATA 5700 event bytes regarding 3D LF Receiver component.
    if (g_sLfRx_flash.bLfFlags) {
        _CLI;
        g_sAta5700_flash.events_lf_flags |= g_sLfRx_flash.bLfFlags;
        g_sLfRx_flash.bLfFlags = 0x00U;
        _SEI;
    }

    // Update ATA 5700 event bytes regarding 3D LF Receiver RSSI component.
    if( g_sLfRssi.bFlags & (LFRSSI_FLAGS_BM_ERROR_FLAG | LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG) )
    {
        _CLI;

        g_sAta5700_flash.events_lf_flags |= (g_sLfRssi.bFlags & (BM_LFRXCONFIG_LF_FLAGS_RSSI_ERR | BM_LFRXCONFIG_LF_FLAGS_RSSI_RDY));

        if( g_sEventHandling_flash.bLf3dRx & (BM_LFRXCONFIG_LF_FLAGS_RSSI_ERR | BM_LFRXCONFIG_LF_FLAGS_RSSI_RDY) )
        {
            setEventPin = 1;
        }

        // Clear component flags, otherwise the event pin is triggered more than once for the same action.
        g_sLfRssi.bFlags &= (~LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG);
        g_sLfRssi.bFlags &= (~LFRSSI_FLAGS_BM_ERROR_FLAG);
        _SEI;
    }

    // Update ATA 5700 event bytes regarding Transponder component.
    if (g_sTpRxTx.bTpFlags) {
        _CLI;
        // Transponder and 3D LF Receiver share the CRC bit error information CRCEF within the PH event data.
        g_sAta5700_flash.events_tp_flags |= g_sTpRxTx.bTpFlags;
        g_sTpRxTx.bTpFlags = 0x00U;
        _SEI;
    }

    if ( setEventPin )
    {
        ATA_systemSetEventPin_flash_ASM();
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
            ATA_globalsActivateXTO_C();
            ATA_globalsClkSwitchXTO_C(0x07U);
        }
        else
        {
            /* Clock switch to FRC */
            ATA_globalsClkSwitchFrc_C();
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
        /* Antenna tuning */
        ATA_rfTxStartAnt_C(config, pService);
    }
    else if (trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_SRC_CALIB)
    {
        /* SRC calibration */
        ATA_triggerSrcCalibration_flash_C();

    }
    else if (trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_FRC_CALIB)
    {
        /* FRC calibration */
        ATA_triggerFrcCalibration_flash_C();
    }
    else if (trxConf.tuneCheckConfig & BM_TUNE_CHECK_CONFIG_VCO_CALIB)
    {
        /* VCO tuning */
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

    /* convert configuration from SIX to P2P */
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
    uint16_t wAddress = 0U;

    if (service == 1)
    {
        /* Get indirect address for RF Tx EEPROM configuration Service 1 */
        wAddress = (uint16_t)&g_sCustomerEEPromSection.eepRfTxSer1Ptr_l;
    }
    else
    {
        /* Get indirect address for RF Tx EEPROM configuration Service 0 */
        wAddress = (uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l;
    }

    uint8_t *pService = (uint8_t*)ATA_rfTxGetIndirectEEPromServiceConfigAddr_flash_C(wAddress);

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


/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_triggerSrcCalibration_C</b>
    triggers SRC calibration feature.

    \return none

    \image html ATA_triggerSrcCalibration_C.png
    \image rtf ATA_triggerSrcCalibration_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
static VOIDFUNC ATA_triggerSrcCalibration_flash_C(void)
{
    /* indicates a calibration process is ongoing */
    g_sAta5700_flash.status |= BM_ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG;

    /* Do SRC calibration */
    ATA_calibInit_C();

    if ((g_sCalibConfig.bFlags & BM_CALIB_CONFIG_FLAGS_ERROR) == 0x00U)
    {
        ATA_calibStartCalibration_C(BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC);

        /* Shutdown XTO and AVCC and switch to FRC as default clock. */
        // ATA_shutDownFe_C();

        trxConf.tuneCheckConfig &= (uint8_t)~BM_TUNE_CHECK_CONFIG_SRC_CALIB;
        g_sAta5700_flash.status &= (uint8_t)~BM_ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG;

        g_sAta5700_flash.events_rf_flags_1 |= BM_ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY;
    }
}

