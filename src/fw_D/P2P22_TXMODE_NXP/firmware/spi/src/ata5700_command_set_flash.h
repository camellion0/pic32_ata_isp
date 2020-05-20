//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Branches/VB_PRIMUS2P_ROM_2.0/firmware/spi/src/ata5700_command_set_flash.h $
  $LastChangedRevision: 280653 $
  $LastChangedDate: 2014-09-30 16:45:31 +0200 (Di, 30 Sep 2014) $
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
/** \file ata5700_command_set.h
 */

//lint -restore

#ifndef ATA5700_COMMAND_SET_H
#define ATA5700_COMMAND_SET_H

#ifdef __IAR_SYSTEMS_ICC__
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"
#include "../../rftx/src/rftx_defs.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/* ------------------------------------------------------------------------- */
/* g_sAta5700_flash                                                                   */
/* ------------------------------------------------------------------------- */

/* g_sAta5700_flash.status */
#define ATA5700_STATUS_UPDATE_FLAG                  BIT_7
#define ATA5700_STATUS_TRIGGER_MAIN_LOOP_FLAG       BIT_6
#define ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG     BIT_5
#define ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG   BIT_4
#define ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG   BIT_3

#define BM_ATA5700_STATUS_UPDATE_FLAG                   BITMASK(ATA5700_STATUS_UPDATE_FLAG)
#define BM_ATA5700_STATUS_TRIGGER_MAIN_LOOP_FLAG        BITMASK(ATA5700_STATUS_TRIGGER_MAIN_LOOP_FLAG)
#define BM_ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG      BITMASK(ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG)
#define BM_ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG    BITMASK(ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG)
#define BM_ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG    BITMASK(ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG)


/* g_sAta5700_flash.events_system */
#define ATA5700_EVENTS_SYSTEM_SYS_ERR               BIT_7
#define ATA5700_EVENTS_SYSTEM_SYS_RDY               BIT_6
#define ATA5700_EVENTS_SYSTEM_AVCCLOW               BIT_5
#define ATA5700_EVENTS_SYSTEM_LOWBATT               BIT_4
#define ATA5700_EVENTS_SYSTEM_EXTCLOCK_ERR          BIT_3
#define ATA5700_EVENTS_SYSTEM_TIMER_0               BIT_0


#define BM_ATA5700_EVENTS_SYSTEM_SYS_ERR            BITMASK(ATA5700_EVENTS_SYSTEM_SYS_ERR)
#define BM_ATA5700_EVENTS_SYSTEM_SYS_RDY            BITMASK(ATA5700_EVENTS_SYSTEM_SYS_RDY)
#define BM_ATA5700_EVENTS_SYSTEM_AVCCLOW            BITMASK(ATA5700_EVENTS_SYSTEM_AVCCLOW)
#define BM_ATA5700_EVENTS_SYSTEM_LOWBATT            BITMASK(ATA5700_EVENTS_SYSTEM_LOWBATT)
#define BM_ATA5700_EVENTS_SYSTEM_EXTCLOCK_ERR       BITMASK(ATA5700_EVENTS_SYSTEM_EXTCLOCK_ERR)
#define BM_ATA5700_EVENTS_SYSTEM_TIMER_0            BITMASK(ATA5700_EVENTS_SYSTEM_TIMER_0)

/* g_sAta5700_flash.rfFlags */
#define ATA5700_EVENTS_RFFLAGS_ANT_TUNE_RDY         BIT_7
#define ATA5700_EVENTS_RFFLAGS_VCO_TUNE_RDY         BIT_6
#define ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY        BIT_5
#define ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY        BIT_4

#define BM_ATA5700_EVENTS_RFFLAGS_ANT_TUNE_RDY      BITMASK(ATA5700_EVENTS_RFFLAGS_ANT_TUNE_RDY)
#define BM_ATA5700_EVENTS_RFFLAGS_VCO_TUNE_RDY      BITMASK(ATA5700_EVENTS_RFFLAGS_VCO_TUNE_RDY)
#define BM_ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY     BITMASK(ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY)
#define BM_ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY     BITMASK(ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY)

/* g_sAta5700_flash.events_reset */
#define ATA5700_EVENTS_RESET_LF_CALRY               BIT_7

#define ATA5700_EVENTS_RESET_LF_TPRF                BIT_5
#define ATA5700_EVENTS_RESET_LF_JTRF                BIT_4
#define ATA5700_EVENTS_RESET_LF_WDRF                BIT_3

#define ATA5700_EVENTS_RESET_LF_EXTRF               BIT_1
#define ATA5700_EVENTS_RESET_LF_PORF                BIT_0

#define BM_ATA5700_EVENTS_RESET_LF_CALRY            BITMASK(ATA5700_EVENTS_RESET_LF_CALRY)

#define BM_ATA5700_EVENTS_RESET_LF_TPRF             BITMASK(ATA5700_EVENTS_RESET_LF_TPRF)
#define BM_ATA5700_EVENTS_RESET_LF_JTRF             BITMASK(ATA5700_EVENTS_RESET_LF_JTRF)
#define BM_ATA5700_EVENTS_RESET_LF_WDRF             BITMASK(ATA5700_EVENTS_RESET_LF_WDRF)

#define BM_ATA5700_EVENTS_RESET_LF_EXTRF            BITMASK(ATA5700_EVENTS_RESET_LF_EXTRF)
#define BM_ATA5700_EVENTS_RESET_LF_PORF             BITMASK(ATA5700_EVENTS_RESET_LF_PORF)

/* g_sAta5700_flash.events_components */
#define ATA5700_EVENTS_COMPONENTS_AES_ERR  BIT_3
#define ATA5700_EVENTS_COMPONENTS_AES_RDY  BIT_2
#define ATA5700_EVENTS_COMPONENTS_I2C_ERR  BIT_1
#define ATA5700_EVENTS_COMPONENTS_I2C_RDY  BIT_0

#define BM_ATA5700_EVENTS_COMPONENTS_AES_ERR  BITMASK(ATA5700_EVENTS_COMPONENTS_AES_ERR)
#define BM_ATA5700_EVENTS_COMPONENTS_AES_RDY  BITMASK(ATA5700_EVENTS_COMPONENTS_AES_RDY)
#define BM_ATA5700_EVENTS_COMPONENTS_I2C_ERR  BITMASK(ATA5700_EVENTS_COMPONENTS_I2C_ERR)
#define BM_ATA5700_EVENTS_COMPONENTS_I2C_RDY  BITMASK(ATA5700_EVENTS_COMPONENTS_I2C_RDY)

/* ------------------------------------------------------------------------- */
/* sExtReq.serviceInitConfig                                                 */
/* ------------------------------------------------------------------------- */
#define EXT_REQ_SERVICE_INIT_CONFIG_UPDATE_FLAG         BIT_7
#define EXT_REQ_SERVICE_INIT_CONFIG_EEPROM_NUMBER       BIT_1
#define EXT_REQ_SERVICE_INIT_CONFIG_SRAM_NUMBER         BIT_0

#define BM_EXT_REQ_SERVICE_INIT_CONFIG_UPDATE_FLAG      BIT_MASK_7
#define BM_EXT_REQ_SERVICE_INIT_CONFIG_EEPROM_NUMBER   ( BIT_MASK_1 | BIT_MASK_2 )
#define BM_EXT_REQ_SERVICE_INIT_CONFIG_SRAM_NUMBER      BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* sExtReq.miscTrigger                                                       */
/* ------------------------------------------------------------------------- */
#define EXT_REQ_MISC_TRIGGER_START_RSSI                 BIT_0

#define BM_EXT_REQ_MISC_TRIGGER_START_RSSI              BIT_MASK_0

/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/
/** \brief <b>patchSpiFuncPtr</b>
    is used for function pointer definition of "PATCH SPI" command.
*/
typedef void (*patchSpiFuncPtr)(void);

/*----------------------------------------------------------------------------- */
/** \brief <b>sExtReq</b>
    contains the configuration which was selected via SPI command
*/
/*----------------------------------------------------------------------------- */
typedef struct {
   
    /** \brief <b>events_system</b>
        contains the event flags with ATA5700 system application information
        \li Bit 7:    SYS_ERR
        \li Bit 6:    SYS_RDY
        \li Bit 5:    AVCCLOW
        \li Bit 4:    LOWBATT
        \li Bit 3:    EXTCLOCK_ERR
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    T0F
     */
    uint8_t events_system;
    
    /** \brief <b>events_wakeup</b>
        contains the event flags with powerOn nPowerOn information
        \li Bit 7: PWRON
        \li Bit 6: NPWRON6
        \li Bit 5: NPWRON5
        \li Bit 4: NPWRON4
        \li Bit 3: NPWRON3
        \li Bit 2: NPWRON2
        \li Bit 1: NPWRON1
        \li Bit 0: NPWRON0
     */
    uint8_t events_wakeup;
    
    /** \brief <b>events_rf_flags_0</b>
        contains the event flags with ATA5700 RF TX information 
        (see rfTx component)
        \li Bit 7:    TX_ERR
        \li Bit 6:    TX_RDY
        \li Bit 5:    rfu
        \li Bit 4:    DFIFO_ERR
        \li Bit 3:    SFIFO_ERR
        \li Bit 2:    DFL_BUF_FILLLVL
        \li Bit 1:    SFL_BUF_FILLLVL
        \li Bit 0:    EOT
    */
    uint8_t events_rf_flags_0;
    
    /** \brief <b>events_rf_flags_1</b>
        contains the event flags with ATA5700 RF TX information 
        (see rfTx component)
        \li Bit 7:    ANT_TUNE_RDY
        \li Bit 6:    VCO_TUNE_RDY
        \li Bit 5:    FRC_CALIB_RDY
        \li Bit 4:    SRC_CALIB_RDY
        \li Bit 3:    rfu
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    rfu
    */
    uint8_t events_rf_flags_1;

    /** \brief <b>events_ph_flags_0</b>
        contains the event flags for the Protocol Handler
        (see ph component)
        \li Bit 7: PHCOF
        \li Bit 6: rfu
        \li Bit 5: PHID1F
        \li Bit 4: PHID0F
        \li Bit 3: PHIDFF
        \li Bit 2: PHDFF
        \li Bit 1: PHTBLF
        \li Bit 0: CRCEF
    */
    uint8_t events_ph_flags_0;
    
    /** \brief <b>events_ph_flags_1</b>
        contains the event flags for the Protocol Handler FIFO
        (see LDFIMfo component)
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: rfu
        \li Bit 2: rfu
        \li Bit 1: OUFLF
        \li Bit 0: FLRF
    */
    uint8_t events_ph_flags_1;
    
    /** \brief <b>events_lf_flags</b>
        contains the event flags for the 3D LF Receiver
        (see lfrx component)
        \li Bit 7: RSSI_ERR
        \li Bit 6: RSSI_RDY
        \li Bit 5: rfu
        \li Bit 4: LF_RDY
        \li Bit 3: LFTOF
        \li Bit 2: LFEOF
        \li Bit 1: LFDEF
        \li Bit 0: LFSYDF
     */
    uint8_t events_lf_flags;
    
    /** \brief <b>events_tp_flags</b>
        contains the event flags for the Transponder
        (see tp component)
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: TPBERF
        \li Bit 2: TPNFTF
        \li Bit 1: TPFTF
        \li Bit 0: TPF
    */
    uint8_t events_tp_flags;

    /** \brief <b>events_components</b>
        contains the component event flags
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: AES_ERR
        \li Bit 2: AES_RDY
        \li Bit 1: I2C_ERR
        \li Bit 0: I2C_RDY
    */
    uint8_t events_components;
    
    /** \brief <b>events_config</b>
        holds the current selected service channel configuration
    */
    uint8_t events_config;
    
    /** \brief <b>events_reset</b>
        holds the reset information of register MCUSR after power-up
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: TPRF
        \li Bit 4: JTRF
        \li Bit 3: WDRF
        \li Bit 2: rfu
        \li Bit 1: EXTRF
        \li Bit 0: PORF
    */
    uint8_t events_reset;
    
    /** \brief <b>events_pinChangePortB</b>
        holds port B pin change events
    */
    uint8_t events_pinChangePortB;
    
    /** \brief <b>events_pinChangePortD</b>
        holds port D pin change events
    */
    uint8_t events_pinChangePortD;
    
    /** \brief <b>status</b>
        contains the status information for ATA5700 command set decoding
        \li Bit 7:  UPDATE_FLAG
        \li Bit 6:  TRIGGER_MAIN_LOOP_FLAG
        \li Bit 5:  SPI_CMD_IN_PROGRESS_FLAG
        \li Bit 4:  FRC_CALIB_IN_PROGRESS_FLAG
        \li Bit 3:  SRC_CALIB_IN_PROGRESS_FLAG
        \li Bit 2:  rfu
        \li Bit 1:  rfu
        \li Bit 0:  rfu
    */
    uint8_t status;

    
}sAta5700;

/*----------------------------------------------------------------------------- */
/** \brief <b>sExtReq</b>
    contains the configuration which was selected via SPI command
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>tuneCheckConfig</b>
        contains the Tune and Check settings from external
     */
    tuneCheckConfig_t tuneCheckConfig;

    /** \brief <b>systemModeConfig</b>
        contains the System mode configuration from external\
     */
    sysModeConfig_t systemModeConfig;

    /** \brief <b>serviceChannelConfig</b>
        contains the Service/Channel configuration
     */
    svcChConfig_t serviceChannelConfig;

    /** \brief <b>serviceInitConfig</b>
        holds the service init configuration
        \li Bit7: updateFlag
        \li Bit6: rfu
        \li Bit5: rfu
        \li Bit4: rfu
        \li Bit3: rfu
        \li Bit2: eepromService1
        \li Bit1: eepromService0 0..2
        \li Bit0: sramService    0..1
     */
    uint8_t    serviceInitConfig;

    /** \brief <b>miscTrigger</b>
        holds the miscellaneous trigger (Could be removed?!?)
        \li Bit7: rfu
        \li Bit6: rfu
        \li Bit5: rfu
        \li Bit4: rfu
        \li Bit3: rfu
        \li Bit2: rfu
        \li Bit1: rfu
        \li Bit0: start RSSI measurment
     */
    uint8_t    miscTrigger;

    /** \brief <b>lfrxRftxConfig</b>
        holds the LF RX for AOIP configuration
        \li Bit7: start LF TP initialization
        \li Bit6: rfu
        \li Bit5: rfu
        \li Bit4: rfu
        \li Bit3: rfu
        \li Bit2: rfu
        \li Bit1: rfu
        \li Bit0: start LF RX initialization
     */
    uint8_t    lfrxRftxConfig;
    
    /** \brief <b>tpEmModeConfig</b>
        holds the EM transponder configuration
        \li Bit7: rfu
        \li Bit6: rfu
        \li Bit5: rfu
        \li Bit4: rfu
        \li Bit3: rfu
        \li Bit2: rfu
        \li Bit1: rfu
        \li Bit0: start EM Mode initialization
     */
    uint8_t    tpEmModeConfig;
    
    /** \brief <b>rfRemoteKeylessEntryConfig</b>
        holds the Remote Keyless Entry configuration
        \li Bit7: rfu
        \li Bit6: rfu
        \li Bit5: rfu
        \li Bit4: rfu
        \li Bit3: rfu
        \li Bit2: rfu
        \li Bit1: rfu
        \li Bit0: trigger Remote Keyless Entry function
     */
    uint8_t    rfRemoteKeylessEntryConfig;

}sExtReq;

/*----------------------------------------------------------------------------- */
/** \brief <b>sTrxConfig</b>
    holds the settings for the Transceiver Configuration
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>tuneCheckConfig</b>
        contains the Tune and Check Configuration which shall be done
     */
    tuneCheckConfig_t tuneCheckConfig;

    /** \brief <b>systemModeConfig</b>
        contains the current System mode configuration
     */
    sysModeConfig_t systemModeConfig;

    /** \brief <b>serviceChannelConfig</b>
        contains the Service/Channel configuration
     */
    svcChConfig_t serviceChannelConfig;

}sTrxConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sRfRemoteKeylessEntryConf</b>
    holds the settings for the Remote Keyless Entry function
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>bSecretKeyIdx</b>
        contains the Secret Key Index to be used to calculate the CMAC
     */
    uint8_t bSecretKeyIdx;

    /** \brief <b>bCmdId</b>
        contains the command to be sent to the Base Station
     */
    uint8_t bCmdId;
    
} sRfRemoteKeylessEntryConf;

/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/
extern patchSpiFuncPtr patchSpi;
extern VOIDFUNC ATA_5700InitCommandSet_flash_C(void);
extern VOIDFUNC ATA_5700CommandRequest_flash_C(void);
extern VOIDFUNC ATA_5700PerformEventHandling_flash_C(void);

extern sExtReq extReq;
extern sTrxConfig trxConf;
extern sAta5700 g_sAta5700_flash;
extern sRfRemoteKeylessEntryConf g_sRfRemoteKeylessEntryConf;

#elif defined __IAR_SYSTEMS_ASM__

/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/
EXTERN  patchSpi
EXTERN  ATA_5700InitCommandSet_flash_C
EXTERN  extReq
EXTERN  g_sRfRemoteKeylessEntryConf


/*startSimExtraction*/

/* ------------------------------------------------------------------------- */
/* g_sAta5700_flash                                                                   */
/* ------------------------------------------------------------------------- */
ATA5700_EVENTS_SYSTEM               EQU 0
ATA5700_EVENTS_WAKEUP               EQU ATA5700_EVENTS_SYSTEM + 1
ATA5700_EVENTS_RF_FLAGS_0           EQU ATA5700_EVENTS_WAKEUP + 1
ATA5700_EVENTS_RF_FLAGS_1           EQU ATA5700_EVENTS_RF_FLAGS_0 + 1
ATA5700_EVENTS_PH_FLAGS_0           EQU ATA5700_EVENTS_RF_FLAGS_1 + 1
ATA5700_EVENTS_PH_FLAGS_1           EQU ATA5700_EVENTS_PH_FLAGS_0 + 1
ATA5700_EVENTS_LF_FLAGS             EQU ATA5700_EVENTS_PH_FLAGS_1 + 1
ATA5700_EVENTS_TP_FLAGS             EQU ATA5700_EVENTS_LF_FLAGS + 1
ATA5700_EVENTS_COMPONENTS           EQU ATA5700_EVENTS_TP_FLAGS + 1
ATA5700_EVENTS_CONFIG               EQU ATA5700_EVENTS_COMPONENTS + 1
ATA5700_EVENTS_RESET                EQU ATA5700_EVENTS_CONFIG + 1
ATA5700_EVENTS_PIN_CHANGE_PORT_B    EQU ATA5700_EVENTS_RESET + 1
ATA5700_EVENTS_PIN_CHANGE_PORT_D    EQU ATA5700_EVENTS_PIN_CHANGE_PORT_B + 1
ATA5700_STATUS                      EQU ATA5700_EVENTS_PIN_CHANGE_PORT_D + 1

/* g_sAta5700_flash.EVENTS_SYSTEM */
ATA5700_EVENTS_SYSTEM_SYS_ERR               EQU BIT_7
ATA5700_EVENTS_SYSTEM_SYS_RDY               EQU BIT_6
ATA5700_EVENTS_SYSTEM_AVCCLOW               EQU BIT_5
ATA5700_EVENTS_SYSTEM_LOWBATT               EQU BIT_4
ATA5700_EVENTS_SYSTEM_EXT_CLOCK_ERR         EQU BIT_3
ATA5700_EVENTS_SYSTEM_TIMER_0               EQU BIT_0

BM_ATA5700_EVENTS_SYSTEM_SYS_ERR            EQU BIT_MASK_7
BM_ATA5700_EVENTS_SYSTEM_SYS_RDY            EQU BIT_MASK_6
BM_ATA5700_EVENTS_SYSTEM_AVCCLOW            EQU BIT_MASK_5
BM_ATA5700_EVENTS_SYSTEM_LOWBATT            EQU BIT_MASK_4
BM_ATA5700_EVENTS_SYSTEM_EXTCLOCK_ERR       EQU BIT_MASK_3
BM_ATA5700_EVENTS_SYSTEM_TIMER_0            EQU BIT_MASK_0

/* g_sAta5700_flash.EVENTS_WAKEUP*/
ATA5700_EVENTS_POWER_PWRON                 EQU BIT_7
ATA5700_EVENTS_POWER_NPWRON6               EQU BIT_6
ATA5700_EVENTS_POWER_NPWRON5               EQU BIT_5
ATA5700_EVENTS_POWER_NPWRON4               EQU BIT_4
ATA5700_EVENTS_POWER_NPWRON3               EQU BIT_3
ATA5700_EVENTS_POWER_NPWRON2               EQU BIT_2
ATA5700_EVENTS_POWER_NPWRON1               EQU BIT_1
ATA5700_EVENTS_POWER_NPWRON0               EQU BIT_0

BM_ATA5700_EVENTS_POWER_PWRON              EQU BIT_MASK_7
BM_ATA5700_EVENTS_POWER_NPWRON6            EQU BIT_MASK_6
BM_ATA5700_EVENTS_POWER_NPWRON5            EQU BIT_MASK_5
BM_ATA5700_EVENTS_POWER_NPWRON4            EQU BIT_MASK_4
BM_ATA5700_EVENTS_POWER_NPWRON3            EQU BIT_MASK_3
BM_ATA5700_EVENTS_POWER_NPWRON2            EQU BIT_MASK_2
BM_ATA5700_EVENTS_POWER_NPWRON1            EQU BIT_MASK_1
BM_ATA5700_EVENTS_POWER_NPWRON0            EQU BIT_MASK_0

ATA5700_EVENTS_RF_TX_0_TX_ERR                 EQU BIT_7
ATA5700_EVENTS_RF_TX_0_TX_RDY                 EQU BIT_6
ATA5700_EVENTS_RF_TX_0_TX_DFIFO_ERR           EQU BIT_4
ATA5700_EVENTS_RF_TX_0_TX_SFIFO_ERR           EQU BIT_3
ATA5700_EVENTS_RF_TX_0_TX_DFL_BUF_FILLLVL     EQU BIT_2
ATA5700_EVENTS_RF_TX_0_TX_SFL_BUF_FILLLVL     EQU BIT_1
ATA5700_EVENTS_RF_TX_0_EOT                    EQU BIT_0

BM_ATA5700_EVENTS_RF_TX_0_TX_ERR                EQU BIT_MASK_7
BM_ATA5700_EVENTS_RF_TX_0_TX_RDY                EQU BIT_MASK_6
BM_ATA5700_EVENTS_RF_TX_0_TX_DFIFO_ERR          EQU BIT_MASK_4
BM_ATA5700_EVENTS_RF_TX_0_TX_SFIFO_ERR          EQU BIT_MASK_3
BM_ATA5700_EVENTS_RF_TX_0_TX_DFL_BUF_FILLLVL    EQU BIT_MASK_2
BM_ATA5700_EVENTS_RF_TX_0_TX_SFL_BUF_FILLLVL    EQU BIT_MASK_1
BM_ATA5700_EVENTS_RF_TX_0_EOT                   EQU BIT_MASK_0


/* g_sAta5700_flash.EVENTS_RF_TX_1 */
ATA5700_EVENTS_RF_TX_1_ANT_TUNE_RDY           EQU BIT_7
ATA5700_EVENTS_RF_TX_1_VCO_TUNE_RDY           EQU BIT_6
ATA5700_EVENTS_RF_TX_1_FRC_CALIB_RDY          EQU BIT_5
ATA5700_EVENTS_RF_TX_1_SRC_CALIB_RDY          EQU BIT_4

BM_ATA5700_EVENTS_RF_TX_1_ANT_TUNE_RDY        EQU BIT_MASK_7
BM_ATA5700_EVENTS_RF_TX_1_VCO_TUNE_RDY        EQU BIT_MASK_6
BM_ATA5700_EVENTS_RF_TX_1_FRC_CALIB_RDY       EQU BIT_MASK_5
BM_ATA5700_EVENTS_RF_TX_1_SRC_CALIB_RDY       EQU BIT_MASK_4


/* g_sAta5700_flash.EVENTS_PH_0 */
ATA5700_EVENTS_PH_FLAGS_0_PHCOF             EQU BIT_7
ATA5700_EVENTS_PH_FLAGS_0_PHID1F            EQU BIT_5
ATA5700_EVENTS_PH_FLAGS_0_PHID0F            EQU BIT_4
ATA5700_EVENTS_PH_FLAGS_0_PHIDFF            EQU BIT_3
ATA5700_EVENTS_PH_FLAGS_0_PHDFF             EQU BIT_2
ATA5700_EVENTS_PH_FLAGS_0_PHTBLF            EQU BIT_1
ATA5700_EVENTS_PH_FLAGS_0_CRCEF             EQU BIT_0

BM_ATA5700_EVENTS_PH_FLAGS_0_PHCOF          EQU BIT_MASK_7
BM_ATA5700_EVENTS_PH_FLAGS_0_PHID1F         EQU BIT_MASK_5
BM_ATA5700_EVENTS_PH_FLAGS_0_PHID0F         EQU BIT_MASK_4
BM_ATA5700_EVENTS_PH_FLAGS_0_PHIDFF         EQU BIT_MASK_3
BM_ATA5700_EVENTS_PH_FLAGS_0_PHDFF          EQU BIT_MASK_2
BM_ATA5700_EVENTS_PH_FLAGS_0_PHTBLF         EQU BIT_MASK_1
BM_ATA5700_EVENTS_PH_FLAGS_0_CRCEF          EQU BIT_MASK_0


/* g_sAta5700_flash.EVENTS_PH_1 */
ATA5700_EVENTS_PH_FLAGS_1_OFLF              EQU BIT_2
ATA5700_EVENTS_PH_FLAGS_1_UFLF              EQU BIT_1
ATA5700_EVENTS_PH_FLAGS_1_FLRF              EQU BIT_0

BM_ATA5700_EVENTS_PH_FLAGS_1_OFLF           EQU BIT_MASK_2
BM_ATA5700_EVENTS_PH_FLAGS_1_UFLF           EQU BIT_MASK_1
BM_ATA5700_EVENTS_PH_FLAGS_1_FLRF           EQU BIT_MASK_0


/* g_sAta5700_flash.EVENTS_LF */
ATA5700_EVENTS_LF_FLAGS_RSSI_ERR            EQU BIT_7
ATA5700_EVENTS_LF_FLAGS_RSSI_RDY            EQU BIT_6
ATA5700_EVENTS_LF_FLAGS_LF_RDY              EQU BIT_4
ATA5700_EVENTS_LF_FLAGS_LFTOF               EQU BIT_3
ATA5700_EVENTS_LF_FLAGS_FLEOF               EQU BIT_2
ATA5700_EVENTS_LF_FLAGS_LFDEF               EQU BIT_1
ATA5700_EVENTS_LF_FLAGS_LFSYDF              EQU BIT_0

BM_ATA5700_EVENTS_LF_FLAGS_RSSI_ERR         EQU BIT_MASK_7
BM_ATA5700_EVENTS_LF_FLAGS_RSSI_RDY         EQU BIT_MASK_6
BM_ATA5700_EVENTS_LF_FLAGS_LF_RDY           EQU BIT_MASK_4
BM_ATA5700_EVENTS_LF_FLAGS_LFTOF            EQU BIT_MASK_3
BM_ATA5700_EVENTS_LF_FLAGS_FLEOF            EQU BIT_MASK_2
BM_ATA5700_EVENTS_LF_FLAGS_LFDEF            EQU BIT_MASK_1
BM_ATA5700_EVENTS_LF_FLAGS_LFSYDF           EQU BIT_MASK_0


/* g_sAta5700_flash.EVENTS_TP */
ATA5700_EVENTS_TP_FLAGS_TPBERF            EQU BIT_3
ATA5700_EVENTS_TP_FLAGS_TPNFTF            EQU BIT_2
ATA5700_EVENTS_TP_FLAGS_TPFTF             EQU BIT_1
ATA5700_EVENTS_TP_FLAGS_TPF               EQU BIT_0

BM_ATA5700_EVENTS_TP_FLAGS_TPBERF         EQU BIT_MASK_3
BM_ATA5700_EVENTS_TP_FLAGS_TPNFTF         EQU BIT_MASK_2
BM_ATA5700_EVENTS_TP_FLAGS_TPFTF          EQU BIT_MASK_1
BM_ATA5700_EVENTS_TP_FLAGS_TPF            EQU BIT_MASK_0

/* g_sAta5700_flash.events_reset */
ATA5700_EVENTS_RESET_LF_CALRY           EQU BIT_7
ATA5700_EVENTS_RESET_LF_TPRF            EQU BIT_5
ATA5700_EVENTS_RESET_LF_JTRF            EQU BIT_4
ATA5700_EVENTS_RESET_LF_WDRF            EQU BIT_3
ATA5700_EVENTS_RESET_LF_EXTRF           EQU BIT_1
ATA5700_EVENTS_RESET_LF_PORF            EQU BIT_0

BM_ATA5700_EVENTS_RESET_LF_CALRY        EQU BIT_MASK_7
BM_ATA5700_EVENTS_RESET_LF_TPRF         EQU BIT_MASK_5
BM_ATA5700_EVENTS_RESET_LF_JTRF         EQU BIT_MASK_4
BM_ATA5700_EVENTS_RESET_LF_WDRF         EQU BIT_MASK_3
BM_ATA5700_EVENTS_RESET_LF_EXTRF        EQU BIT_MASK_1
BM_ATA5700_EVENTS_RESET_LF_PORF         EQU BIT_MASK_0

/* g_sAta5700_flash.status */
ATA5700_STATUS_UPDATE_FLAG                  EQU BIT_7
ATA5700_STATUS_TRIGGER_MAIN_LOOP_FLAG       EQU BIT_6
ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG     EQU BIT_5
ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG   EQU BIT_4
ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG   EQU BIT_3

BM_ATA5700_STATUS_UPDATE_FLAG                   EQU BIT_MASK_7
BM_ATA5700_STATUS_TRIGGER_MAIN_LOOP_FLAG        EQU BIT_MASK_6
BM_ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG      EQU BIT_MASK_5
BM_ATA5700_STATUS_FRC_CALIB_IN_PROGRESS_FLAG    EQU BIT_MASK_4
BM_ATA5700_STATUS_SRC_CALIB_IN_PROGRESS_FLAG    EQU BIT_MASK_3

/* g_sAta5700_flash.events_components */
ATA5700_EVENTS_COMPONENTS_AES_ERR  EQU  BIT_3
ATA5700_EVENTS_COMPONENTS_AES_RDY  EQU  BIT_2
ATA5700_EVENTS_COMPONENTS_I2C_ERR  EQU  BIT_1
ATA5700_EVENTS_COMPONENTS_I2C_RDY  EQU  BIT_0

BM_ATA5700_EVENTS_COMPONENTS_AES_ERR  EQU BIT_MASK_3
BM_ATA5700_EVENTS_COMPONENTS_AES_RDY  EQU BIT_MASK_2
BM_ATA5700_EVENTS_COMPONENTS_I2C_ERR  EQU BIT_MASK_1
BM_ATA5700_EVENTS_COMPONENTS_I2C_RDY  EQU BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* sExtReq                                                                   */
/* ------------------------------------------------------------------------- */
EXT_REQ_TUNE_CHECK_CONFIG        EQU 0
EXT_REQ_SYSTEM_MODE_CONFIG       EQU EXT_REQ_TUNE_CHECK_CONFIG + 1
EXT_REQ_SERVICE_CHANNEL_CONFIG   EQU EXT_REQ_SYSTEM_MODE_CONFIG + 1
EXT_REQ_SERVICE_INIT_CONFIG      EQU EXT_REQ_SERVICE_CHANNEL_CONFIG + 1
EXT_REQ_MISC_TRIGGER             EQU EXT_REQ_SERVICE_INIT_CONFIG + 1
EXT_REQ_LFRCRFTX_CONFIG          EQU EXT_REQ_MISC_TRIGGER + 1
EXT_REQ_TP_EM_MODE_CONFIG        EQU EXT_REQ_LFRCRFTX_CONFIG + 1
EXT_REQ_RF_REMOTE_KEYLESS_ENTRY  EQU EXT_REQ_TP_EM_MODE_CONFIG + 1

/* ------------------------------------------------------------------------- */
/* sExtReq.systemModeConfig                                                  */
/* ------------------------------------------------------------------------- */
BM_EXT_REQ_SYSTEM_MODE_CONFIG_CLKIDLE_SELECTOR      EQU BIT_MASK_4

/* ------------------------------------------------------------------------- */
/* sExtReq.serviceInitConfig                                                 */
/* ------------------------------------------------------------------------- */
EXT_REQ_SERVICE_INIT_CONFIG_UPDATE_FLAG         EQU BIT_7
EXT_REQ_SERVICE_INIT_CONFIG_EEPROM_NUMBER       EQU BIT_1
EXT_REQ_SERVICE_INIT_CONFIG_SRAM_NUMBER         EQU BIT_0

BM_EXT_REQ_SERVICE_INIT_CONFIG_UPDATE_FLAG      EQU BIT_MASK_7
BM_EXT_REQ_SERVICE_INIT_CONFIG_EEPROM_NUMBER    EQU ( BIT_MASK_1 | BIT_MASK_2 )
BM_EXT_REQ_SERVICE_INIT_CONFIG_SRAM_NUMBER      EQU BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* sExtReq.miscTrigger                                                       */
/* ------------------------------------------------------------------------- */
EXT_REQ_MISC_TRIGGER_START_RSSI     EQU BIT_0

BM_EXT_REQ_MISC_TRIGGER_START_RSSI  EQU BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* sTrxConf                                                                  */
/* ------------------------------------------------------------------------- */
TRXCONF_TUNE_CHECK_CONFIG       EQU 0
TRXCONF_SYSTEM_MODE_CONFIG      EQU TRXCONF_TUNE_CHECK_CONFIG + 1
TRXCONF_SERVICE_CHANNEL_CONFIG  EQU TRXCONF_SYSTEM_MODE_CONFIG + 1

/*stopSimExtraction*/

#endif
#endif /* ATA5700_COMMAND_SET_H */

