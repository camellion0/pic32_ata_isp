//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/rftx/src/rftx.h $
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
/** \file rftx.h
 */
//lint -restore

#ifndef RFTX_H
#define RFTX_H

#ifdef __IAR_SYSTEMS_ICC__
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
#include "rftx_defs.h"

/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/
/** \brief <b>sysFlowStateMachineFunc_t</b>
    is used for function pointer definition of system flow state machines.
*/
typedef void (*sysFlowStateMachineFunc_t)(void);

/** \brief <b>sysFlowStateMachineFuncLut_t</b>
    is used for function pointer definition of system flow state machine
    look up tables in code section.
*/
typedef const sysFlowStateMachineFunc_t __flash sysFlowStateMachineFuncLut_t;

/** \brief <b>sSystemFlowCtrl</b>
    contains the information to control the flow of the software state machines.
*/
typedef struct{
    /** brief <b>bIndex</b>
        contains the current state machine index
    */
    volatile uint8_t bIndex;

    /** \brief <b>fpLut</b>
        is used as pointer to currently used state machine look up table
    */
    sysFlowStateMachineFuncLut_t *fpLut;
}sSystemFlowCtrl;

typedef struct {
    /** \brief <b>bFlags</b>
        contains the RF Tx module external flags
    
        \li Bit 7: RF Tx module error flag (0: Not Set, 1: Set)
        \li Bit 6: RF Tx module ready for transmission (0: Not Ready, 1: Ready)
        \li Bit 5: rfu
        \li Bit 4: DFIFO overflow/underflow error (0: Not Set, 1: Set)
        \li Bit 3: SFIFO overflow/underflow error (0: Not Set, 1: Set)
        \li Bit 2: DFIFO fill level reached (0: Not reached, 1: Reached)
        \li Bit 1: SFIFO fill level reached (0: Not reached, 1: Reached)
        \li Bit 0: End of telegram reached (0: Not reached, 1: Reached)
     */
    uint8_t bFlags;

    /** \brief <b>bTuneFlags</b>
        contains the RF Tx module tuning status 
    
        \li Bit 7: Antenna Tuning ready flag (0: No, 1: Yes)
        \li Bit 6: VCO Tuning ready flag (0: No, 1: Yes)
        \li Bit 5..0: rfu
     */
    uint8_t bTuneFlags;
    
     /** \brief <b>bStatus</b>
        contains the RF Tx module status
    
        \li Bit 7: Direct Switching (0: No, 1: Yes)
        \li Bit 6: RF Tx SSM ready flag (0: SSM not ready, 1: SSM ready)
        \li Bit 5: RF Tx module transmission complete (0: No, 1: Yes)
        \li Bit 4: RF Tx module active flag (0: Running, 1: finished/not started)
        \li Bit 3..0: rfu
     */
    uint8_t bStatus;

    /** \brief <b>bConfig</b>
        contains the RF Tx module configuration
    
        \li Bit 7: Antenna Tuning (0: No, 1: Yes)
        \li Bit 6: VCO Tuning (0: No, 1: Yes)
        \li Bit 5: TX Mode (0: Buffered, 1: Transparent)
        \li Bit 4: ShutDown TXMode (0: IDLEMode(RC), 1: IDLEMode(XTO))
        \li Bit 3: Service location (0: SRAM, 1: EEPROM)
        \li Bit 2: stay in TX after transmission complete (0: No, 1: Yes)
        \li Bit 1..0: Channel
                    - Channel 1 = 0
                    - Channel 2 = 1
                    - Channel 3 = 2
     */
    uint8_t bConfig;

    /** \brief <b>pAddress</b>
        contains the address of the service configuration (SRAM or EEPROM)
     */
    uint8_t *pAddress;
    
    /* --P2P-3655-- */
    /** \brief <b>bCmcrSetting</b>
        contains the original cmcr setting
     */
    uint8_t bCmcrSetting;
}sRfTxConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sRfTxServicePathConfig</b>
    is the path specific configuration structure for the RF TX feature
*/
/*----------------------------------------------------------------------------- */
typedef struct {
    /** \brief <b>btxDev</b>
        frequency deviation settings (HighByte and LowByte), will be used for SW calculations
     */
    uint8_t btxDev[2];
     /** \brief <b>bGACDIV</b>
        Gauss Clock Divider Register settings
     */
    uint8_t bGACDIV[2];
    /** \brief <b>bFSFCR</b>
        Tx DSP Filter Control Register settings
     */
    uint8_t bFSFCR;
    /** \brief <b>bTXDR</b>
        TX Data Rate setting (HighByte and LowByte) will be used for SW
        calculations
     */
    uint8_t bTXDR[2];
    /** \brief <b>txSetPath</b>
        is used for TXMode configuration

        <b>bTxSetPath[0]</b>
        \li Bit 7: GAUS
        \li Bit 6: PREE
        \li Bit 5..Bit 0: StartTxDataFillLevel<br>

        <b>bTxSetPath[1]</b>
        \li Bit 7: TX modulation (0=FSK/1=ASK)
        \li Bit 6: ASK shaping enable
        \li Bit 5: rfu
        \li Bit 4..0: StartPreambleFillLevel
     */
    uint8_t bTxSetPath[2];
    /** \brief <b>bTxSysEvent</b>
        is used for event configuration in TXMode for transmission
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5..Bit 0: TxBufFillLevel
     */
    uint8_t bTxSysEvent;
    /** \brief <b>bTxPreambleSysEvent</b>
        is used preamble buffer event configuration in TXMode for transmission
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4..Bit 0: PreambleBufFillLevel
     */
    uint8_t bTxPreambleSysEvent;
    /** \brief <b>bTMCR2</b>
        Tx Modulator Control Register 2
     */
    uint8_t bTMCR2;
    /** \brief <b>bTMSSC</b>
        Tx Modulator Stop Sequence Configuration
     */
    uint8_t bTMSSC;
    /** \brief <b>bTMTL</b>
        Tx Modulator Telegram Length
     */
    uint8_t bTMTL[2];
    /** \brief <b>bTMCP</b>
        Tx Modulator CRC Polynomial
     */
    uint8_t bTMCP[2];
    /** \brief <b>bTMCI</b>
        Tx Modulator CRC Init value
     */
    uint8_t bTMCI[2];
    /** \brief <b>bTMCSB</b>
        Tx Modulator CRC Skip Bit number
     */
    uint8_t bTMCSB;
}sRfTxServicePathConfig;


/*----------------------------------------------------------------------------- */
/** \brief <b>sRfTxServiceSpecificConfig</b>
    is the service specific configuration structure for the RF TX feature
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>bSSMFBR</b>
        contains the SSMFBR register setting
     */
    uint8_t bSSMFBR;

    /** \brief <b>bFEALR_FEANT</b>
        this variable contains a combination of FEANT and FEALR register
        \li Bit 7:    rfu
        \li Bit 6:    rfu
        \li Bit 5:    FEALR.RNGE1
        \li Bit 4:    FEALR.RNGE0
        \li Bit 3:    FEANT.LVLC3
        \li Bit 2:    FEANT.LVLC2
        \li Bit 1:    FEANT.LVLC1
        \li Bit 0:    FEANT.LVLC0
     */
    uint8_t bFEALR_FEANT;
    /** \brief <b>bFEAT</b>
        Frontend Amplifier Bias and Antenna Tuning Register settings
     */
    uint8_t bFEAT;
    /** \brief <b>bFEPAC</b>
        Frontend Power Amplifier Register settings
     */
    uint8_t bFEPAC;
    /** \brief <b>bFEVCO</b>
        Frontend VCO and PLL Control Register settings
     */
    uint8_t bFEVCO;
    /** \brief <b>bFEVCT</b>
        Frontend VCO Tuning Register settings
     */
    uint8_t bFEVCT;
    /** \brief <b>bIF</b>
        IF setting (HighByte and LowByte) will be used for SW calculations
     */
    uint8_t bIF[2];
}sRfTxServiceSpecificConfig;
/*----------------------------------------------------------------------------- */
/** \brief <b>sRfTxServiceConfig</b>
    is the service specific configuration structure for the RF TX feature
*/
/*----------------------------------------------------------------------------- */
typedef struct{

    sRfTxServiceSpecificConfig sServiceConfig;
    sRfTxServicePathConfig     sPathConfig[NUM_PATHES_PER_SERVICE];
}sRfTxServiceConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sRfTxChannelConfig</b>
    is the channel specific configuration structure for RF TX functionality
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief  <b>bFFREQ</b>
        3 bytes FFRQ base setting
     */
    uint8_t bFFREQ[3];
    /** \brief  <b>bFEMS</b>
        Frontend Mode and Swallow Register setting
        \li Bit 7..4: PLL Mode
        \li Bit 3..0: PLL Swallow
     */
    uint8_t bFEMS;
    /** \brief <b>bFECR</b>
        Frontend Control Register settings for temperature measurement channel
        \li  Bit 7..6:  rfu
        \li  Bit 5:     ANPS
        \li  Bit 4:     PLCKG
        \li  Bit 3:     ADHS
        \li  Bit 2:     ANDP
        \li  Bit 1:     S4N3
        \li  Bit 0:     LBNHB
     */
    uint8_t bFECR;
}sRfTxChannelConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sRfTxCurrentServiceChannelConfiguration</b>
    is the service configuration structure used for current service, which
    is a copy from EEPROM/SRAM service and used for HW initialization
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    sRfTxServiceSpecificConfig  sService;
    sRfTxServicePathConfig      sPath;
    sRfTxChannelConfig          sChannel;
}sRfTxCurrentServiceChannelConfiguration;

/*----------------------------------------------------------------------------- */
/** \brief <b>sRfTxServiceChannelConfig</b>
    is the service configuration structure used for SRAM/EEPROM services
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>sService</b>
        contains the service specific part of a service configuration
     */
    sRfTxServiceConfig  sService;
    /** \brief <b>sChannel</b>
        contains the channel specific part of a service configuration
     */
    sRfTxChannelConfig sChannel[NUM_CHANNELS_PER_SERVICE];
}sRfTxServiceChannelConfig;

/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/
extern VOIDFUNC ATA_rfTxInit_C(void);
extern VOIDFUNC ATA_rfTxClose_C(void);

extern VOIDFUNC ATA_rfTxStartTx_C(uint8_t config, uint8_t *pAddress);
extern VOIDFUNC ATA_rfTxFillDFifo_C(uint8_t len, uint8_t *pData);
extern VOIDFUNC ATA_rfTxFillSFifo_C(uint8_t len, uint8_t *pData);

extern VOIDFUNC ATA_rfTxProcessing_C(void);
extern VOIDFUNC ATA_rfTxStop_C(void);

extern VOIDFUNC ATA_rfTxInitTxSSM_C(void);
extern VOIDFUNC ATA_rfTxWait4AVCC_C(void);
extern VOIDFUNC ATA_rfTxInitFrontEnd_C(void);
extern VOIDFUNC ATA_rfTxWait4XTO_C(void);
extern VOIDFUNC ATA_rfTxStartSSM_C(uint8_t bState);
extern VOIDFUNC ATA_rfTxStartTxBufSSM_C(void);
extern VOIDFUNC ATA_rfTxStartTxTransSSM_C(void);
extern VOIDFUNC ATA_rfTxWait4SSMrdy_C(void);
extern VOIDFUNC ATA_rfTxTransparentMode_C(void);
extern VOIDFUNC ATA_rfTxWait4FillLevel_C(void);
extern VOIDFUNC ATA_rfTxStartTxModulator_C(void);
extern VOIDFUNC ATA_rfTxWait4TransmissionComplete_C(void);
extern VOIDFUNC ATA_rfTxShutdown_C(void);

extern VOIDFUNC ATA_rfTxInitCurrentService_C(void);

extern UINT8FUNC ATA_rfTxStartSsmWatchdog_C(void);
extern VOIDFUNC ATA_rfTxStopSsmWatchdog_C(void);

extern sRfTxConfig g_sRfTx;
extern sRfTxCurrentServiceChannelConfiguration g_sRfTxCurrentService;

extern sysFlowStateMachineFuncLut_t *g_pRfTxBufStateMachine;
extern sysFlowStateMachineFuncLut_t *g_pRfTxTransStateMachine;
extern sSystemFlowCtrl g_sRfTxFlowCtrl;

extern sysFlowStateMachineFuncLut_t g_sRfTxBufStateMachineLut[];
extern sysFlowStateMachineFuncLut_t g_sRfTxTransStateMachineLut[];

#elif defined __IAR_SYSTEMS_ASM__
#endif
#endif /* RFTX_H */
