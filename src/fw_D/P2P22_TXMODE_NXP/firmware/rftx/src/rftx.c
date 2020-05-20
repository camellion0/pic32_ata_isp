//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/rftx/src/rftx.c $
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
/** \file rftx.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "rftx.h"
#include "rftx_vco.h"
#include "rftx_ant.h"

#include "..\..\eep\src\eep.h"
#include "..\..\timer2\src\timer2.h"
#include "..\..\timer3\src\timer3.h"
#include "..\..\globals\src\globals.h"
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/

static VOIDFUNC  ATA_rfTxOpenTimer3forTxing_C(void);
static VOIDFUNC  ATA_rfTxFrequencySettings_C(void);
static VOIDFUNC  ATA_rfTxSetupTxModulator_C(void);
static VOIDFUNC  ATA_rfTxConfigureTxSSM_C(void);
static UINT8FUNC ATA_rfTxFevcoCalculation_C(void);
static VOIDFUNC  ATA_rfTxEnd_C(void);
static VOIDFUNC  ATA_rfTxStartSendTelegramSSM_C(void);
static VOIDFUNC  ATA_rfTxStartShutDownSSM_C(void);

/** \brief <b>g_sRfTx</b>
    contains the configuration and status information for module RF Tx.
 */
#pragma location = ".rftx"
__root __no_init sRfTxConfig g_sRfTx;

/** \brief <b>g_sRfTxCurrentService</b>
    contains the service configuration of the currently used service/channel
    configuration. The selected configuration is copied to
    ::ATA_rfTxInitCurrentService_C at startup of RFTX.
 */
#pragma location = ".rftx"
__root __no_init sRfTxCurrentServiceChannelConfiguration g_sRfTxCurrentService;

/** \brief <b>flowCtrl</b>
    is used for software state machine control.
    The complete SW state machine flow is controlled with this variable.
*/
#pragma location = ".flowctrl"
__root __no_init sSystemFlowCtrl g_sRfTxFlowCtrl;

/** \brief <b>g_pRfTxBufStateMachine</b>
    points to look up table of the SW state machine used for
    TXMode(Buffered).
    \details
    This pointer is initialized at RFTX module initialization
    in function ::ATA_rfTxInit_C. If the user wants to change this flow,
    the pointer should be modified after initialization.
*/
#pragma location = ".flowctrl"
__root __no_init sysFlowStateMachineFuncLut_t *g_pRfTxBufStateMachine;

//lint -esym(9003, g_sRfTxBufStateMachineLut) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sRfTxBufStateMachineLut' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
/** <b>g_sRfTxBufStateMachineLut</b>
    look up table of the SW state machine used for TXMode (Buffered only).
    \details
    \li ATA_rfTxInitTxSSM_C:
        this sw state does the register initialization according to the selected
        service channel configuration. Note: FE register are initialized after AVCC
        is stable in SW state ::ATA_rfTxInitFrontEnd_C
    \li ATA_rfTxWait4AVCC_C:
        waits until AVCC is stable and switches to next sw state if AVCC is stable.
    \li ATA_rfTxInitFrontEnd_C:
        initializes the frontend registers according to the selected service channel
        configuration
    \li ATA_rfTxWait4XTO_C:
        waits until XTO is ready. If AVR is not running with CLKXTO4 the
        core clock is switched to CLKXTO4
    \li ATA_rfTxStartSSM_C:
        starts the sequencer state machine
    \li ATA_rfTxWait4SSMrdy_C:
        waits until the sequencer state machine is ready
    \li ATA_rfTxWait4FillLevel_C:
        checks the fill levels of S-FIFO and D-FIFO
    \li ATA_rfTxStartTxModulator_C:
        configures the fill level interrupts, starts the TX Modulator and
        SendTelegramSSM
    \li ATA_rfTxWait4TransmissionComplete_C:
        waits until Transmission has completed
    \li ATA_rfTxShutdown_C:
        shut down the RFTX module related state machine for TXMode(buffered and transparent),
        VCO tuning and antenna tuning.

 */
__root sysFlowStateMachineFuncLut_t g_sRfTxBufStateMachineLut[] = {
    ATA_rfTxInitTxSSM_C,
    ATA_rfTxWait4AVCC_C,
    ATA_rfTxInitFrontEnd_C,
    ATA_rfTxWait4XTO_C,
    ATA_rfTxStartTxBufSSM_C,
    ATA_rfTxWait4SSMrdy_C,
    ATA_rfTxWait4FillLevel_C,
    ATA_rfTxStartTxModulator_C,
    ATA_rfTxWait4TransmissionComplete_C,
    ATA_rfTxShutdown_C
};

/** \brief <b>g_pRfTxTransStateMachine</b>
    points to look up table of the SW state machine used for
    TXMode(Transparent mode only).
    \details
    This pointer is initialized at RFTX module initialization
    in function ATA_rfTxInit_C. If the user wants to change this flow,
    the pointer should be modified after initialization.
*/
#pragma location = ".flowctrl"
__root __no_init sysFlowStateMachineFuncLut_t *g_pRfTxTransStateMachine;


//lint -esym(9003, g_sRfTxTransStateMachineLut) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sRfTxTransStateMachineLut' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
/** <b>g_sRfTxTransStateMachineLut</b>
    look up table of the SW state machine used for TXMode (Transparent mode only).
    \details
    \li ATA_rfTxInitTxSSM_C:
        this SW state does the register initialization according to the selected
        service channel configuration. Note: FE register are initialized after AVCC
        is stable in SW state ::ATA_rfTxInitFrontEnd_C
    \li ATA_rfTxWait4AVCC_C:
        waits until AVCC is stable and switches to next sw state if AVCC is stable.
    \li ATA_rfTxInitFrontEnd_C:
        initializes the frontend registers according to the selected service channel
        configuration
    \li ATA_rfTxWait4XTO_C:
        waits until XTO is ready. If AVR is not running with CLKXTO4 the
        core clock is switched to CLKXTO4
    \li ATA_rfTxStartSSM_C:
        starts the sequencer state machine
    \li ATA_rfTxWait4SSMrdy_C:
        waits until the sequencer state machine is ready
    \li ATA_rfTxTransparentMode_C:
        is an empty function. This sw state is reached after TXMode(transparent)
        has started correctly
    \li ATA_rfTxShutdown_C:
        shut down the RFTX module related state machine for TXMode(buffered and transparent),
        VCO tuning and antenna tuning.

 */
__root sysFlowStateMachineFuncLut_t g_sRfTxTransStateMachineLut[] = {
    ATA_rfTxInitTxSSM_C,
    ATA_rfTxWait4AVCC_C,
    ATA_rfTxInitFrontEnd_C,
    ATA_rfTxWait4XTO_C,
    ATA_rfTxStartTxTransSSM_C,
    ATA_rfTxWait4SSMrdy_C,
    ATA_rfTxTransparentMode_C,
    ATA_rfTxShutdown_C
};


/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxInit_C</b>
    initializes the RFTX module. This function should be executed before
    starting RFTX.

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_pRfTxBufStateMachine Global RF Tx (buffered) statemachine
    \li [out] ::g_pRfTxTransStateMachine Global RF Tx (transparent) statemachine
    \li [out] ::g_pRfTxVcoTuneStateMachine Global RF Tx (VCO tuning) statemachine
    \li [out] ::g_pRfTxAntTuneStateMachine Global RF Tx (Antenna tuning) statemachine
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data
    \li [out] ::g_bAntennaTuningResult Global RF Tx Antenna tuning component data
    \li [in] ::g_sRfTxBufStateMachineLut Global RF Tx (buffered) statemachine lookup table
    \li [in] ::g_sRfTxTransStateMachineLut Global RF Tx (transparent) statemachine lookup table
    \li [in] ::g_sRfTxVcoTuneStateMachineLut Global RF Tx (VCO tuning) statemachine lookup table
    \li [in] ::g_sRfTxAntTuneStateMachineLut Global RF Tx (Antenna tuning) statemachine lookup table

    \image html ATA_rfTxInit_C.png

    \internal
    \li 010: Power-up/initialize D-FIFO and S-FIFO used for data bufferint in RFTX
    \li 020: Enable overflow/underflow interrupts for D-FIFO and S-FIFO to signal
              errorneous usage of D-FIFO and S-FIFO to the customer application
    \li 030: Initialize the state machine pointers for controlling
                 - TXMode buffered (::g_pRfTxBufStateMachine)
                 - TXMode transparent (::g_pRfTxTransStateMachine)
                 - VCO Tuning (::g_pRfTxVcoTuneStateMachine)
                 - Antenna Tuning (::g_pRfTxAntTuneStateMachine)
    \li 040: Initialize RFTX Module global variables which contains
                 - VCO tuning result (::g_bVcoTuningResult)
                 - Antenna tuning result (::g_bAntennaTuningResult)
    \li 050: Initialize RFTX Module configuration ::g_sRfTx

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-844}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxInit_C()
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxInit_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_POWERON_C(PRR2, PRDF)
    ATA_POWERON_C(PRR2, PRSF)

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(DFI,BM_DFERIM)
    ATA_SETBITMASK_C(SFI,BM_SFERIM)

    /* LLR-Ref: 030 */
    g_pRfTxBufStateMachine     = g_sRfTxBufStateMachineLut;
    g_pRfTxTransStateMachine   = g_sRfTxTransStateMachineLut;
    g_pRfTxVcoTuneStateMachine = g_sRfTxVcoTuneStateMachineLut;
    g_pRfTxAntTuneStateMachine = g_sRfTxAntTuneStateMachineLut;

    /* LLR-Ref: 040 */
    g_bVcoTuningResult = 0U;
    g_bAntennaTuningResult = 0U;

    /* LLR-Ref: 050 */
    g_sRfTx.bFlags      = 0x00U;
    g_sRfTx.bTuneFlags  = 0x00U;
    g_sRfTx.bStatus     = 0x00U;
    g_sRfTx.bConfig     = 0x00U;
    g_sRfTx.pAddress   = 0x0000U;

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxClose_C</b>
    closes the RFTX module by disabling RFTX specific parts.
    
    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data
    \li [out] ::g_bAntennaTuningResult Global RF Tx Antenna tuning component data

    \image html ATA_rfTxClose_C.png

    \internal
    \li 010: Close Timer2 by calling function ::ATA_timer2Close_C and Timer3 by
             calling function ::ATA_timer3Close_C to allow usage in customer
             application
    \li 020: Remove power from the following HW IPs
                 - TXDSP module
                 - CRC module
                 - D-FIFO
                 - S-FIFO
                 - Sequencer State Machine
                 - TX Modulator module
    \li 030: Reset RFTX Module global variables which contains
                 - VCO tuning result (::g_bVcoTuningResult)
                 - Antenna tuning result (::g_bAntennaTuningResult)
    \li 040: Reset RFTX Module configuration ::g_sRfTx

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-846}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxClose_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxClose_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_timer2Close_C();
    ATA_timer3Close_C();

    /* LLR-Ref: 020 */
    ATA_POWEROFF_C(PRR0, PRTXDC)
    ATA_POWEROFF_C(PRR0, PRCRC)

    ATA_POWEROFF_C(PRR2, PRDF)
    ATA_POWEROFF_C(PRR2, PRSF)
    ATA_POWEROFF_C(PRR2, PRSSM)
    ATA_POWEROFF_C(PRR2, PRTM)

    /* LLR-Ref: 030 */
    g_bVcoTuningResult = 0U;
    g_bAntennaTuningResult = 0U;

    /* LLR-Ref: 040 */
    g_sRfTx.bFlags      = 0x00U;
    g_sRfTx.bTuneFlags  = 0x00U;
    g_sRfTx.bStatus     = 0x00U;
    g_sRfTx.bConfig     = 0x00U;
    g_sRfTx.pAddress   = 0x0000U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxFillDFifo_C</b>
    writes payload data to the D-FIFO.

    \param[in]      bLen     Number of payload bytes to be written to D-FIFO
    \param[in]      pData    Pointer to the start of payload bytes

    \image html ATA_rfTxFillDFifo_C.png

    \internal
    \li 010: Set data direction of D-FIFO to TX and store previous setting
    \li 020: Copy length payload bytes starting from address pData to DFD
              register of D-FIFO.
    \li 030: Restore data direction of D-FIFO

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-845}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxFillDFifo_C(uint8_t bLen, uint8_t *pData)
{
    /* LLR-Ref: 010 */
    uint8_t tmp = DFC & BM_DFDRA;
    ATA_SETBITMASK_C(DFC,BM_DFDRA)

    /* LLR-Ref: 020 */
    for(uint8_t i=0; i<bLen;i++){
        DFD = *pData++;
    }

    /* LLR-Ref: 030 */
    if (!tmp) {
        ATA_CLEARBITMASK_C(DFC,BM_DFDRA)
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxFillSFifo_C</b>
    writes preamble data to the S-FIFO.

    \param[in]      bLen     Number of payload bytes to be written to S-FIFO
    \param[in]      pData    Pointer to the start of preamble bytes

    \image html ATA_rfTxFillSFifo_C.png

    \internal
    \li 010: Set data direction of S-FIFO to TX and store previous setting
    \li 020: Copy length preamble bytes starting from address pData to SFD
              register of S-FIFO
    \li 030: Restore data direction of S-FIFO

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-845}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxFillSFifo_C(uint8_t bLen, uint8_t *pData)
{
    /* LLR-Ref: 010 */
    uint8_t tmp = SFC & BM_SFDRA;
    ATA_SETBITMASK_C(SFC,BM_SFDRA)

    /* LLR-Ref: 020 */
    for(uint8_t i=0; i<bLen;i++){
        SFD = *pData++;
    }

    /* LLR-Ref: 030 */
    if (!tmp) {
        ATA_CLEARBITMASK_C(SFC,BM_SFDRA)
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartTx_C</b>
    this function should be called to start RFTX.

    \param[in]      bConfig     Configuration of selected RFTXMode. For details see ::g_sRfTx .bConfig
    \param[in]      pAddress    Pointer to Service/Channel configuration in EEPROM/SRAM

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in] ::g_pRfTxBufStateMachine Global RF Tx (buffered) statemachine
    \li [in] ::g_pRfTxTransStateMachine Global RF Tx (transparent) statemachine

    \image html ATA_rfTxStartTx_C.png

    \internal
    \li 010: Stop currently running sequencer state machines to avoid
             resource conflicts of sequencer state machine hardware\n\n
             Note: Frontend register initialization is done in SW state
             ::ATA_rfTxInitFrontEnd_C for all state machines in module RFTX

    \li 020: Enable AVCC to access the Frontend register after AVCC is stable

    \li 030: Initialize ::g_sRfTx with content of function arguments config and
             pAddress. Signal RFTX Module active via flag ::g_sRfTx .bStatus[4]

    \li 040: Initialize ::g_sRfTxCurrentService by calling function
             ::ATA_rfTxInitCurrentService_C

    \li 050: IF an error occured during initialization of the current RF Tx service,
             THEN\n
                Deactivate active signal of RF Tx module via flag ::g_sRfTx
                .bStatus[4] being set to 0, AND Disable AVCC since it is not needed.

    \li 060: ELSE\n
               Signal Direct Switching if possible, AND
               Initialize SW state machine by setting SW state machine index to 0
               and apply correct sw state machine to SW state machine pointer
               in case of TXMode(transparent) set TMDI pin to input.\n
             ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartTx_C(uint8_t bConfig, uint8_t *pAddress)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStartTx_C, bConfig);

    uint8_t bDirectSwitching = 0U;

    /* LLR-Ref: 010 */
    SSMRR = BM_SSMST;
    bDirectSwitching = g_sRfTx.bFlags & BM_RFTXCONFIG_BFLAGS_RDY4TX;

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(SUPCR,BM_AVEN)

    /* LLR-Ref: 030 */
    g_sRfTx.bFlags = 0x00U;
    g_sRfTx.bStatus = BM_RFTXCONFIG_BSTATUS_ACTIVE;
    g_sRfTx.bConfig = bConfig;
    g_sRfTx.pAddress = pAddress;
    /* --P2P-3655-- */
    g_sRfTx.bCmcrSetting = CMCR;

    /* LLR-Ref: 040 */
    ATA_rfTxInitCurrentService_C();

    if (g_sRfTx.bFlags & BM_RFTXCONFIG_BFLAGS_ERROR)
    {
        /* LLR-Ref: 050 */
        ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_ACTIVE)
        ATA_CLEARBITMASK_C(SUPCR,BM_AVEN)
    }
    else
    {
        /* LLR-Ref: 060 */
        if(bDirectSwitching)
        {
            ATA_SETBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_DIRECT_SWITCH)
        }

        g_sRfTxFlowCtrl.bIndex = 0U;

        if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_TRANSPARENT_MODE)
        {
            /* TXMode(transparent) */
            g_sRfTxFlowCtrl.fpLut  = g_pRfTxTransStateMachine;
            
            ATA_CLEARBITMASK_C(DDRD,BM_TMDI)
            ATA_CLEARBITMASK_C(PORTD,BM_TMDI)
        }
        else
        {
            /* TXMode(buffered) */
            g_sRfTxFlowCtrl.fpLut  = g_pRfTxBufStateMachine;
        }
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStop_C</b>
    is used as API function to shutdown a currently running TXMode
    
    \image html ATA_rfTxStop_C.png

    \internal
    \li 010: Shutdown currently running TXMode by calling function
             ::ATA_rfTxShutdown_C

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-895}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStop_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStop_C, 0x00);
    /* LLR-Ref: 010 */
    ATA_rfTxShutdown_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxProcessing_C</b>
    triggers the active SW state of currently running SW state machine. This
    function should be placed in main loop context or should be run periodically
    to ensure RFTX functionality.

    Variable Usage:
    \li [in] ::g_sRfTxFlowCtrl Global RF Tx statemachine data

    \image html ATA_rfTxProcessing_C.png

    \internal
    \li 010: Decode and execute currently active SW state

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxProcessing_C(void)
{
    sysFlowStateMachineFunc_t fpFunc;

    /* LLR-Ref: 010 */
    fpFunc = *(sysFlowStateMachineFunc_t)g_sRfTxFlowCtrl.fpLut[g_sRfTxFlowCtrl.bIndex];
    fpFunc();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxInitTxSSM_C</b>
    this sw state does the register initialization according to the selected
    service channel configuration. Note: FE register are initialized after AVCC
    is stable in sw state ::ATA_rfTxInitFrontEnd_C

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in] ::g_sRfTx Global RF Tx component data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxInitTxSSM_C.png

    \internal
    \li 010: Power up the TX DSP HW block and the Sequencer State Machine HW block
    \li 020: Initialize the frequency registers with selected Service/Channel
             configuration via function ::ATA_rfTxFrequencySettings_C
    \li 030: Enable the Sigma-Delta Modulator by accessing FSEN.SDEN
    \li 040: Set FSCR.TXMS[1..0] according to selected TXMode
                 - TXMode(buffered)      FSCR.TXMS = 10 - Tx Modulator serial output
                 - TXMode(transparent)   FSCR.TXMS = 01 - TMDI input pin
    \li 050: Set FSCR.TXMOD according to selected modulation type
                 - ASK Modulation: FSCR.SFM   = 0
                                   FSCR.TXMOD = 1
                 - FSK Modulation: FSCR.SFM   = 1
    \li 060: Initialize following register according to service channel configuration
                 - FSFCR
                 - GACDIVL/H
    \li 070: Initialize TX Modulator registers by calling function
             ::ATA_rfTxSetupTxModulator_C
    \li 080: Initialize Sequencer State Machine registers by calling function
             ::ATA_rfTxConfigureTxSSM_C
    \li 090: Switch to next sw state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxInitTxSSM_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxInitTxSSM_C, 0x00);
    /* LLR-Ref: 010 */
    ATA_POWERON_C(PRR0, PRTXDC)
    ATA_POWERON_C(PRR2, PRSSM)

    /* LLR-Ref: 020 */
    ATA_rfTxFrequencySettings_C();

    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(FSEN,BM_SDEN)

    /* LLR-Ref: 040 */
    ATA_CLEARBITMASK_C(FSCR,BM_TXMOD)

    if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_TRANSPARENT_MODE) {
        /* Transparent mode */
        ATA_SETBITMASK_C(FSCR,BM_TXMS0)
    }
    else {
        /* Buffered mode */
        ATA_SETBITMASK_C(FSCR,BM_TXMS1)
    }

    /* LLR-Ref: 050 */
    if( g_sRfTxCurrentService.sPath.bTxSetPath[1] & BM_RFTXSERVICE_BTXSETPATH_MODULATION ) {
        /* ASK modulation */
        ATA_SETBITMASK_C(FSCR,BM_TXMOD)
    }
    else {
        /* FSK modulation */
        ATA_SETBITMASK_C(FSCR,BM_SFM)
    }

    /* LLR-Ref: 060 */
    FSFCR   = g_sRfTxCurrentService.sPath.bFSFCR;
    GACDIVL = g_sRfTxCurrentService.sPath.bGACDIV[0];
    GACDIVH = g_sRfTxCurrentService.sPath.bGACDIV[1];

    /* LLR-Ref: 070 */
    ATA_rfTxSetupTxModulator_C();

    /* LLR-Ref: 080 */
    ATA_rfTxConfigureTxSSM_C();

    /* LLR-Ref: 090 */
    g_sRfTxFlowCtrl.bIndex++;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4AVCC_C</b>
    waits until AVCC is stable and switches to next sw state if AVCC is stable.

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in] ::g_sRfTx Global RF Tx component data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxWait4AVCC_C.png

    \internal
    \li 010: Clear AVCC related flags SUPFR.AVCCLF and SUPFR.AVCCRF
    \li 020: IF AVCC is stable, indicated by both flags SUPFR.AVCCLF and
             SUPFR.AVCCRF being set to 0,\n
             switch to next sw state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4AVCC_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4AVCC_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_SETBITMASK_C(SUPFR,(BM_AVCCLF | BM_AVCCRF))

    /* LLR-Ref: 020 */
    if ( (SUPFR & (BM_AVCCLF|BM_AVCCRF)) == 0 )
    {
        g_sRfTxFlowCtrl.bIndex++;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxInitFrontEnd_C</b>
    initializes the frontend registers according to the selected service channel
    configuration.

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxInitFrontEnd_C.png

    \internal
    \li 010: Initialize FETN4 register with EEPROM content ::g_sEepFacLockRfFrontend
             .bFETN4
    \li 020: Switch on XTO by setting FEEN1.XTOEN
    \li 040: Initialize FEAT register
    \li 050: Initialize FEBT register with with EEPROM content
             ::g_sEepFacLockRfFrontend .bFEBT
    \li 060: Initialize FEPAC register
    \li 070: Initialize FEVCO register by calling function
             ::ATA_rfTxFevcoCalculation_C
    \li 080: Set FEEN2.SDTX to switch the antenna SPDT_ANT to the TX power
             amplifier SPDT_TX
    \li 090: Initialize FECR register
    \li 100: Initialize FEMS register
    \li 110: Initialize FEVCT register
    \li 120: Increase the internal bias current of the AVCC regulator if FEPAC
             setting is less than 0x2B
    \li 130: Initialize FEANT register
    \li 140: Initialize FEALR register
    \li 150: Switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868,\
                  Primus2P-2443}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxInitFrontEnd_C(void)
{
    uint8_t tmp;
    eEepErrorCode sEepErrCode;

    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxInitFrontEnd_C, 0x00);

    /* LLR-Ref: 010 */
    sEepErrCode = ATA_eepReadBytes_C(&tmp, (uint16_t)&g_sAtmelEEPromSection.eepFETN4, 1U);
    if(sEepErrCode != EEC_NO_ERROR)
    {
        ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR;
    }
    
    FETN4 = tmp;

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(FEEN1,BM_XTOEN)

    /* LLR-Ref: 040 */
    FEAT = g_sRfTxCurrentService.sService.bFEAT;

    /* LLR-Ref: 050 */
    sEepErrCode = ATA_eepReadBytes_C(&tmp, (uint16_t)&g_sAtmelEEPromSection.eepFEBT, 1U);
    if(sEepErrCode != EEC_NO_ERROR)
    {
        ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR;
    }

    FEBT = tmp;
    
    /* LLR-Ref: 060 */
    FEPAC = g_sRfTxCurrentService.sService.bFEPAC;

    /* LLR-Ref: 070 */
    FEVCO = ATA_rfTxFevcoCalculation_C();

    /* LLR-Ref: 090 */
    FECR  = g_sRfTxCurrentService.sChannel.bFECR;
    /* LLR-Ref: 100 */
    FEMS  = g_sRfTxCurrentService.sChannel.bFEMS;
    /* LLR-Ref: 110 */
    FEVCT = g_sRfTxCurrentService.sService.bFEVCT;

    /* LLR-Ref: 120 */
    if (g_sRfTxCurrentService.sService.bFEPAC < 0x2BU)
    {}
    else
    {
        ATA_SETBITMASK_C(SUPCR,BM_AVDIC)
    }
    /* setting of FEALR.RNGE necessary for Antenna tuning (SSM review 22jul2011) */
    /* LLR-Ref: 130 */
    FEANT = (g_sRfTxCurrentService.sService.bFEALR_FEANT & 0x0FU);
    /* LLR-Ref: 140 */
    FEALR = ((g_sRfTxCurrentService.sService.bFEALR_FEANT & 0x30U) >> 4U);

    /* LLR-Ref: 150 */
    g_sRfTxFlowCtrl.bIndex++;
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4XTO_C</b>
    waits until XTO is ready. If AVR is not running with CLKXTO4 the
    core clock is switched to CLKXTO4.

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data

    \image html ATA_rfTxWait4XTO_C.png

    \internal
    \li 010: Check if XTO is available via flag FESR.XRDY
             IF XTO is ready\n
    \li 020:   Check if AVR core is running with other clock than CLKXTO4 via
               flag CMCR.CMM2 and CMCR.CCS\n
               IF AVR core is not running with CLKXTO4\n
    \li 030:     Switch AVR core clock to CLKXTO4 by calling function
                 ::ATA_globalsClkSwitchXTO_C
    \li 040:     Switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4XTO_C(void)
{
    /* LLR-Ref: 010 */
    if (FESR & BM_XRDY) {

        ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4XTO_C, 0x00);

        /* LLR-Ref: 020 */
        if (  ((CMCR & BM_CMM2) == 0)
            ||((CMCR & BM_CCS ) == 0)
           ){
            /* LLR-Ref: 030 */
            ATA_globalsClkSwitchXTO_C(0x07U);
        }

        /* LLR-Ref: 040 */
        g_sRfTxFlowCtrl.bIndex++;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartTxBufSSM_C</b>
    starts the sequencer state machine for TxMode(buffered).

    \image html ATA_rfTxStartTxBufSSM_C.png

    \internal
    \li 010: Call function ::ATA_rfTxStartSSM_C to configure the RF Tx statemachine
             to execute the Tx buffered shutdown sequence

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartTxBufSSM_C(void)
{
    /* LLR-Ref: 010 */
    ATA_rfTxStartSSM_C(RFTX_BUF_STATE_SHUTDOWN);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartTxTransSSM_C</b>
    starts the sequencer state machine for TxMode(transparent)

    \image html ATA_rfTxStartTxTransSSM_C.png

    \internal
    \li 010: Call function ::ATA_rfTxStartSSM_C to configure the RF Tx statemachine
             to execute the Tx transparent shutdown sequence

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-848}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartTxTransSSM_C(void)
{
    /* LLR-Ref: 010 */
    ATA_rfTxStartSSM_C(RFTX_TRANS_STATE_SHUTDOWN);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartSSM_C</b>
    starts the sequencer state machine.

    \param[in]      bState              SW state to switch to in case of an immediate RF Tx shutdown

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_rfTxStartSSM_C.png

    \internal
    \li 010: Clear SSM Interrupt Flag Register (SSMIFR)
    \li 020: Clear SSM status register (SSMSR)
    \li 030: Clear flag ::g_sRfTx .bStatus[6]
    \li 040: Enable SSM finished interrupt by setting flag SSMIMR.SSMIM in
             SSM Interrupt Mask Register (SSMIMR)
    \li 050: Start Watchdog with timeout for SSM by calling function
             ::ATA_rfTxStartSsmWatchdog_C\n
             IF SSM Watchdog is locked
    \li 060:     Signal error via flag ::g_sRfTx .bFlags[7]
    \li 070:     Write error code to ::g_sDebug .bErrorCode and ::g_sDebug
                 .bSsmErrorCode
    \li 080:     Switch to shutdown sw state by setting ::g_sRfTxFlowCtrl .bIndex\n
             IF SSM Watchdog has started correctly
    \li 090:     Start SSM by setting flag SSMR in SSM Run Register (SSMRR)
    \li 100:     Switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartSSM_C(uint8_t bState)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStartSSM_C, 0x00);

    /* LLR-Ref: 010 */
    SSMIFR = 0x00U;

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(SSMSR,BM_SSMERR)

    /* LLR-Ref: 030 */
    ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)

    /* LLR-Ref: 040 */
    SSMIMR = BM_SSMIM;

    /* LLR-Ref: 050 */
    if( ATA_rfTxStartSsmWatchdog_C() )
    {
        /* LLR-Ref: 060 */
        ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)

        /* LLR-Ref: 070 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_STARTSSM_TIMER_LOCKED;
        g_sDebug.bSsmErrorCode = 0U;

        /* LLR-Ref: 080 */
        g_sRfTxFlowCtrl.bIndex = bState;
    }
    else
    {
        /* LLR-Ref: 090 */
        SSMRR = BM_SSMR;

        /* LLR-Ref: 100 */
        g_sRfTxFlowCtrl.bIndex++;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4SSMrdy_C</b>
    waits until the sequencer state machine is ready.

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx component data
    \li [in,out] ::g_sRfTxCurrentService Global RF Tx current service data
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_sDebug Global Debug component data
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data
    \li [out] ::g_bAntennaTuningResult Global RF Tx Antenna tuning component data

    \image html ATA_rfTxWait4SSMrdy_C.png

    \internal
            Note:
            The SSM watchdog is stopped, i.e. Timer 2 is closed in function 
            ATA_rfTxShutDown_C() in case the SSM watchdog has expired (ELSE-clause)

    \li 010: Check if SSM has finished with success via ::g_sRfTx .bStatus[6]
             and SSM status register SSMSR\n
             IF SSM has finished with success
    \li 020:   Stop SSM Watchdog
    \li 030:   Clear RF Tx SSM ready in ::g_sRfTx .bStatus[6]
    \li 040:   Signal RF Tx Module ready for transmission in ::g_sRfTx .bFlags[6]
    \li 045:   Store Antenna Tuning and VCO Tuning values in current service
               and dedicated tuning result variables
               (::g_bVcoTuningResult,::g_bAntennaTuningResult)
    \li 050:   IF TXMode(transparent) is selected, activate the power amplifier
                by setting FSCR.PAOER
    \li 060:     Switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex\n
             ELSE (SSM has not finished)
    \li 070:   Check for
                 - SSM Watchdog timeout via flag T2IFR.T2COF
                 - SSM Error via register SSMSR\n
                 IF Watchdog timeout or SSM Error has occured
    \li 080:       Reset current running sequencer state machine
    \li 100:       Signal error via flag ::g_sRfTx .bFlags[7]
    \li 110:       Write error code to ::g_sDebug .bErrorCode and ::g_sDebug
                   .bSsmErrorCode
    \li 120:       Switch to shutdown SWstate by setting ::g_sRfTxFlowCtrl .bIndex
                     - to RFTX_TRANS_STATE_SHUTDOWN in TXMode(transparent)
                     - to RFTX_BUF_STATE_SHUTDOWN in TXMode(buffered)

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-849,Primus2P-848,Primus2P-847}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4SSMrdy_C(void)
{
    /* LLR-Ref: 010 */
    if( (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_SSMREADY) && (SSMSR == 0x00U) )
    {
        ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4SSMrdy_C, 0x00);

        /* LLR-Ref: 020 */
        ATA_rfTxStopSsmWatchdog_C();

        /* LLR-Ref: 030 */
        ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)

        /* LLR-Ref: 040 */
        ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_RDY4TX)

        /* LLR-Ref: 045 */
        g_sRfTxCurrentService.sService.bFEAT = FEAT;
        g_bAntennaTuningResult = g_sRfTxCurrentService.sService.bFEAT;
        g_sRfTxCurrentService.sService.bFEVCT = FEVCT;
        g_bVcoTuningResult = g_sRfTxCurrentService.sService.bFEVCT;

        /* LLR-Ref: 050 */
        if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_TRANSPARENT_MODE) {
            ATA_SETBITMASK_C(FSCR,BM_PAOER)
        }

        /* LLR-Ref: 060 */
        g_sRfTxFlowCtrl.bIndex++;
    }
    else {
        /* LLR-Ref: 070 */
        if( (T2IFR & BM_T2COF) || SSMSR )
        {
            /* LLR-Ref: 080 */
            SSMRR = BM_SSMST;       /* reset current state machine */

            /* LLR-Ref: 100 */
            ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)

            /* LLR-Ref: 110 */
            g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_WAIT4SSMRDY_TIMEOUT;
            g_sDebug.bSsmErrorCode = SSMSR;

            /* LLR-Ref: 120 */
            if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_TRANSPARENT_MODE) {
                g_sRfTxFlowCtrl.bIndex = RFTX_TRANS_STATE_SHUTDOWN;
            }
            else {
                g_sRfTxFlowCtrl.bIndex = RFTX_BUF_STATE_SHUTDOWN;
            }
        }
    }

}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxTransparentMode_C</b>
    is an empty function. This SW state is reached after TXMode(transparent)
    has started correctly.

    \image html ATA_rfTxTransparentMode_C.png

    \internal
    \Derived{Yes}

    \Rationale{This function is required as a placeholder to stay in Tx transparent
               mode until the Application SW shuts down Tx transparent mode}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxTransparentMode_C(void)
{
    /* remove trace to avoid trace overflow
     * ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxTransparentMode_C, 0x00);
     * do nothing --> system in TXMode(transparent) */
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4FillLevel_C</b>
    checks the fill levels of S-FIFO and D-FIFO

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxWait4FillLevel_C.png

    \internal
    \li 010:    read out start fill levels for S-FIFO (Preamble FIFO) and D-FIFO
                (Data FIFO) from current service channel configuration
    \li 020:    check if S-FIFO and D-FIFO fill levels are greater or equal the
                corresponding start fill levels
                IF both fill level conditions are true
    \li 030:        switch to next sw state by incrementing variable g_sRfTxFlowCtrl.bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4FillLevel_C(void)
{
    /* LLR-Ref: 010 */
    uint8_t bTxFifoStartFillLevel = g_sRfTxCurrentService.sPath.bTxSetPath[0] & 0x3F;
    uint8_t bPreambleFifoStartFillLevel = g_sRfTxCurrentService.sPath.bTxSetPath[1] & 0x1F;

    /* LLR-Ref: 020 */
    if(    ( bTxFifoStartFillLevel       <= (DFL & BM_DFFLS) )
        && ( bPreambleFifoStartFillLevel <= (SFL & BM_SFFLS) ) ) {

        ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4FillLevel_C, 0x00);

        /* LLR-Ref: 030 */
        g_sRfTxFlowCtrl.bIndex++;
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartTxModulator_C</b>
    configures the fill level interrupts, starts the TX Modulator and
    SendTelegramSSM.

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxStartTxModulator_C.png

    \internal
    \li 010: Configure the S-FIFO for data transmission by setting SFC.SFDRA 
             (TxMode for preamble FIFO)
    \li 020: Set the S-FIFO fill level configuration in SFC.SFFLC[4..0]
             according to current Service/Channel configuration
             ::g_sRfTxCurrentService
    \li 030: Enable S-FIFO fill level interrupt
    \li 040: Configure the D-FIFO for data transmission by setting DFC.DFDRA
             (TxMode for data FIFO)
    \li 050: Set the D-FIFO fill level configuration in DFC.DFFLC[5..0]
             according to current Service/Channel configuration
             ::g_sRfTxCurrentService
    \li 060: Enable D-FIFO fill level interrupt
    \li 070: Activate the power amplifier by setting FSCR.PAOER
    \li 080: Reset TX Modulator by toggling PRR2.PRTM
    \li 090: Configure Timer 3 for TXMode via function
             ::ATA_rfTxOpenTimer3forTxing_C
    \li 100: Switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartTxModulator_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStartTxModulator_C, 0x00);
    
    /* LLR-Ref: 010 */
    SFC = BM_SFDRA;
    
    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(SFC,g_sRfTxCurrentService.sPath.bTxPreambleSysEvent & 0x1F)
    
    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(SFI,BM_SFFLIM)

    /* LLR-Ref: 040 */
    DFC = BM_DFDRA;
    
    /* LLR-Ref: 050 */
    ATA_SETBITMASK_C(DFC,g_sRfTxCurrentService.sPath.bTxSysEvent & 0x3F)
    
    /* LLR-Ref: 060 */
    ATA_SETBITMASK_C(DFI,BM_DFFLIM)

    /* LLR-Ref: 070 */
    ATA_SETBITMASK_C(FSCR,BM_PAOER)

    /* LLR-Ref: 080 */
    ATA_POWEROFF_C(PRR2, PRTM)
    ATA_POWERON_C(PRR2, PRTM)

    /* LLR-Ref: 090 */
    ATA_rfTxOpenTimer3forTxing_C();

    /* LLR-Ref: 100 */
    ATA_rfTxStartSendTelegramSSM_C();

    /* LLR-Ref: 110 */
    g_sRfTxFlowCtrl.bIndex++;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4TransmissionComplete_C</b>
    waits until Transmission has completed.

    Variable Usage:
    \li [in,out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in,out] ::g_sRfTx Global RF Tx current service data

    \image html ATA_rfTxWait4TransmissionComplete_C.png

    \internal
    \li 010: Check if RFTX transmission is complete via ::g_sRfTx .bStatus[5]\n
             IF RFTX transmission is complete
    \li 020:   Clear transmission complete flag in ::g_sRfTx .bStatus[5]
    \li 030:   Clear fill level interrupts for S-FIFO and D-FIFO
    \li 040:   IF RFTX module shall stay in TXMode (configured in ::g_sRfTx .bConfig[2])
                 Switch to wait for fill level SW state by setting
                 ::g_sRfTxFlowCtrl .bIndex\n
               ELSE switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4TransmissionComplete_C(void)
{
    /* LLR-Ref: 010 */
    if(g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_TRANSMISSION_COMPLETE)
    {
        ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4TransmissionComplete_C, 0x00);

        /* LLR-Ref: 020 */
        g_sRfTx.bStatus &= (uint8_t)~BM_RFTXCONFIG_BSTATUS_TRANSMISSION_COMPLETE;

        /* LLR-Ref: 030 */
        ATA_CLEARBITMASK_C(SFI,BM_SFFLIM)
        ATA_CLEARBITMASK_C(DFI,BM_DFFLIM)

        /* LLR-Ref: 040 */
        if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_STAY_TX) {
            g_sRfTxFlowCtrl.bIndex = RFTX_BUF_STATE_WAIT_FILLLEVEL;
        }
        else{
            g_sRfTxFlowCtrl.bIndex++;
        }
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxShutdown_C</b>
    shut down the RFTX module related state machine for TXMode(buffered and transparent),
    VCO tuning and antenna tuning.

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx current service data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_rfTxShutdown_C.png

    \internal
    \li 010: Clear SSM ready flag ::g_sRfTx .bStatus[6]
    \li 020: Start shutdown SSM by calling function ::ATA_rfTxStartShutDownSSM_C
    \li 030: Apply 5us wait time before checking that shutdown SSM has finished
              no SSM watchdog functionality implemented here, since timer startup phase
              extends SSM runtime
    \li 040: Check if SSM has finished with success via ::g_sRfTx .bStatus[6]
              and SSM status register SSMSR\n
             IF SSM has finished with success
    \li 050:   Clear SSM ready flag ::g_sRfTx .bStatus[6]\n
             ELSE (SMM has finished with error)
    \li 060:   Signal error via flag ::g_sRfTx .bFlags[7]
    \li 070:   Write error code to ::g_sDebug .bErrorCode and ::g_sDebug
               .bSsmErrorCode\n
             ENDIF
    \li 080: Clear S-FIFO and D-FIFO content by setting SFL.SFCLR and DFL.DFCLR
    \li 090: Remove power from RFTX module by calling function ::ATA_rfTxEnd_C
    \li 100: Clear RFTX module active flag ::g_sRfTx .bStatus[4]
    \li 110: Clear RFTX module ready for transmission flag ::g_sRfTx .bFlags[6]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxShutdown_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxShutdown_C, 0x00);
    
    /* LLR-Ref: 010 */
    g_sRfTx.bStatus &= (uint8_t)~BM_RFTXCONFIG_BSTATUS_SSMREADY;
    
    /* LLR-Ref: 020 */
    ATA_rfTxStopSsmWatchdog_C();
    ATA_rfTxStartShutDownSSM_C();

    /* LLR-Ref: 030 */
    ATA_globalsWaitNus_ASM(5U);

    /* LLR-Ref: 040 */
    if (   (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_SSMREADY)
        && (SSMSR == 0U) ) {
        /* LLR-Ref: 050 */
        ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)
    }
    else {
        /* LLR-Ref: 060 */
        ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
        
        /* LLR-Ref: 070 */
        g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_SHUTDOWN_ERROR;
        g_sDebug.bSsmErrorCode = SSMSR;
    }

    /* LLR-Ref: 080 */
    ATA_SETBITMASK_C(SFL,BM_SFCLR)
    ATA_SETBITMASK_C(DFL,BM_DFCLR)

    /* LLR-Ref: 090 */
    ATA_rfTxEnd_C();

    /* LLR-Ref: 100 */
    ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_ACTIVE)

    /* LLR-Ref: 110 */
    ATA_CLEARBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_RDY4TX)
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_rfTxInitCurrentService_C</b>
    does the initialization of ::g_sRfTxCurrentService

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx current service data
    \li [out] ::g_sDebug Global Debug component data
    \li [out] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxInitCurrentService_C.png

    \internal
    \li 010: Check for EEPROM or SRAM service configuration via ::g_sRfTx .bConfig[3]\n
             IF service configuration is located in EEPROM,
             THEN

    \li 020: Copy service specific configuration from EEPROM to
             ::g_sRfTxCurrentService .sService

    \li 025: IF after the EEPROM read access to retrieve the requested data, an
               uncorrectable EEPROM error was detected, indicated by
               bit "E2FF" in register EECR2 being set 1,
             THEN
               Set an EEPROM read error indication in ::g_sRfTx .bFlags of and
               set ::g_sDebug .bErrorCode to the RF Tx module EEPROM read error\n
             ENDIF

    \li 030: Copy path specific configuration from EEPROM to
             ::g_sRfTxCurrentService .sPath

    \li 035: IF after the EEPROM read access to retrieve the requested data, an
              uncorrectable EEPROM error was detected, indicated by
              bit "E2FF" in register EECR2 being set 1,
             THEN
               Set an EEPROM read error indication in ::g_sRfTx .bFlags and
               set ::g_sDebug .bErrorCode to the RF Tx module EEPROM read error\n
             ENDIF

    \li 040: Copy channel specific configuration from EEPROM to
             ::g_sRfTxCurrentService .sChannel

    \li 045: IF after the EEPROM read access to retrieve the requested data, an
              uncorrectable EEPROM error was detected, indicated by
              bit "E2FF" in register EECR2 being set 1,\n
             THEN
               Set an EEPROM read error indication in ::g_sRfTx .bFlags and
               set ::g_sDebug .bErrorCode" to the RF Tx module EEPROM read error\n
             ENDIF

             ELSE (service configuration is located in SRAM)
    \li 050:   Copy service specific configuration from SRAM to
               ::g_sRfTxCurrentService .sService

    \li 060:   Copy path specific configuration from SRAM to
               ::g_sRfTxCurrentService .sPath

    \li 070:   Copy channel specific configuration from SRAM to
               ::g_sRfTxCurrentService .sChannel\n
             ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868,\
                  Primus2P-2443}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_rfTxInitCurrentService_C(void)
{
    eEepErrorCode sEepErrCode;
    
    /* LLR-Ref: 010 */
    if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_SVC_LOCATION)
    {
        /* LLR-Ref: 020 */
        uint8_t *pSrcData = g_sRfTx.pAddress;
        uint8_t *pDstData = (uint8_t*)&g_sRfTxCurrentService.sService;
        sEepErrCode = ATA_eepReadBytes_C(pDstData, (uint16_t)pSrcData, sizeof(sRfTxServiceSpecificConfig));

        /* LLR-Ref: 025 */
        if(sEepErrCode != EEC_NO_ERROR)
        {
            ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR;
        }

        /* LLR-Ref: 030 */
        pSrcData += sizeof(sRfTxServiceSpecificConfig);
        pDstData = (uint8_t*)&g_sRfTxCurrentService.sPath;
        sEepErrCode = ATA_eepReadBytes_C(pDstData, (uint16_t)pSrcData, sizeof(sRfTxServicePathConfig));

        /* LLR-Ref: 035 */
        if(sEepErrCode != EEC_NO_ERROR)
        {
            ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR;
        }

        /* LLR-Ref: 040 */
        pSrcData += sizeof(sRfTxServicePathConfig);
        pDstData = (uint8_t*)&g_sRfTxCurrentService.sChannel;

        uint8_t channel = g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_CHANNEL;

        if (channel > 0) {
            pSrcData += sizeof(sRfTxChannelConfig);
        }

        if (channel > 1) {
            pSrcData += sizeof(sRfTxChannelConfig);
        }

        sEepErrCode = ATA_eepReadBytes_C(pDstData, (uint16_t)pSrcData, sizeof(sRfTxChannelConfig));

        /* LLR-Ref: 045 */
        if(sEepErrCode != EEC_NO_ERROR)
        {
            ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
            g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR;
        }
    }
    else
    {
        /* LLR-Ref: 050 */
        uint8_t *pSrcData = g_sRfTx.pAddress;
        uint8_t *pDstData = (uint8_t*)&g_sRfTxCurrentService.sService;
        ATA_globalsCopySramSpace_C(pDstData, pSrcData, sizeof(sRfTxServiceSpecificConfig));

        /* LLR-Ref: 060 */
        pSrcData += sizeof(sRfTxServiceSpecificConfig);
        pDstData = (uint8_t*)&g_sRfTxCurrentService.sPath;
        ATA_globalsCopySramSpace_C(pDstData, pSrcData, sizeof(sRfTxServicePathConfig));

        /* LLR-Ref: 070 */
        pSrcData += sizeof(sRfTxServicePathConfig);
        pDstData = (uint8_t*)&g_sRfTxCurrentService.sChannel;

        uint8_t channel = g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_CHANNEL;

        if (channel > 0) {
            pSrcData += sizeof(sRfTxChannelConfig);
        }

        if (channel > 1) {
            pSrcData += sizeof(sRfTxChannelConfig);
        }
        ATA_globalsCopySramSpace_C(pDstData, pSrcData, sizeof(sRfTxChannelConfig));
    }
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_rfTxFrequencySettings_C</b>
    does the Tx specific frequency settings according to the current
    Service/Channel configuration

    Variable Usage:
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxFrequencySettings_C.png

    \internal
    \li 010: Calculate f_tx_ask = f_lo + f_if
               f_lo = ::g_sRfTxCurrentService .sChannel.FREQ[2..0]
               f_if = ::g_sRfTxCurrentService .sService.bIF[1..0]
    \li 020: Check modulation type via ::g_sRfTxCurrentService .sPath.bTxSetPath[1][7]\n
             IF ASK modulation is used
    \li 030:   Copy FFREQ1 settings to FFREQ2
    \li 040:   Switch to FFREQ2 to avoid PLL problems during programming of FFREQ1
    \li 050:   Program FFREQ1 registers with f_tx_ask
    \li 060:   Switch back to FFREQ1\n
             ELSE (FSK modulation is used)
    \li 070:   Copy FFREQ1 settings to FFREQ2
    \li 080:   Switch to FFREQ2 to avoid PLL problems during programming of FFREQ1
    \li 090:   Calculate f_tx_fsk1 = f_tx_ask - f_tx_dev / 2
                 f_tx_dev = ::g_sRfTxCurrentService .sPath.btxDev[1..0]
    \li 100:   Program FFREQ1 with f_tx_fsk1
    \li 110:   Switch back to FFREQ1
    \li 120:   Calculate f_tx_fsk2 = f_tx_fsk1 + f_tx_dev (= f_tx_ask + f_tx_dev / 2)
    \li 130:   Program FFREQ2 with f_tx_fsk2

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
static VOIDFUNC ATA_rfTxFrequencySettings_C(void)
{
    /* LLR-Ref: 010 */
    uint32_t dwFrequency = ((uint32_t)g_sRfTxCurrentService.sChannel.bFFREQ[2] << 16U)
                        |((uint32_t)g_sRfTxCurrentService.sChannel.bFFREQ[1] <<  8U)
                        |((uint32_t)g_sRfTxCurrentService.sChannel.bFFREQ[0] <<  0U);
    /* --- F_TX = F_LO + F_IF --- */
    dwFrequency +=  ((uint32_t)g_sRfTxCurrentService.sService.bIF[1] <<  8U)
                   |((uint32_t)g_sRfTxCurrentService.sService.bIF[0] <<  0U);

    /* LLR-Ref: 020 */
    if (g_sRfTxCurrentService.sPath.bTxSetPath[1] & BM_RFTXSERVICE_BTXSETPATH_MODULATION) {
        /* ASK Modulation */
        /* --- F_TX_ASK = F_LO + F_IF --- */
        /* LLR-Ref: 030 */
        FFREQ2H = FFREQ1H;
        FFREQ2M = FFREQ1M;
        FFREQ2L = FFREQ1L;

        /* LLR-Ref: 040 */
        FSCR    = BM_TXMOD | BM_SFM;

        /* LLR-Ref: 050 */
        FFREQ1H = (uint8_t)(dwFrequency >> 16U);
        FFREQ1M = (uint8_t)(dwFrequency >>  8U);
        FFREQ1L = (uint8_t)(dwFrequency >>  0U);

        /* LLR-Ref: 060 */
        FSCR    = 0x00U;
    }
    else {   /* FSK Modulation */
        uint32_t dwFreqdev = (  ((uint32_t)g_sRfTxCurrentService.sPath.btxDev[1] <<  8U)
                               |((uint32_t)g_sRfTxCurrentService.sPath.btxDev[0] <<  0U));
        /* LLR-Ref: 070 */
        FFREQ2H = FFREQ1H;
        FFREQ2M = FFREQ1M;
        FFREQ2L = FFREQ1L;

        /* LLR-Ref: 080 */
        FSCR    = BM_TXMOD | BM_SFM;

        /* LLR-Ref: 090 */
        dwFrequency -= dwFreqdev/2U;

        /* LLR-Ref: 100 */
        FFREQ1H = (uint8_t)(dwFrequency >> 16U);
        FFREQ1M = (uint8_t)(dwFrequency >>  8U);
        FFREQ1L = (uint8_t)(dwFrequency >>  0U);

        /* LLR-Ref: 110 */
        FSCR    = 0x00U;

        /* LLR-Ref: 120 */
        dwFrequency += dwFreqdev;

        /* LLR-Ref: 130 */
        FFREQ2H = (uint8_t)(dwFrequency >> 16U);
        FFREQ2M = (uint8_t)(dwFrequency >>  8U);
        FFREQ2L = (uint8_t)(dwFrequency >>  0U);
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxSetupTxModulator_C</b>
    this function initializes the TX modulator according to the settings in
    variable ::g_sRfTxCurrentService .sPath.

    Variable Usage:
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxSetupTxModulator_C.png

    \internal
    \li 010: Initialize the following registers
                 - TMCR1
                 - TMCR2
                 - TMSR
                 - TMSSC
                 - TMTLL
                 - TMTLH
                 - TMCPL
                 - TMCPH
                 - TMCIL
                 - TMCIH
                 - TMCSB

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
\n
 */
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_rfTxSetupTxModulator_C(void)
{
    /* LLR-Ref: 010 */
    TMCR1 = BM_TMCIM | BM_TMSCS;
    TMCR2 = g_sRfTxCurrentService.sPath.bTMCR2;
    TMSR  = BM_TMTCF;
    TMSSC = g_sRfTxCurrentService.sPath.bTMSSC;
    TMTLL = g_sRfTxCurrentService.sPath.bTMTL[0];
    TMTLH = g_sRfTxCurrentService.sPath.bTMTL[1];
    TMCPL = g_sRfTxCurrentService.sPath.bTMCP[0];
    TMCPH = g_sRfTxCurrentService.sPath.bTMCP[1];
    TMCIL = g_sRfTxCurrentService.sPath.bTMCI[0];
    TMCIH = g_sRfTxCurrentService.sPath.bTMCI[1];
    TMCSB = g_sRfTxCurrentService.sPath.bTMCSB;
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_rfTxConfigureTxSSM_C</b>
    this function configures the Sequencer State Machine for TxMode(buffered).

    Variable Usage:
    \li [in] ::g_sRfTx Global RF Tx component data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxConfigureTxSSM_C.png

    \internal
    \li 010: Check SSM configuration via ::g_sRfTx .bConfig[7..6]
    \li 020: Stop current running SSM via SSMRR.SSMST\n
             IF no tuning is selected
    \li 030:   Write following configuration to MSMCR1..4
                   - SSM_PLL_EN_STATE
                   - SSM_PLL_LOCK_STATE
                   - SSM_TX_DSP_EN_STATE
                   - SSM_END_STATE\n
             IF only VCO tuning is selected
    \li 040:   Write following configuration to MSMCR1..4
                   - SSM_PLL_EN_STATE
                   - SSM_VCO_TUNING_STATE
                   - SSM_PLL_LOCK_STATE
                   - SSM_TX_DSP_EN_STATE
                   - SSM_END_STATE\n
             IF only Antenna tuning is selected
    \li 050:   Write following configuration to MSMCR1..4
                   - SSM_PLL_EN_STATE
                   - SSM_PLL_LOCK_STATE
                   - SSM_TX_DSP_EN_STATE
                   - SSM_ANTENNA_TUNING_STATE
                   - SSM_END_STATE\n
             IF VCO and Antenna tuning is selected
    \li 060:   Write following configuration to MSMCR1..4
                   - SSM_PLL_EN_STATE
                   - SSM_VCO_TUNING_STATE
                   - SSM_PLL_LOCK_STATE
                   - SSM_TX_DSP_EN_STATE
                   - SSM_ANTENNA_TUNING_STATE
                   - SSM_END_STATE
    \li 070: Check for direct switch via ::g_sRfTx .bStatus[7]. In case of a direct
              switch, replace PLL_EN SSM with TX_DSP_DIS
    \li 080: Initialize SSMFBR register
    \li 090: Reset SSM by clearing SSMCR register content
    \li 100: Set SSMCR.SSMTGE according to ::g_sRfTxCurrentService
              .sPath.bTxSetPath[0][7]
    \li 110: Set SSMCR.SSMTPE according to ::g_sRfTxCurrentService
              .sPath.bTxSetPath[0][6]
    \li 120: Set SSMCR.SSMTAE according to ::g_sRfTxCurrentService
              .sPath.bTxSetPath[1][6]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
static VOIDFUNC ATA_rfTxConfigureTxSSM_C(void)
{
    /* LLR-Ref: 010 */
    uint8_t config = g_sRfTx.bConfig & (BM_RFTXCONFIG_BCONFIG_ANT_TUNING | BM_RFTXCONFIG_BCONFIG_VCO_TUNING);

    /* LLR-Ref: 020 */
    SSMRR = BM_SSMST;

    if (config == 0U) {
        /* NO VCO TUNING */
        /* NO ANTENNA TUNING */
        /* LLR-Ref: 030 */
        MSMCR1 = SSM_PLL_EN_STATE           | (uint8_t)(SSM_PLL_LOCK_STATE << 4U);
        MSMCR2 = SSM_TX_DSP_EN_STATE        | (uint8_t)(SSM_END_STATE << 4U);
        MSMCR3 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
        MSMCR4 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
    }
    else if (config == BM_RFTXCONFIG_BCONFIG_VCO_TUNING) {
        /* ONLY VCO TUNING */
        /* NO ANTENNA TUNING */
        /* LLR-Ref: 040 */
        MSMCR1 = SSM_PLL_EN_STATE           | (uint8_t)(SSM_VCO_TUNING_STATE << 4U);
        MSMCR2 = SSM_PLL_LOCK_STATE         | (uint8_t)(SSM_TX_DSP_EN_STATE << 4U);
        MSMCR3 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
        MSMCR4 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
    }
    else if (config == BM_RFTXCONFIG_BCONFIG_ANT_TUNING) {
        /* NO VCO TUNING */
        /* ONLY ANTENNA TUNING */
        /* LLR-Ref: 050 */
        MSMCR1 = SSM_PLL_EN_STATE           | (uint8_t)(SSM_PLL_LOCK_STATE << 4U);
        MSMCR2 = SSM_TX_DSP_EN_STATE        | (uint8_t)(SSM_ANTENNA_TUNING_STATE << 4U);
        MSMCR3 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
        MSMCR4 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
    }
    else {
        /* VCO TUNING */
        /* ANTENNA TUNING */
        /* LLR-Ref: 060 */
        MSMCR1 = SSM_PLL_EN_STATE           | (uint8_t)(SSM_VCO_TUNING_STATE << 4U);
        MSMCR2 = SSM_PLL_LOCK_STATE         | (uint8_t)(SSM_TX_DSP_EN_STATE << 4U);
        MSMCR3 = SSM_ANTENNA_TUNING_STATE   | (uint8_t)(SSM_END_STATE << 4U);
        MSMCR4 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
    }

    /* LLR-Ref: 070 */
    if (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_DIRECT_SWITCH){
        /* in case of a direct switch -> replace PLL_EN SSM with TX_DSP_DIS */
        MSMCR1 &= (uint8_t)(BM_MSMSM13 | BM_MSMSM12 | BM_MSMSM11 | BM_MSMSM10);
        MSMCR1 |= SSM_TX_DSP_DIS_STATE;
    }

    /* LLR-Ref: 080 */
    SSMFBR = g_sRfTxCurrentService.sService.bSSMFBR;

    /* LLR-Ref: 090 */
    SSMCR = 0U;

    /* LLR-Ref: 100 */
    /* SSMCR.SSMTGE - Sequencer State Machine Tx Gauss Enable */
    if (g_sRfTxCurrentService.sPath.bTxSetPath[0] & BM_RFTXSERVICE_BTXSETPATH_GAUS) {
        ATA_SETBITMASK_C(SSMCR,BM_SSMTGE)
    }
    /* LLR-Ref: 110 */
    /* SSMCR.SSMTPE - Sequencer State Machine Tx Preemphasis Enable */
    if (g_sRfTxCurrentService.sPath.bTxSetPath[0] & BM_RFTXSERVICE_BTXSETPATH_PREE) {
        ATA_SETBITMASK_C(SSMCR,BM_SSMTPE)
    }
    /* LLR-Ref: 120 */
    /* SSMCR.SSMTAE - Sequencer State Machine Tx Ask-Shaping Enable */
    if (g_sRfTxCurrentService.sPath.bTxSetPath[1] & BM_RFTXSERVICE_BTXSETPATH_ASK_SHAPING) {
        ATA_SETBITMASK_C(SSMCR,BM_SSMTAE)
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxFevcoCalculation_C</b>
    does calculation for FEVCO register setting.

    Variable Usage:
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxFevcoCalculation_C.png

    \internal
    \li 010: Read offset values for vcob and cpcc from EEPROM g_sEepFacLockRfFrontend.bFEVCOoffset
                 - vcob_offset = g_sEepFacLockRfFrontend.bFEVCOoffset[7..4]
                 - cpcc_offset = g_sEepFacLockRfFrontend.bFEVCOoffset[3..0]
    \li 020: Eead vcob value from ::g_sRfTxCurrentService .sService.bFEVCO[7..4]
    \li 030: Read cpcc value from ::g_sRfTxCurrentService .sService.bFEVCO[3..0]
    \li 040: Extend signed 4 bit value vcob to 8 bit
    \li 050: Extend signed 4 bit value cpcc to 8 bit
    \li 060: Calucate 7 + vcob + vcob_offset and limit negative results to 0
    \li 070: Move 4 bit result of calculation to vcob[7..4]
    \li 080: Calucate 7 + cpcc + cpcc_offset and limit negative results to 0
    \li 090: Move 4 bit result of calculation to cpcc[3..0]
    \li 100: Combine vcob and cpcc for retVal

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868,
                  Primus2P-2443}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static UINT8FUNC ATA_rfTxFevcoCalculation_C(void)
{
    uint8_t bRetVal      = 0U;
    int8_t  bResult      = 0;
    eEepErrorCode sEepErrCode;

    /* LLR-Ref: 010 */
    uint8_t bFevcoOffset = 0U;
    sEepErrCode = ATA_eepReadBytes_C(&bFevcoOffset, (uint16_t)&g_sAtmelEEPromSection.eepFEVCOoffset, 1U);
    if(sEepErrCode != EEC_NO_ERROR)
    {
        ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR;
    }

    /* LLR-Ref: 020 */
    uint8_t bVcob        = (g_sRfTxCurrentService.sService.bFEVCO & 0xF0U) >> 4U;
    uint8_t bVcobOffset  = (bFevcoOffset & 0xF0U) >> 4U;

    /* LLR-Ref: 030 */
    uint8_t bCpcc        = g_sRfTxCurrentService.sService.bFEVCO & 0x0FU;
    uint8_t bCpccOffset  = bFevcoOffset & 0x0FU;

    /* LLR-Ref: 040 */
    if (bVcob > 7U) {
        ATA_SETBITMASK_C(bVcob,0xF0U)
    }

    /* LLR-Ref: 050 */
    if (bCpcc > 7U) {
        ATA_SETBITMASK_C(bCpcc,0xF0U)
    }

    /* LLR-Ref: 060 */
    bResult = 7 + (int8_t)bVcob + (int8_t)bVcobOffset;
    if(bResult < 0) {
        bResult = 0;
    }

    /* LLR-Ref: 070 */
    bVcob = (uint8_t)((uint8_t)bResult << 4U);
    bVcob &= 0xF0U;

    /* LLR-Ref: 080 */
    bResult = 7 + (int8_t)bCpcc + (int8_t)bCpccOffset;
    if (bResult < 0){
        bResult = 0;
    }

    /* LLR-Ref: 090 */
    bCpcc = (uint8_t)bResult;
    bCpcc &= 0x0FU;

    /* LLR-Ref: 100 */
    bRetVal = bVcob | bCpcc;

    return bRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxEnd_C</b>
    closes and switches all used peripherals off.

    Variable Usage:
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [in] ::g_sRfTx Global RF Tx component data

    \image html ATA_rfTxEnd_C.png

    \internal
    \li 010: Switch off Timer 3 by calling function ::ATA_timer3Close_C
                Note: Timer 2 already switched off in function ATA_rfTxShutdown_C
    \li 020: Reset FSCR and FSEN register
    \li 030: Disable SSM interrupt and stop currently running SSM anyway
    \li 040: Remove power from SSM, TX Modulator, CRC and TX DSP HW blocks
    \li 050: Check if system shall return to IDLEMode(RC) or IDLEMode(XTO)\n
             IF return to IDLEMode(XTO)
    \li 060: Reset SUPCR.AVDIC, reset FEEN1/FEEN2 register, set FEEN1.XTOEN\n
             ELSE (return to IDLEMode(RC))
    \li 070: Switch AVR core clock to FRC via function ::ATA_globalsClkSwitchFrc_C
             Reset FEEN1/FEEN2 and SUPCR register\n
             ENDIF
    \li 080: Reset SW statemachine data ::g_sRfTxFlowCtrl

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_rfTxEnd_C(void)
{
    /* LLR-Ref: 010 */
    ATA_timer3Close_C();

    /* LLR-Ref: 020 */
    FSCR = 0x00U;
    FSEN = 0x00U;

    /* LLR-Ref: 030 */
    SSMIMR = 0x00U;
    SSMRR  = BM_SSMST;

    /* LLR-Ref: 040 */
    ATA_POWEROFF_C(PRR2,PRSSM)
    ATA_POWEROFF_C(PRR2,PRTM)
    ATA_POWEROFF_C(PRR0,PRCRC)
    ATA_POWEROFF_C(PRR0,PRTXDC)

    /* LLR-Ref: 050 */
    if (g_sRfTx.bConfig & BM_RFTXCONFIG_BCONFIG_SHUTDOWN_MODE) {
        /* LLR-Ref: 060 */
        ATA_CLEARBITMASK_C(SUPCR,BM_AVDIC)
        FEEN2 = 0U;
        FEEN1 = BM_XTOEN;
    }
    else {
        /* LLR-Ref: 070 */
        /* --P2P-3655-- */
        if( g_sRfTx.bCmcrSetting & BM_CCS) 
        {
            uint8_t bClockSource = g_sRfTx.bCmcrSetting & (BM_CMM2|BM_CMM1|BM_CMM0);
            if ( bClockSource== 0)  // CLK_SRC
            {
                ATA_globalsClkSwitchSrc_C();
            }
            else if (bClockSource == 2) // CLK_EXT
            {
                if (SUPCR & BM_DVHEN)
                {
                    ATA_globalsClkSwitchExt_C(TRUE);
                } else {
                    ATA_globalsClkSwitchExt_C(FALSE);
                }
            }
            else // CLK_FRC
            {
                ATA_globalsClkSwitchFrc_C();
            }
        } else {    // CLK_MRC
            ATA_globalsClkSwitchMrc_C();
        }
        FEEN1 = 0U;
        ATA_CLEARBITMASK_C(SUPCR,(BM_AVEN | BM_AVCCRM | BM_AVCCLM | BM_AVDIC))
    }

    /* LLR-Ref: 080 */
    g_sRfTxFlowCtrl.bIndex    = 0U;
    g_sRfTxFlowCtrl.fpLut     = 0x0000U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartSendTelegramSSM_C</b>
    configures and starts the SendTelegram SSM.

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data

    \image html ATA_rfTxStartSendTelegramSSM_C.png

    \internal
    \li 010: Stop currently running SSM via SSMRR.SSMST
    \li 020: Configure the SSM via MSMCR1..4 as follows
                 - SSM_SEND_TELEGRAM_STATE
                 - SSM_END_STATE
    \li 030: Clear previous pending SSM interrupts via SSMIFR register
    \li 040: Clear previous pending SSM error flag via setting flag SSMSR.SSMERR
    \li 050: Clear rfTx SSM ready flag in ::g_sRfTx .bStatus[6]
    \li 060: Start SendTelegram SSM via SSMRR.SSMR with SEND_TELEGRAM state

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_rfTxStartSendTelegramSSM_C(void)
{
    /* LLR-Ref: 010 */
    SSMRR = BM_SSMST;

    /* LLR-Ref: 020 */
    MSMCR1 = SSM_SEND_TELEGRAM_STATE    | (uint8_t)(SSM_END_STATE << 4U);
    MSMCR2 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
    MSMCR3 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);
    MSMCR4 = SSM_END_STATE              | (uint8_t)(SSM_END_STATE << 4U);

    /* LLR-Ref: 030 */
    SSMIFR = 0x00U;

    /* LLR-Ref: 040 */
    ATA_SETBITMASK_C(SSMSR,BM_SSMERR)

    /* LLR-Ref: 050 */
    ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)

    /* LLR-Ref: 060 */
    SSMRR = BM_SSMR;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxOpenTimer3forTxing_C</b>
    configures timer 3 for TXMode(buffered).

    Variable Usage:
    \li [out] ::g_sTimer3 Global Timer 3 component data
    \li [in] ::g_sRfTxCurrentService Global RF Tx current service data

    \image html ATA_rfTxOpenTimer3forTxing_C.png

    \internal
    \li 010: Lock Timer 3 via ::g_sTimer3 .bStatus[7]
    \li 020: Switch on Timer 3 HW block
    \li 030: Configure Timer 3 registers for TXMode
                 - T3IMR, disable Timer 3 interrupts
                 - T3MRB, reset vlaue
                 - T3MRA, clk = xto/4 prescaler = 1
    \li 040: Setup T3CORL/H for Tx Data Rate with content from
              ::g_sRfTxCurrentService .sPath.bTXDR[1..0]
    \li 050: Enable Timer 3 in T3CR register with
                 - T3CR.T3CRM
                 - T3CR.T3CTM

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-847}
    \endinternal
    \n
*/
/*---------------------------------------------------------------------------*/
static VOIDFUNC ATA_rfTxOpenTimer3forTxing_C(void)
{
    /* LLR-Ref: 010 */
    ATA_SETBITMASK_C(g_sTimer3.bStatus,TMR3LOCK)

    /* LLR-Ref: 020 */
    ATA_POWERON_C(PRR1, PRT3)

    /* LLR-Ref: 030 */
    T3IMR = 0U;
    T3MRB = 0U;
    T3MRA = BM_T3CS1;

    /* LLR-Ref: 040 */
    T3CORL = g_sRfTxCurrentService.sPath.bTXDR[0];
    T3CORH = g_sRfTxCurrentService.sPath.bTXDR[1];

    /* LLR-Ref: 050 */
    ATA_SETBITMASK_C(T3CR,(BM_T3ENA | BM_T3CRM | BM_T3CTM))
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxSFifoFillLevelReached_ISR_C</b>
    signals S-FIFO fill level reached in ::g_sRfTx .bFlags[1].

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data

    \image html ATA_rfTxSFifoFillLevelReached_ISR_C.png

    \internal
    \li 010: Set fill level reached flag in ::g_sRfTx .bFlags[1]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-845}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
//lint -esym(765, ATA_rfTxSFifoFillLevelReached_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_rfTxSFifoFillLevelReached_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_rfTxSFifoFillLevelReached_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_rfTxSFifoFillLevelReached_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=SFFLR_vect */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_rfTxSFifoFillLevelReached_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_rfTxSFifoFillLevelReached_ISR_C, 0x00);
    
    /* LLR-Ref: 010 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_SFIFO_FILLLEVEL)
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxSFifoError_ISR_C</b>
    signals S-FIFO overflow/underflow error in ::g_sRfTx .bFlags[7]
    and ::g_sRfTx .bFlags[3].

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_rfTxSFifoError_ISR_C.png

    \internal
    \li 010: Disable SFIFO error interrupt by clearing SFI.SFERIM to avoid
              multiple signalling
    \li 020: Set SFIFO overflow/underflow error flag ::g_sRfTx .bFlags[3]
    \li 030: Set RF Tx module error flag ::g_sRfTx .bFlags[7]
    \li 040: Set error code ::g_sDebug .bErrorCode and ::g_sDebug .bSsmErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-845}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
//lint -esym(765, ATA_rfTxSFifoError_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_rfTxSFifoError_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_rfTxSFifoError_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_rfTxSFifoError_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=SFOUE_vect */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_rfTxSFifoError_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_rfTxSFifoError_ISR_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_CLEARBITMASK_C(SFI,BM_SFERIM)

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_SFIFO_ERROR)

    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)

    /* LLR-Ref: 040 */
    g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_SFIFO_ERROR;
    g_sDebug.bSsmErrorCode = 0U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxDFifoFillLevelReached_ISR_C</b>
    signals D-FIFO fill level reached in g_sRfTx.bFlags[2].

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data

    \image html ATA_rfTxDFifoFillLevelReached_ISR_C.png

    \internal
    \li 010: Set fill level reached flag ::g_sRfTx .bFlags[2]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-845}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
//lint -esym(765, ATA_rfTxDFifoFillLevelReached_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_rfTxDFifoFillLevelReached_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_rfTxDFifoFillLevelReached_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_rfTxDFifoFillLevelReached_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=DFFLR_vect */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_rfTxDFifoFillLevelReached_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_rfTxDFifoFillLevelReached_ISR_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_DFIFO_FILLLEVEL)
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxDFifoError_ISR_C</b>
    signals D-FIFO overflow/underflow error in ::g_sRfTx .bFlags[7]
    and ::g_sRfTx .bFlags[4].

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_rfTxDFifoError_ISR_C.png

    \internal
    \li 010: Disable D-FIFO error interrupt by clearing DFI.DFERIM to avoid
              multiple signalling
    \li 020: Set SFIFO overflow/underflow error flag in ::g_sRfTx .bFlags[4]
    \li 030: Set RF Tx module error flag ::g_sRfTx .bFlags[7]
    \li 040: Set error code ::g_sDebug .bErrorCode and ::g_sDebug .bSsmErrorCode

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-845}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
//lint -esym(765, ATA_rfTxDFifoError_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_rfTxDFifoError_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_rfTxDFifoError_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_rfTxDFifoError_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=DFOUE_vect */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_rfTxDFifoError_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_rfTxDFifoError_ISR_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_CLEARBITMASK_C(DFI,BM_DFERIM)

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_DFIFO_ERROR)

    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)

    /* LLR-Ref: 040 */
    g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_DFIFO_ERROR;
    g_sDebug.bSsmErrorCode = 0U;
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_rfTxSsmRdy_ISR_C</b>
    signals RF Tx SSM ready flag ::g_sRfTx .bStatus[6].

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data

    \image html ATA_rfTxSsmRdy_ISR_C.png

    \internal
    \li 010: Set RF Tx SSM ready flag ::g_sRfTx .bStatus[6]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
//lint -esym(765, ATA_rfTxSsmRdy_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_rfTxSsmRdy_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_rfTxSsmRdy_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_rfTxSsmRdy_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=SSM_vect */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_rfTxSsmRdy_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_rfTxSsmRdy_ISR_C, 0x00);

    /* LLR-Ref: 010 */
    ATA_SETBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxModulatorTelegramFinish_ISR_C</b>
    signals RF Tx Module transmission complete in ::g_sRfTx .bStatus[5]
    and End of Telegram in ::g_sRfTx .bFlags[0].

    Variable Usage:
    \li [out] ::g_sRfTx Global RF Tx component data

    \image html ATA_rfTxModulatorTelegramFinish_ISR_C.png

    \internal
    \li 010: Stop SendTelegram SSM via SSMRR.SSMST
    \li 020: Set RF Tx module transmission complete flag ::g_sRfTx .bStatus[5]
    \li 030: Set End of Telegram flag ::g_sRfTx .bFlags[0]

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-898}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
//lint -esym(765, ATA_rfTxModulatorTelegramFinish_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - external 'ATA_rfTxModulatorTelegramFinish_ISR_C' could be made static
 * interrupt service routine shall be accessed from outside via flash software 
 */
//lint -esym(714, ATA_rfTxModulatorTelegramFinish_ISR_C) FlSc (26.05.2014)
/* disable lint note 765 - Symbol 'ATA_rfTxModulatorTelegramFinish_ISR_C' not referenced
 * interrupt service routines are not directly referenced but are called
 * by the HW interrupt handler
 */
/* #pragma vector=TMTCF_vect */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_rfTxModulatorTelegramFinish_ISR_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_ISR_C(ATA_rfTxModulatorTelegramFinish_ISR_C, 0x00);

    /* LLR-Ref: 010 */
    SSMRR = BM_SSMST;

    /* LLR-Ref: 020 */
    ATA_SETBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_TRANSMISSION_COMPLETE)

    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_EOT)
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_rfTxStartShutDownSSM_C</b>
    starts the SSM shutdown.
    
    \image html ATA_rfTxStartShutDownSSM_C.png

    \internal
    \li 010: Reset FSCR register (except FSCR.PAOER --> done via SSM)
    \li 020: Reset FSEN register (except FSCR.ANTT  --> done via SSM)
    \li 030: Power up SSM HW block
    \li 040: Stop currently running SSM via SSMRR.SSMST
    \li 050: Reset SSM by clearing SSMCR register content
    \li 060: Configure SSM via MSMCR1..4 as follows
                 - SSM_SHUTDOWN_STATE
                 - SSM_END_STATE
    \li 070: Reset SSMIFR to clear previous SSM interrupt flags
    \li 080: Reset SSMSR.SSMERR flag
    \li 090: Enable SSM ready interrupt in SSMIMR.SSMIM

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
static VOIDFUNC ATA_rfTxStartShutDownSSM_C(void)
{
    /* LLR-Ref: 010 */
    ATA_CLEARBITMASK_C(FSCR,BM_PAOER)

    /* LLR-Ref: 020 */
    ATA_CLEARBITMASK_C(FSEN,BM_ANTT)

    /* LLR-Ref: 030 */
    ATA_POWERON_C(PRR2, PRSSM)

    /* LLR-Ref: 040 */
    SSMRR = BM_SSMST;

    /* LLR-Ref: 050 */
    SSMCR = 0x00U;

    /* LLR-Ref: 060 */
    MSMCR1 = SSM_SHUTDOWN_STATE | (uint8_t)(SSM_END_STATE << 4U);

    /* LLR-Ref: 070 */
    SSMIFR = 0x00U;

    /* LLR-Ref: 080 */
    ATA_SETBITMASK_C(SSMSR,BM_SSMERR)

    /* LLR-Ref: 090 */
    SSMIMR = BM_SSMIM;

    /* LLR-Ref: 100 */
    SSMRR  = BM_SSMR;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartSsmWatchdog_C</b>
    starts Timer2 to check timeouts of HW sequencer state machines.

    \return     OK on success, FAIL on failure

    \image html ATA_rfTxStartSsmWatchdog_C.png

    \internal
    \li 010: Open Timer 2 by calling function ::ATA_timer2Open_C in order to start
             the SSM watchdog timer.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_rfTxStartSsmWatchdog_C(void)
{
    sTimerAsyn8BitParams  sTimer2Params = {
        BM_T2ENA,                   /* T2CR */
        0x00U,                      /* T2MR */
        0xFFU,                      /* T2COR */
        0x00U,                      /* T2IMR */
        (timerIRQHandler)0x0000,    /* g_sTimer2.ovfIsr */
        (timerIRQHandler)0x0000     /* g_sTimer2.compIsr */
        };

    /* LLR-Ref: 010 */
    return ATA_timer2Open_C(&sTimer2Params);
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_rfTxStopSsmWatchdog_C</b>
    stops the sequencer state machine watchdog.

    \image html ATA_rfTxStopSsmWatchdog_C.png

    \internal
    \li 010: Close Timer 2 by calling function ::ATA_timer2Close_C in order to
             stop the SSM watchdog timer.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869,Primus2P-849,Primus2P-848,Primus2P-847,Primus2P-868}
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_rfTxStopSsmWatchdog_C(void)
{
    /* LLR-Ref: 010 */
    ATA_timer2Close_C();
}
