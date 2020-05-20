//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/rftx/src/rftx_vco.c $
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
/** \file rftx_vco.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "rftx_vco.h"
#include "rftx.h"

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
/** \brief <b>g_bVcoTuningResult</b>
    contains the last VCO tuning result
 */
 #pragma location = ".rftx"
__root __no_init uint8_t g_bVcoTuningResult;

/** \brief <b>g_pRfTxVcoTuneStateMachine</b>
    points to look up table of the SW state machine used for
    VCO tuning.
*/
#pragma location = ".flowctrl"
__no_init sysFlowStateMachineFuncLut_t *g_pRfTxVcoTuneStateMachine;

//lint -esym(9003, g_sRfTxVcoTuneStateMachineLut) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sRfTxVcoTuneStateMachineLut' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
/** <b>g_sRfTxVcoTuneStateMachineLut</b>
    look up table of the SW state machine used for VCO tuning.
    \details
    \li ATA_rfTxInitVcoSSM_C:
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
    \li ATA_rfTxWait4VCoSSMrdy_C:
        waits until the sequencer state machine is ready
    \li ATA_rfTxShutdown_C:
        shut down the RFTX module related state machine for TXMode(buffered and transparent),
        VCO tuning and antenna tuning.

 */
__root sysFlowStateMachineFuncLut_t g_sRfTxVcoTuneStateMachineLut[] = {
    ATA_rfTxInitVcoSSM_C,
    ATA_rfTxWait4AVCC_C,
    ATA_rfTxInitFrontEnd_C,
    ATA_rfTxWait4XTO_C,
    ATA_rfTxStartVcoSSM_C,
    ATA_rfTxWait4VcoSSMrdy_C,
    ATA_rfTxShutdown_C
};

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartVco_C</b>
    starts a VCO tuning sequence
    
    \param[in]      bConfig         Configuration of selected RFTXMode. For details see ::g_sRfTx .bConfig
    \param[in]      pAddress        Pointer to Service/Channel configuration

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data

    \image html ATA_rfTxStartVco_C.png

    \internal
    \li 005: Set HW trace point for this function

    \li 010: Initialize ::g_bVcoTuningResult

    \li 020: Initialize RF Tx module by calling function ::ATA_rfTxStartTx_C

    \li 030: Patch RF Tx module configuration ::g_sRfTx .bConfig by setting 
             VCO tuning explicitly

    \li 040: IF no error occured during startup of the RF Tx state machine 
              indicated by ::g_sRfTx .bFlags,
             THEN
               Patch SW state machine pointer ::g_sRfTxFlowCtrl .bfpLutwith with 
               the VCO tuning SW state machine
             ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartVco_C(uint8_t bConfig, uint8_t *pAddress)
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStartVco_C, bConfig);

    /* LLR-Ref: 010 */
    g_bVcoTuningResult = 0U;
    
    /* LLR-Ref: 020 */
    ATA_rfTxStartTx_C(bConfig, pAddress);
    
    /* LLR-Ref: 030 */
    ATA_SETBITMASK_C(g_sRfTx.bConfig,BM_RFTXCONFIG_BCONFIG_VCO_TUNING)

    /* LLR-Ref: 040 */
    if ( (g_sRfTx.bFlags & BM_RFTXCONFIG_BFLAGS_ERROR) == 0x00U )
    {
        g_sRfTxFlowCtrl.fpLut = g_pRfTxVcoTuneStateMachine;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStopVco_C</b>
    shuts down a currently running VCO tuning sequence

    \image html ATA_rfTxStopVco_C.png

    \internal
    \li 010: Shutdown currently running VCO tuning by calling function 
             ::ATA_rfTxShutdown_C

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStopVco_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStopVco_C, 0x00);
    
    /* LLR-Ref: 010*/
    ATA_rfTxShutdown_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxGetVcoTuningResult_C</b>
    returns last VCO tuning result

    Variable Usage:
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data

    \image html ATA_rfTxGetVcoTuningResult_C.png

    \internal
    \li 010: Return ::g_bVcoTuningResult

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_rfTxGetVcoTuningResult_C(void)
{
    /* LLR-Ref: 010*/
    return g_bVcoTuningResult;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxInitVcoSSM_C</b>
    this sw state does the register initialization according to the selected 
    service channel configuration via patching the function ATA_rfTxInitTxSSM_C.

    Variable Usage:
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data
    
    \image html ATA_rfTxInitVcoSSM_C.png

    \internal
    \li 010: Initialize via function ::ATA_rfTxInitTxSSM_C
    \li 020: Patch SSM configuration by set end state after VCO tuning

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxInitVcoSSM_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxInitVcoSSM_C, 0x00);

    /* LLR-Ref: 010*/
    ATA_rfTxInitTxSSM_C();
    
    /* LLR-Ref: 020*/
    MSMCR2 = SSM_END_STATE | (uint8_t)(SSM_END_STATE << 4U);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartVcoSSM_C</b>
    starts the sequencer state machine for VCO tuning

    \image html ATA_rfTxStartVcoSSM_C.png

    \internal
    \li 010: Call function ::ATA_rfTxStartSSM_C to configure the RF Tx statemachine
             to execute the VCO tuning shutdown sequence

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartVcoSSM_C(void)
{
    ATA_rfTxStartSSM_C(RFTX_VCO_STATE_SHUTDOWN);
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4VcoSSMrdy_C</b>
    waits until the sequencer state machine is ready

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_bVcoTuningResult Global RF Tx VCO tuning component data
    \li [out] ::g_sRfTxCurrentService Global RF Tx current service data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_rfTxWait4VcoSSMrdy_C.png

    \internal
    \li 010: Check if SSM has finished with success via variable ::g_sRfTx 
              .bStatus[6] and SSM status register SSMSR\n
             IF SSM has finished with success
    \li 020:     Stop SSM Watchdog
    \li 030:     Clear RF Tx SSM ready in ::g_sRfTx .bStatus[6]
    \li 040:     Store VCO tuning result in ::g_bVcoTuningResult
                  and update service channel configuration ::g_sRfTxCurrentService 
                  .sService.bFEVCT
    \li 045:     Set VCO_TUNE_RDY flag in ::g_sRfTx .bTuneFlags to indicate the end 
                    of an VCO Tuning sequence
    \li 050:     Switch to next SW state by incrementing ::g_sRfTxFlowCtrl .bIndex\n
             ELSE (SSM has not finished)
    \li 060:  Check for 
                 - SSM Watchdog timeout via flag T2IFR.T2COF
                 - SSM Error via register SSMSR\n
                 IF Watchdog timeout or SSM Error has occured
    \li 070:         Reset current running sequencer state machine
    \li 080:         Stop SSM Watchdog
    \li 090:         Signal error via flag ::g_sRfTx .bFlags[7]
    \li 100:         Write error code to ::g_sDebug .bErrorCode and ::g_sDebug
                      .bSsmErrorCode
    \li 110:         Switch to shutdown sw state by setting ::g_sRfTxFlowCtrl 
                      .bIndex to RFTX_VCO_STATE_SHUTDOWN    

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-868}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4VcoSSMrdy_C(void)
{
    /* LLR-Ref: 010*/
    if( (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_SSMREADY) && (SSMSR == 0x00U) ) {

        ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4VcoSSMrdy_C, 0x00);

        /* LLR-Ref: 020*/
        ATA_rfTxStopSsmWatchdog_C();

        /* LLR-Ref: 030*/
        ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)

        /* LLR-Ref: 040*/
        g_sRfTxCurrentService.sService.bFEVCT = FEVCT;
        g_bVcoTuningResult = g_sRfTxCurrentService.sService.bFEVCT;
        
        /* LLR-Ref: 045*/
        ATA_SETBITMASK_C(g_sRfTx.bTuneFlags,BM_RFTXCONFIG_BTUNEFLAGS_VCO_TUNE_RDY)
        
        /* LLR-Ref: 050*/
        g_sRfTxFlowCtrl.bIndex++;
    }
    else {
        /* LLR-Ref: 060*/
        if( (T2IFR & BM_T2COF) || SSMSR ) {    // timeout of SSM occured
            /* LLR-Ref: 070*/
            SSMRR = BM_SSMST;       // reset current state machine

            /* LLR-Ref: 080*/
            ATA_rfTxStopSsmWatchdog_C();

            /* LLR-Ref: 090*/
            ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)

            /* LLR-Ref: 100*/
            g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_WAIT4VCOSSMRDY_TIMEOUT;
            g_sDebug.bSsmErrorCode = SSMSR;
  
            /* LLR-Ref: 110*/
            g_sRfTxFlowCtrl.bIndex = RFTX_VCO_STATE_SHUTDOWN;
        }
    }
}
