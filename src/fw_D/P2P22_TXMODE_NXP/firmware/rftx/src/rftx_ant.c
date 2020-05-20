//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/rftx/src/rftx_ant.c $
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
/** \file rftx_ant.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "rftx_ant.h"
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
/** \brief <b>g_bAntennaTuningResult</b>
    contains the last antenna tuning result
 */
#pragma location = ".rftx"
__root __no_init uint8_t g_bAntennaTuningResult;

/** \brief <b>g_pRfTxAntTuneStateMachine</b>
    points to look up table of the SW state machine used for
    antenna tuning.
*/
#pragma location = ".flowctrl"
__root __no_init sysFlowStateMachineFuncLut_t *g_pRfTxAntTuneStateMachine;

//lint -esym(9003, g_sRfTxAntTuneStateMachineLut) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sRfTxAntTuneStateMachineLut' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
/** <b>g_sRfTxAntTuneStateMachineLut</b>
    look up table of the SW state machine used for antenna tuning.
    \details
    \li ATA_rfTxInitTxSSM_C:
        this sw state does the register initialization according to the selected 
        service channel configuration. Note: FE register are initialized after AVCC 
        is stable in sw state ::ATA_rfTxInitFrontEnd_C
    \li ATA_rfTxWait4AVCC_C:
        waits until AVCC is stable and switches to next sw state if AVCC is stable.
    \li ATA_rfTxInitFrontEnd_C:
        initializes the frontend registers according to the selected service channel
        configuration
    \li ATA_rfTxWait4XTO_C:
        waits until XTO is ready. If AVR is not running with CLKXTO4 the
        core clock is switched to CLKXTO4
    \li ATA_rfTxStartAntSSM_C:
        starts the sequencer state machine
    \li ATA_rfTxWait4AntSSMrdy_C:
        waits until the sequencer state machine is ready
    \li ATA_rfTxShutdown_C:
        shut down the RFTX module related state machine for TXMode(buffered and transparent),
        VCO tuning and antenna tuning.

 */
__root sysFlowStateMachineFuncLut_t g_sRfTxAntTuneStateMachineLut[] = {
    ATA_rfTxInitTxSSM_C,
    ATA_rfTxWait4AVCC_C,
    ATA_rfTxInitFrontEnd_C,
    ATA_rfTxWait4XTO_C,
    ATA_rfTxStartAntSSM_C,
    ATA_rfTxWait4AntSSMrdy_C,
    ATA_rfTxShutdown_C
};

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartAnt_C</b>
    starts an antenna tuning sequence

    \param[in]      bConfig         Configuration of selected RFTXMode. For details see ::g_sRfTx .bConfig
    \param[in]      pAddress        Pointer to Service/Channel configuration

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_bAntennaTuningResult Global RF Tx Antenna tuning component data

    \image html ATA_rfTxStartAnt_C.png

    \internal
    \li 005: Set HW trace point for this function

    \li 010: Initialize ::g_bAntennaTuningResult

    \li 020: Initialize RF Tx module by calling function ::ATA_rfTxStartTx_C

    \li 030: Patch RF Tx module configuration ::g_sRfTx .bConfig by setting 
             VCO and Antenna tuning explicitly

    \li 040: IF no error occured during startup of the RF Tx state machine 
              indicated by ::g_sRfTx .bFlags,
             THEN
               Patch SW state machine pointer ::g_sRfTxFlowCtrl .bfpLutwith with 
               the Antenna tuning SW state machine
             ENDIF

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartAnt_C(uint8_t bConfig, uint8_t *pAddress)
{
    /* LLR-Ref: 005*/
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStartAnt_C, bConfig);
    
    /* LLR-Ref: 010*/
    g_bAntennaTuningResult = 0U;
    
    /* LLR-Ref: 020*/
    ATA_rfTxStartTx_C(bConfig, pAddress);
    
    /* LLR-Ref: 030*/
    ATA_SETBITMASK_C(g_sRfTx.bConfig,(BM_RFTXCONFIG_BCONFIG_VCO_TUNING
                                      | BM_RFTXCONFIG_BCONFIG_ANT_TUNING))
    
    /* LLR-Ref: 040*/
    if ((g_sRfTx.bFlags & BM_RFTXCONFIG_BFLAGS_ERROR) == 0x00U)
    {
        g_sRfTxFlowCtrl.fpLut = g_pRfTxAntTuneStateMachine;
    }
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStopAnt_C</b>
    shuts down a currently running antenna tuning sequence
    
    \image html ATA_rfTxStopAnt_C.png

    \internal
    \li 010: Shutdown currently running antenna tuning by calling function 
            ::ATA_rfTxShutdown_C

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStopAnt_C(void)
{
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxStopAnt_C, 0x00);
    
    /* LLR-Ref: 010 */
    ATA_rfTxShutdown_C();
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxGetAntennaTuningResult_C</b>
    returns last antenna tuning result

    Variable Usage:
    \li [out] ::g_bAntennaTuningResult Global RF Tx Antenna tuning component data

    \image html ATA_rfTxGetAntennaTuningResult_C.png

    \internal
    \li 010: Return ::g_bAntennaTuningResult

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_rfTxGetAntennaTuningResult_C(void)
{
    /* LLR-Ref: 010*/
    return g_bAntennaTuningResult;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxStartAntSSM_C</b>
    starts the sequencer state machine for antenna tuning

    \image html ATA_rfTxStartVcoSSM_C.png

    \internal
    \li 010: Call function ::ATA_rfTxStartSSM_C to configure the RF Tx statemachine
             to execute the Antenna tuning shutdown sequence

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxStartAntSSM_C(void)
{
    /* LLR-Ref: 010*/
    ATA_rfTxStartSSM_C(RFTX_ANT_STATE_SHUTDOWN);
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_rfTxWait4AntSSMrdy_C</b>
    waits until the sequencer state machine is ready

    Variable Usage:
    \li [in,out] ::g_sRfTx Global RF Tx component data
    \li [out] ::g_sRfTxFlowCtrl Global RF Tx statemachine data
    \li [out] ::g_bAntennaTuningResult Global RF Tx VCO tuning component data
    \li [out] ::g_sRfTxCurrentService Global RF Tx current service data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_rfTxWait4AntSSMrdy_C.png

    \internal
    \li 010: Check if SSM has finished with success via variable g_sRfTx.bStatus[6]
             and SSM status register SSMSR\n
             IF SSM has finished with success
    \li 020:     Stop SSM Watchdog
    \li 030:     Clear RF Tx SSM ready in ::g_sRfTx .bStatus[6]
    \li 040:     Store antenna tuning result in ::g_bAntennaTuningResult
                 and update Service/Channel configuration ::g_sRfTxCurrentService 
                 .sService.bFEAT
    \li 045:     Set ANT_TUNE_RDY flag in ::g_sRfTx .bTuneFlags to indicate the end 
                 of an antenna tuning sequence
    \li 050:     Switch to next sw state by incrementing ::g_sRfTxFlowCtrl .bIndex\n
             ELSE (SSM has not finished)
    \li 060:     Check for 
                  - SSM Watchdog timeout via flag T2IFR.T2COF
                  - SSM Error via register SSMSR\n
                  IF Watchdog timeout or SSM Error has occured
    \li 070:        Reset current running sequencer state machine
    \li 080:        Stop SSM Watchdog
    \li 090:        Signal error via flag ::g_sRfTx .bFlags[7]
    \li 100:        Write error code to ::g_sDebug .bErrorCode and ::g_sDebug 
                    .bSsmErrorCode
    \li 110:        Switch to shutdown sw state by setting ::g_sRfTxFlowCtrl
                    .bIndex to RFTX_ANT_STATE_SHUTDOWN

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-869}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_rfTxWait4AntSSMrdy_C(void)
{
    /* LLR-Ref: 010*/
    if( (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_SSMREADY) && (SSMSR == 0x00U) ) {
        
        ATA_SET_FUNCTION_TRACE_POINT_C(ATA_rfTxWait4AntSSMrdy_C, 0x00);
        
        /* LLR-Ref: 020*/
        ATA_rfTxStopSsmWatchdog_C();
        
        /* LLR-Ref: 030*/
        ATA_CLEARBITMASK_C(g_sRfTx.bStatus,BM_RFTXCONFIG_BSTATUS_SSMREADY)
        
        /* LLR-Ref: 040*/
        g_sRfTxCurrentService.sService.bFEAT = FEAT;
        g_bAntennaTuningResult = g_sRfTxCurrentService.sService.bFEAT;
        
        /* LLR-Ref: 045*/
        ATA_SETBITMASK_C(g_sRfTx.bTuneFlags,BM_RFTXCONFIG_BTUNEFLAGS_ANT_TUNE_RDY)
        
        /* LLR-Ref: 050*/
        g_sRfTxFlowCtrl.bIndex++;
    }
    else {
        /* LLR-Ref: 060*/
        if( (T2IFR & BM_T2COF) || SSMSR ) {
            /* LLR-Ref: 070*/
            SSMRR = BM_SSMST;

            /* LLR-Ref: 080*/
            ATA_rfTxStopSsmWatchdog_C();

            /* LLR-Ref: 090*/
            ATA_SETBITMASK_C(g_sRfTx.bFlags,BM_RFTXCONFIG_BFLAGS_ERROR)

            /* LLR-Ref: 100*/
            g_sDebug.bErrorCode    = DEBUG_ERROR_CODE_RFTX_WAIT4ANTSSMRDY_TIMEOUT;
            g_sDebug.bSsmErrorCode = SSMSR;

            /* LLR-Ref: 110*/
            g_sRfTxFlowCtrl.bIndex = RFTX_ANT_STATE_SHUTDOWN;
        }
    }
}
