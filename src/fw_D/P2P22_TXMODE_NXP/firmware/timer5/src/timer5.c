//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer5/src/timer5.c $
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
/** \file timer5.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "timer5.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/** \brief <b>g_sTimer5</b>
    contains the configuration settings for Timer5.
*/
#pragma location = ".tmr5ram"
__no_init sTmr5Config g_sTimer5;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer5Init_C</b>
    shall initialize Timer5 data structure sTmr5Config.

    Variable Usage:
    \li [out] ::g_sTimer5  global Timer 5 configuration

    \image html ATA_timer5Init_C.png

    \internal
        \li 010:   Set all ::g_sTimer5 attributes to 0.

        \Derived{Yes}

        \Rationale{Since the hardware SRAM initialization is turned off, all SRAM
               variables need initialization prior to its usage}

        \Traceability   N/A
    \endinternal
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer5Init_C(void)
{
    /* LLR-Ref: 010 */
    g_sTimer5.bStatus  = 0x00U;
    g_sTimer5.fpCompIsr = (timerIRQHandler)0x0000U;
    g_sTimer5.fpOvfIsr  = (timerIRQHandler)0x0000U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer5Open_C</b>
    shall open and initialize Timer 5, in case it is not yet locked.

    \param[in]      pTimer5Params    synchronous timer configuration settings
    \return OK on success, FAIL on failure

    Variable Usage:
    \li [in,out] ::g_sTimer5      Global Timer 5 component data

    \image html ATA_timer5Open_C.png


    \internal
             Note:
             Power up of Timer 5 via register PRR1 needs to occur before any 
             Timer 5 register is accessed in order for the intended action to 
             take effect.

             IF Timer 5 is not locked, i.e. not already used by another component,
             THEN
    \li 010:   Lock Timer 5 by setting the lock status in global variable 
               ::g_sTimer5 in order to synchronize the usage of Timer 5.

    \li 020:   Power up Timer 5 by setting bit PRT5 in register PRR1 to 0 to 
               disable the power reduction feature for Timer 5.

    \li 030:   Stop Timer 5 by setting control register T5CCR to 0x00 in order to 
               securely configure Timer 5.

    \li 040:   Reset Timer 5 compare and overflow flags by writing a 1 to the
               bits T5COF and T5OFF of register T5IFR.

    \li 050:   Store the given function pointer parameters "ovf" and "comp" of
               parameter "pTimer5Params" within to
               the global variable ::g_sTimer5 in order for those functions to be 
               called when the corresponding interrupt, overflow and compare, is
               triggered.

    \li 060:   Set Timer 5 registers T5CNTH, T5CNTL, T5OCRH, T5OCRL T5IMR and T5CCR 
               to the given attributes of parameter "pTimer5Params".

             ENDIF

    \li 070: Return OK to the calling function if Timer 5 was successfully 
             configured.
             Return FAIL, since Timer 5 was already locked and therefore could 
             not be configured.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-889,Primus2P-1328}
    \endinternal
    \n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_timer5Open_C(const sTimerSyn16BitParams * const pTimer5Params)
{
    uint8_t fRetVal = FAIL;
    
    if( !(g_sTimer5.bStatus & TMR5LOCK) )
    {
        /* LLR-Ref: 010 */
        g_sTimer5.bStatus |= TMR5LOCK;
        
        /* LLR-Ref: 020 */
        ATA_POWERON_C(PRR1, PRT5)

        /* LLR-Ref: 030 */
        T5CCR = 0x00U;

        /* LLR-Ref: 040 */
        g_sTimer5.fpCompIsr = pTimer5Params->compIsr;
        g_sTimer5.fpOvfIsr = pTimer5Params->ovfIsr;

        /* LLR-Ref: 050 */
        T5IFR |= ((uint8_t)(BM_T5COF|BM_T5OFF));
        
        /* LLR-Ref: 060 */
        _CLI;
        T5CNTH = pTimer5Params->countH;
        T5CNTL = pTimer5Params->countL;
        _SEI;
        
        _CLI;
        T5OCRH = pTimer5Params->compH;
        T5OCRL = pTimer5Params->compL;
        _SEI;

        T5IMR = pTimer5Params->irqMask;
        T5CCR = pTimer5Params->ctrl;
        
        fRetVal = OK;
    }
    
    /* LLR-Ref: 070 */
    return fRetVal;
}

/* ---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer5Close_C</b>
    shall stop Timer 5 and enable the power reduction feature of Timer 5.

    Variable Usage:
    \li [out] ::g_sTimer5  Global Timer 5 component data

    \image html ATA_timer5Close_C.png

    \internal
             Note:
             Disabling of the Power Reduction feature of Timer 5 via register PRR1
             needs to occur before any Timer 5 register is accessed in order for the
             intended actions to take effect.
             Enabling of the Power Reduction feature of Timer 5 via register PRR1 
             needs to occur after any Timer 5 register is accessed in order for the 
             intended action to take effect.

    \li 005: Disable the power reduction feature of Timer 5 by setting bit PRT5 in 
             register PRR1 to 0.

    \li 010: Disable Timer 5 by setting all bits of register T5CCR to 0.

    \li 020: Unlock Timer 5 by setting the status indication of the global variable
             ::g_sTimer5 to 0.

    \li 030: Enable the power reduction feature of Timer 5 by setting bit PRT5 in 
             register PRR1 to 1.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-890}
    \endinternal
    \n
*/
/* ---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer5Close_C(void)
{
    /* LLR-Ref: 005 */
    ATA_POWERON_C(PRR1, PRT5)
  
    /* LLR-Ref: 010 */
    T5CCR = 0x00U;
    
    /* LLR-Ref: 020 */
    g_sTimer5.bStatus = 0x00U;
    
    /* LLR-Ref: 030 */
    ATA_POWEROFF_C(PRR1, PRT5);
}

