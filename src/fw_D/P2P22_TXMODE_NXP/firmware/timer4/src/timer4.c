//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer4/src/timer4.c $
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
/** \file timer4.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "timer4.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/** \brief <b>g_sTimer4</b>
    contains the configuration settings for Timer4.
*/
#pragma location = ".tmr4ram"
__no_init sTmr4Config g_sTimer4;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer4Init_C</b>
    shall initialize Timer4 data structure sTmr4Config.

    Variable Usage:
    \li [out] ::g_sTimer4  Global Timer 4 component data

    \image html ATA_timer4Init_C.png

    \internal
        \li 010:   Set all ::g_sTimer4 attributes to 0.

        \Derived{Yes}

        \Rationale{Since the hardware SRAM initialization is turned off, all SRAM
                   variables need initialization prior to its usage}

        \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer4Init_C(void)
{
    g_sTimer4.bStatus  = 0x00U;
    g_sTimer4.fpCompIsr = (timerIRQHandler)0x0000U;
    g_sTimer4.fpOvfIsr  = (timerIRQHandler)0x0000U;
    g_sTimer4.fpCapIsr  = (timerIRQHandler)0x0000U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer4Open_C</b>
    shall open and initialize Timer 4, in case it is not yet locked.

    \param[in]      pTimer4Params    Asynchronous timer configuration settings
    \return OK on success, FAIL on failure
    
    Variable Usage:
    \li [in,out] ::g_sTimer4 Global Timer 4 component data
    
    \image html ATA_timer4Open_C.png

    \internal
             Note:
             Power up of Timer 4 via register PRR1 needs to occur before any 
             Timer 4 register is accessed in order for the intended action to 
             take effect.

             IF Timer 4 is not locked, i.e. not already used by another component,
             THEN
    \li 010:   Lock Timer 4 by setting the lock status in global variable 
               ::g_sTimer4 in order to synchronize the usage of Timer 4.

    \li 020:   Power up Timer 4 by setting bit PRT4 in register PRR1 to 0 to 
               disable the power reduction feature for Timer 4.

    \li 030:   Disable and reset Timer 4 by writing a 1 to the bit T4RES in
               register T4CR and a 0 to bit T4ENA.

    \li 040:   Store the given function pointer parameters "ovf", "comp" and "cap"
               of parameter "pTimer4Params"
               to the global variable ::g_sTimer4 in order for those functions 
               to be called when the corresponding interrupt, overflow and compare 
               and capture, is triggered.

    \li 050:   Clear Timer 4 interrupt flags T4COF, T4OFF and T4ICF in register
               T4IFR.

    \li 060:   Set Timer 4 registers T4CORH, T4CORL, T4MRA, T4MRB T4IMR and T4CR 
               to the given attributes of parameter "pTimer4Params".

             ENDIF

    \li 070: Return OK to the calling function if Timer 4 was successfully 
             configured.
             Return FAIL if Timer 4 was already locked and therefore could 
             not be configured.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-887,Primus2P-1328}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_timer4Open_C(const sTimerAsyn16BitParams * const pTimer4Params)
{
    uint8_t fRetVal = FAIL;
    
    if( !(g_sTimer4.bStatus & TMR4LOCK) )
    {
        /* LLR-Ref: 010 */
        g_sTimer4.bStatus |= TMR4LOCK;
      
        /* LLR-Ref: 020 */
        ATA_POWERON_C(PRR1, PRT4)
        
        /* LLR-Ref: 030 */
        T4CR &= ~BM_T4ENA;
        T4CR = BM_T3RES;

        /* LLR-Ref: 040 */
        g_sTimer4.fpCompIsr = pTimer4Params->compIsr;
        g_sTimer4.fpOvfIsr = pTimer4Params->ovfIsr;
        g_sTimer4.fpCapIsr = pTimer4Params->capIsr;
        
        /* LLR-Ref: 050 */
        T4IFR |= ((uint8_t)(BM_T4ICF|BM_T4COF|BM_T4OFF));
        
        /* LLR-Ref: 060 */
        T4MRA  = pTimer4Params->modeA;
        T4MRB  = pTimer4Params->modeB;
        T4CORL = pTimer4Params->compL;
        T4CORH = pTimer4Params->compH;
        T4IMR  = pTimer4Params->irqMask;
        T4CR   = pTimer4Params->ctrl;
        
        fRetVal = OK;
    }
    
    /* LLR-Ref: 070 */
    return fRetVal;
}


/* ---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer4Close_C</b>
    shall stop Timer 4 and enable the power reduction feature of Timer 4.

    Variable Usage:
    \li [out] ::g_sTimer4  Global Timer 4 component data

    \image html ATA_timer4Close_C.png

    \internal
             Note:
             Disabling of the Power Reduction feature of Timer 4 via register PRR1
             needs to occur before any Timer 4 register is accessed in order for the
             intended actions to take effect.
             Enabling of the Power Reduction feature of Timer 4 via register PRR1 
             needs to occur after any Timer 4 register is accessed in order for the 
             intended action to take effect.

    \li 005: Disable the power reduction feature of Timer 4 by setting bit PRT4 in 
             register PRR1 to 0.

    \li 010: Disable Timer 4 by setting all bits of register T4CR to 0,
             except bit T4RES which is set to 1 to reset the prescaler and counter
             value.

    \li 020: Unlock Timer 4 by setting the status indication of the global variable
             ::g_sTimer4 to 0.

    \li 030: Enable the power reduction feature of Timer 4 by setting bit PRT4 in 
             register PRR1 to 1.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-888}
    \endinternal
\n
*/
/* ---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer4Close_C(void)
{
    /* LLR-Ref: 005 */
    ATA_POWERON_C(PRR1, PRT4)
  
    /* LLR-Ref: 010 */
    T4CR = BM_T4RES;
    
    /* LLR-Ref: 020 */
    g_sTimer4.bStatus = 0x00;
  
    /* LLR-Ref: 030 */
    ATA_POWEROFF_C(PRR1, PRT4);
}

