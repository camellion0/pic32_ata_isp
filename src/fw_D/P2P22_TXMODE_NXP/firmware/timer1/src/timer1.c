//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer1/src/timer1.c $
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
/** \file timer1.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "timer1.h"
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/** \brief <b>g_sTimer1</b>
    contains the configuration settings for Timer 1.
*/
#pragma location = ".tmr1ram"
__no_init sTmr1Config g_sTimer1;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer1Init_C</b>
    shall initialize Timer1 data structure sTmr1Config.

    Variable Usage:
    \li [out] ::g_sTimer1 Global Timer 1 component data

    \image html ATA_timer1Init_C.png

    \internal
    \li 010:   Set all ::g_sTimer1 attributes to 0.

    \Derived{Yes}

    \Rationale{Since the hardware SRAM initialization is turned off, all SRAM
               variables need initialization prior to its usage}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer1Init_C(void)
{
    /* LLR-Ref: 010 */
    g_sTimer1.bStatus  = 0x00U;
    g_sTimer1.fpCompIsr = (timerIRQHandler)0x0000U;
    g_sTimer1.fpOvfIsr  = (timerIRQHandler)0x0000U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer1Open_C</b>
    shall open and initialize Timer1, in case it is not yet locked.

    \param[in]      pTimer1Params   Asynchronous timer configuration settings
    \return     OK on success, FAIL on failure
    
    Variable Usage:
    \li [in,out] ::g_sTimer1 Global Timer 1 component data

    \image html ATA_timer1Open_C.png

    \internal
             Note:
             Power up of Timer 1 via register PRR1 needs to occur before any 
             Timer 1 register is accessed in order for the intended action to 
             take effect.

             IF Timer 1 is not locked, i.e. not already used by another component,
             THEN

    \li 010:   Lock Timer 1 by setting the lock status in global variable 
               ::g_sTimer1 in order to synchronize the usage of Timer 1.

    \li 020:   Power up Timer 1 by setting bit PRT1 in register PRR1 to 0 to 
               disable the power reduction feature for Timer 1.

    \li 030:   Disable and reset Timer 1 by writing a 1 to the bit T1RES in
                register T1CR and a 0 to bit T1ENA.

    \li 040:   Store the given function pointer parameters "ovf" and "comp" of 
               parameter "pTimer1Params" within to the global variable ::g_sTimer1
               in order for those functions to be called when the corresponding
               interrupt, overflow and compare, is triggered.

    \li 050:   Clear Timer 1 interrupt flags in T1IFR. 

    \li 060:   Set Timer 1 registers T1COR, T1MR, T1IMR and T1CR to the 
                given attributes of parameter "pTimer1Params".

             ENDIF

    \li 070: Return OK to the calling function if Timer 1 was successfully 
              configured.
             Return FAIL if Timer 1 was already locked and therefore could 
              not be configured.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-881,Primus2P-1328}
    \endinternal 
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_timer1Open_C(const sTimerAsyn8BitParams * const pTimer1Params)
{
    uint8_t retVal = FAIL;

    if( !(g_sTimer1.bStatus & TMR1LOCK) )
    {   
        /* LLR-Ref: 010 */
        g_sTimer1.bStatus |= TMR1LOCK;   
        
        /* LLR-Ref: 020 */
        ATA_POWERON_C(PRR1, PRT1)
        
        /* LLR-Ref: 030 */
        T1CR &= ~BM_T1ENA;
        T1CR = BM_T1RES;

        /* LLR-Ref: 040 */
        g_sTimer1.fpCompIsr = pTimer1Params->compIsr;
        g_sTimer1.fpOvfIsr = pTimer1Params->ovfIsr;
        
        /* LLR-Ref: 050 */
        T1IFR |= ((uint8_t)(BM_T1COF|BM_T1OFF));
        
        /* LLR-Ref: 060 */
        T1COR = pTimer1Params->comp;
        T1MR = pTimer1Params->mode;
        T1IMR = pTimer1Params->irqMask;
        T1CR = pTimer1Params->ctrl;

        retVal = OK;
    }
    
    /* LLR-Ref: 070 */
    return retVal;
}


/* ---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer1Close_C</b>
    shall stop Timer 1 and enable the power reduction feature of Timer 1.

    Variable Usage:
    \li [out] ::g_sTimer1 Global Timer 1 component data

    \image html ATA_timer1Close_C.png

    \internal
             Note:
             Disabling of the Power Reduction feature of Timer 1 via register PRR1
             needs to occur before any Timer 1 register is accessed in order for the
             intended actions to take effect.
             Enabling of the Power Reduction feature of Timer 1 via register PRR1 
             needs to occur after any Timer 1 register is accessed in order for the 
             intended action to take effect.

    \li 005: Disable the power reduction feature of Timer 1 by setting bit PRT1 in 
             register PRR1 to 0.

    \li 010: Disable Timer 1 by setting all bits of register T1CR to 0,
             except bit T1RES which is set to 1 to reset the prescaler and counter
             value.

    \li 020: Unlock Timer 1 by setting the status indication of the global variable
             ::g_sTimer1 to 0.

    \li 030: Enable the power reduction feature of Timer 1 by setting bit PRT1 in 
             register PRR1 to 1.
        
    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-882}
    \endinternal            
\n
*/
/* ---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer1Close_C(void)
{
    /* LLR-Ref: 005 */
    ATA_POWERON_C(PRR1, PRT1)
  
    /* LLR-Ref: 010 */
    T1CR = BM_T1RES;
    
    /* LLR-Ref: 020 */
    g_sTimer1.bStatus = 0x00U;
    
    /* LLR-Ref: 030 */
    ATA_POWEROFF_C(PRR1, PRT1);
}
