//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer2/src/timer2.c $
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
/** \file timer2.c
*/
//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "timer2.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/


/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>g_sTimer2</b>
    contains the configuration settings for Timer2.
*/
#pragma location = ".tmr2ram"
__no_init sTmr2Config g_sTimer2;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer2Init_C</b>
    shall initialize Timer2 data structure sTmr2Config.

    Variable Usage:
    \li [out] ::g_sTimer2     Global Timer 2 component data

    \image html ATA_timer2Init_C.png

    \internal
    \li 010:   Set all ::g_sTimer2 attributes to 0.

    \Derived{Yes}

    \Rationale{Since the hardware SRAM initialization is turned off, all SRAM
               variables need initialization prior to its usage}

    \Traceability   N/A
    \endinternal
\n
  */
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer2Init_C(void)
{
    /* LLR-Ref: 010 */
    g_sTimer2.bStatus  = 0x00U;
    g_sTimer2.fpCompIsr = (timerIRQHandler)0x0000U;
    g_sTimer2.fpOvfIsr  = (timerIRQHandler)0x0000U;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer2Open_C</b>
    shall open and initialize Timer 2, in case it is not yet locked.

    \param[in]      pTimer2Params    asynchronous timer configuration settings
    \return     OK on success, FAIL on failure

    Variable Usage:
    \li [in,out] ::g_sTimer2 Global Timer 2 component data

    \image html ATA_timer2Open_C.png

    \internal
             Note:
             Power up of Timer 2 via register PRR1 needs to occur before any 
             Timer 2 register is accessed in order for the intended action to 
             take effect.

             IF Timer 2 is not locked, i.e. not already used by another component,
             THEN
    \li 010:   Lock Timer 2 by setting the lock status in global variable 
               ::g_sTimer2 in order to synchronize the usage of Timer 2.

    \li 020:   Power up Timer 2 by setting bit PRT2 in register PRR1 to 0 to 
               disable the power reduction feature for Timer 2.

    \li 030:   Disable and reset Timer 2 by writing a 1 to the bit T2RES in
               register T2CR and a 0 to bit T2ENA.

    \li 040:   Store the given function pointer parameters "ovf" and "comp" of 
               parameter "pTimer2Params" within to the global variable ::g_sTimer2
               in order for those functions to be called when the corresponding
               interrupt, overflow and compare, is triggered.

    \li 050:   Clear Timer 2 interrupt flags in T2IFR. 

    \li 060:   Set Timer 2 registers T2COR, T2MR, T2IMR and T2CR to the 
               given attributes of parameter "pTimer2Params".

             ENDIF

    \li 060: Return OK to the calling function if Timer 2 was successfully 
             configured.
             Return FAIL if Timer 2 was already locked and therefore could 
             not be configured.
        
    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-883,Primus2P-1328}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_timer2Open_C(const sTimerAsyn8BitParams * const pTimer2Params)
{
    uint8_t fRetVal = FAIL;
    
    if( !(g_sTimer2.bStatus & TMR2LOCK) )
    {
        /* LLR-Ref: 010 */
        g_sTimer2.bStatus |= TMR2LOCK;
    
        /* LLR-Ref: 020 */
        ATA_POWERON_C(PRR1, PRT2)
        
        /* LLR-Ref: 030 */
        T2CR &= ~BM_T2ENA;
        T2CR = BM_T2RES;
        
        /* LLR-Ref: 040 */
        g_sTimer2.fpCompIsr = pTimer2Params->compIsr;
        g_sTimer2.fpOvfIsr = pTimer2Params->ovfIsr;
        
        /* LLR-Ref: 050 */
        T2IFR |= ((uint8_t)(BM_T2COF|BM_T2OFF));
        
        /* LLR-Ref: 060 */
        T2COR = pTimer2Params->comp;
        T2MR = pTimer2Params->mode;
        T2IMR = pTimer2Params->irqMask;
        T2CR = pTimer2Params->ctrl;
        
        fRetVal = OK;
    }
    
    /* LLR-Ref: 070 */
    return fRetVal;
}

/* ---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer2Close_C</b>
    shall stop Timer 2 and enable the power reduction feature of Timer 2.

    Variable Usage:
    \li [out] ::g_sTimer2 Global Timer 2 component

    \image html ATA_timer2Close_C.png

    \internal
             Note:
             Disabling of the Power Reduction feature of Timer 2 via register PRR1
             needs to occur before any Timer 2 register is accessed in order for the
             intended actions to take effect.
             Enabling of the Power Reduction feature of Timer 2 via register PRR1 
             needs to occur after any Timer 2 register is accessed in order for the 
             intended action to take effect.

    \li 005: Disable the power reduction feature of Timer 2 by setting bit PRT2 in 
             register PRR1 to 0.

    \li 010: Disable Timer 2 by setting all bits of register T2CR to 0,
             except bit T2RES which is set to 1 to reset the prescaler and counter
             value.

    \li 020: Unlock Timer 2 by setting the status indication of the global variable
             ::g_sTimer2 to 0.

    \li 030: Enable the power reduction feature of Timer 2 by setting bit PRT2 in 
             register PRR1 to 1.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-884}
    \endinternal
\n
*/
/* ---------------------------------------------------------------------------*/
VOIDFUNC ATA_timer2Close_C(void)
{
    /* LLR-Ref: 005 */
    ATA_POWERON_C(PRR1, PRT2)
      
    /* LLR-Ref: 010 */
    T2CR = BM_T2RES;
    
    /* LLR-Ref: 020 */
    g_sTimer2.bStatus = 0x00U;
    
    /* LLR-Ref: 030 */
    ATA_POWEROFF_C(PRR1, PRT2);
}
