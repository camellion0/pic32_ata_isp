//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer4/src/timer4.h $
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
/** \file timer4.h
 */
//lint -restore

#ifndef TIMER4_H
#define TIMER4_H

#ifdef __IAR_SYSTEMS_ICC__
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../globals/src/globals_types.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

/** \brief <b>TMR4LOCK</b>
    is the set-mask for sTmr4Config.status lock bit.
*/
#define TMR4LOCK    (uint8_t)0x80

/** \brief <b>TMR4UNLOCK</b>
    is the reset-mask for sTmr4Config.status lock bit.
*/
#define TMR4UNLOCK  (uint8_t)0x7F

/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------- */
/** \brief <b>sTmr4Config</b> 
    contains the status and the function pointers for Timer4 ISRs.
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>bStatus</b>
        contains timer 4 lock indicator
        \li Bit 7:    lock indicator (0:unlocked/1:locked)
        \li Bit 6..0: rfu
    */
    uint8_t bStatus;

    /** \brief <b>fpCompIsr</b>
        is used as function pointer which is executed at timer4 compare interrupt 
        to allow flexible usage of this interrupt source in firmware and user software
    */
    timerIRQHandler fpCompIsr;

    /** \brief <b>fpOvfIsr</b>
        is used as function pointer which is executed at timer4 overflow interrupt 
        to allow flexible usage of this interrupt source in firmware and user software
    */
    timerIRQHandler fpOvfIsr;

    /** \brief <b>fpCapIsr</b>
        is used as function pointer which is executed at timer4 capture interrupt 
        to allow flexible usage of this interrupt source in firmware and user software
    */
    timerIRQHandler fpCapIsr;
}sTmr4Config;

/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/

extern sTmr4Config g_sTimer4;

extern VOIDFUNC ATA_timer4Init_C(void);

extern UINT8FUNC ATA_timer4Open_C(const sTimerAsyn16BitParams * const pTimer4Params);

extern VOIDFUNC ATA_timer4Close_C(void);

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/

/* ------------------------------------------------------------------------- */
/* sTmr4Config                                                               */
/* ------------------------------------------------------------------------- */
TMR4CONFIG_STATUS   EQU 0
TMR4CONFIG_COMPISR  EQU TMR4CONFIG_STATUS + 1
TMR4CONFIG_OVFISR   EQU TMR4CONFIG_COMPISR + 2
TMR4CONFIG_CAPISR   EQU TMR4CONFIG_OVFISR + 2
/*stopSimExtraction*/

#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* TIMER4_H */
