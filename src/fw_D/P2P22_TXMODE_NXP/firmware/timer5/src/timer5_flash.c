//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Branches/VB_PRIMUS2P_ROM_2.0/firmware/timer5/src/timer5.h $
  $LastChangedRevision: 272903 $
  $LastChangedDate: 2014-07-23 13:46:18 +0200 (Mi, 23 Jul 2014) $
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
/** \file timer5.h
*/
//lint -restore

#ifndef TIMER5_H
#define TIMER5_H

#ifdef __IAR_SYSTEMS_ICC__

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../globals/src/globals_types.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

/** \brief <b>TMR5LOCK</b>
    is the set-mask for sTmr5Config.status lock bit.
*/
#define TMR5LOCK    (uint8_t)0x80

/** \brief <b>TMR5UNLOCK</b>
    is the reset-mask for sTmr5Config.status lock bit.
*/
#define TMR5UNLOCK  (uint8_t)0x7F


#define TIMER5COMPARETRUE       BIT_7
#define TIMER5OVERFLOWTRUE      BIT_6
#define BM_TIMER5COMPARETRUE    BIT_MASK_7
#define BM_TIMER5OVERFLOWTRUE   BIT_MASK_6

/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------- */
/** \brief <b>sTmr5Config</b> 
    contains the status and the function pointers for Timer5 ISRs.
*/
/*----------------------------------------------------------------------------- */
 typedef struct{
     /** \brief <b>bStatus</b>
         contains timer5 lock indicator
         \li Bit 7:    lock indicator (0:unlocked/1:locked)
         \li Bit 6..0: rfu
     */
     uint8_t bStatus;
     
     /** \brief <b>fpCompIsr</b>
        is used as function pointer which is executed at timer5 compare interrupt
        to allow flexible usage of this interrupt source in firmware and user software
      */
     timerIRQHandler fpCompIsr;
     
     /** \brief <b>fpOvfIsr</b>
        is used as function pointer which is executed at timer5 overflow interrupt 
        to allow flexible usage of this interrupt source in firmware and user software
      */
     timerIRQHandler fpOvfIsr;
 }sTmr5Config;
 typedef uint8_t tTimer5Status;
 /** \brief <b>Timer5Status</b>
        contains timer5 status flags  
        (see rfTx component)
        \li Bit 7:    TIMERCOMPARETRUE
        \li Bit 6:    TIMEROVERFLOWTRUE
        \li Bit 5:    rfu
        \li Bit 4:    rfu
        \li Bit 3:    rfu
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    rfu
    */

/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/

extern sTmr5Config g_sTimer5;

extern VOIDFUNC ATA_timer5Init_C(void);

extern UINT8FUNC ATA_timer5Open_C(const sTimerSyn16BitParams * const pTimer5Params);

extern VOIDFUNC ATA_timer5Close_C(void);

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/

/* ------------------------------------------------------------------------- */
/* sTmr5Config                                                               */
/* ------------------------------------------------------------------------- */
TMR5CONFIG_STATUS   EQU 0
TMR5CONFIG_COMPISR  EQU TMR5CONFIG_STATUS + 1
TMR5CONFIG_OVFISR   EQU TMR5CONFIG_COMPISR + 2
/*stopSimExtraction*/

#endif /* __IAR_SYSTEMS_ASM__ */
#endif /* TIMER5_H */
