//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer4/src/timer4_flash.c $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.3.18.0
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
//lint -restore
/** \file timer4_flash.c
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "timer4_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/


/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>tmr4CaptureValues</b>
    contains the capture events traced for Timer 4.
*/
//#pragma location = ".sram_FlashApp_Timer4"
//__root __no_init uint16_t g_bTimer4CaptureArray_flash[MAX_TIMER4_CAPTURE_VALUES];


/** \brief <b>tmr4CaptureIndex</b>
    contains the capture event counter to index the traced capture events in
    tmr3CaptureValues.
*/
//#pragma location = ".sram_FlashApp_Timer4"
//__root __no_init uint8_t g_bTimer4CaptureIndex_flash;


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_timer4ComIntHandler_ISR_flash_C</b>
    function is used to handle the Timer 4 Compare Match interrupt.
    On occurrence, the stored function is executed.

\param[in,out]  ::g_sTimer4     Global Timer 4 component data

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_timer4ComIntHandler_ISR_flash_C.png

\internal
\li 010:  Get stored address to function which has to be executed.

\li 020:  NULL pointer check.

\li 030:  Invoke routine.

\Rationale{N/A}

\Traceability   N/A
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
/* #pragma vector=T4COMP_vect */
//lint -esym(714, ATA_timer4ComIntHandler_ISR_flash_C) FlSc (10.06.2014)
//lint -esym(765, ATA_timer4ComIntHandler_ISR_flash_C) FlSc (10.06.2014)
/* disable lint rule 714 - symbol 'ATA_timer4ComIntHandler_ISR_flash_C' not referenced
 * interrupt assignment to Interrupt Vector Table is done by Flash application
 *
 * disable lint rule 765 - external symbol 'ATA_timer4ComIntHandler_ISR_flash_C' could be made static
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
#pragma diag_suppress=Ta006
__interrupt VOIDFUNC ATA_timer4ComIntHandler_ISR_flash_C(void)
{
    /* LLR-Ref: 010 */
    void (*fpFunc)(void) = g_sTimer4.fpCompIsr;

    /* LLR-Ref: 020 */
    if( fpFunc )
    {
        /* LLR-Ref: 030 */
        fpFunc();
    }
}
