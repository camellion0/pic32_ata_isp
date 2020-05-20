//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/system/src/system_flash.c $
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
/** \file system.c
*/

//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "system_flash.h"
#include "../../globals/src/globals.h"
#include "../../timer2/src/timer2.h"
#include "../../spi/src/ata5700_command_set_flash.h"
#include "../../init/src/init_flash.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_systemErrorLoop_flash_C</b>
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_systemErrorLoop_flash_C(void)
{
    ATA_globalsWdtDisable_C();
    for(;;)
    {}
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_systemLowBatt_ISR_flash_C</b>
    is used as interrupt handler for voltage monitor. If interrupt occurs the
    LOWBATT event in events_system is set and if configured the event pin is
    set as configured. To avoid multiple signalizations the interrupt is disabled.

    \return none

    \image html ATA_systemLowBatt_ISR_flash_C.png
    \image rtf ATA_systemLowBatt_ISR_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
#pragma vector=VMON_vect
__interrupt void ATA_systemLowBatt_ISR_flash_C(void)
{   /*lint !e957 GeWi (30jun11) ISR needs no prototype therefore Note 957 is disabled */
    VMCR &= (uint8_t)~BM_VMIM;
    g_sAta5700_flash.events_system |= BM_ATA5700_EVENTS_SYSTEM_LOWBATT;
    
    if (g_sEventHandling_flash.bSystem & BIT_MASK_4) {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_systemAvccLow_ISR_flash_C</b>
    interrupt handler for AVCCLOW. If interrupt occurs the AVCCLOW event in
    events_system is set and if configured the event pin is set as configured.
    To avoid multiple signalizations the interrupt is disabled.

    \return none

    \image html ATA_systemAvccLow_ISR_flash_C.png
    \image rtf ATA_systemAvccLow_ISR_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
#pragma vector=AVCCL_vect
__interrupt void ATA_systemAvccLow_ISR_flash_C(void)
{   /*lint !e957 GeWi (30jun11) ISR needs no prototype therefore Note 957 is disabled */
    SUPCR &= (uint8_t)~BM_AVCCLM;
    g_sAta5700_flash.events_system |= BM_ATA5700_EVENTS_SYSTEM_AVCCLOW;
    
    /*  */
    if (g_sEventHandling_flash.bSystem & BIT_MASK_5) {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_systemAvccReset_ISR_flash_C</b>
    interrupt handler for AVCCRESET. There is no signalization for this interrupt
    source available in the event bytes. To avoid multiple signalization the
    interrupt is disabled.

    \return none
*/
/*----------------------------------------------------------------------------- */
#pragma vector=AVCCR_vect
__interrupt void ATA_systemAvccReset_ISR_flash_C(void)
{   /*lint !e957 GeWi (30oct11) ISR needs no prototype therefore Note 957 is disabled */
    SUPCR &= (uint8_t)~BM_AVCCRM;
}

