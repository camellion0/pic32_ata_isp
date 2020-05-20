//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/timer4/src/timer4_flash.h $
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

/** \file timer4_flash.h
 */

#ifndef TIMER4_FLASH_H
#define TIMER4_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../globals/src/globals_types.h"
#include "timer4.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

#define MAX_TIMER4_CAPTURE_VALUES       0x20U    /* 32 entries*/
#define BM_TIMER4COMPARETRUE    BIT_MASK_7
#define BM_TIMER4OVERFLOWTRUE   BIT_MASK_6
   
   
typedef uint8_t tTimer4Status;
 /** \brief <b>Timer5Status</b>
        contains timer4 status flags  
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
/*  Globals                                                                  */
/*---------------------------------------------------------------------------*/

extern uint8_t      g_bTimer4CaptureIndex_flash;

extern uint16_t     g_bTimer4CaptureArray_flash[MAX_TIMER4_CAPTURE_VALUES];

extern VOIDFUNC     ATA_timer4CaptureEvents_flash_ASM(void);


#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/


MAX_TIMER4_CAPTURE_VALUES   EQU 0x20



/*stopSimExtraction*/
#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* TIMER3_FLASH_H */