//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/stdc/src/stdc.h $
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
/** \file stdc.h
*/
//lint -restore

#ifndef STDC_H
#define STDC_H

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../compiler/src/compiler.h"
/*lint --e{*}*/
#include "intrinsics.h"
#include "ioATA5702M322.h"
/*lint -restore */

/* include defines for target MCU */
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/* own firmware definitions */
#define TRUE    (uint8_t)1
#define FALSE   (uint8_t)0
#define OK      (uint8_t)0
#define FAIL    (uint8_t)1

#define _SLEEP        __asm ("sleep")
#define _NOP          __asm ("nop")
#define _IJMP         __asm ("ijmp")
#define _SEI          __asm ("sei")
#define _CLI          __asm ("cli")
#define _WDR          __asm ("wdr")

#define OUTPUT_DEBUG_WARNINGS   // --> TODO: place in ewp file
// #undef  OUTPUT_DEBUG_WARNINGS
/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/

#endif /* STDC_H */
