//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/extif/src/extif_flash.c $
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
/** \file extif_flash.c
*/

//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "extif_flash.h"

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>extIf</b>
    contains the variables used pin change handling.
*/
#pragma location = ".sram_FlashApp_ExtIf"
__root __no_init sIFData        g_sExtIf;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_initExtIf_flash_C</b>
    does the initialization of extif software module.

    \param fpPortBisr function pointer for PC int on portB
    \param pcIntHandler function pointer for PC int on portD

    \return none

    \image html ATA_initExtIf_flash_C.png
    \image rtf ATA_initExtIf_flash_C.png
    \n
*/
/*----------------------------------------------------------------------------- */
VOIDFUNC ATA_initExtIf_flash_C(pcIntHandler fpPortBisr, pcIntHandler fpPortDisr)
{
    /**/
    g_sExtIf.pcInt0old = PINB;
    g_sExtIf.pcInt1old = PIND;
    g_sExtIf.pcInt0    = fpPortBisr;
    g_sExtIf.pcInt1    = fpPortDisr;
}
