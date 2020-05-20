/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplIRQ.c $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 5.51.0
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

/** \file FlashApplIRQ.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set.h"

#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"

#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/globals/src/globals.h"

#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"

#include "../../../firmware/extif/src/extif_flash.h"

#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"

#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define conSwitch1              1
#define conSwitch2              2
#define conSwitch3              4
/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/
uint8_t guiButton =0;
uint8_t gucNewButtonAvailable=0;
/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*----------------------------------------------------------------------------- */
/*brief <b>PCINTO Interrupt handler</b>*/
            
/*    The function contains the interrupt vector for the Pin Change interrupts 0-7.
    This routine determins the specefic pin that generated the interupt and sets
    variables to alert the main loop that a new button press has been detected.*/
 
/*---------------------------------------------------------------------------*/
#pragma vector=PCINT0_vect
__interrupt VOIDFUNC ATA_pinChangeInterrupt0Handler_ISR_flash_C(void)
{
   unsigned char ucdebounce=0;
   unsigned char ucSwitchStatus;
   unsigned char ucOldSwitchStatus;

   for(ucdebounce=0; ucdebounce<15;)
    {
      ucSwitchStatus = ~PINB & 0x07;             // read switches
      if(ucSwitchStatus == ucOldSwitchStatus)
      {
         ucdebounce++;
      }
      else
      {
         ucdebounce = 0;
         ucOldSwitchStatus = ucSwitchStatus;
      }
        __delay_cycles(500);
        __delay_cycles(500);
    }
   ucdebounce = 0;
   if (ucSwitchStatus !=0)   gucNewButtonAvailable = 1;
   switch (ucSwitchStatus)
   {
   case conSwitch1:
     guiButton = 1;
     break;
   case conSwitch2:
      guiButton = 2;
     break;
   case conSwitch3:
     guiButton = 4;
   break;
   }

}
/*----------------------------------------------------------------------------- */