/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplInOut.c $
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

/** \file FlashApplInOut.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set.h"
#include "../../../firmware/stdc/src/ioATA5700.h"
#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"

#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/timer5/src/timer5_flash.h"
#include "../../../firmware/globals/src/globals.h"

#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"

#include "../../../firmware/extif/src/extif_flash.h"

#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"

#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"
#include "FlashApplVars.h"


/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/




/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
extern sFlashApplState gFlashApplState;
extern sFlashApplVars gFlashApplVars;
extern tTimer5Status gTimer5Status;
static uint8_t FilterCount=0;//was 3
static uint8_t NoPressCount=0;
extern uint8_t ButtonTimerCnt;

/*===========================================================================*/
/*  Modul Prototypes                                                         */
/*===========================================================================*/
void SendLongCommand(void);
void SendShortCommand(void);
//void ATA_Flash_IOinit(void);
/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
//-----------------------------------------------------------------------------
/** \brief <b>ATA_Flash_RKEbuttonfilter(uint8_t buttinstate, uint8_t count)</b>
    Shall configure the 3D LF receiver into LF listen mode and activate
      the ID0 wake-up interrupt

    \param[in]  bLfBdrate       selects the LF baud rate
                bSense          selects the LF RX sensitivity
                pLf_Id          pointer to the LF wake-up ID
                bLf_IdLength    number of LF ID bits

    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
uint8_t ATA_Flash_RKEbuttonfilter(uint8_t ButtonState, uint8_t Cnt)
{
  if (ButtonTimerCnt==1)NoPressCount=1;
  else  NoPressCount++;
  
  if (ButtonState != 0)//Button Pressed?
  {    
    if (gFlashApplVars.RKEcommand==ButtonState)//if last = current
    {
        if (FilterCount<3) FilterCount++;
	if (FilterCount>=3)gFlashApplState.Buttons |= BM_BUTTONFILTERON; //If filter count >=3, Filter on
        if (ButtonTimerCnt>61)
        {
          SendLongCommand();            
	}		
    }
    else //Last != current
    {
      if (FilterCount>0) FilterCount--;//Decrement filter count
      if (FilterCount==0) gFlashApplState.Buttons &=  ~(BM_BUTTONFILTERON); //If filter count = 0, Filter off
      if ((gFlashApplState.Buttons&BM_BUTTONFILTERON)!= BM_BUTTONFILTERON)//Filter off? 
      {
        gFlashApplVars.RKEcommand=ButtonState;//Update with new command
      }
    }
  }
  else //No button press
  {
     if (FilterCount>0) FilterCount--;//Decrement filter count	 
     if (FilterCount==0 && (gFlashApplState.Buttons & BM_BUTTONFILTERON))//FIlter on and filter count 0?
     {
        SendShortCommand();                 
     }
     else if (FilterCount==0 && ~(gFlashApplState.Buttons & BM_BUTTONFILTERON))
     {
       gFlashApplState.Buttons |= BM_BUTTONDATAVALID;//set Done//Glitch exit no message
     }
  }
  return(NoPressCount);
}
void SendLongCommand(void)
{
  if (gFlashApplVars.RKEcommand==1)  gFlashApplVars.RKEcommand=0x81;
  else if (gFlashApplVars.RKEcommand==2)  gFlashApplVars.RKEcommand=0x82;
  // Two button pressed together (SW1, SW2)
  else if (gFlashApplVars.RKEcommand==3)  gFlashApplVars.RKEcommand=0xC3;
  else gFlashApplVars.RKEcommand=0x84;
  
  gFlashApplState.Buttons |= BM_NEWCMNDVALID;
  gFlashApplState.Buttons |= BM_BUTTONDATAVALID;//set Done  
  NoPressCount=0xa5;
   
}
void SendShortCommand(void)
{
  //if (gFlashApplVars.RKEcommand==4)gFlashApplVars.RKEcommand=3;
  gFlashApplState.Buttons |= BM_NEWCMNDVALID;
  gFlashApplState.Buttons |= BM_BUTTONDATAVALID;//set Done
  NoPressCount=0xa6;
   
}

/*----------------------------------------------------------------------------- */