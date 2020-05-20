/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashIrq.c $
  $LastChangedRevision: 586334 $
  $LastChangedDate: 2020-01-27 13:26:46 -0700 (Mon, 27 Jan 2020) $
  $LastChangedBy: grueter $
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
#include "../../../firmware/stdc/src/ioATA5700.h"

#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"

#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/timer5/src/timer5_flash.h"
#include "../../../firmware/timer4/src/timer4_flash.h"
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
/*  Modul Globals                                                             */
/*===========================================================================*/
//uint8_t guiButton =0;
uint8_t ButtonTimerCnt=0;
tTimer5Status gTimer5Status;
tTimer4Status gTimer4Status;
tPCINTStatus gPCINTStatus;
extern sFlashApplState gFlashApplState;
extern sFlashApplVars gFlashApplVars;
//extern sTmr5Config g_sTimer5;


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/


/*---------------------------------------------------------------------------*/
#pragma vector=PCINT1_vect
__interrupt VOIDFUNC ATA_pinChangeInterrupt1Handler_ISR_flash_C(void)
{
   gPCINTStatus |= BM_PCINT1TRUE;  
   PCICR &= ~(1<<PCIE1);//Disable Pin change interrupt
   PCIFR |= 0x02; //clear pin change interrupt bank 1 flag
}
/*----------------------------------------------------------------------------- */
#pragma vector=T5COMP_vect
__interrupt VOIDFUNC ATA_timer5CompareMatchInterruptHandler_ISR_flash_C(void)
{
  gTimer5Status |= BM_TIMER5COMPARETRUE;  
}
/*----------------------------------------------------------------------------- */
#pragma vector=T4COMP_vect
__interrupt VOIDFUNC ATA_timer4CompareMatchInterruptHandler_ISR_flash_C(void)
{
  gTimer4Status |= BM_TIMER4COMPARETRUE;
}
/*----------------------------------------------------------------------------- */
#pragma vector=T5OVF_vect
__interrupt VOIDFUNC ATA_timer5OverflowInterruptHandler_ISR_flash_C(void)
{
  gTimer5Status |= BM_TIMER5OVERFLOWTRUE;  
}
/*------------------------------------------------------------------------------*/
void ATA_RKEtimerStart()
{
   gPCINTStatus &= ~(BM_PCINT1TRUE);//Clear the flag
    gFlashApplState.State |= BM_RKETIMER_ACTIVE; 
   Intr_Disable(SW1_INTR);    //Disable SW1,2,3 interrupts 
   Intr_Disable(SW2_INTR);
   Intr_Disable(SW3_INTR);
   gFlashApplState.Buttons |=  BM_BUTTONPROCCESSINGACTIVE;  
   ButtonTimerCnt=0;
   uint16_t timer5Count = 0x0048;//72 * 1/(1 MHz/256) = 
   uint8_t timer5Config = (BM_T5CS1|BM_T5CS2|BM_T5CTC);//MRC 1 MHz clock/256
   ATA_FLashAppTimer5Start_C(timer5Count, timer5Config);  
}
/*------------------------------------------------------------------------------*/
void ATA_RKEtimer4Start()
{
   gPCINTStatus &= ~(BM_PCINT1TRUE);//Clear the flag
   gFlashApplState.State |= BM_RKETIMER4_ACTIVE; 
   Intr_Disable(SW1_INTR);    //Disable SW1,2,3 interrupts 
   Intr_Disable(SW2_INTR);
   Intr_Disable(SW3_INTR);
   gFlashApplState.Buttons |=  BM_BUTTONPROCCESSINGACTIVE;  
   ButtonTimerCnt=0;
   uint16_t timer4Count = 0x0048;//72 * 1/(1 MHz/256) = 
   uint8_t timer4Config = (BM_T4ENA|BM_T4CRM);//MRC 1 MHz clock/256
   uint8_t timer4ModeA = (BM_T4CS1|BM_T4PS1|BM_T4PS2);//MRC 1 MHz clock/256
   ATA_FLashAppTimer4Start_C(timer4Count, timer4Config, timer4ModeA);
}
/*------------------------------------------------------------------------------*/
void ATA_RKEtimerProcess()
{
  uint8_t ButtonState;  
  //_WDR; 
  gTimer5Status &= ~(BM_TIMER5COMPARETRUE);
  if (ButtonTimerCnt==0)
  {
  gFlashApplState.Buttons &= ~(BM_BUTTONFILTERON);//set filter off
  gFlashApplState.Buttons |= BM_BUTTONPROCCESSINGACTIVE;//set active true
  gFlashApplState.Buttons &= ~(BM_BUTTONDATAVALID);
  }  
  if (bit_test(LED2)) bit_clear(LED2);
  else bit_set(LED2);
  ButtonTimerCnt++;
  ButtonState = ~(PIND | SW_BM);
  ButtonState = ButtonState>>5;//New SW inputs
 if (   ((gFlashApplState.Buttons & BM_BUTTONDATAVALID)==BM_BUTTONDATAVALID)     ||  (ButtonTimerCnt>75))
  { 
    gFlashApplState.State &= ~(BM_RKETIMER_ACTIVE); 
    ATA_timer5Close_C(); 
    T5IMR = 0x00;
    PCIFR =0x03;
    if (  (gFlashApplState.Buttons & BM_NEWCMNDVALID)==0)//False trigger re-enable INTs
    { 
    Intr_Enable(SW1_INTR);    //Disable SW1,2,3 interrupts 
    Intr_Enable(SW2_INTR);
    Intr_Enable(SW3_INTR);
    }
    PORTC &= ~(1<<PORTC1);   
    ButtonTimerCnt=0;
    gFlashApplState.Buttons &= ~(BM_BUTTONPROCCESSINGACTIVE);//set active false
     gTimer5Status &= ~(BM_TIMER5COMPARETRUE);
  }
  else
  {
   uint8_t cmnd =  ATA_Flash_RKEbuttonfilter(ButtonState, ButtonTimerCnt); 
  }
}
/*------------------------------------------------------------------------------*/
void ATA_RKEtimer4Process()
{
  uint8_t ButtonState;  
  //_WDR; 
  gTimer4Status &= ~(BM_TIMER4COMPARETRUE);
  if (ButtonTimerCnt==0)
  {
    gFlashApplState.Buttons &= ~(BM_BUTTONFILTERON);//set filter off
    gFlashApplState.Buttons |= BM_BUTTONPROCCESSINGACTIVE;//set active true
    gFlashApplState.Buttons &= ~(BM_BUTTONDATAVALID);
  }   
  if (bit_test(LED2)) bit_clear(LED2);
  else bit_set(LED2);
  ButtonTimerCnt++;
  ButtonState = ~(PIND | SW_BM); 
  ButtonState = ButtonState>>5;//New SW inputs
  
  if (((gFlashApplState.Buttons & BM_BUTTONDATAVALID)==BM_BUTTONDATAVALID) || (ButtonTimerCnt>75))
  { 
    gFlashApplState.State &= ~(BM_RKETIMER_ACTIVE); 
    ATA_timer4Close_C(); 
    T4IMR = 0x00;
    PCIFR =0x03;
    if (  (gFlashApplState.Buttons & BM_NEWCMNDVALID)==0)//False trigger re-enable INTs
    { 
      Intr_Enable(SW1_INTR);    //Disable SW1,2,3 interrupts 
      Intr_Enable(SW2_INTR);
      Intr_Enable(SW3_INTR);
    }
    bit_clear(LED2);    
    ButtonTimerCnt=0;
    gFlashApplState.Buttons &= ~(BM_BUTTONPROCCESSINGACTIVE);//set active false
    gTimer4Status &= ~(BM_TIMER4COMPARETRUE);
  }
  else
  {
    uint8_t cmnd =  ATA_Flash_RKEbuttonfilter(ButtonState, ButtonTimerCnt); 
  }
}
/*------------------------------------------------------------------------------*/
void ATA_FLashAppTimer5Start_C(uint16_t Timer5CompareCnt, uint8_t Timer5Config)
{
   sTimerSyn16BitParams sTimer5Params;
   sTimer5Params.ctrl = Timer5Config;     
   sTimer5Params.compL = (Timer5CompareCnt&0xff);                     
   sTimer5Params.compH = (Timer5CompareCnt>>8); 
   sTimer5Params.countL = 0x00U;                      
   sTimer5Params.countH = 0x00U;                       
   sTimer5Params.irqMask = (BM_T5CIM);                 // T5IrqMask
   sTimer5Params.ovfIsr = (timerIRQHandler)0x0000;     // g_sTimer5.ovfIsr
   sTimer5Params.compIsr = (timerIRQHandler)0x0000;    // g_sTimer5.compIsr
   if (ATA_timer5Open_C(&sTimer5Params) == FAIL){
   ATA_systemErrorLoop_flash_C();
   }  
}
/*------------------------------------------------------------------------------*/
void ATA_FLashAppTimer4Start_C(uint16_t Timer4CompareCnt, uint8_t Timer4Config, uint8_t timer4ModeA)
{
   sTimerAsyn16BitParams sTimer4Params;
   sTimer4Params.ctrl = Timer4Config;     
   sTimer4Params.modeA = timer4ModeA;
   sTimer4Params.compL = (Timer4CompareCnt&0xff);                     
   sTimer4Params.compH = (Timer4CompareCnt>>8);                       
   sTimer4Params.irqMask = (BM_T4CIM);                 // T4IrqMask
   sTimer4Params.ovfIsr = (timerIRQHandler)0x0000;     // g_sTimer4.ovfIsr
   sTimer4Params.compIsr = (timerIRQHandler)0x0000;    // g_sTimer4.compIsr
   if (ATA_timer4Open_C(&sTimer4Params) == FAIL){
   ATA_systemErrorLoop_flash_C();
   }  
}
/*------------------------------------------------------------------------------*/
void ATA_FlashApplTimer5Process_C()
{
  if ((gFlashApplState.State & BM_RKETIMER_ACTIVE))ATA_RKEtimerProcess();
  else if ((gFlashApplState.State & BM_PEPSRFTIMERACTIVE))ATA_rfTx_PEPSrftimingprocess_flash_C();// ATA_PEPStimerProcess();  
}
void ATA_FlashApplTimer4Process_C()
{
  if ((gFlashApplState.State & BM_RKETIMER_ACTIVE))ATA_RKEtimer4Process();
  else if ((gFlashApplState.State & BM_PEPSRFTIMERACTIVE))ATA_rfTx_PEPSrftimingprocess_flash_C();// ATA_PEPStimerProcess();  
}
/*------------------------------------------------------------------------------*/
void ATA_PEPStimerStart(uint8_t mscount)
{
   Intr_Disable(SW1_INTR);    //Disable SW1,2,3 interrupts 
   Intr_Disable(SW2_INTR);
   Intr_Disable(SW3_INTR);
   uint16_t timer4Count = mscount*0x219;//72 * 1/(1 MHz/256) = 
   uint8_t timer4Config = (BM_T4ENA|BM_T4CRM);//MRC 1 MHz clock/256
   uint8_t timer4ModeA = (BM_T4CS1|BM_T4PS0);//MRC 1 MHz clock/256
   ATA_FLashAppTimer4Start_C(timer4Count, timer4Config, timer4ModeA);
}
/*------------------------------------------------------------------------------*/
void ATA_PEPStimerProcess()
{
  //_WDR; 
  ATA_timer5Close_C(); 
  T5IMR = 0x00;
  gTimer5Status &= ~(BM_TIMER5COMPARETRUE);  
  gFlashApplState.State &= ~(BM_MSGTIMER_ACTIVE); 
  Intr_Enable(SW1_INTR);    //Enable SW1,2,3 interrupts 
  Intr_Enable(SW2_INTR);
  Intr_Enable(SW3_INTR);   
}
void ATA_transTXtimerStart(uint16_t mscount)
{
   uint16_t timer4Count = mscount;//72 * 1/(1 MHz/256) = 
   uint8_t timer4Config = (BM_T4ENA|BM_T4CRM);//MRC 1 MHz clock/256
   uint8_t timer4ModeA = (BM_T4CS1|BM_T4PS0|BM_T4PS2);//MRC 1 MHz clock/256
   gTimer4Status = 0x00;
   ATA_FLashAppTimer4Start_C(timer4Count, timer4Config, timer4ModeA);
}
uint8_t ATA_transTXtimerProcess()
{
  if((gTimer4Status & BM_TIMER4COMPARETRUE) == 0x80 )
  {
    ATA_timer4Close_C(); 
   // T4IFR |= 0x02;// CLear the T4 compare flag
    T4IMR = 0x00;
    gTimer4Status = 0x00;
    return (1);
  }
  return(0); 
}
