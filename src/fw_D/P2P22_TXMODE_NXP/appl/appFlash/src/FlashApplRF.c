/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplRF.c $
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

/** \file FlashApplRF.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/rftx/src/rftx_flash.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"
#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"

#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/globals/src/globals.h"
#include "../../../firmware/timer5/src/timer5_flash.h"
#include "../../../firmware/timer4/src/timer4_flash.h"

#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"

#include "../../../firmware/extif/src/extif_flash.h"

#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"

#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"

//--------

#include "../src/FlashApplPEPS.h"
#include "../src/FlashApplRF.h"
#include "../src/FlashApplLF.h" 
#include "../src/micro.h"
#include "../src/FlashApplVars.h"
#include "rfrcc_flash.h"
#include "FlashApplVars.h"
#include "FlashApplMsg.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define TX_PAYLOADDATALENGTH    32
#define RF_FRAME_LENGTH(NB_BYTES) (1000.0/CFG_RF1_BITRATE*8.0*(NB_BYTES+0.5))
/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/

extern sRfTxServicePathConfig g_sRfTxServicePathConfig0;
extern sFlashApplVars gFlashApplVars;
extern RFMSG_FRAME_TS g_MsgRXbuffer;
extern RFMSG_FRAME_TS g_MsgTXbuffer;
extern sFlashApplState gFlashApplState;
extern sFlashApplVars gFlashApplVars;
extern tTimer5Status gTimer5Status;
extern tTimer4Status gTimer4Status;
//RF message length (with only one byte of preamble)
static CONST uint8_t caub_frame_auth_delay[8] =
{
  0,
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*1 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*2 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*3 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*4 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*5 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*6 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*7 + 0.5),
};
static CONST uint8_t caub_tx_length[16] =
{
  PEPS_SIZE_TX_MSG_ID,//Needs EOM GR - Removed
  PEPS_SIZE_TX_MSG_PARAM, //PEPS_CID_RD_PARAM  1
  PEPS_SIZE_TX_MSG_PARAM, //PEPS_CID_WR_PARAM  2
  0,
  PEPS_SIZE_TX_MSG_2WAY, //PEPS_CID_2WAY 4
  PEPS_SIZE_TX_MSG_AUTH, //PEPS_CID_UNI_AUTH 5//*GR* add 4 with coil phase  
  PEPS_SIZE_TX_MSG_AUTH, //PEPS_CID_BI_AUTH  6//*GR* add 4 with coil phase  
  PEPS_SIZE_TX_MSG_AUTH, //PEPS_CID_UNI_AUTH_SINGLE 7
  PEPS_SIZE_TX_MSG_AUTH, //PEPS_CID_UNI_AUTH_SINGLE 8
  0,
  PEPS_SIZE_TX_MSG_LF_TST, //PEPS_CID_LF_TST  10
  PEPS_SIZE_TX_MSG_LF_PARAM, //PEPS_CID_LF_PARAM 11
  0,
  0,
  PEPS_SIZE_TX_MSG_SWID, //PEPS_CID_SWID 14
  PEPS_SIZE_TX_MSG_MODE  //PEPS_CID_MODE 15
};
extern uint8_t rub_cid;
/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
void ATA_rfTx_PEPSmsg_flash_C(void);
void ATA_rfTx_PEPSframe_flash_C(void);
/*----------------------------------------------------------------------------- */
/**\brief  TODO - code here

 */
/*----------------------------------------------------------------------------- */
void ATA_rfTx_PEPSmsg_flash_C(void)
{
  /*
  Notes on P2P RF response collision avoidance and timing 11 Feb 2016 GeRu
  
  1. EEPROM transcations are time consuming, a read modify write cycle takes
  about 18 mS based on measurements of the CRC disable function. EEPROM read
  cycles I did not measure but based on the delay from LF off to RF function
  calls they are also time consuming. 
  
  2. Misc timing observations 
  LF message end to RF start - 16 mS
  Car Access Kit RF receive on interval from LF message end - 210 mS
  P2P autehnticcation RF Telegram length 1/4K period * 8 bits * 10 bytes - 19 mS
  
  3. P2P TImer 4 implementation
  Timer 4 clock source is programmed to be FRC/8. In function
  ATA_PEPStimerStart(uint8_T tempo)the argument is multiplpied by 506 and the 
  product programmed into the comparison registers. Measured results are as follows;
  Tempo     Delay time (mS)   Delay per count (mS)
  50        45.3              0.9
  10        9.1               0.9
  2         1.9               0.95
    
  4. LDL code is very abstracted so here is the #DEFINES used to calculate the
  tempo;
  
  Fob_tempo = (uint8_t)INTERFRAME + caub_frame_auth_delay[Fob_tempo]; 
  Note caub_interframe_delay os decelared as a uint8_t so all results are rounded
  to the nearest integer

  static CONST uint8_t caub_frame_auth_delay[8] =
{
  0,//0
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*1 + 0.5),//12.5
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*2 + 0.5),//24.5
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*3 + 0.5),//36.5
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*4 + 0.5),//48.5
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*5 + 0.5),//60.5
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*6 + 0.5),//72.5
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_AUTH) + INTERFRAME)*7 + 0.5),//84.5
};
#define INTERFRAME    3.0
#define RF_FRAME_LENGTH(NB_BYTES) (1000.0/CFG_RF1_BITRATE*8.0*(NB_BYTES+0.5))//NB_BYTES=10 then 8.75 (9) 
#define CFG_RF1_BITRATE       9600

#define PEPS_SIZE_TX_MSG_AUTH        (PEPS_SIZE_TX_MSG_COMMON+1+CFG_PEPS_CHALLENGE_LENGTH)//10
#define PEPS_SIZE_TX_MSG_COMMON      (1 + CFG_PEPS_PREAMBLE_LENGTH)//5
#define CFG_PEPS_PREAMBLE_LENGTH        4 
#define CFG_PEPS_CHALLENGE_LENGTH       4  

  
  
  */
  
  //Send message in common and FOB index time slots
  uint8_t Fob_tempo;//Delay timing position (slot)
  uint8_t Fob_slots;//TX slots 
  // compute interframe for anti-collicion process
  Fob_tempo = 0;
  Fob_slots = SLOT_COMMON;
 
  if ((rub_cid==PEPS_CID_UNI_AUTH) || (rub_cid==PEPS_CID_BI_AUTH))
  {
  Fob_slots = SLOT_COMMON|SLOT_SINGLES;//1 | 2
    // common slot + wait xxx ms before 2nd frame
    Fob_tempo = MSG_TX_DATA.peps.fidx - ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->fidx;//TX Data FOB Index - RX message FOB Index    
    if (Fob_tempo & 0x80)  
    {
      // Fob index lower than index requested
      Fob_tempo += PEPS_NB_FOB_MAX; //Add 4 to it?
    }
    // interframe is 3ms, and frame length is around 8ms
    // =((1000/CFG_RF1_BITRATE*8*(PEPS_SIZE_TX_MSG_AUTH+CFG_PEPS_PREAMBLE_LENGTH-1+0.5)+0.5)
    Fob_tempo = (uint8_t)INTERFRAME + caub_frame_auth_delay[Fob_tempo];    
  }
  // send first frame
  if (Fob_slots&SLOT_COMMON)
  {
     ATA_rfTx_PEPSframe_flash_C();    
  }  
  // send second frame
  if (Fob_slots&SLOT_SINGLES)
  {
    if (Fob_tempo)
    {
     ATA_PEPStimerStart(Fob_tempo);
     gFlashApplState.State |= BM_PEPSRFTIMERACTIVE;     
     }    
  
  }   
}

void ATA_rfTx_PEPSrftimingprocess_flash_C(void)
{ 
      T4IMR = 0x00; 
      //_WDR;  
      ATA_PEPStimerProcess();
      gFlashApplState.State &= ~(BM_PEPSRFTIMERACTIVE);   
      gTimer4Status &= ~(BM_TIMER4COMPARETRUE);
    // send 2nd frame
      ATA_rfTx_PEPSframe_flash_C();  
      ATA_timer4Close_C();   
}

void ATA_rfTx_PEPSframe_flash_C(void)
{
   if (bit_test(LED2)) bit_clear(LED2);
   else bit_set(LED2);
  g_MsgTXbuffer.ub_size = caub_tx_length[rub_cid];//Need EOM byte
 // for (uint8_t loopcnt=0;loopcnt<3;loopcnt++)
 // { 
  
     ATA_rfTxInit_C();   
     ATA_rfTxFillDFifo_C(g_MsgTXbuffer.ub_size, g_MsgTXbuffer.aub_data);//gFlashApplVars.RfTxbuffer);
   //  ATA_rfTxFillSFifo_C(g_MsgTXbuffer.ub_size, g_MsgTXbuffer.aub_data);//gFlashApplVars.RfTxbuffer);
   //  ATA_rfTxStartTx_C(0x48, (uint8_t *) 0x06D0);
     uint16_t eepService = ATA_rfTxGetIndirectEEPromServiceConfigAddr_flash_C((uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l);
   //  ATA_rfTxStartTx_C((BM_RFTXCONFIG_BCONFIG_SVC_LOCATION|BM_RFTXCONFIG_BCONFIG_VCO_TUNING|loopcnt), (uint8_t*)eepService);
     ATA_rfTxStartTx_C((BM_RFTXCONFIG_BCONFIG_SVC_LOCATION|BM_RFTXCONFIG_BCONFIG_VCO_TUNING|0), (uint8_t*)eepService);//Only TX on 433.02 MHz
     do {
        ATA_rfTxProcessing_C();
        }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
      ATA_rfTxStop_C();
 // }
   SUPCR &= ~BM_AVEN;
   if (bit_test(LED2)) bit_clear(LED2);
   else bit_set(LED2);   
}