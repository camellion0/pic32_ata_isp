/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/GR_inwork/appl/appFlash/src/FlashApplRKE.c $
  $LastChangedRevision: 333314 $
  $LastChangedDate: 2015-08-14 16:42:10 -0600 (Fri, 14 Aug 2015) $
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

/** \file FlashApplPEPSRF.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/lfrx/src/lfrx.h"
//#include "../../../firmware/rfrcc/src/rfrcc.h"
#include "../../../firmware/spi/src/ata5700_command_set.h"

#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/globals/src/globals.h"

#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"

#include "../../../firmware/extif/src/extif_flash.h"

#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"

#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"

#include "rfrcc_flash.h"
#include "FlashApplVars.h"
#include "FlashApplPEPSRF.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/


#define RF_FRAME_LENGTH(NB_BYTES) (1000.0/CFG_RF1_BITRATE*8.0*(NB_BYTES+0.5))
/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/


// extern eAesSecretKeySelection bAesSecretKeySelection = AES_USE_SECRET_KEY_A;
//uint8_t_t bUserCmd=0xa5;
extern  uint16_t g_EepRFRcc_flash;
extern uint8_t guiButton;
extern sFlashApplState gFlashApplState;
extern sFlashApplVars gFlashApplVars;
extern uint16_t wEepRfrccAddress;
extern sCustomerEEPromSection g_sCustomerEEPromSection;

//extern RFMSG_FRAME_TS msg_rx_buffer;
//extern RFMSG_FRAME_TS msg_tx_buffer;
/*   
static uint8_t caub_tx_length[16] =
{
  PEPS_SIZE_TX_MSG_ID,
  PEPS_SIZE_TX_MSG_PARAM, //PEPS_CID_RD_PARAM  1
  PEPS_SIZE_TX_MSG_PARAM, //PEPS_CID_WR_PARAM  2
  0,
  PEPS_SIZE_TX_MSG_2WAY, //PEPS_CID_2WAY 4
  PEPS_SIZE_TX_MSG_AUTH, //PEPS_CID_UNI_AUTH 5
  PEPS_SIZE_TX_MSG_AUTH, //PEPS_CID_BI_AUTH  6
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

//LF message length
static uint8_t caub_lf_length[16] =
{
  PEPS_SIZE_RX_MSG_ID,
  PEPS_SIZE_RX_MSG_PARAM_RD, //PEPS_CID_RD_PARAM  1
  PEPS_SIZE_RX_MSG_PARAM_WR, //PEPS_CID_WR_PARAM  2
  0,
#ifdef CFG_APP_2WAYRF  
  PEPS_SIZE_RX_MSG_2WAY,     //PEPS_CID_2WAY     4
#else
  0,
#endif
  PEPS_SIZE_RX_MSG_AUTH_UNI, //PEPS_CID_UNI_AUTH 5
  PEPS_SIZE_RX_MSG_AUTH_BI,  //PEPS_CID_BI_AUTH  6
  PEPS_SIZE_RX_MSG_AUTH_UNI, //PEPS_CID_UNI_AUTH_SING 7
  PEPS_SIZE_RX_MSG_AUTH_BI,  //PEPS_CID_BI_AUTH_SING  8
  0,
  PEPS_SIZE_RX_MSG_LF_TST,   //PEPS_CID_LF_TST   10
  PEPS_SIZE_RX_MSG_LF_PARAM, //PEPS_CID_SET_REF  11
  0,
  0,
  PEPS_SIZE_RX_MSG_SWID, //PEPS_CID_SWID 14
  PEPS_SIZE_RX_MSG_MODE  //PEPS_CID_MODE 15
};

//Command with LF CW ?
static uint8_t cabb_cmd_with_cw[16] =
{
  FALSE,
  FALSE, //PEPS_CID_RD_PARAM  1
  FALSE, //PEPS_CID_WR_PARAM  2
  FALSE,
  TRUE,  //PEPS_CID_2WAY     4
  TRUE,  //PEPS_CID_UNI_AUTH 5
  TRUE,  //PEPS_CID_BI_AUTH  6
  TRUE,  //PEPS_CID_UNI_AUTH_SING 7
  TRUE,  //PEPS_CID_BI_AUTH_SING  8
  FALSE,
  TRUE,  //PEPS_CID_LF_TST   10
  TRUE, //PEPS_CID_SET_REF  11
  FALSE,
  FALSE,
  FALSE, //PEPS_CID_SWID 14
  FALSE  //PEPS_CID_MODE 15
};

// Access codes for diagnostic (OEM, AS)

static uint8_t caub_diag_code[2][4] =
{
  {
    (CFG_APP_OEM_CODE>>24)&0xFF,
    (CFG_APP_OEM_CODE>>16)&0xFF,
    (CFG_APP_OEM_CODE>>8)&0xFF,
    (CFG_APP_OEM_CODE&0xFF)
  },
  {
    (CFG_APP_AS_CODE>>24)&0xFF,
    (CFG_APP_AS_CODE>>16)&0xFF,
    (CFG_APP_AS_CODE>>8)&0xFF,
    (CFG_APP_AS_CODE&0xFF)
  }
};

static uint8_t caub_frame_auth_delay[8] =
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
static uint8_t caub_frame_2way_delay[8] =
{
  0,
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*1 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*2 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*3 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*4 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*5 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*6 + 0.5),
  (uint8_t)((RF_FRAME_LENGTH(PEPS_SIZE_TX_MSG_2WAY) + INTERFRAME)*7 + 0.5),
};
*/
// command ID received
//static uint8_t rub_cid;

// information on source message (channel LF/RF and wake-up id)
//static uint8_t rub_wuip;

// Fob Index
//static uint8_t rub_fob_idx;

// Fob ID
//static uint32_t rul_fob_id;

// RF channel to use
//static uint8_t rub_rf_chan;

 //RFMSG_FRAME_TS msg_rx_buffer;
// RFMSG_FRAME_TS msg_tx_buffer;

//==============================================================================
// Local functions Prototypes
//==============================================================================
// PEPS main task
//static void _app_peps_task(void);

// Check LF message received
//static uint8_t _peps_cmd_validity(void);

// Build PEPS RF message
//static void _peps_build_msg(void);

// Send PEPS RF message
//static void _peps_send_msg(void);

// Send PEPS RF message
//static void _peps_send_frame(void);



/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
//** \brief <b>ATA_Flash_RKEsend(void)</b>

// PEPS main task
//static void _app_peps_task(void);



/**
 * \brief PEPS Task on frame reception
 *        Start LF RSSI acquisiotns if needed
 *        Check WUID is coming from RF link
 *        Call PEPS handler
 *
 * \return none
 */
/*
void app_peps_handler(uint8_t lub_channel)
{
  uint8_t laub_data[4];
  
  // accept only frames with more than 5 bytes (WUID + CID + CKS)
  //if (MSG_RX_CNT >= 5)//This define is confusing
   if (msg_rx_buffer.ub_size >= 5) 
  {
    // get CID
    //rub_cid = ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->cid;// Another confusing defince
    rub_cid = ((RX_MSG_PEPS_TS*) msg_rx_buffer.aub_data)->cid;
    rub_wuip = lub_channel;
    
    if (rub_wuip&RX_CHAN_LF_MSK)
    { 
      // message received by LF
      rub_wuip |= (rub_wuip<<3);    // set WUID source
      // RSSI acquisitions ?
      if (cabb_cmd_with_cw[rub_cid])
      {
        // acquire internal RSSI (CW OFF)
        lf_measureRSSI(LF_MEAS_INT);
 #ifdef RSSI_COMPENSATION
        // wait before internal field measurement
        TIMEB_WAIT_US((16-CFG_LF_RSSI_ACQ)*100);
        // acquire external RSSI (CW ON)
        lf_measureRSSI(LF_MEAS_EXT);
 #endif
      }
      // force emission on channel 1 when command is received by LF
      rub_rf_chan = 1;
    }
    else
    {
      // message received by RF
      // Note: ATA5831 returns 1 additionnal byte due to additionnal bit in EOF
      msg_rx_buffer.ub_size -= 1;
      
      // check if wake-up ID match WUID0
#ifdef CFG_LF_WUP0_IN_EEPROM    
      EEPDATA_READ(laub_data, CFG_LF_WUP0);
#else
      laub_data[3] = (CFG_LF_WUP0>>((CFG_LF_WUP0_LENGTH-32)&0x1F)) & 0xFF;
      laub_data[2] = (CFG_LF_WUP0>>((CFG_LF_WUP0_LENGTH-24)&0x1F)) & 0xFF;
      laub_data[1] = (CFG_LF_WUP0>>((CFG_LF_WUP0_LENGTH-16)&0x1F)) & 0xFF;
      laub_data[0] = (CFG_LF_WUP0>>((CFG_LF_WUP0_LENGTH-8)&0x1F)) & 0xFF;
#endif
      if (memory_compare(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->vid,
                          laub_data, CFG_PEPS_VID_LENGTH))
      {
        rub_wuip = RX_WUID0;
      }
      
      // check if wake-up ID match WUID1
#ifdef CFG_LF_WUP1_IN_EEPROM    
      EEPDATA_READ(laub_data, CFG_LF_WUP1);
#else
      laub_data[3] = (CFG_LF_WUP1>>((CFG_LF_WUP0_LENGTH-32)&0x1F)) & 0xFF;
      laub_data[2] = (CFG_LF_WUP1>>((CFG_LF_WUP0_LENGTH-24)&0x1F)) & 0xFF;
      laub_data[1] = (CFG_LF_WUP1>>((CFG_LF_WUP0_LENGTH-16)&0x1F)) & 0xFF;
      laub_data[0] = (CFG_LF_WUP1>>((CFG_LF_WUP0_LENGTH-8)&0x1F)) & 0xFF;
#endif
      if (memory_compare(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->vid,
                         laub_data, CFG_PEPS_VID_LENGTH))
      {
        rub_wuip = RX_WUID1;
      }
    }
    LF_ENABLE(FALSE);
    
    if (rub_wuip&RX_WUID_MSK)
    {
      _app_peps_task();
    }
  }
}
*/
/**
 * \brief PEPS common Task
 *        Analyse and execute frame receive
 *        Prepare RF message reply
 *        Send RF message reply
 *        Acquire battery status
 *        Light ON LED for 50ms
 *
 * \return none
 */
/*
static void _app_peps_task(void)
{
  // execute command
  if (_peps_cmd_validity())
  {
    // light on LED
    SET_LED_STATUS(TRUE);
    
#ifdef CFG_APP_2WAYRF
    // wake-up RF
    rf_ata5831_setmode(E_STATE_IDLE, (RF_ATA5831_CONFIG_TU){0});
#else // ONE WAY
    // start XTO quartz now so that it is stabilized when we need to transmit
//    rf_ata5791_setmode(RF_MODE_IDLE);//
#endif

    // Build RF message
    _peps_build_msg();

    //activate voltage monitor
//    PRR0 &= ~(1<<PRVM); // remove power reduction on voltage monitor (2.0V)
 //   VMCR = (VMCR & ((1<<BODLS) | (1<<BODPD))) | (1<<VMPS) | (0<<VMIM) 
//           | (0<<VMLS3) | (0<<VMLS2) | (0<<VMLS1) | (1<<VMLS1);
//    VMSR = (1<<VMF); // clear VMF flag

    // Send RF message
    _peps_send_msg();

    // filter and update battery flag
  //  raub_filt_input[CFG_FILT_BATTERY] = NUB_BAT;
 //   gpio_input_filtering(CFG_FILT_BATTERY, (VMSR & (1<<VMF)));
 //   NUB_BAT = raub_filt_input[CFG_FILT_BATTERY];

    // stop voltage monitor
  //  VMCR &= (1<<BODLS) | (1<<BODPD);
    PRR0 |= (1<<PRVM);

    // shut down LED if emission length < 50ms (1000 is here to get result in ms)
#if ((1000/CFG_RF1_BITRATE*8*PEPS_SIZE_MSG_AUTH) < 50)
    // do not wait if another command is expected soon
    if (rub_cid != PEPS_CID_2WAY)
    {
       // LED must be light on at least 50ms
//      timeb_timer_start_ms(CFG_TIMER_APP, 
              //    (uint8_t)(50 - 1000.0/CFG_RF1_BITRATE*8*PEPS_SIZE_TX_MSG_AUTH));
//      timeb_timer_wait_end(CFG_TIMER_APP);
    }
#endif
//    SET_LED_STATUS(OFF);
  }
}
*/


