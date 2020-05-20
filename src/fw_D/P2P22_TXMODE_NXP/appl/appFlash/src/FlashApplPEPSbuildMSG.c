/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/ATAK51003-V3_P2P_Demo/branch/P2P_Demo01/appl/appFlash/src/FlashAppl.c $
  $LastChangedRevision: 241023 $
  $LastChangedDate: 2014-01-28 16:15:50 -0700 (Tue, 28 Jan 2014) $
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

/** \file FlashApplPEPSbuildMSG.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set_flash.h"

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
#include "../../../firmware/eep/src/eep.h"

#include "../src/FlashApplPEPS.h"
#include "../src/FlashApplLF.h" 
#include "../src/micro.h"

#include "../src/FlashApplVars.h"

#include "FlashApplMsg.h"
#include <stdbool.h>//OK




/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define CRC_INIT_00   0x00
#define CRC_INIT_FF   0xFF

#define CRC_POLY_CCIT 0x07


/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/

extern uint8_t gLfMessageReceived; 
extern uint8_t gLfRxData[CFG_LF_BUFFER_SIZE];           // max LF buffer size = 32 bytes
extern uint8_t gLfNmbrRxByts;
extern uint16_t gExtLfRssi[3];
extern uint16_t gIntLfRssi[3];
extern uint16_t gLfRssiRes[3]; 
extern uint16_t wLfRssiref[3];
extern uint16_t wLfRssiNorm[3];
extern uint16_t wBref;

// command ID received
extern uint8_t rub_cid;
// information on source message (channel LF/RF and wake-up id)
extern uint8_t rub_wuip;
// Fob Index
extern uint8_t rub_fob_idx;
// Fob ID
extern uint8_t rul_fob_id;
// RF channel to use
extern uint8_t rub_rf_chan;
extern RFMSG_FRAME_TS msg_rx_buffer;
extern RFMSG_FRAME_TS msg_tx_buffer;
void ATA_rfTx_PEPSbuildmsg_flash_C(void)
{
  // UINT32* lpul_id;  
  // common message data
  for (uint8_t index = 0; index < (RKE_RFMESSAGEPREAMPLELENGTH-1); index++)
    {
       MSG_TX_DATA.rke.preamble0[index]=0xff;
    }  
  MSG_TX_DATA.rke.preamble1 = 0xfe;//Start bit in preamble byte 16
  MSG_TX_DATA.peps.fidx = rub_fob_idx&0x07;
  MSG_TX_DATA.peps.bat = 0x01;//update battery flag
  MSG_TX_DATA.peps.cid = rub_cid;

  // Process message per LF received ID
  switch (rub_cid)
  {
  case PEPS_CID_RD_ID:
      MSG_TX_DATA.peps.data_id.fid = rul_fob_id;
      MSG_TX_DATA.peps.data_id.vid[0] = 0x5a; ////update Vehicle ID 
      MSG_TX_DATA.peps.data_id.cks = 0x44;
      break;
  case PEPS_CID_SWID:
      MSG_TX_DATA.peps.data_swid.swid[0]=0xa5;  ///6 bytes - where is this????
      MSG_TX_DATA.peps.data_swid.cks = 0x11;     
      break;
  case PEPS_CID_UNI_AUTH_SINGLE:
      MSG_TX_DATA.peps.data_authent.mac[0]=0xbb;    
      break;
  case PEPS_CID_UNI_AUTH:
     MSG_TX_DATA.peps.data_authent.rssi=0xaa;
     MSG_TX_DATA.peps.data_authent.mac[0]=0xbb;
     break;
  case PEPS_CID_BI_AUTH:
     MSG_TX_DATA.peps.data_authent.rssi=0xaa;
     MSG_TX_DATA.peps.data_authent.mac[0]=0xbb;
     break;
  case PEPS_CID_BI_AUTH_SINGLE:
     MSG_TX_DATA.peps.data_authent.mac[0]=0xbb;    
     break;
  case PEPS_CID_WR_PARAM:     
     break;
  case PEPS_CID_LF_TST:
     break;
  case PEPS_CID_LF_PARAM:
     break;     
  case PEPS_CID_MODE:
     break;    
  default:
    break;
  }
}
     

 
/*
    case PEPS_CID_UNI_AUTH:
    case PEPS_CID_BI_AUTH:
    case PEPS_CID_UNI_AUTH_SINGLE:
    case PEPS_CID_BI_AUTH_SINGLE:
    TIMEB_CLK_SPEED(CLK_SPEED_FULL);
    MSG_TX_DATA.peps.data_authent.rssi = app_rssi_compute();
      // compute MAC
      // load AES key
      EEPDATA_READ(raub_aes_keyin_cyphout, eaub_aes_key_fob);
      // load message and cypher it
      memory_copy(MSG_TX_DATA.peps.data_authent.mac, 
                  ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_auth_uni.challenge,
                  CFG_PEPS_CHALLENGE_LENGTH);
      lpul_id = (UINT32*)(MSG_TX_DATA.peps.data_authent.mac
                          +CFG_PEPS_CHALLENGE_LENGTH);
      *lpul_id = rul_fob_id;
      //compute AES
      aes_calc((UINT8*)&MSG_TX_DATA.peps.data_authent-1,
               1+1+CFG_PEPS_CHALLENGE_LENGTH+4,
               TRUE,
               TRUE);
      memory_copy(MSG_TX_DATA.peps.data_authent.mac,
                  raub_aes_keyin_cyphout,
                  CFG_PEPS_CHALLENGE_LENGTH);
      TIMEB_CLK_SPEED(CLK_SPEED_NORM);
      break;

    case PEPS_CID_WR_PARAM:
      // store parameter in EEprom
      EEPDATA_WRITE(eaub_param(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_index),
                    ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_data);
      // no break in switch case to send reply
    case PEPS_CID_RD_PARAM:
      // return VID
      // read 4 bytes in every case, but last byte may be overwritte by cks
      EEPDATA_READ((UINT8*)&MSG_TX_DATA.peps.data_param.vid, eaub_vid);
      // read parameter in EEprom
      EEPDATA_READ(&MSG_TX_DATA.peps.data_param.param_data[0],
                   eaub_param(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_index));
      // complete message (index + checksum)
      MSG_TX_DATA.peps.data_param.param_index =
        ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_index;
      break;

    case PEPS_CID_2WAY:
      MSG_TX_DATA.peps.data_2way.rssi = app_rssi_compute();
      // save rF channel for reply
      rub_rf_chan = (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->fidx)>>1;
      // enable RF reception for 1s
      timeb_timer_start_s(CFG_TIMER_RFRX, 1);
      // return msg already completed
      break;

    case PEPS_CID_LF_TST:
      MSG_TX_DATA.peps.data_lf_tst.rss = app_rssi_compute();
      MSG_TX_DATA.peps.data_lf_tst.norm_x = raub_rssi_norm_factor[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.norm_y = raub_rssi_norm_factor[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.norm_z = raub_rssi_norm_factor[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_tst.ref_x = raub_rssi_int_ref[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.ref_y = raub_rssi_int_ref[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.ref_z = raub_rssi_int_ref[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_tst.ext_x = raub_lf_rssi[LF_MEAS_EXT][LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.ext_y = raub_lf_rssi[LF_MEAS_EXT][LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.ext_z = raub_lf_rssi[LF_MEAS_EXT][LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_tst.int_x = raub_lf_rssi[LF_MEAS_INT][LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.int_y = raub_lf_rssi[LF_MEAS_INT][LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.int_z = raub_lf_rssi[LF_MEAS_INT][LF_AXIS_Z];
      break;

    case PEPS_CID_LF_PARAM:
      // update normalization factors
      if ((((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_x != 0xFF) ||
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_y != 0xFF) ||
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_z != 0xFF))
      {
        app_rssi_set_norm(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_x,
                          ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_y,
                          ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_z);
      }

      // update compensation factors
      if ((((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_x == 0xFF) &&
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_y == 0xFF) &&
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_z == 0xFF))
      {
        // do not change compensation
      }
      else if ((((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_x == 0xFE) &&
               (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_y == 0xFE) &&
               (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_z == 0xFE))
      {
        // auto acquire new internal reference
        app_rssi_set_ref(TRUE);
      }
      else
      {
        // set new internal reference = data received
        raub_lf_rssi[LF_MEAS_INT][LF_AXIS_X] =
          ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_x;
        raub_lf_rssi[LF_MEAS_INT][LF_AXIS_Y] =
          ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_y;
        raub_lf_rssi[LF_MEAS_INT][LF_AXIS_Z] =
          ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_z;
        app_rssi_set_ref(FALSE);
      }

      app_rssi_load_factors();
      // fill RF message
      MSG_TX_DATA.peps.data_lf_param.norm_x = raub_rssi_norm_factor[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_param.norm_y = raub_rssi_norm_factor[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_param.norm_z = raub_rssi_norm_factor[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_param.ref_x = raub_rssi_int_ref[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_param.ref_y = raub_rssi_int_ref[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_param.ref_z = raub_rssi_int_ref[LF_AXIS_Z];
      break;

    case PEPS_CID_SWID:
      // send Software ID
      memory_copy_const(MSG_TX_DATA.peps.data_swid.swid,
                        (const UINT8*)cts_appli_data.aub_app_id,
                        6);
      break;

    case PEPS_CID_MODE:
      // check mode and code
      if (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode > 2)
      {
        // unknown mode, back to diag OFF
        NTE_DIAG_MODE = DIAG_OFF;
      }
      else if (memory_compare_const(
                 (UINT8*)&((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.code,
                 (const UINT8*)caub_diag_code[((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode-1],
                 4))
      {
        NTE_DIAG_MODE = (DIAG_MODE_TE)((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode;
        // activate long timeout @8s to exit diag
        timeb_timer_start_s(CFG_TIMER_MODE, 8);
      }
      else
      {
        //wrong code
        NTE_DIAG_MODE = DIAG_OFF;
      }
      MSG_TX_DATA.peps.data_mode.fid = ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.fid;
      MSG_TX_DATA.peps.data_mode.mode = (UINT8)NTE_DIAG_MODE;
      break;
      
    default:
      break;
  }

  // compute frame checksum
  if ((rub_cid < PEPS_CID_UNI_AUTH) || (rub_cid > PEPS_CID_BI_AUTH_SINGLE))
  {
    *((UINT8*)&MSG_TX_DATA.peps + caub_tx_length[rub_cid]-1) =
      crc_compute((UINT8*)&MSG_TX_DATA.peps + CFG_PEPS_PREAMBLE_LENGTH,
                  caub_tx_length[rub_cid]-(CFG_PEPS_PREAMBLE_LENGTH+1),
                  CRC_INIT_00,
                  CRC_POLY_CCIT);
  }
  
  // rearm RFRX tieout if needed
  if (!IS_TIMER_ENDED(CFG_TIMER_RFRX))
  {
    timeb_timer_start_s(CFG_TIMER_RFRX, 1);
  }
  */
