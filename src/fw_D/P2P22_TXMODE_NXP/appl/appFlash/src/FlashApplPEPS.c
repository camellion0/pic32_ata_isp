/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplPEPS.c $
  $LastChangedRevision: 571136 $
  $LastChangedDate: 2019-08-14 14:11:27 -0600 (Wed, 14 Aug 2019) $
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

/** \file FlashApplPEPS.c
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
#include "FlashApplMSG.h"
#include <stdbool.h>
#include <pgmspace.h>
#include <math.h>

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

#define MSG_RX_DATA (g_MsgRXbuffer.aub_data)
/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/

uint8_t CalcCoilPhaseSig(void);

uint8_t gLfMessageReceived; 
uint8_t gLfRxData[CFG_LF_BUFFER_SIZE];           // max LF buffer size = 32 bytes
uint8_t gLfNmbrRxByts;
uint16_t gExtLfRssi[3];
uint16_t gIntLfRssi[3];
uint16_t gLfRssiRes[3];
uint8_t gLfqcTrim[3] = {LFRX_R_Trim90k, LFRX_R_Trim90k, LFRX_R_Trim117k};
uint16_t wLfRssiref[3];
uint16_t wLfRssiNorm[3];
uint16_t wBref;
uint8_t gRSSI_ResBuffer[6];
uint8_t g3dVector[2];
uint8_t g3dVectorLin[2];
uint8_t g3dVectorFlag;
uint8_t gAES_DataBuffer[16];
uint8_t gVbat_Status; 
uint8_t fubar[6] = {0,0,0,0,0,0};  
sFlashApplignSignDetect gFlashApplignSignDetect;

/* Software identifiers*/
#ifdef CFG_APP_2WAYRF
uint8_t SW_ID[]={0x57,0x02,0x02/*Device*/,0x02/*Kitphase*/,0x02/*RFdirection*/,0x00/*Rev Major/Minor*/};         
#else // ONE_WAY
uint8_t SW_ID[]={0x57,0x02,0x22/*Device*/,0x03/*Kitphase*/,0x01/*RFdirection*/,0x05/*Rev Major/Minor*/};   
#endif

extern uint8_t g_EepFlashApp_PARAMdata[32][16];

sRfTxServicePathConfig g_sRfTxServicePathConfig0;
extern sFlashApplState gFlashApplState;



#pragma location = ".sram_FlashApp_MsgTXbuffer"
__no_init RFMSG_FRAME_TS g_MsgTXbuffer;

#pragma location = ".sram_FlashApp_MsgRXbuffer"
__no_init RFMSG_FRAME_TS g_MsgRXbuffer;

// command ID received
uint8_t rub_cid;

// information on source message (channel LF/RF and wake-up id)
uint8_t rub_wuip;

// Fob Index
uint8_t rub_fob_idx;

// Fob ID
uint32_t rul_fob_id; //KarM_CH_20160115_change to 32bits FOB ID;

// RF channel to use
uint8_t rub_rf_chan;

//uint8_t gBattStatus; 
uint8_t gNTE_DIAG_MODE;

VOIDFUNC ATA_lfRxEnableWakeup_flash_C(uint8_t bLfBdrate,uint8_t bSense);
VOIDFUNC ATA_CheckLfData_flash_C(void);
VOIDFUNC Init_LfRssi_flash_C(void);
VOIDFUNC ATA_PerformLfRSSI_flash_C(uint8_t bmode, uint8_t bsign);
extern VOIDFUNC ATA_StartRssi_flash_C(uint8_t bmode);
VOIDFUNC ATA_lFRssiGetResult_flash_C(uint8_t bmode, uint8_t bsign);
static VOIDFUNC ATA_GetIntRssiValues(void);
VOIDFUNC app_rssi_set_ref(bool IntMeasure);
VOIDFUNC app_peps_handler(uint8_t lub_channel);
static VOIDFUNC _app_peps_task(void);

bool _peps_cmd_validity(void);
// Build PEPS RF message
static void _peps_build_msg(void);

extern void memory_copy(uint8_t*, uint8_t*, uint8_t);
extern bool memory_compare(uint8_t*,uint8_t*,uint8_t);
extern void memory_set(uint8_t* lpub_dst,
                       uint8_t lub_value, 
                       uint8_t lub_length);
extern void memory_copy_const(uint8_t* lpub_dst,
                              const uint8_t* lpub_src,
                              uint8_t lub_length);
extern bool memory_compare_const(uint8_t* lpub_src1,
                                    const uint8_t* lpub_src2,                                    
                                    uint8_t lub_length);


extern VOIDFUNC app_rssi_load_factors(void);
extern VOIDFUNC ATA_StartRssi_flash_C(uint8_t bmode);
extern VOIDFUNC CalcLinVector(void);

extern sEepFlashApp_RKEPEPS g_sEepFlashApp_RKEPEPS;


//Command with LF CW ?

uint8_t cabb_cmd_with_cw[16] =
{
  FALSE, //PEPS_CID_Read_UID
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
static CONST uint8_t caub_diag_code[2][4] =
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

/* Software identifiers*/
//#ifdef CFG_APP_2WAYRF
//uint8_t SW_ID[]={0x57,0x02,0x02/*Device*/,0x02/*Kitphase*/,0x02/*RFdirection*/,0x00/*Rev Major/Minor*/};         
//#else // ONE_WAY
//uint8_t SW_ID[]={0x57,0x02,0x02/*Device*/,0x02/*Kitphase*/,0x01/*RFdirection*/,0x01/*Rev Major/Minor*/};   
//#endif


NO_INIT_DATA CRAM_TS rts_cram;


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxEnableWakeup_flash_C</b>
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
void ATA_lfRxEnableWakeup_flash_C(uint8_t bLfBdrate,uint8_t bSense)
{
  uint8_t laub_data[4];
  
  LDFCKSW |= (1<<LDFSCSW); 
  while ((LDFCKSW & (1<<LDFSCKS)) ==0);            // wait until clock source is switched
 

  LFQC1 = gLfqcTrim[0];         //LFRX_R_Trim90k;
  LFQC2 = gLfqcTrim[1];         //LFRX_R_Trim90k;
  LFQC3 = gLfqcTrim[2];         //LFRX_R_Trim117k;
  LFCR0 = 0x80 | bLfBdrate | BM_LFCE1 | BM_LFCE2 | BM_LFCE3;    // activate all channels and set baudrate
  
  
  LFCR1 = BM_LFRE | BM_LFPEEN;                              // enable RX, ID and Data Mode
  LFCR3 |= ( (1<<LFSBEN) | (1<<LFTS2) | (1<<LFTS0) );//LF polling enable 3072 uS standby time
  LTEMR = BM_ID0EM | ID1EM;
  //LTEMR = BM_FLEM;
  
  LFCR2 = bSense;                                        // select sensitivity
  //LFCR3 = ;                                            // at first without trimming function
  LDFFL =0x80;
  
  
  
  
  #ifdef CFG_LF_WUP0_IN_EEPROM
  // Wake-up ID stored in EEprom as a buffer (MSB first)
  ATA_eepReadBytes_C(laub_data, CFG_LF_WUP0_Adr, 0x04 );
 
  
  PHID00 = laub_data[(CFG_LF_WUP0_LENGTH/8-1)%4];//2
  PHID01 = laub_data[(CFG_LF_WUP0_LENGTH/8-2)%4];//1
  PHID02 = laub_data[(CFG_LF_WUP0_LENGTH/8-3)%4];//0
  PHID03 = laub_data[(CFG_LF_WUP0_LENGTH/8)%4];//3
  
#else
  // Wake-up ID stored in RAM or Flash
  PHID00 = (CFG_LF_WUP0) & 0xFF;
  PHID01 = (CFG_LF_WUP0>> 8) & 0xFF;
  PHID02 = (CFG_LF_WUP0>>16) & 0xFF;
  PHID03 = (CFG_LF_WUP0>>24) & 0xFF;
#endif

  //Low Frequency IDentifier 1 data register (LFID1)
#ifdef CFG_LF_WUP1_IN_EEPROM
  // Wake-up ID stored in EEprom as a buffer (MSB first)
  ATA_eepReadBytes_C(laub_data, CFG_LF_WUP1_Adr);
  PHID10 = laub_data[(CFG_LF_WUP1_LENGTH/8-1)%4];
  PHID11 = laub_data[(CFG_LF_WUP1_LENGTH/8-2)%4];
  PHID12 = laub_data[(CFG_LF_WUP1_LENGTH/8-3)%4];
  PHID13 = laub_data[(CFG_LF_WUP1_LENGTH/8)%4];
#else
  // Wake-up ID stored in RAM or Flash
  PHID10 = (CFG_LF_WUP1) & 0xFF;
  PHID11 = (CFG_LF_WUP1>> 8) & 0xFF;
  PHID12 = (CFG_LF_WUP1>>16) & 0xFF;
  PHID13 = (CFG_LF_WUP1>>24) & 0xFF;
#endif
  
  
  // Settings for the protocol handler
 
  PHID0L = CFG_LF_WUP0_LENGTH;
  PHID1L = CFG_LF_WUP1_LENGTH;
  PHIDFR = LF_IDFRAMELENGTH;
  
  PHTBLR = 0xFF;                // Protocol Handler Telegram bit length
//  PHDFR = 48;
  
  LFSYSY0 = 0x09;               //Define wakeup ID pattern;
  LFSYLE = 0x04;
  
  PHIMR |= BM_PHID0IM | BM_PHID1IM;       // enable both wake-up ID interrupt
  
  LDFC = (1<<LDFMSB) | 5;//KarM_Data Fifo setting
  ID0_Wake = 0x00;
  ID1_Wake = 0x00;
  LF_DecErrFlag = 0x00; 
}


//-----------------------------------------------------------------------------
/** \brief <b>Init_LfRssi_flash_C</b>
    Prepare LF RSSI block for measurements
    

    \param[in]  none


    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC Init_LfRssi_flash_C(void)
{
  /* Temp. initialization of LF RSSI component data for use during System
  Verification */
  g_sLfRssi.bFlags = LFRSSI_FLAGS_RESET;
  g_sLfRssi.bStatus = LFRSSI_STATUS_RESET;
  ATA_lfRssiInit_C();
  
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_PerformLfRSSI_flash_C</b>
    Contains the complete flow for performing an LF RSSI measurement
    

    \param[in]  bMode       Contains internal or external LF RSSI measurement request


    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_PerformLfRSSI_flash_C(uint8_t bmode, uint8_t bsign)
{
  ATA_StartRssi_flash_C(bmode);
  __delay_cycles(3000);
  ATA_lfRssiMeasStart_C( &g_sLfRssiRegConfig_flash, bmode, bsign );
  __delay_cycles(10);
  do 
  {
    
  }
  while ((g_sLfRssi.bStatus & LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG)==0);
  
  if (bmode==LFRSSI_INT) 
  {
    ATA_lfRssiGetAverageResult_C((uint8_t*)&gIntLfRssi[0], (uint8_t*)0x0000);
  }
  if (bmode==LFRSSI_EXT)
  {  
    ATA_lfRssiGetAverageResult_C((uint8_t*)&gExtLfRssi[0], (uint8_t*)0x0000);
    gFlashApplignSignDetect.SD12result = SD12RR;
    gFlashApplignSignDetect.SD13result = SD13RR;
    gFlashApplignSignDetect.SD23result = SD23RR;
    gFlashApplignSignDetect.SD360result = SD360R;
  }
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_CheckLfData_flash_C</b>
    Reads out the received LF telegram from the internal LF data buffer

    \param[in]  none

    \return none

    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_CheckLfData_flash_C(void)
{
  uint8_t index;
  
  LDFCKSW |= (1<<LDFSCSW); 
  while ((LDFCKSW & (1<<LDFSCKS)) ==0);            // wait until clock source is switched
  gLfNmbrRxByts = LDFFL; 
  if (gLfNmbrRxByts !=0)
  {
    for (index=0; index < gLfNmbrRxByts; index++)
    {     
      g_MsgRXbuffer.aub_data[index + 3] = LDFD;//VID is only 3 bytes, so here start from 3;
    }
    g_MsgRXbuffer.ub_size = gLfNmbrRxByts;
  }
  LDFCKSW &= ~(1<<LDFSCSW);
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_TuneLfAllChannels</b>
    Tunes the LF circuitry using CTRIM to match the resonant frequency.
    
    \param[in]  none

    \return none

    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
void ATA_TuneLfAllChannels(void)
{
  uint8_t rtrim, ctrim;
  uint16_t last_RssiValue;
  
  // Enable the Trim feature
  LFCR3 |= 0x07;
  
  for (uint8_t channel = 0; channel < 3; channel++)
  {
    // Tune LF channel X
    rtrim = LFRX_R_Trim270k;               // Default value of QTRIM, fix during C trimming
    ctrim = LFRX_C_TrimOff;                    // Start value of trimming capacitors
    last_RssiValue = 0;
    
    do
    {
      if (channel == 0)
      {                  
        gLfqcTrim[0] = ctrim | rtrim;           // Set trimming register
        LFQC1 = gLfqcTrim[0];
      }
      else if (channel == 1)
      {
        gLfqcTrim[1] = ctrim | rtrim;
        LFQC2 = gLfqcTrim[1];
      }
      else if (channel == 2)
      {
        gLfqcTrim[2] = ctrim | rtrim;
        LFQC3 = gLfqcTrim[2];
        
      }
      Init_LfRssi_flash_C();
      // Internal RSSI measurement
      //__delay_cycles(18000);
      ATA_PerformLfRSSI_flash_C(LFRSSI_INT, NO_SIGNDET);
      if (gIntLfRssi[channel] >= last_RssiValue) 
      {
        ctrim += LFRX_C_Trim6pf;               // Increase trim capacitor by 6pF
        last_RssiValue = gIntLfRssi[channel];
      }
    } while ((gIntLfRssi[channel] == last_RssiValue) && (ctrim != 0x00));
  }
  
  ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE);
}

/**
 * \brief Update internal reference (compensation factor) in EEprom
 *        Values stored in raub_lf_rssi[LF_MEAS_INT] are used
 *
 * \param[in] lbb_with_acq TRUE execute an internal RSSI measurements and update
 *                         raub_lf_rssi[LF_MEAS_INT] before updating EEprom
 *                         FALSE otherwise
 *
 * \return void
 */
VOIDFUNC app_rssi_set_ref(bool IntMeasure)
{
  if (IntMeasure == TRUE)
  {
  // acquire RSSI internal
  Init_LfRssi_flash_C();
  ATA_PerformLfRSSI_flash_C(LFRSSI_INT, NO_SIGNDET);
  }
  
  // Store RSSI in EEProm
  ATA_eepWriteBytes_C((uint8_t*)&gIntLfRssi[0],(uint16_t)&g_sEepFlashApp_RKEPEPS.aub_rssi_intref,6);
}


/**
 * \brief PEPS Task on frame reception
 *        Start LF RSSI acquisiotns if needed
 *        Check WUID is coming from RF link
 *        Call PEPS handler
 *
 * \return none
 */
void app_peps_handler(uint8_t lub_channel)
{
 
  // accept only frames with more than 2 bytes (WUID + CID + CKS)
  if (gLfNmbrRxByts >= 2)              
  {  
    rub_cid = ((g_MsgRXbuffer.aub_data[3]& BM_CID)>>4);  
    rub_wuip = lub_channel;
    
    if (rub_wuip&RX_CHAN_LF_MSK)
    { 
      // message received by LF
      rub_wuip |= (rub_wuip<<3);    // set WUID source
      // RSSI acquisitions ?
      if (cabb_cmd_with_cw[rub_cid])
      {
        // acquire internal RSSI (CW OFF)
        Init_LfRssi_flash_C();      
        if (bit_test(LED2)) bit_clear(LED2);
        else bit_set(LED2);
        ATA_PerformLfRSSI_flash_C(LFRSSI_EXT, SIGNDET);
         __delay_cycles(18000);//12000  - 1 in 50 miss
        if (bit_test(LED2)) bit_clear(LED2);
        else bit_set(LED2);
         ATA_PerformLfRSSI_flash_C(LFRSSI_INT, NO_SIGNDET);
      }
      // force emission on channel 1 when command is received by LF
      rub_rf_chan = 0; //Toby - was 1
    }
    else
    {

    }
      _app_peps_task();         

  }
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lFRssiGetResult_flash_C<void>
    Read out the result from the last LF RSSI measurements and store them 
      to the global variables

    \param[in] none

    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/

void ATA_lFRssiGetResult_flash_C(uint8_t bmode, uint8_t bsign)
{

  RSMS1R &= ~(1<<RSSSV);        // for output the everage value
  if (bmode == 0)
  {
    gExtLfRssi[0] = (RSRES1H<<8) ;
    gExtLfRssi[0] |= RSRES1L;
    gExtLfRssi[1] = (RSRES2H<<8) ;
    gExtLfRssi[1] |= RSRES2L;
    gExtLfRssi[2] = (RSRES3H<<8) ;
    gExtLfRssi[2] |= RSRES3L;
    
  }
  else 
  {
    gIntLfRssi[0] = (RSRES1H<<8) ;
    gIntLfRssi[0] |= RSRES1L;
    gIntLfRssi[1] = (RSRES2H<<8) ;
    gIntLfRssi[1] |= RSRES2L;
    gIntLfRssi[2] = (RSRES3H<<8) ;
    gIntLfRssi[2] |= RSRES3L;
  }
}



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
static void _app_peps_task(void)
{
  // execute command
  if (_peps_cmd_validity())
  
  {
    // light on LED
    bit_set(LED1);
    
//#ifdef CFG_APP_2WAYRF
//    // wake-up RF
//    rf_ata5831_setmode(E_STATE_IDLE, (RF_ATA5831_CONFIG_TU){0});
//#else // ONE WAY
//    // start XTO quartz now so that it is stabilized when we need to transmit
//    rf_ata5791_setmode(RF_MODE_IDLE);
//#endif

    // Build RF message
    _peps_build_msg();     

    //activate voltage monitor
   
    uint8_t VMCR_set = BM_VM_VBAT | BM_VM_2_5V; 
    ATA_globalsSetVoltageMonitor_C(VMCR_set);//Set Voltage montitor @ 2.5V
        
    //load up the data message to be tested
    for (uint8_t index = 0; index < (CFG_PEPS_PREAMBLE_LENGTH-1); index++)
    {
      MSG_TX_DATA.peps.preamble0[index]=0xff;
    }     
    MSG_TX_DATA.peps.preamble1 = 0xfe;
    

    // Send RF message
    ATA_rfTx_PEPSmsg_flash_C();      
   
    // update battery flag
    gVbat_Status = (VMSCR & BM_VMF);
 
    // stop voltage monitor
    VMCR = BM_VM_DISABLE;
    PRR0 |= (1<<PRVM);

    // must be checked if delay must be added
    bit_clear(LED1);
  }
}

/**
 * \brief Verify LF frame command validity
 *        Check managed CID, Frame lenght, CRC if any, WUP ID used, Diag mode
 *        Command parameter (with cyphered challenge in case of bilateral authent)
 *
 * \return TRUE if command is valid, FALSE otherwise
 */
static bool _peps_cmd_validity(void)
{
  uint16_t AES_KeyAddr; 
  ATA_eepReadBytes_C((uint8_t*)&rub_fob_idx, (uint16_t) &g_sEepFlashApp_RKEPEPS.ub_fidx, 0x01);
  ATA_eepReadBytes_C((uint8_t*)&rul_fob_id, eul_key_id, 0x04);
  
  // check wake-up ID source
  switch (rub_cid)
  {
    case PEPS_CID_WR_PARAM:
    case PEPS_CID_RD_PARAM:
      if ((rub_wuip&RX_WUID_MSK) == RX_WUID1)
      {
        // wrong wake-up or wrong index
        return FALSE;
      }
      else if (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->fidx != rub_fob_idx)
      {
        // wrong fob index
        return FALSE;
      }
      break;

    case PEPS_CID_2WAY:
      // allowed everytime
      break;
      
    case PEPS_CID_UNI_AUTH_SINGLE:
    case PEPS_CID_BI_AUTH_SINGLE:
      if (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->fidx != rub_fob_idx)
      {
        // wrong fob index
        return FALSE;
      }
    case PEPS_CID_UNI_AUTH:
    case PEPS_CID_BI_AUTH:
      if ((rub_wuip&RX_WUID_MSK) == RX_WUID1)
      {
        // wrong wake-up
        return FALSE;
      }
      break;
      
    default:
      if ((rub_wuip&RX_WUID_MSK) == RX_WUID0)
      {
        // vehicle wake-up
        if (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->fidx != rub_fob_idx)
        {
          // wrong fob index
          return FALSE;
        }
      }
  }

  // check mode
  switch (rub_cid)
  {
    case PEPS_CID_RD_PARAM:
    case PEPS_CID_WR_PARAM:
      if (gNTE_DIAG_MODE == DIAG_OFF)
      {
        // forbiddden in DIAG OFF
        return FALSE;
      }
      break;

    case PEPS_CID_LF_PARAM:
      if (gNTE_DIAG_MODE != DIAG_OEM)
      {
        // only allowed in OEM mode
        return FALSE;
      }
      break;
      
    default:
      break;
  }

   // check command parameters
  switch (rub_cid)
  {
    case PEPS_CID_RD_PARAM:
    case PEPS_CID_WR_PARAM:
      
      //**
      if (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_rd_param.param_index >=32)
      {
        // wrong parameter index
        return FALSE;
      }
      break;

    case PEPS_CID_BI_AUTH:
    case PEPS_CID_BI_AUTH_SINGLE:
    
     PRR0 &= ~BM_PRCU;         //disable power reduction for AES
     AESCR = BM_AESRES;
     
     ATA_eepReadBytes_C((uint8_t*)&AES_KeyAddr, (uint16_t)&g_sCustomerEEPromSection.eepSecKeyAddrB, 0x02); //KeyB(Vehicle_SK) for Authentication; 
     ATA_aesTriggerKeyDma_C(AES_KeyAddr);
     __delay_cycles(30);    
     memory_copy(&gAES_DataBuffer[0], (uint8_t*)((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_auth_bi.challenge, CFG_PEPS_CHALLENGE_LENGTH);
     for (uint8_t index = CFG_PEPS_CHALLENGE_LENGTH; index < 16; index++)
      {
          gAES_DataBuffer[index] = gAES_DataBuffer[(index & 3)]; 
      }
      ATA_aesLoadData_C(&AESDR, &gAES_DataBuffer[0]);
      AESCR = BM_AESE; 
      while ((AESSR & BM_AESRF)== 0x00U);       //wait until ready
      AESSR |= BM_AESRF;                        // clear flag
      for (uint8_t index = 0; index < 16; index++)
      {
        gAES_DataBuffer[index] = AESDR;
      }
      PRR0 |= BM_PRCU;         //enable power reduction for AES
      
      // compare challenge cyphered with the on received
      if (!memory_compare(&gAES_DataBuffer[0],
                          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_auth_bi.cyph_challenge),
                          CFG_PEPS_CHALLENGE_CYPH_LENGTH))
      {
        return FALSE;
      }
      break;

    case PEPS_CID_MODE:
      // check FOB ID
      if (rul_fob_id != ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.fid)
      {
        // wrong ID
        return FALSE;
      }
      break;
      
    default:
      break;
  }

  return TRUE;
}

/**
 * \brief Execute command and build PEPS RF message
 *        RF message is stored in MSG_TX_DATA variable
 *        Authent : compute AES and RSSI values
 *        Write param: update param in EEprom
 *        LF test: compute RSSI
 *        LF param: update parameters in EEprom
 *        Mode: change DIAG mode
 *
 * \return void
 */
static void _peps_build_msg(void)
{
  uint32_t* lpul_id;
  uint16_t AES_KeyAddr;
  uint16_t Normvalues[3];
  uint16_t msg_length = 0;
  
 // common message data
  memory_set(MSG_TX_DATA.rke.preamble0, 0x00, CFG_RKE_PREAMBLE_LENGTH-1);
  MSG_TX_DATA.peps.preamble1 = 0x01;
  MSG_TX_DATA.peps.fidx = rub_fob_idx&0x07;
  MSG_TX_DATA.peps.bat = gVbat_Status;
  MSG_TX_DATA.peps.cid = rub_cid;

  // specific message data
  switch (rub_cid)
  {
    case PEPS_CID_RD_ID://Read ID command
      MSG_TX_DATA.peps.data_id.fid = rul_fob_id;//Load Fob ID in response
      ATA_eepReadBytes_C((uint8_t*)&MSG_TX_DATA.peps.data_id.vid, (uint16_t)&g_sEepFlashApp_RKEPEPS.aub_vid, VID_LENGTH);//Load vehicle ID in resposne      
      CRCenable();//Enable CRC appended to RF response
      break;

    case PEPS_CID_UNI_AUTH:
    case PEPS_CID_BI_AUTH:
    case PEPS_CID_UNI_AUTH_SINGLE:
    case PEPS_CID_BI_AUTH_SINGLE:
        CalcLinVector();
       
        MSG_TX_DATA.peps.data_authent.rssil = g3dVector[0];   
        MSG_TX_DATA.peps.data_authent.rssih = g3dVector[1];
        MSG_TX_DATA.peps.data_authent.rssiFlag = (g3dVectorFlag & 0x7f);  // Clear RFFI error flag - stop red box in CARS GUI

        MSG_TX_DATA.peps.data_authent.CoilPhase12 = gFlashApplignSignDetect.SD12result; 
        MSG_TX_DATA.peps.data_authent.CoilPhase13 = gFlashApplignSignDetect.SD13result; 
        MSG_TX_DATA.peps.data_authent.CoilPhase23 = gFlashApplignSignDetect.SD23result; 
        MSG_TX_DATA.peps.data_authent.CoilPhase360 = gFlashApplignSignDetect.SD360result; 
        
        
        /* Sign Detect Data to be added - 1 byte - needs to be construncted 
        MSG_TX_DATA.peps.data_lf_tst.phase12 = gFlashApplignSignDetect.SD12result;
        MSG_TX_DATA.peps.data_lf_tst.phase13 = gFlashApplignSignDetect.SD13result;
        MSG_TX_DATA.peps.data_lf_tst.phase23 = gFlashApplignSignDetect.SD23result;
        */
                   
          // compute MAC
          // load AES key
          PRR0 &= ~BM_PRCU;         //disable power reduction for AES
          AESCR = BM_AESRES;
          ATA_eepReadBytes_C((uint8_t*)&AES_KeyAddr, (uint16_t)&g_sCustomerEEPromSection.eepSecKeyAddrA, 0x02);//KEYA is used for Auth;KeyB is not defined in the EEP;

          ATA_aesTriggerKeyDma_C(AES_KeyAddr);
          // load message and cypher it
          memory_copy(MSG_TX_DATA.peps.data_authent.mac, 
                      ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_auth_uni.challenge,
                      CFG_PEPS_CHALLENGE_LENGTH);
          lpul_id = (uint32_t*)(MSG_TX_DATA.peps.data_authent.mac + CFG_PEPS_CHALLENGE_LENGTH);
          *lpul_id = rul_fob_id;
                
          msg_length = (CFG_PEPS_CID_LENGTH + CFG_PEPS_RSSI_LENGTH + CFG_PEPS_PHASEDATA_LENGTH 
                        +CFG_PEPS_CHALLENGE_LENGTH + CFG_PEPS_FID_LENGTH);
          memory_copy(&gAES_DataBuffer[0], (uint8_t*)&MSG_TX_DATA.peps.data_authent-1, msg_length);
         
          // Pad the message to match AES length.
          for (uint8_t index = msg_length; index < 16; index++)
          {
            gAES_DataBuffer[index] = AES_PADD_PATTERN;
          }
          
          ATA_aesLoadData_C(&AESDR, &gAES_DataBuffer[0]);
          //compute AES
          AESCR = BM_AESE; 
          while ((AESSR & BM_AESRF)== 0x00U);       //wait until ready
          AESSR |= BM_AESRF;                        // clear flag
          for (uint8_t index = 0; index < 16; index++)
          {
            gAES_DataBuffer[index] = AESDR;
          }
          memory_copy(MSG_TX_DATA.peps.data_authent.mac,
                      &gAES_DataBuffer[0],
                      CFG_PEPS_CHALLENGE_LENGTH);
          PRR0 |= BM_PRCU;         //enable power reduction for AES

    #ifdef CFG_APP_2WAYRF
          // enable ATA5831 communication back
        #endif      
      break;

    case PEPS_CID_WR_PARAM:
      // store parameter in EEprom
      ATA_eepWriteBytes_C(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_data,
                    eaub_param +(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_index)* 3 * PARAMBLOCK_SIZE, PARAMBLOCK_SIZE);
      // no break in switch case to send reply
    case PEPS_CID_RD_PARAM:
      // return VID
      // read 4 bytes in every case, but last byte may be overwritte by cks
      ATA_eepReadBytes_C((uint8_t*)&MSG_TX_DATA.peps.data_param.vid, (uint16_t)&g_sEepFlashApp_RKEPEPS.aub_vid, 4);      
      // read parameter in EEprom
      ATA_eepReadBytes_C(&MSG_TX_DATA.peps.data_param.param_data[0],
                   eaub_param + (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_index) * 3 * PARAMBLOCK_SIZE, PARAMBLOCK_SIZE);
      // complete message (index + checksum)
      MSG_TX_DATA.peps.data_param.param_index =
        ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_wr_param.param_index;
       CRCenable();
      break;

    case PEPS_CID_2WAY:
      CalcLinVector();
      //MSG_TX_DATA.peps.data_2way.rssil = g3dVectorLin[0];
      //MSG_TX_DATA.peps.data_2way.rssih = g3dVectorLin[1];
      MSG_TX_DATA.peps.data_2way.rssil = g3dVector[0];
      MSG_TX_DATA.peps.data_2way.rssih = g3dVector[1];
      MSG_TX_DATA.peps.data_2way.rssiFlag = g3dVectorFlag;
      MSG_TX_DATA.peps.data_authent.CoilPhase12 = gFlashApplignSignDetect.SD12result; 
      MSG_TX_DATA.peps.data_authent.CoilPhase13 = gFlashApplignSignDetect.SD13result; 
      MSG_TX_DATA.peps.data_authent.CoilPhase23 = gFlashApplignSignDetect.SD23result; 
      MSG_TX_DATA.peps.data_authent.CoilPhase360 = gFlashApplignSignDetect.SD360result; 
      // save rF channel for reply
      rub_rf_chan = (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->fidx)>>1;
      // enable RF reception for 1s
      // return msg already completed
      break;

    case PEPS_CID_LF_TST:
      CalcLinVector();
      // MSG_TX_DATA.peps.data_lf_tst.rss = g3dVectorLin[1]<<8 | g3dVectorLin[0];
      MSG_TX_DATA.peps.data_lf_tst.rss = ((((uint16_t)g3dVector[1]) << 8) | g3dVector[0]);
      MSG_TX_DATA.peps.data_lf_tst.rssiFlag = g3dVectorFlag;
      // MSG_TX_DATA.peps.data_lf_tst.norm_x = wLfRssiNorm[LF_AXIS_X];
      // MSG_TX_DATA.peps.data_lf_tst.norm_y = wLfRssiNorm[LF_AXIS_Y];
      // MSG_TX_DATA.peps.data_lf_tst.norm_z = wLfRssiNorm[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_tst.SD_12 = gFlashApplignSignDetect.SD12result;
      MSG_TX_DATA.peps.data_lf_tst.SD_13 = gFlashApplignSignDetect.SD13result;
      MSG_TX_DATA.peps.data_lf_tst.SD_23 = gFlashApplignSignDetect.SD23result;
      MSG_TX_DATA.peps.data_lf_tst.SD_360 = gFlashApplignSignDetect.SD360result;
      MSG_TX_DATA.peps.data_lf_tst.ref_x = wLfRssiref[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.ref_y = wLfRssiref[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.ref_z = wLfRssiref[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_tst.ext_x = gExtLfRssi[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.ext_y = gExtLfRssi[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.ext_z = gExtLfRssi[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_tst.int_x = gIntLfRssi[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_tst.int_y = gIntLfRssi[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_tst.int_z = gIntLfRssi[LF_AXIS_Z];
      CRCenable();
      
      break;

    case PEPS_CID_LF_PARAM:
      // update normalization factors
      if ((((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_x != 0xFFFF) ||
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_y != 0xFFFF) ||
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_z != 0xFFFF))
      {
        /* Update phase 3 - modifed GUI and CARS board to eliminate offset. Now
        receved external RSSI values are returned inthe LF parameter command */
        
        /*Normalization values returned as difference from a user input setpoint 
        by the GUI. Unfortunately this yields negative values that the 
        P2P cannot use. So here if all are positive we will use them as is,
        Else if any one is negative we will offset subtracte all from 155 and use 
        the difference. It is only neccessary to use the difference in the coil 
        response for normalization as the only relevant info is the difference
        between the axis measurements. .  
        
        Much discussion on this point to accomodate the existing GUI one byte
        delta values. For now I shift left 8 bits and use that. This discards
        four fractional bits and one integer bit. When the GUI is upgraded 
        the RAW RSSI two byte measurement for each axis should be returned. 
        GeRu 11 Mar 2013         
        */
        
        /*initialize some convenient variables to process the norm values. Extracting
        them form the RX message gets to be messy*/        
        
        Normvalues[0] =(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_z); 
        Normvalues[1] =(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_y);   
        Normvalues[2] =(((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.norm_x); 
       
        
       // /* they are receved as 2s complemet values with bit 7 flipped. How was
       // this scheme arrived at? So if bit 7 is set it is a positive value*/
        /*
        if (Normvalues[0]<128 || Normvalues[1]<128 || Normvalues[2]<128) //Any negatives?
        {
          uint8_t nrmoffset=0;
        */  
        /* Find the least value (greatest negative magnitude)*/ 
        /* Phase 3 
        for (uint8_t nrmcntr=0; nrmcntr<3; nrmcntr++)
         {
           if (nrmcntr==0) nrmoffset = Normvalues[nrmcntr];
           else if(Normvalues[nrmcntr]<nrmoffset) nrmoffset=Normvalues[nrmcntr];
         }
         nrmoffset |= 0x80;//Flip bit 7
         nrmoffset = ~nrmoffset+1;//Complement and add one
         for (uint8_t nrmcntr=0; nrmcntr<3; nrmcntr++)//Get rid of LDL +128 offset
         {
           if (Normvalues[nrmcntr] & 0x80)Normvalues[nrmcntr] &= 0x7f;
           else Normvalues[nrmcntr] |= 0x80;
         }
         Normvalues[0] =  Normvalues[0] + nrmoffset;
         Normvalues[1] =  Normvalues[1] + nrmoffset;
         Normvalues[2] =  Normvalues[2] + nrmoffset;
        }
        else //All positive - clear the offset bit
        {
          Normvalues[0] &= 0x7f;
          Normvalues[1] &= 0x7f;
          Normvalues[2] &= 0x7f;                 
        }
         Normvalues[0] = 255 - Normvalues[0];
         Normvalues[1] = 255 - Normvalues[1];
         Normvalues[2] = 255 - Normvalues[2];
        */
        wLfRssiNorm[LF_AXIS_X] = Normvalues[2];
        wLfRssiNorm[LF_AXIS_Y] = Normvalues[1];
        wLfRssiNorm[LF_AXIS_Z] = Normvalues[0];   
        
         //ATA_eepWriteBytes_C((uint8_t*)&wLfRssiNorm[0],LF_RSSI_NORM_X,6);
         ATA_eepWriteBytes_C((uint8_t*)&wLfRssiNorm[0], (uint16_t)&g_sEepFlashApp_RKEPEPS.aub_rssi_norm,6);
      
      }
      // update compensation factors
      if ((((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_x != 0xFFFF) &&
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_y != 0xFFFF) &&
          (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_z != 0xFFFF))
      { 
        gIntLfRssi[LF_AXIS_X] = (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_x);
        gIntLfRssi[LF_AXIS_Y] = (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_y);
        gIntLfRssi[LF_AXIS_Z] = (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_z);
        app_rssi_set_ref(FALSE);//Move to EEPROM
        
      }
      else if ((((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_x == 0xFE) &&
               (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_y == 0xFE) &&
               (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_lf_param.ref_z == 0xFE))
      {
        // auto acquire new internal reference
        
        app_rssi_set_ref(TRUE);//Measure and move to EEPROM
      }
      else
      {
        // No change 
      }

      // Populate RF resposne message - Shift right 4 bits and for NOrm values flip the MSbit   
      MSG_TX_DATA.peps.data_lf_param.norm_x =wLfRssiNorm[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_param.norm_y =wLfRssiNorm[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_param.norm_z =wLfRssiNorm[LF_AXIS_Z];
      MSG_TX_DATA.peps.data_lf_param.ref_x =wLfRssiref[LF_AXIS_X];
      MSG_TX_DATA.peps.data_lf_param.ref_y =wLfRssiref[LF_AXIS_Y];
      MSG_TX_DATA.peps.data_lf_param.ref_z =wLfRssiref[LF_AXIS_Z];
      CRCenable();      
      break;

    case PEPS_CID_SWID:
      // send Software ID  
 
  memory_copy(MSG_TX_DATA.peps.data_swid.swid,
                        SW_ID,
                       6);   
      
 
      CRCenable();//Enable CRC appended to RF response
      break;

    case PEPS_CID_MODE:
      // check mode and code
      if (((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode > 2)
      {
        // unknown mode, back to diag OFF
        //NTE_DIAG_MODE = DIAG_OFF;
        rts_cram.data.te_diag_mode = DIAG_OFF;
      }
      else if (memory_compare_const(
                 (uint8_t*)&((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.code,
                (const uint8_t*)caub_diag_code[((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode-1],                 
                 4))
      {
        gNTE_DIAG_MODE = (DIAG_MODE_TE)((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode;
        // activate long timeout @8s to exit diag
        // timeb_timer_start_s(CFG_TIMER_MODE, 8); //ToDo
      }
      else if (memcmp_G(
                 (uint8_t*)&((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.code,
                (const uint8_t*)caub_diag_code[((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode-1],                 
                 4))
        gNTE_DIAG_MODE = (DIAG_MODE_TE)((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.mode;

       
      else
      {
        //wrong code
        //NTE_DIAG_MODE = DIAG_OFF;
        rts_cram.data.te_diag_mode = DIAG_OFF;
      }
      MSG_TX_DATA.peps.data_mode.fid = ((RX_MSG_PEPS_TS*)MSG_RX_DATA)->data_mode.fid;
      MSG_TX_DATA.peps.data_mode.mode = (uint8_t)gNTE_DIAG_MODE;
      CRCenable();
      break;
     
    default:
      break;
  }
 
}


void CRCdisable(void)
{
   uint16_t serv0Addr = 0;
   // Get the Service 0 address from the EEPROM
   ATA_eepReadBytes_C((uint8_t*)&serv0Addr, (uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l, 0x02);
   // Load the bTMCR2 value from the EEPROM                 
   ATA_eepReadBytes_C((uint8_t*)&g_sRfTxServicePathConfig0.bTMCR2, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig->bTMCR2, 0x01);
   
   if (g_sRfTxServicePathConfig0.bTMCR2 & 0x01)  //CRC enabled? then disable
   {  
     g_sRfTxServicePathConfig0.bTMCR2 &= ~(1 << TMCRCE);  //Turn off CRC
     // Save to EEPROM
     ATA_eepWriteBytes_C((uint8_t *)&g_sRfTxServicePathConfig0.bTMCR2, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig->bTMCR2, 0x01);
   }
   gFlashApplState.State &= ~(BM_RFTXCRCACTIVE); //Clear state variable
}


void CRCenable(void)
{
   uint16_t serv0Addr = 0;
   // Get the Service 0 address from the EEPROM
   ATA_eepReadBytes_C((uint8_t*)&serv0Addr, (uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l, 0x02);
   
   // Load the bTMCR2...bTMCSB values from the EEPROM                 
   ATA_eepReadBytes_C((uint8_t*)&g_sRfTxServicePathConfig0.bTMCR2, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig->bTMCR2, 0x09); 
   
   if ((g_sRfTxServicePathConfig0.bTMCR2 & 0x01) == 0)  //CRC disabled? then enable 
   { 
     g_sRfTxServicePathConfig0.bTMCSB = ((sizeof(MSG_TX_DATA.peps.preamble0) + 1))*8;  //CRC calculation skip preamble
     g_sRfTxServicePathConfig0.bTMCR2 |= ((1 << TMCRCE) | (1 << TMCRCSE0));            //ENable CRC engine - 8 bit
     g_sRfTxServicePathConfig0.bTMCP[0] = 0x07;                                        //Load CRC polynomial; 
     // Save to EEPROM 
     ATA_eepWriteBytes_C((uint8_t *)&g_sRfTxServicePathConfig0.bTMCR2, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig->bTMCR2, 0x09); 
   }
   gFlashApplState.State |= BM_RFTXCRCACTIVE; //Set state variable 
}


void Stopbyteenable(void)
{
   uint16_t serv0Addr = 0;
   // Get the Service 0 address from the EEPROM
   ATA_eepReadBytes_C((uint8_t*)&serv0Addr, (uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l, 0x02);
   
   // Load the bTMCR2...bTMSSC values from the EEPROM                 
   ATA_eepReadBytes_C((uint8_t*)&g_sRfTxServicePathConfig0.bTMCR2, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig->bTMCR2, 0x02);
   
   if (((g_sRfTxServicePathConfig0.bTMCR2 & 0x20) == 0))    //Stop sequence enabled? then skip 
   {  
      g_sRfTxServicePathConfig0.bTMCR2 |= (1 << TMSSE);     //ENable STop sequence engine - 8 bit
      g_sRfTxServicePathConfig0.bTMSSC &= ~(1 << TMSSH);    //Hold mode off 
      g_sRfTxServicePathConfig0.bTMSSC |= ((1 << TMSSP0) | (1 << TMSSP1) | (1 << TMSSP3)); //0010 stop sequence
      g_sRfTxServicePathConfig0.bTMSSC &= ~(1 << TMSSP2);
      g_sRfTxServicePathConfig0.bTMSSC &= ~((1 << TMSSL0) | (1 << TMSSL1) | (1 << TMSSL2)); //length 8 bits (SSL = 0)
      // Save to EEPROM 
      ATA_eepWriteBytes_C((uint8_t *)&g_sRfTxServicePathConfig0.bTMCR2, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig->bTMCR2, 0x02); 
   }
   gFlashApplState.State |= BM_RFTXSTOPACTIVE;
}


uint8_t CalcCoilPhaseSig(void)
{
  //#define NumDeg 360
  //#define quad1 90
  //#define quad3 270  
  uint8_t CoilPhaseSig=0x00;
  uint32_t fubar32;
  
  fubar32 = ((uint32_t)gFlashApplignSignDetect.SD12result*0x168)/(uint32_t)gFlashApplignSignDetect.SD360result;
  if ((fubar32>0x5a) && (fubar32<0x10e)) CoilPhaseSig |= 0x80;
  fubar32 = ((uint32_t)gFlashApplignSignDetect.SD13result*0x168)/(uint32_t)gFlashApplignSignDetect.SD360result;
  if ((fubar32>0x5a) && (fubar32<0x10e)) CoilPhaseSig |= 0x40;
  fubar32 = ((uint32_t)gFlashApplignSignDetect.SD23result*0x168)/(uint32_t)gFlashApplignSignDetect.SD360result;
  if ((fubar32>0x5a) && (fubar32<0x10e)) CoilPhaseSig |= 0x20;
  return CoilPhaseSig;
}


