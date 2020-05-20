/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplLF.c $
  $LastChangedRevision: 463332 $
  $LastChangedDate: 2017-06-15 09:38:17 -0600 (Thu, 15 Jun 2017) $
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

/** \file FlashApplLF.c
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
#include "../../../firmware/aes/src/aes.h"

#include "../src/FlashApplLF.h"
#include "../src/FlashApplVars.h"


/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define DEFAULT_RSSI_REF_VALUE          (0x0A00)
#define DEFAULT_RSSI_NORM_VALUE         (0x0A00)
/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/
extern uint16_t gExtLfRssi[3];
extern uint16_t gIntLfRssi[3];
extern uint16_t gLfRssiRes[3]; 
extern uint16_t wLfRssiref[3];
extern uint16_t wLfRssiNorm[3];
extern uint16_t wBref;
extern uint8_t gRSSI_ResBuffer[6];
extern uint8_t g3dVector[2];
extern uint8_t g3dVectorLin[2];
extern uint8_t g3dVectorFlag;
extern sEepFlashApp_RKEPEPS g_sEepFlashApp_RKEPEPS;


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*----------------------------------------------------------------------------- */
/**\brief  TODO - code here

 */
/*----------------------------------------------------------------------------- */
   
//-----------------------------------------------------------------------------
/** \brief <b>ATA_StartRssi_flash_C</b>
    Contains the complete flow for performing an LF RSSI measurement
    

    \param[in]  none


    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_StartRssi_flash_C(uint8_t bmode)
{
  uint8_t srcVal =0;
  ATA_lfRssiSetEepromConfig_C( srcVal );        
  ATA_lfRssiOpen_C();       // Check!!! 
  ATA_lfRssiMeasEnableLfReceiver_flash_C();
 // g_sLfRssiRegConfig_flash.bRscr = 0x08;
  g_sLfRssiRegConfig_flash.bRscr = (BM_RSOFM | RSSDEN);
 
  //g_sLfRssiRegConfig_flash.bRsdlyr = 24; //select the RSSI Tracking time 0x18 8uS sampling rate/484 uS duration
  g_sLfRssiRegConfig_flash.bRsdlyr = 26; //select the RSSI Tracking time 0x18 8uS sampling rate/525 uS duration

  g_sLfRssiRegConfig_flash.bRsms1r = 0x07;// select all channels
  
  //g_sLfRssiRegConfig_flash.bRsms2r = 0x70; // select number of samples for averaging 128 samples * 8 uS sample rate = 625 uS
  g_sLfRssiRegConfig_flash.bRsms2r = 0x60; // select number of samples for averaging 64 samples * 8 uS sample rate = 512 uS

  g_sLfRssiRegConfig_flash.bRssrcr = 0x00; // no SRC calibration

  do 
  { 
    __no_operation();
  } while ((RSSR & 0x01) ==0);
 
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRssiCalcCorr_C</b>
    Contains the complete flow for performing an LF RSSI measurement
    

    \param[in]  none


    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/

VOIDFUNC ATA_lfRssiCalcCorr_C(void)
{
  uint8_t index; 
  RSMS1R &= ~BM_RSSCAL;
  RSCR   &= ~(BM_RSMODE1 | BM_RSOFM | BM_RSEOR);
  RSCR   |= BM_RSMODE0;
  
  for( index = 0; index < 3; index++ )
  {
    RSFR = BM_RSOFF;
    
    RSRES1L = gExtLfRssi[index] & 0x00FF;
    RSRES1H = (gExtLfRssi[index]>>8);
    RSRES2L = gIntLfRssi[index] & 0x00FF;
    RSRES2H = (gIntLfRssi[index]>>8);
    RSRES3L = g_sLfRssi.bChanCalibVal[index + index];
    RSRES3H = g_sLfRssi.bChanCalibVal[index + index + 1];
    
    _CLI;
    RSCR |= BM_RSOS;    // start correction calculation
    
    while( !(RSFR & BM_RSOFF) ){}
    
    gLfRssiRes[index]  = RSRES4L;
    gLfRssiRes[index] |= (RSRES4H<<8);
  }

  RSFR = BM_RSOFF;
  
  
}



VOIDFUNC CalcLinVector(void)
{
  uint8_t bMargin; 
  
  ATA_eepReadBytes_C(&bMargin, MARGIN_EEADR, 0x01);
  ATA_eepReadBytes_C((uint8_t*)&wLfRssiref[0], (uint16_t)&g_sEepFlashApp_RKEPEPS.aub_rssi_intref, 0x06);
  ATA_eepReadBytes_C((uint8_t*)&wLfRssiNorm[0], (uint16_t)&g_sEepFlashApp_RKEPEPS.aub_rssi_norm, 0x06);
  
  /*--- EEPROM not yet intitialized, load dummy values in SRAM variables ---*/
  if (wLfRssiref[0] == 0xFFFF) { wLfRssiref[0] = DEFAULT_RSSI_REF_VALUE; }
  if (wLfRssiref[1] == 0xFFFF) { wLfRssiref[1] = DEFAULT_RSSI_REF_VALUE; }
  if (wLfRssiref[2] == 0xFFFF) { wLfRssiref[2] = DEFAULT_RSSI_REF_VALUE; }
  
  if (wLfRssiNorm[0] == 0xFFFF) { wLfRssiNorm[0] = DEFAULT_RSSI_NORM_VALUE; }
  if (wLfRssiNorm[1] == 0xFFFF) { wLfRssiNorm[1] = DEFAULT_RSSI_NORM_VALUE; }
  if (wLfRssiNorm[2] == 0xFFFF) { wLfRssiNorm[2] = DEFAULT_RSSI_NORM_VALUE; }
  
 /*---- Coil connections ATAB5702A-V2.3 PCB -----
  
  Z coil is variable index 0 connected to A1x
  Y coil is variable index 1 connected to A2x
  X coil is variable index 2 connected to A3x
  
  ----------------------------------------------*/
   
  bMargin=0; //Margin variable is no lnger usued so set to 0 
  
  ATA_lfRssiCalcChanCalibVal_C(bMargin, &wLfRssiref[0],&wLfRssiNorm[0]);
  ATA_lfRssiCalcCorr_C();
  for (uint8_t index=0;index < 3;index++)
  {
    gRSSI_ResBuffer[index+index]=(gLfRssiRes[index] & 0xFF);
    gRSSI_ResBuffer[index+index+1]=(gLfRssiRes[index] >>8);
  }
  ATA_lfRssiCalc3dVec_C(&gRSSI_ResBuffer[0],&g3dVector[0]);
  while (g_sLfRssi.bStatus & LFRSSI_STATUS_BM_3DVEC_DATA_AVAILABLE_FLAG ==0);//Wait for it
  g3dVectorFlag = g_sLfRssi.bFlags;
  if ((g_sLfRssi.bFlags & LFRSSI_FLAGS_BM_ERROR_FLAG) != 0) {
    // Error in getting 3dVec value
    //g3dVector[0] = g3dVector[1] = 0;
    //g3dVectorLin[0] = g3dVectorLin[1] = 0;
  } 
  else {
    
    ATA_lfRssiCalcLog2Lin_C(&g3dVector[0],&g3dVectorLin[0]);
    while (g_sLfRssi.bStatus & LFRSSI_STATUS_BM_LINEAR_DATA_AVAILABLE_FLAG ==0);//Wait for it
    if ((g_sLfRssi.bFlags & LFRSSI_FLAGS_BM_ERROR_FLAG) != 0) {
        // Error in getting 3D vector liner value
        //g3dVectorLin[0] = g3dVectorLin[1] = 0;
    }
  }
}

/**
 * \brief Load compensation / normalization factors from EEprom
 *
 * \return void
 */
void app_rssi_load_factors(void)
{
#ifdef RSSI_COMPENSATION
  // get RSSI internal ref
  ATA_eepReadBytes_C(wLfRssiref[LF_AXIS_X], LF_RSSI_INTREF_X,2);
  ATA_eepReadBytes_C(wLfRssiref[LF_AXIS_Y], LF_RSSI_INTREF_Y,2);
  ATA_eepReadBytes_C(wLfRssiref[LF_AXIS_Z], LF_RSSI_INTREF_Z,2);
#else
  wLfRssiref[LF_AXIS_X] = 0x00;
  wLfRssiref[LF_AXIS_Y] = 0x00;
  wLfRssiref[LF_AXIS_Z] = 0x00;
#endif
#ifdef RSSI_NORMALISATION
  // get RSSI compensation factor
  ATA_eepReadBytes_C(wLfRssiNorm[LF_AXIS_X], LF_RSSI_INTREF_X,2);
  ATA_eepReadBytes_C(wLfRssiNorm[LF_AXIS_Y], LF_RSSI_INTREF_Y,2);
  ATA_eepReadBytes_C(wLfRssiNorm[LF_AXIS_Z], LF_RSSI_INTREF_Z,2);
#else
  wLfRssiNorm[LF_AXIS_X] = 0x0100;                        //MiHa value unclear
  wLfRssiNorm[LF_AXIS_Y] = 0x0100;
  wLfRssiNorm[LF_AXIS_Z] = 0x0100;

             
#endif
}