/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashAppl.c $
  $LastChangedRevision: 591974 $
  $LastChangedDate: 2020-03-16 09:23:12 -0600 (Mon, 16 Mar 2020) $
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

/** \file FlashAppl.c
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

#include "../src/micro.h"
#include "rfrcc_flash.h"
#include "FlashApplVars.h"

#include "../src/FlashApplPEPS.h"
#include "FlashApplMSG.h"



#include "../src/FlashApplLF.h" 

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define FLASH_MAJOR_VERSION     0x02U
#define FLASH_MINOR_VERSION     0x05U
#define FLASH_CUSTOMER_VERSION  0x00U


#define TX_PREAMBLEDATALENGTH   5
#define TX_PAYLOADDATALENGTH    32


/*===========================================================================*/
/*  Modul Globals                                                             */
/*===========================================================================*/
#pragma location = ".versions"
__root __flash static uint16_t flashVersion = (uint16_t)(((uint16_t)FLASH_MINOR_VERSION<<8U) | FLASH_MAJOR_VERSION);

#pragma location = ".versions"
__root __flash static uint8_t  customerVersion = FLASH_CUSTOMER_VERSION;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/


extern VOIDFUNC ATA_lfRxEnableWakeup_flash_C(uint8_t bLfBdrate,uint8_t bSense);
extern VOIDFUNC ATA_CheckLfData_flash_C(void);
extern VOIDFUNC Init_LfRssi_flash_C(void); 
extern VOIDFUNC gpio_init(void);
extern VOIDFUNC app_peps_handler(uint8_t lub_channel);
extern void ATA_TuneLfAllChannels(void);
extern sRfTxServicePathConfig g_sRfTxServicePathConfig0;

extern char ID0_Wake=0x00;
extern char ID1_Wake=0x00;
extern char LF_DecErrFlag=0x00; 
extern uint8_t gLfMessageReceived;

extern tTimer5Status gTimer5Status;
extern tTimer4Status gTimer4Status;
extern tPCINTStatus gPCINTStatus;

extern sCustomerEEPromSection g_sCustomerEEPromSection;
extern sEepFlashApp_AESKey g_sEepFlashApp_AESKey;
//extern uint8_t g_EepFlashApp_FOBindx;
extern uint8_t g_EepFlashApp_USRID[4];

sFlashApplState gFlashApplState;
sFlashApplVars gFlashApplVars;


/*----------------------------------------------------------------------------- */
/**\brief  <b>main routine</b>
            for ATA5700 Flash application software to support the Primus2P
            test mode for the Tx library.

    The main function checks for a valid system wake up, initializes the system
    and enters the main loop which is responsible for command decoding, system
    control and sleep mode control.

 */
/*----------------------------------------------------------------------------- */
int16_t main(void)
{
    /* Use INTVEC at Flash start address 0x4000 */
    MCUCR = BM_IVSEL | BM_IVL0;
    MCUCR |= (1<<ENPS);   // Enable Port Settings
 
    ATA_initAvrRegister_flash_C();

    /* Init global system events before performing */
    ATA_5700InitCommandSet_flash_C();

    /* Store reset events and clear register. */
    g_sAta5700_flash.events_reset = MCUSR;
    MCUSR = 0x00U;

    /* Disable Watchdog during initialization. */
    ATA_globalsWdtDisable_C();
    ATA_globalsInitDebug_C();

    /* Perform Application initialization */
    if (ATA_initAta5700_flash_C() == FAIL){
        ATA_systemErrorLoop_flash_C();
    }


 /* Initialization of LF calibration data, only if LFVCC was off before. */
    if (1) //( (LFCPR & BM_LFCALRY) == 0x00U )
    {
    /* Initialization of LF calibration data */
        ATA_lfRxInit_C();

        /* System Verification Requirement, referenced as BUG "Primus2P-2165" */
        LFCPR = BM_LFCPCE;
        LFCPR = BM_LFCALRY;

        /* Primus2P-1827 and Primus2P-1670:
           This is a Flash Validation requirement to set T0 to the given value. */
        T0CR  = 0x12U;
        T0IFR = 0x01U;
        
        // Initialize LFREC, Power Mode
       ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE);
    }
    else
    {
        /* Enable clock for LF Receiver to be able to update its registers. */
        ATA_POWERON_C(PRR1, PRLFR);

        /* Set LF Calibration available indication */
        g_sAta5700_flash.events_reset |= BM_ATA5700_EVENTS_RESET_LF_CALRY;
    }
 
    /* Initialization of LF and TP module */
    ATA_lfRxInit_flash_C();

    ATA_tpRxTxInit_flash_C();
     
    /* Initialization of RFTX module */
    ATA_rfTxInit_C();
    
    Init_LfRssi_flash_C();

    ATA_lfRssiInit_C();
    
    /* Initialize calibration component */
    ATA_calibInit_C();

    /* initialization of AES module */
    ATA_aesInit_C();
    /* Application Board IO init */
    gpio_init();   
   
    char volatile count;
   
   /* Enable global interrupts */
    _SEI;
    
    ATA_TuneLfAllChannels();
    
    //ATA_eepReadBytes_C(gFlashApplVars.RfTxbuffer, 0xE8, 0x08);
    
    uint16_t serv0Addr = 0;
    // Get the Service 0 address from the EEPROM
    ATA_eepReadBytes_C((uint8_t*)&serv0Addr, (uint16_t)&g_sCustomerEEPromSection.eepRfTxSer0Ptr_l, 0x02);
    // Load the Service 0 RF Path configuration
    ATA_eepReadBytes_C((uint8_t*)&g_sRfTxServicePathConfig0, (uint16_t)&((sRfTxServiceConfig *)serv0Addr)->sPathConfig, sizeof(sRfTxServicePathConfig));
    
    if ((g_sRfTxServicePathConfig0.bTMCR2 & 0x01) == 1) CRCdisable();//Turn off CRC-
    if (((g_sRfTxServicePathConfig0.bTMCR2 & 0x20) == 0)) Stopbyteenable();//Set stop byte on 

    ATA_spiOpenSlaveFlash_C((1<<SPE),0x00);
   // ATA_spiOpenSlaveFlash_C(uint8_t bSpcr, uint8_t bSpsr)
   
    for(;;)
    {
      ATA_globalsClkSwitchFrc_C();
      /* toggle GPIOR0.2 to indicate main loop frequency */
      GPIOR0 ^= BIT_MASK_2;   
      
      // uint8_t Conf_WDTCR = ((1<<WDPS0) | (1<<WDPS1) | (1<<WDE));//Enable Watch Dog Timer with 2.1 S interval
      // ATA_globalsWdtEnable_C(Conf_WDTCR);//Caution - Won't debug with WDT enabled
     
      if ((gTimer4Status & BM_TIMER4COMPARETRUE)) ATA_FlashApplTimer4Process_C();  
      
      if ((gFlashApplState.State&BM_SPI_received_flag)==BM_SPI_received_flag){
         ATA_spiCloseSlaveFlash_C();
         ATA_Flash_RKEbuttonProcessOut();//Amazon SPI command to TX
         ATA_spiOpenSlaveFlash_C((1<<SPE),0x00);

      }
      
      
      if ((gTimer4Status & BM_TIMER4COMPARETRUE)) ATA_FlashApplTimer4Process_C();      
     // if (gPCINTStatus & BM_PCINT1TRUE) ATA_RKEtimer4Start();//Process Pin Change RKE intrs
      if (gPCINTStatus & BM_PCINT1TRUE){
        if ((PIND & 0x04)== 0x04){         //Amazon RF on signal true
           gFlashApplVars.RKEcommand = 0x01;      
           ATA_Flash_RKEbuttonProcessOut();//CHanged for Amazon from EXTINT0 to PCMSK10
           gPCINTStatus &= ~(BM_PCINT1TRUE);//Clear the flag
           PCICR |= (1<<PCIE1); //re-enable piin change interrupts 
           }        
        else{      //Bush button interrupt
            ATA_RKEtimer4Start();//Process Pin Change RKE intrs
            }
        }
        if ( (gFlashApplState.Buttons&BM_INT0TRUE)==BM_INT0TRUE){//Amazon SPI NSS asserted low
             gFlashApplState.Buttons &= ~(BM_INT0TRUE);
             }
        
      if (LF_DecErrFlag ==0x01)
      {
          
        // LF wake-up and data was received 
        ATA_CheckLfData_flash_C();
        
        // Check which LF wake-up ID was received
        if (ID0_Wake == 0x01)
        {
          // vehicle ID received
          app_peps_handler(RX_CHAN_LF0);
        }
        else 
        {
          // broadcast ID received
          app_peps_handler(RX_CHAN_LF1);
        }
        
        T0CR = 0;      
   
        
        ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE);

      }
      
      if (((gFlashApplState.Buttons & BM_BUTTONPROCCESSINGACTIVE)==0) && ((gFlashApplState.Buttons & BM_NEWCMNDVALID)))  //New Button press command available
      {
        ATA_Flash_RKEbuttonProcessOut(); 
      }
      if ((gFlashApplState.State & BM_RFTXCRCACTIVE) &&  ((gFlashApplState.State & BM_PEPSRFTIMERACTIVE)==0 ) )
      {
           CRCdisable();//!!!!! 18 mS execution time -
      }
    //  g_bSleepModeConfig_flash = IDLE; 
        g_bSleepModeConfig_flash = POWER_DOWN;    // Set Sleep ModeEXT_POWER_DOWN
   // g_bSleepModeConfig_flash = EXT_POWER_DOWN;    // Set Sleep ModeEXT_POWER_DOWN
     gFlashApplState.State |= BM_PEPSRFTIMERACTIVE;// For SPI debug
      /* Check whether sleep mode can be entered. */
     if (
                (        (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE) == 0x00U)              &&
                (  (g_sAta5700_flash.status & BM_ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG) == 0x00U) &&
                (  (gFlashApplState.Buttons & BM_BUTTONPROCCESSINGACTIVE) != BM_BUTTONPROCCESSINGACTIVE) &&
                (   (gFlashApplState.State & BM_PEPSRFTIMERACTIVE) ==0x00)
             
          )
      {
        
        if ( (g_bSleepModeConfig_flash & BIT_MASK_0) != 0x00U )
        {
          bit_clear(LED2);
          ATA_globalsWdtDisable_C();
          ATA_globalsClkSwitchSrc_C();//attempt to reduce sleep current
          /*Enter Sleep using g_bSleepModeConfig_flash variable*/
          ATA_globalsSleep_C(g_bSleepModeConfig_flash);
          /* Disable Sleep */
          g_bSleepModeConfig_flash &= ~BIT_MASK_0;
        }
      }
    }
}









