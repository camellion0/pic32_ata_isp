/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/OldFlashAppl.c $
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

/** \file FlashAppl.c
    this file contains an ATA5700 Flash application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/rftx/src/rftx_flash.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set_sysver_flash.h"

#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"

#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/timer5/src/timer5.h"
#include "../../../firmware/globals/src/globals.h"

#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"

#include "../../../firmware/extif/src/extif_flash.h"

#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"

#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"
#include "../../../firmware/eep/src/eep.h"
#include "FlashApplVars.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define FLASH_MAJOR_VERSION     0x02U
#define FLASH_MINOR_VERSION     0x05U
#define FLASH_CUSTOMER_VERSION  0x00U

#define LFRX_BDR_1_95           0x00U
#define LFRX_BDR_3_90           0x08U
#define LFRX_BDR_7_81           0x10U

#define LFRX_H_SENSE            0x04U //Setting DAMP allows up to OVP
#define LFRX_M_SENSE            0x05U
#define LFRX_L_SENSE            0x07U

#define LFRX_R_TrimOff          0x00
#define LFRX_R_Trim18k          0x01
#define LFRX_R_Trim36k          0x02
#define LFRX_R_Trim54k          0x03
#define LFRX_R_Trim90k          0x04
#define LFRX_R_Trim117k         0x05
#define LFRX_R_Trim162k         0x06
#define LFRX_R_Trim198k         0x07
#define LFRX_R_Trim270k         0x08

#define LFRX_C_TrimOff          0x00
#define LFRX_C_Trim6pf          0x10
#define LFRX_C_Trim12pf         0x20
#define LFRX_C_Trim18pf         0x30
#define LFRX_C_Trim24pf         0x40
#define LFRX_C_Trim30pf         0x50
#define LFRX_C_Trim36pf         0x60
#define LFRX_C_Trim42pf         0x70
#define LFRX_C_Trim48pf         0x80
#define LFRX_C_Trim54pf         0x90
#define LFRX_C_Trim60pf         0xA0
#define LFRX_C_Trim66pf         0xB0
#define LFRX_C_Trim72pf         0xC0
#define LFRX_C_Trim78pf         0xD0
#define LFRX_C_Trim84pf         0xE0
#define LFRX_C_Trim90pf         0xF0

#define LFRX_IDLENGTH           32

#define TX_PREAMBLEDATALENGTH   5
#define TX_PAYLOADDATALENGTH    24

#define conSwitch1              1
#define conSwitch2              2
#define conSwitch3              4

/**********************
SMCR
0x01 -> Idle
0x05 -> Power Down
0x09 -> Ext. Power Down 
************************/
#define LF_SLEEP_MODE    5



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

static VOIDFUNC ATA_lfRxEnableWakeup_flash_C(uint8_t bLfBdrate,uint8_t bSense,uint8_t * pLf_Id,uint8_t bLf_IdLength);
static VOIDFUNC ATA_rfTx_lFRssi_flash_C(void);
static VOIDFUNC ATA_GetIntRssiValues(void);


extern char ID0_Wake=0x00;
extern uint8_t guiButton;


uint8_t PREAMBLEDATA [ TX_PREAMBLEDATALENGTH ]; //5
uint8_t PAYLOADDATA  [ TX_PAYLOADDATALENGTH ]; //32
//uint8_t guiButton =0;
//uint8_t gucNewButtonAvailable=0;
uint16_t gExtRssiValues[3];
uint16_t gIntRssiValues[3];
uint8_t gResSignDet[4];
sFlashApplState gFlashApplState;
sFlashApplVars gFlashApplVars;

struct sLfRssiApplResult
{
    // add additional flags and internal status for each measurement
    uint16_t wRawLfRssi[3];
    uint8_t  bSampleVal[12];
    uint8_t  bSignDetect[4];
    uint16_t wCorrLfRssi[3];
    uint16_t w3dVecVal;
    uint16_t wLinearVal;
    uint16_t wLinearVoltageValue;

}g_sLfRssiApplResult_flash;


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

  uint8_t bLFRX_IDLENGTH[4] ={0x11,0x12,0x13,0x14};  
  uint8_t index;
  uint8_t checksum;
  
    /* Use INTVEC at Flash start */
  MCUCR = BM_IVSEL | BM_IVL0;
  MCUCR |= (1<<ENPS);   // Enable Port Settings
  
    /* Default EEPROM settings used for different application types exceeds the
       unprotected EEPROM memory space (0x00 - 0xFF). For that reason, protected
       EEPROM memory locations (WP0 ... WP3) are enabled. */
//    EEPR = (BM_EEAP3 | BM_EEAP2 | BM_EEAP1 | BM_EEAP0);  // MH V1.3

    ATA_initAvrRegister_flash_C();

    /* Init global system events before performing */
    ATA_5700InitCommandSet_flash_C();

    /* Store reset events and clear register. */
    g_sAta5700_flash.events_reset = MCUSR;
    MCUSR = 0x00U;

    /* Check port wakeup events */
    if ( (g_sAta5700_flash.events_reset & BM_TPRF) == 0x00U)
    {
        g_sAta5700_flash.events_wakeup = ATA_initCheckWakeupSource_C();
    }
    else
    {
        /* POWER and NPWRON indicate no power */
        g_sAta5700_flash.events_wakeup = 0x00U;
    }


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
        ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE,&bLFRX_IDLENGTH[0],LFRX_IDLENGTH);
//        ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_L_SENSE,&bLFRX_IDLENGTH[0],LFRX_IDLENGTH);
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

    /* Initialize Pin Change handling */
    /* Check port wakeup events */
    if ( (g_sAta5700_flash.events_reset & BM_TPRF) == 0x00U )
    {
        ATA_initExtIf_flash_C(ATA_portB_flash_ASM, ATA_portD_flash_ASM);
    }
    

    /* Initialization of RFTX module */
    ATA_rfTxInit_C();

     /* Do event pin handling for wakeup events only once */
    if ( g_sAta5700_flash.events_wakeup & g_sEventHandling_flash.bWakeup )
    {
        ATA_systemSetEventPin_flash_ASM();
    }

    /* Do event pin handling for SYS_RDY event */
    g_sAta5700_flash.events_system |= BM_ATA5700_EVENTS_SYSTEM_SYS_RDY;

    if ( g_sAta5700_flash.events_system & g_sEventHandling_flash.bSystem )
    {
        ATA_systemSetEventPin_flash_ASM();
    }

    /* Do event pin handling for reset events */
    if ( g_sAta5700_flash.events_reset & g_sEventHandling_flash.bReset )
    {
        ATA_systemSetEventPin_flash_ASM();
    }

    /* Temp. initialization of LF RSSI component data for use during System
       Verification */
    g_sLfRssi.bFlags = 0;
    g_sLfRssi.bStatus = 0;
    ATA_lfRssiInit_C();
    
    ATA_lfRssiMeasConfig_flash_C(PARALLEL_LFRSSI_MEASUREMENT);
    
    /* Initialize calibration component */
    ATA_calibInit_C();

    /* initialization of AES module */
    ATA_aesInit_C();

    /* Application Board IO init */
   
    bit_clear(LED1);          //LED off        
    bit_set(LED1_DDR);        //LED port as output
    bit_clear(LED2);    
    bit_set(LED2_DDR);
    
    bit_clear(SW1_DDR);       // Push button as input
    bit_set(SW1);             // set pull-up 
    bit_clear(SW2_DDR);     
    bit_set(SW2); 
    bit_clear(SW3_DDR);     
    bit_set(SW3); 
    
    Intr_Enable(SW1_INTR);    //Enable SW1,2,3 interrupts 
    Intr_Enable(SW2_INTR);
    Intr_Enable(SW3_INTR);
    PCICR |= (1<<PCIE1);      //enable pin change int on PCINT[15:8]
    SPCR &= ~((1<<SPIE) | (1<<SPE)); // Disable SPI port
    SPSR &= ~((1<<SPIF) | (1<<TXIF) | (1<<RXIF));
    
    DDRB = 0;           // all pins on PB are inputs
    PORTB = 7;          // set pull-up on all push buttons 
    
    gFlashApplState.Buttons = 0x00U;//Clear button state
    
    /* ------- Move to IOinit.c someday--------------------------------------*/
    
    /* Enable global interrupts */
    _SEI;
    
    ATA_eepReadBytes_C(PAYLOADDATA, 0xE8, 0x08);
    char volatile count;

    for(;;)
    {
      _WDR;
       ATA_globalsClkSwitchFrc_C();
      /* toggle GPIOR0.2 to indicate main loop frequency */
      GPIOR0 ^= BIT_MASK_2;
      
      if (ID0_Wake == 0x01){
        //*********** Process RSSI ************
        
          for (char i=0; i<5; i++)
          {
           PORTD |= (1<<4);  // Set PD4 high
           PORTD &= ~(1<<4);     // Set PD4 low
           __delay_cycles(3000);    //Tout=1E06 Clks
          }
        uint8_t srcVal =0;
        //ATA_eepReadBytes_C( &srcVal, 0x08AF, 1U );     // Value to be updated at tape out. 
        ATA_lfRssiSetEepromConfig_C( srcVal );        
        ATA_lfRssiOpen_C();       // Check!!! 
        ATA_lfRssiMeasEnableLfReceiver_flash_C();
        g_sLfRssiRegConfig_flash.bRscr = 0x08;
        g_sLfRssiRegConfig_flash.bRsdlyr = 24; //select the RSSI Tracking time
        g_sLfRssiRegConfig_flash.bRsms1r = 0x07; // select all channels
        g_sLfRssiRegConfig_flash.bRsms2r = 0x60; // select number of samples for averaging 64 samples
        g_sLfRssiRegConfig_flash.bRssrcr = 0x00; // no SRC calibration
        do 
        { 
          __no_operation();
        } while ((RSSR & 0x01) ==0);
        ATA_lfRssiMeasStart_C( &g_sLfRssiRegConfig_flash, 0U, 0U );
        __delay_cycles(10);
        PORTD |= (1<<4);  // Set PD4 high
        PORTD &= ~(1<<4);     // Set PD4 low
         
        do 
        {
 
        }
        while ((g_sLfRssi.bStatus & LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG)==0); 
    
        ATA_rfTx_lFRssi_flash_C();                      // Check!!!
        __no_operation();
        
        ATA_lfRssiMeasClose_flash_C();                  // Check!!!
        T0CR = 0;
//        SUPCR=0;
        
        
        PORTD |= (1<<1);   // Set PD3 high (LED1)
        for (char i=0; i<250; i++)
        {
          __delay_cycles(3500);    //Tout=1E06 Clks
        }
        PORTD &= ~(1<<1);     // Set PD3 low
        ID0_Wake = 0x00;
        ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE,&bLFRX_IDLENGTH[0],LFRX_IDLENGTH);
//        ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_L_SENSE,&bLFRX_IDLENGTH[0],LFRX_IDLENGTH);
        
      } 
      if (gFlashApplState.Buttons & BM_NEWCMNDVALID)  //New Button press command available
      {
        //ATA_globalsSwitchMvccRegulator_C(TRUE);        // activate Vmem
        bit_set(LED1);
        gFlashApplState.Buttons &= ~BM_NEWCMNDVALID;  //Clear new command available flag
        for (index = 0; index < 14; index++)
        {
          PAYLOADDATA[index] = 0xFF;
        }
        PAYLOADDATA[14] = 0xFEU; 
        PAYLOADDATA[15] = 0x39U; 
        
        PAYLOADDATA[16] = 0;
        PAYLOADDATA[17] = 0;
        
        PAYLOADDATA[18] = 0;
        PAYLOADDATA[19] = 0;
        
        PAYLOADDATA[20] = 0;
        PAYLOADDATA[21] = 0;
        
        PAYLOADDATA[22] = guiButton; 
        
        for (index = 0; index < 23; index++)
        {
          checksum += PAYLOADDATA[index];
        }
        PAYLOADDATA[23] = ~checksum+1; 
        
        ATA_rfTxInit_C();
        ATA_rfTxFillDFifo_C(0x19U, PAYLOADDATA);
        // start rfTx with eeprom service g_sEepRfTxServiceConfig1
        // __delay_cycles(100);
        //ATA_rfTxStartTx_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
        ATA_rfTxStartTx_C(0x48, (uint8_t *) 0x06D0);
        do {
          ATA_rfTxProcessing_C();
          
        }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
        ATA_rfTxStop_C();      
        SUPCR &= ~BM_AVEN;
         for (char i=0; i<250; i++)
        {
         // __delay_cycles(3500);    //Tout=1E06 Clks
         // __delay_cycles(35);    //Tout=1E06 Clks
        }
        bit_clear(LED1);
        //ATA_globalsSwitchMvccRegulator_C(FALSE);       // deactivate Vmem
        T0CR = 0;
      }
      
      ATA_globalsClkSwitchMrc_C();
      if ((gFlashApplState.Buttons & ~BUTTONFILTERACTIVE) == 0)
      {
      g_bSleepModeConfig_flash = LF_SLEEP_MODE;    // Set Sleep Mode
      }
      /* Check whether sleep mode can be entered. */
     if (
                (        (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE) == 0x00U)              &&
                (  (g_sAta5700_flash.status & BM_ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG) == 0x00U) &&
                (gFlashApplState.Buttons & ~BM_BUTTONFILTERACTIVE) == 0x00U
             
          )
      {
        
        if ( (g_bSleepModeConfig_flash & BIT_MASK_0) != 0x00U )
        {
//          ATA_globalsSwitchMvccRegulator_C(FALSE);       // deactivate Vmem
          /*Enter Sleep using g_bSleepModeConfig_flash variable*/
          ATA_globalsSleep_C(g_bSleepModeConfig_flash);
          
          /* Disable Sleep */
          g_bSleepModeConfig_flash &= ~BIT_MASK_0;
        }
      }
    }
}



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
void ATA_lfRxEnableWakeup_flash_C(uint8_t bLfBdrate,uint8_t bSense,uint8_t * pLf_Id,uint8_t bLf_IdLength)
{
  
  LFQC1 = LFRX_R_Trim90k;
  LFQC2 = LFRX_R_Trim90k;
  LFQC3 = LFRX_R_Trim117k;
  LFCR0 = 0x80 | bLfBdrate | BM_LFCE1 | BM_LFCE2 | BM_LFCE3;    // activate all channels and set baudrate
  LFCR1 = BM_LFRE | BM_LFPEEN;                              // enable RX, ID and Data Mode
  LFCR2 = bSense;                                        // select sensitivity
//  LFCR3 |= 0x01;                                          // at first without trimming function, enable trim function
  
  // Settings for the protocol handler
  PHID00 = 0x11; //*pLf_Id++;
  if (bLf_IdLength > 7)
  { 
    PHID01 = 0x12;//*pLf_Id++;
  }
  if (bLf_IdLength > 15)
  {
    PHID02 = 0x13; //*pLf_Id++;
  }
  if (bLf_IdLength > 23)
  {
    PHID03 = 0x14; //*pLf_Id++;
  }
  PHID0L = bLf_IdLength;
  PHIDFR = bLf_IdLength;
  
  PHTBLR = 0xFF;                // Protocol Handler Telegram bit length
  
  LFSYSY0 = 0x09;
  LFSYLE = 0x04;
  
  PHIMR |= (1<<4);       // Set PHID0IM=1 (ID0 IRQ mask)
  
  //Enable port settings to output
  DDRB |= (1<<3);         // Set DDRB1=1 (out)
  DDRC |= 1 | (1<<1);     // Set PC1=1, PC0=1
  DDRD |= (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5); // Set PD1, PD2, PD3, PD4, PD5
  
  //Set PB[3:0] as outputs and set them high
  DDRB |= (1<<3) | (1<<2) | (1<<1) | 1;
  PORTB |= (1<<3) | (1<<2) | (1<<1) | 1;
  
  //Set PC[1:0] as outputs and set them high
  PORTC |= (1<<1) | 1;
  
  //Set PD[7] and set them high
  DDRD |= (1<<7);
  PORTD |= (1<<7);
  
  
  //Set wake-up event on PC2
  LTEMR |= (1<<ID0EM);   // Set ID0EM event mask to '1'
//  LFCR1 |= (1<<6);  // set LFPEEN (LF port event enable bit) to enable events on PC2 (wakeup flag)
//  DDRC |= (1<<2);   // Enable PC2 as output
 
  // Set SleepMode (during Listen)
  g_bSleepModeConfig_flash = LF_SLEEP_MODE;  
}
//-----------------------------------------------------------------------------
/** \brief <b>ATA_rfTx_lFRssi_flash_C<void>
    Shall configure the 3D LF receiver into LF listen mode and activate
      the ID0 wake-up interrupt

    \param[in] none

    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
void ATA_rfTx_lFRssi_flash_C(void)
{
  uint8_t index;  
  uint8_t checksum;

    // only if one of the buttons are pressed
          PORTD |= (1<<4);  // Set PD4 high (LED2)
          for (index = 0; index < 14; index++)
          {
            PAYLOADDATA[index] = 0xFF;
          }
          PAYLOADDATA[14] = 0xFEU; 
          PAYLOADDATA[15] = 0x39U; 
          
          RSMS1R &= ~(1<<RSSSV);        // for output the everage value
          PAYLOADDATA[16] = RSRES1L;
          PAYLOADDATA[17] = RSRES1H;
          
          PAYLOADDATA[18] = RSRES2L;
          PAYLOADDATA[19] = RSRES2H;
          
          PAYLOADDATA[20] = RSRES3L;
          PAYLOADDATA[21] = RSRES3H;
          
          PAYLOADDATA[22] = 0x18U; 
          
          for (index = 0; index < 23; index++)
          {
            checksum += PAYLOADDATA[index];
          }
          PAYLOADDATA[23] = ~checksum+1; 
         ATA_rfTxInit_C();
          ATA_rfTxFillDFifo_C(0x19U, PAYLOADDATA);
          // start rfTx with eeprom service g_sEepRfTxServiceConfig1
         // __delay_cycles(500);// Hang up occurs aobut here
//          ATA_rfTxStartTx_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
          ATA_rfTxStartTx_C(0x48, (uint8_t *) 0x06D0);//Buffer mode
        //  ATA_rfTxStartTx_C(0x68, (uint8_t *) 0x06D0);// CW mode
          do {
            ATA_rfTxProcessing_C();
             }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
          for (uint8_t cnt=0;cnt<255;cnt++);
          ATA_rfTxStop_C();
          PORTD &= 0xef;     // Set PD4 low         
}


//-----------------------------------------------------------------------------
/** \brief <b>ATA_GetIntRssiValues<void>
    Stores the result of the internal RSSI measurement
    

    \param[in] none

    \return none


    \Traceability None

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/

void ATA_GetIntRssiValues(void)
{
//  gIntRssiValues[0] = g_sLfRssi.wRawLfRssi[0];
//  gIntRssiValues[1] = g_sLfRssi.wRawLfRssi[1];
//  gIntRssiValues[2] = g_sLfRssi.wRawLfRssi[2];
}







