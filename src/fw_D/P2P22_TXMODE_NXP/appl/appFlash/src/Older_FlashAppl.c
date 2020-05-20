/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/Older_FlashAppl.c $
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
#define FLASH_MAJOR_VERSION     0x12U
#define FLASH_MINOR_VERSION     0x34U
#define FLASH_CUSTOMER_VERSION  0x56U

#define LFRX_BDR_1_95           0x00U
#define LFRX_BDR_3_90           0x08U
#define LFRX_BDR_7_81           0x10U

#define LFRX_H_SENSE            0x04U //Setting DAMP allows up to OVP
#define LFRX_M_SENSE            0x01U
#define LFRX_L_SENSE            0x02U

#define LFRX_IDLENGTH           24

#define TX_PREAMBLEDATALENGTH   5
#define TX_PAYLOADDATALENGTH    32

#define conSwitch1              1
#define conSwitch2              2
#define conSwitch3              4


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

extern char ID0_Wake=0x00;
uint8_t PREAMBLEDATA [ TX_PREAMBLEDATALENGTH ]; //5
uint8_t PAYLOADDATA  [ TX_PAYLOADDATALENGTH ]; //32
uint8_t guiButton =0;
uint8_t gucNewButtonAvailable=0;

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
 uint8_t bTempSpcr;  
  uint8_t bLFRX_IDLENGTH[4] ={0x11,0x12,0x13,0x14};  
    uint8_t index;
   /* Use INTVEC at Flash start */
    MCUCR = BM_IVCE;
    MCUCR = BM_IVSEL;

    /* Default EEPROM settings used for different application types exceeds the
       unprotected EEPROM memory space (0x00 - 0xFF). For that reason, protected
       EEPROM memory locations (WP0 ... WP3) are enabled. */
    EEPR = (BM_EEAP3 | BM_EEAP2 | BM_EEAP1 | BM_EEAP0);
     /* Read SPI enable, since it is misused to drive PIN access in case of 
       VBAT is 0. */
    ATA_eepReadByte_C((uint16_t)&g_sEepFlashAppSpi_flash, &bTempSpcr);

    ATA_initAvrRegister_flash_C();

    /* Init global system events before performing */
    ATA_5700InitCommandSet_flash_C();

    /* Store reset events and clear register. */
    g_sAta5700_flash.events_reset = MCUSR;
    MCUSR = 0x00U;

    /* Check port wakeup events */
    if ( ((g_sAta5700_flash.events_reset & BM_TPRF) == 0x00U) ||
         ((bTempSpcr & BM_SPIE) != 0x00U )
       )
    {
        g_sAta5700_flash.events_wakeup = ATA_checkWakeupSource_C();
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
        ATA_lfRxInit_C();

        /* System Verification Requirement, referenced as BUG "Primus2P-2165" */
        LFCPR = BM_LFCPCE;
        LFCPR = BM_LFCALRY;

        /* Primus2P-1827 and Primus2P-1670:
           This is a Flash Validation requirement to set T0 to the given value. */
        T0CR  = 0x12U;
        T0IFR = 0x01U;
        T0CR  |= 0x08;
        
        // Initialize LFREC, Power Mode
        ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE,&bLFRX_IDLENGTH[0],LFRX_IDLENGTH);
    }
    else
    {
        /* Enable clock for LF Receiver to be able to update its registers. */
        ATA_POWERON_C(PRR1, PRLFR);

        /* Set LF Calibration available indication */
        g_sAta5700_flash.events_reset |= BM_ATA5700_EVENTS_RESET_LF_CALRY;
    }

    /* Initialization of LF calibration data */
  //  ATA_lfRxInit_C();

    /* Primus2P-1827 and Primus2P-1670:
    This is a Flash Validation requirement to set T0 to the given value. */
 
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
    g_sLfRssi.bFlags = 0;//FL OK
    g_sLfRssi.bStatus = 0;//FL OK
    ATA_lfRssiMeasInit_flash_C();//FL OK
    
   // ATA_lfRssiMeasConfig_flash_C(SERIAL_LFRSSI_MEASUREMENT);//New GR why serial??
   //   ATA_lfRssiMeasConfig_flash_C(PARALLEL_LFRSSI_MEASUREMENT);FL moved
    


    /* Initialize calibration component */
    ATA_calibInit_C();

    /* initialization of AES module */
    ATA_aesInit_C();

    DDRB = 0;           // all pins on PB are inputs
    PORTB = 7;          // set pull-up on all push buttons 
    
    /* enable pin change interrupts */
    PCMSK0 = 7;         // enable pin change int for PB2, PB1 and PB0
    PCICR = 1;          // enable pin change int on PCINT[7:0]


    /* Enable global interrupts */
    _SEI;
    
    char volatile count;

    for(;;)
    {
        _WDR;
        /* toggle GPIOR0.2 to indicate main loop frequency */
        GPIOR0 ^= BIT_MASK_2;
        
        //Set LED on PD3 to ON state
          if (ID0_Wake == 0x01){
          //*********** Process RSSI ************
      
          ATA_globalsWdtDisable_C();
          ATA_lfRssiMeasOpen_C();
          ATA_lfRssiMeasStartExt_C(0x3b, 0xc0);
          while ((g_sLfRssi.bFlags & LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG)==0);//Wait here 
          if ( g_sEventHandling_flash.bPhRxTx0 & BM_LFRXCONFIG_PH_FLAGS_0_PHID0F )
          {
          ATA_systemSetEventPin_flash_ASM();
          }
          ATA_rfTx_lFRssi_flash_C();
          ID0_Wake = 0x00;// Check variables in debug here
          ATA_lfRssiMeasClose_C();
         
          //*********** Process RSSI ************  
                                        
          PORTD |= (1<<3);   // Set PD3 high
          for (char i=0; i<250; i++)
          {
            __delay_cycles(3500);    //Tout=1E06 Clks
          }
          PORTD &= ~(1<<3);     // Set PD3 low
          ID0_Wake = 0x00;
           ATA_lfRxEnableWakeup_flash_C(LFRX_BDR_3_90,LFRX_H_SENSE,&bLFRX_IDLENGTH[0],LFRX_IDLENGTH);//GR
           ATA_globalsWdtEnable_C(0x03U);
        }
        
        if (gucNewButtonAvailable != 0)
        {
          // only if one of the buttons are pressed
          PORTD |= (1<<4);  // Set PD4 high
          gucNewButtonAvailable =0;
          for (index = 0; index < 3; index++)
          {
            PAYLOADDATA[index] = 0xFF;
          }
          PAYLOADDATA[3] = 0xFE; 
          for (index = 4; index < 15; index++)
          {
            PAYLOADDATA[index] = index;
           // PAYLOADDATA[index] = g_sLfRssiData_flash.wLfRssiCoilsComp[index];
          }
          ATA_rfTxFillDFifo_C(0x0EU, PAYLOADDATA);
          // start rfTx with eeprom service g_sEepRfTxServiceConfig1
          ATA_rfTxStartTx_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
          do {
            ATA_rfTxProcessing_C();
          }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
          ATA_rfTxStop_C();          
          PORTD &= ~(1<<4);     // Set PD4 low
        }
        
        
          
                    

        

        /* */
//        ATA_performEventHandling_flash_C();
        
        /**********************
        SMCR
        0x01 -> Idle
        0x05 -> Power Down
        0x09 -> Ext. Power Down /PaLe
       ************************/
 //       g_bSleepModeConfig_flash = 0x05;    // Set Sleep Mode (must have the LSB set to enter sleep!)
        
        /* Check whether sleep mode can be entered. */
        if ( ( (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE) == 0x00U) &&
             ( (g_sAta5700_flash.status & BM_ATA5700_STATUS_SPI_CMD_IN_PROGRESS_FLAG) == 0x00U) )
        {

            if ( (g_bSleepModeConfig_flash & BIT_MASK_0) != 0x00U )
            {
                /*Enter Sleep using g_bSleepModeConfig_flash variable*/
                ATA_globalsSleep_C(g_bSleepModeConfig_flash);

                /* Disable Sleep */
                g_bSleepModeConfig_flash &= ~BIT_MASK_0;
            }
        }
    }
}


/**/
//static VOIDFUNC ATA_performEventHandling_flash_C(void)
//{
//    /* Update ATA 5700 event bytes regarding RF Tx component and clear
//       component event information.*/
//    _CLI;
//    g_sAta5700_flash.events_rf_flags_0 |= g_sRfTx.bFlags;
//    g_sAta5700_flash.events_rf_flags_1 |= g_sRfTx.bTuneFlags;
//    g_sAta5700_flash.events_config = trxConf.serviceChannelConfig;
//
//    if ( (g_sRfTx.bFlags & g_sEventHandling_flash.bRfTx0) ||
//         (g_sRfTx.bTuneFlags & g_sEventHandling_flash.bRfTx1))
//    {
//        ATA_systemSetEventPin_flash_ASM();
//    }
//
//    /* Clear component flags, otherwise the event pin is triggered more than
//       once for the same action. */
//    g_sRfTx.bFlags = 0x00U;
//    g_sRfTx.bTuneFlags = 0x00U;
//    _SEI;
//
//    /* Update ATA 5700 event bytes regarding Protocol Handler (FIFO)
//       component.*/
//    _CLI;
//    g_sAta5700_flash.events_ph_flags_0 |= g_sLfRx_flash.bPhFlags0 ;
//    g_sAta5700_flash.events_ph_flags_1 |= g_sLfRx_flash.bPhFlags1;
//    g_sLfRx_flash.bPhFlags0 = 0x00U;
//    g_sLfRx_flash.bPhFlags1 = 0x00U;
//    _SEI;
//
//    /* Update ATA 5700 event bytes regarding 3D LF Receiver component.*/
//    _CLI;
//    g_sAta5700_flash.events_lf_flags |= g_sLfRx_flash.bLfFlags;
//    g_sLfRx_flash.bLfFlags = 0x00U;
//    _SEI;
//
//    /* Update ATA 5700 event bytes regarding 3D LF Receiver RSSI component.*/
//    _CLI;
//    if ( ((g_sLfRssi.bFlags & LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG) != 0x00U) &&
//         ((g_sLfRssi.bStatus & LFRSSI_STATUS_BM_CHANNELS_TIMEOUT_FLAG) == 0x00U)
//        )
//    {
//        g_sAta5700_flash.events_lf_flags |= BM_LFRXCONFIG_LF_FLAGS_RSSI_RDY;
//
//        if ( g_sEventHandling_flash.bLf3dRx & BM_LFRXCONFIG_LF_FLAGS_RSSI_RDY )
//        {
//            ATA_systemSetEventPin_flash_ASM();
//        }
//    }
//    else if ( ((g_sLfRssi.bFlags & LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG) != 0x00U) &&
//              ((g_sLfRssi.bStatus & LFRSSI_STATUS_BM_CHANNELS_TIMEOUT_FLAG) != 0x00U)
//             )
//    {
//        g_sAta5700_flash.events_lf_flags |= BM_LFRXCONFIG_LF_FLAGS_RSSI_ERR;
//
//        if ( g_sEventHandling_flash.bLf3dRx & BM_LFRXCONFIG_LF_FLAGS_RSSI_ERR )
//        {
//            ATA_systemSetEventPin_flash_ASM();
//        }
//    }
//    else
//    {
//        /* Do nothing. */
//    }
//
//    /* Clear component flags, otherwise the event pin is triggered more than
//       once for the same action. */
//    g_sLfRssi.bFlags &= (~LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG);
//    g_sLfRssi.bFlags &= (~LFRSSI_FLAGS_BM_ERROR_FLAG);
//
//    _SEI;
//
//    /* Update ATA 5700 event bytes regarding Transponder component.*/
//    _CLI;
//    /* Transponder and 3D LF Receiver share the CRC bit error information
//       CRCEF within the PH event data. */
//    g_sAta5700_flash.events_tp_flags |= g_sTpRxTx.bTpFlags;
//    g_sTpRxTx.bTpFlags = 0x00U;
//    _SEI;
//}

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
  
  LFCR0 = 0xc0 | bLfBdrate | BM_LFCE1 | BM_LFCE2 | BM_LFCE3;    // activate all channels and set baudrate
  LFCR1 = BM_LFRE | BM_LFPEEN;                              // enable RX, ID and Data Mode
  LFCR2 = bSense;                                        // select sensitivity
  //LFCR3 = ;                                            // at first without trimming function
  LFIMR = BM_LFEOIM;                                     // enable end of telegram interrupt
  LFDSR1 = BM_CTTHA1 | BM_HITHA0 | BM_LOTHA1;            // set decoder threshold to default values
  LFDSR2 = BM_CTTHB0 | BM_HITHB0 | BM_LOTHB1;
  LFDSR3 = BM_QCTH2 | BM_PBDTH1; 
  LFDSR4 = BM_SRSTC1 | BM_SCTHA0 | BM_SDTHA2; 
  LFDSR5 = 0x0B;                                         // select Atmel preburst detection
  LFDSR6 = 0x09; 
  LFDSR7 = 0xA5;
  LFDSR8 = 0x3C;
  LFDSR9 = 0x10; 
  LFDSR10 = 0x54;
  LFDSR11 = 0x22; 
  
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
    PHID03 = 0x00; //*pLf_Id++;
  }
  PHID0L = bLf_IdLength;
  
  PHTBLR = 0xFF;
  
  LFSYSY0 = 0x09;
  LFSYLE = 0x04;
  SRCCAL = 0x73;
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
  
  //Enable PB3/LFES as a debug port
  //MCUCR |= (1<<3);
  //DBGSW = 10;                       // Enable to drive debug test 10
  
  //Set wake-up event on PC2
  PHTEMR |= 0x01;   // Set ID0EM event mask to '1'
  LFCR1 |= (1<<6);  // set LFPEEN (LF port event enable bit) to enable events on PC2 (wakeup flag)
//  DDRC |= (1<<2);   // Enable PC2 as output
 
  // Set SleepMode (during Listen)
  g_bSleepModeConfig_flash = 0x05;    // 8b0000_0101 -> Power Down Mode
}



//-----------------------------------------------------------------------------
/** \brief <b>ATA_pinChangeInterrupt0Handler_ISR_FLASH</b>
    Called from interrupt 
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
     guiButton = 3;
   break;
   }

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
          PORTD |= (1<<4);  // Set PD4 high
          for (index = 0; index < 14; index++)
          {
            PAYLOADDATA[index] = 0xFF;
          }
          PAYLOADDATA[14] = 0xFEU; 
          PAYLOADDATA[15] = 0x39U; 
          
          PAYLOADDATA[16] = (g_sLfRssi.wRawLfRssi[0]>>8)&0xffU;
          PAYLOADDATA[17] = g_sLfRssi.wRawLfRssi[0]&0xffU;
          
          PAYLOADDATA[18] = (g_sLfRssi.wRawLfRssi[1]>>8)&0xffU;
          PAYLOADDATA[19] = g_sLfRssi.wRawLfRssi[1]&0xffU;
          
          PAYLOADDATA[20] = (g_sLfRssi.wRawLfRssi[2]>>8)&0xffU;
          PAYLOADDATA[21] = g_sLfRssi.wRawLfRssi[2]&0xffU;
          
          PAYLOADDATA[22] = 0x18U; 
          
          for (index = 0; index < 23; index++)
          {
            checksum += PAYLOADDATA[index];
          }
          PAYLOADDATA[23] = ~checksum+1; 
          ATA_rfTxFillDFifo_C(0x19U, PAYLOADDATA);
          // start rfTx with eeprom service g_sEepRfTxServiceConfig1
          ATA_rfTxStartTx_C(0x48, (uint8_t *) &g_sEepRfTxServiceConfig1);
          do {
            ATA_rfTxProcessing_C();
          }while (g_sRfTx.bStatus & BM_RFTXCONFIG_BSTATUS_ACTIVE);
          ATA_rfTxStop_C();
          PORTD &= ~(1<<4);     // Set PD4 low
}