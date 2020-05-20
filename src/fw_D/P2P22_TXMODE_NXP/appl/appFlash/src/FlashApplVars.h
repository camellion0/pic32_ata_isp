/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplVars.h $
  $LastChangedRevision: 591974 $
  $LastChangedDate: 2020-03-16 09:23:12 -0600 (Mon, 16 Mar 2020) $
  $LastChangedBy: grueter $
-------------------------------------------------------------------------------
  Project:      FlashApplVars
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.30.1
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
/** \file FlashApplVars.h
 */

//lint -restore

#ifndef FLASHAPPLVARS_H
#define FLASHAPPLVARS_H

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
//#include "../../stdc/src/stdc.h"
//#include "../../globals/src/globals.h"
//#include "../../rftx/src/rftx_defs.h"
#include "FlashApplPEPS.h"
#include "utils.h"
#include "micro.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
   
/* Software identifiers*/
//#ifdef CFG_APP_2WAYRF
//uint8_t SW_ID[]={0x57,0x02,0x02/*Device*/,0x02/*Kitphase*/,0x02/*RFdirection*/,0x00/*Rev Major/Minor*/};         
//#else // ONE_WAY
//uint8_t SW_ID[]={0x57,0x02,0x02/*Device*/,0x02/*Kitphase*/,0x01/*RFdirection*/,0x01/*Rev Major/Minor*/};   
//#endif


/* ------------------------------------------------------------------------- */
/* g_FlashApplVars                                                           */
/* ------------------------------------------------------------------------- */
#define BUTTON_PROC_ACTIVE_FLAG          BIT_7
#define LF_ACTIVE_FLAG                   BIT_6
#define RF_ACTIVE_FLAG                   BIT_6

#define BM_BUTTON_PROC_ACTIVE_FLAG       BITMASK(BUTTON_PROC_ACTIVE_FLAG)
#define BM_LF_ACTIVE_FLAG                BITMASK(LF_ACTIVE_FLAG)
#define BM_LF_ACTIVE_FLAG                BITMASK(LF_ACTIVE_FLAG)

/*----------------------- IO -------------------------------------------------*/
#define	bit_set(...)                     bit_set_(__VA_ARGS__)
#define	bit_set_(x,y)                    x |= 1<<y		// set a bit
#define	bit_clear(...)                   bit_clear_(__VA_ARGS__)
#define	bit_clear_(x,y)                  x &= ~(1<<y)		// clear a bit
#define	bit_test(...)                    bit_test_(__VA_ARGS__)
#define	bit_test_(x,y)                   (!!(x & (1<<y)))	// test a bit

#define	Intr_Enable(...)                 bit_set_(__VA_ARGS__)
#define	Intr_Enable_(x,y)                x |= 1<<y		// Enable Intr
#define	Intr_Disable(...)                bit_clear_(__VA_ARGS__)
#define	Intr_Disable_(x,y)               x &= ~(1<<y)		// Disable Intr


/*------------- IO usage examples-----------------------------------------------
bit_set( LED1);        // Port set
bit_set( LED1_DDR );   // Port direction set
if( bit_test( KEY0 ))
      bit_set( LED0 );
    else
      bit_clear( LED0 );
------------------------------------------------------------------------------*/
/*
#define SWITCH1                         BIT_3
#define SWITCH2                         BIT_4
#define SWITCH3                         BIT_5
*/

#define SWITCH1                         BIT_5
#define SWITCH2                         BIT_6
#define SWITCH3                         BIT_7
/*
#define	LED1                             PORTD, 1
#define	LED1_DDR                         DDRD, 1
#define	LED2                             PORTD, 2
#define	LED2_DDR                         DDRD, 2

#define	SW1                              PORTD, 3
#define	SW1_DDR                          DDRD, 3
#define	SW2                              PORTD, 4
#define	SW2_DDR                          DDRD, 4
#define	SW3                              PORTD, 5
#define	SW3_DDR                          DDRD, 5
*/

#define	LED1                             PORTC, 0
#define	LED1_DDR                         DDRC, 0
#define	LED2                             PORTC, 1
#define	LED2_DDR                         DDRC, 1

#define	SW1                              PORTD, 5
#define	SW1_DDR                          DDRD, 5
#define	SW2                              PORTD, 6
#define	SW2_DDR                          DDRD, 6
#define	SW3                              PORTD, 7
#define	SW3_DDR                          DDRD, 7

/* for Amazon transparent mode TX application  */
#define CLKOUT                           PORTD,3
#define CLKOUT_DDR                       DDRD, 3
#define TMDI_STATE                       PORTD,4
#define TMDI_DDR                         DDRD, 4
//#define TXACTIVE                         PORTD,1
//#define TXACTIVE_DDR                     DDRD, 1

#define TXACTIVE                         PORTD,2
#define TXACTIVE_DDR                     DDRD, 2



#define SW_BM                            0x1f


//#define SW_BM                            0xc7


/*
#define SW1_INTR                         PCMSK1,3
#define SW2_INTR                         PCMSK1,4
#define SW3_INTR                         PCMSK1,5
*/
#define SW1_INTR                         PCMSK1,5
#define SW2_INTR                         PCMSK1,6
#define SW3_INTR                         PCMSK1,7
/* ------------------------------------------------------------------------- */
/**/
#define NEWCMNDVALID                     BIT_7
#define BUTTONFILTERON                   BIT_6
#define BUTTONPROCCESSINGACTIVE          BIT_5
#define BUTTONDATAVALID                  BIT_4
#define BUTTON1ACTIVE                    BIT_2
#define BUTTON2ACTIVE                    BIT_1
#define BUTTON3ACTIVE                    BIT_0        

#define BM_NEWCMNDVALID                 BIT_MASK_7
#define BM_BUTTONFILTERON               BIT_MASK_6
#define BM_BUTTONPROCCESSINGACTIVE      BIT_MASK_5
#define BM_BUTTONDATAVALID              BIT_MASK_4
#define BM_BUTTON1ACTIVE                BIT_MASK_2
#define BM_BUTTON2ACTIVE                BIT_MASK_1
#define BM_BUTTON3ACTIVE                BIT_MASK_0

#define MSGTIMER_ACTIVE                 BIT_7
#define RKETIMER_ACTIVE                 BIT_6
#define PEPSRFTIMERACTIVE               BIT_5
#define BM_MSGTIMER_ACTIVE              BITMASK(MSGTIMER_ACTIVE)
#define BM_RKETIMER_ACTIVE              BITMASK(RKETIMER_ACTIVE)
#define BM_PEPSRFTIMERACTIVE            BITMASK(PEPSRFTIMERACTIVE)

#define RFTXCRCACTIVE                   BIT_4
#define BM_RFTXCRCACTIVE                BIT_MASK_4
#define RFTXSTOPACTIVE                  BIT_3
#define BM_RFTXSTOPACTIVE               BIT_MASK_3
    
#define RKETIMER4_ACTIVE                 BIT_2
#define BM_RKETIMER4_ACTIVE              BITMASK(RKETIMER_ACTIVE)

#define SPI_received_flag                BIT_0
#define BM_SPI_received_flag             BITMASK(SPI_received_flag)
    
#define BM_INT0TRUE                      BIT_MASK_3

/* g_sAta5700_flash.rfFlags */
#define ATA5700_EVENTS_RFFLAGS_ANT_TUNE_RDY         BIT_7
#define ATA5700_EVENTS_RFFLAGS_VCO_TUNE_RDY         BIT_6
#define ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY        BIT_5
#define ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY        BIT_4

#define BM_ATA5700_EVENTS_RFFLAGS_ANT_TUNE_RDY      BITMASK(ATA5700_EVENTS_RFFLAGS_ANT_TUNE_RDY)
#define BM_ATA5700_EVENTS_RFFLAGS_VCO_TUNE_RDY      BITMASK(ATA5700_EVENTS_RFFLAGS_VCO_TUNE_RDY)
#define BM_ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY     BITMASK(ATA5700_EVENTS_RFFLAGS_FRC_CALIB_RDY)
#define BM_ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY     BITMASK(ATA5700_EVENTS_RFFLAGS_SRC_CALIB_RDY)

/* g_sAta5700_flash.events_reset */
#define ATA5700_EVENTS_RESET_LF_CALRY               BIT_7

#define ATA5700_EVENTS_RESET_LF_TPRF                BIT_5
#define ATA5700_EVENTS_RESET_LF_JTRF                BIT_4
#define ATA5700_EVENTS_RESET_LF_WDRF                BIT_3

#define ATA5700_EVENTS_RESET_LF_EXTRF               BIT_1
#define ATA5700_EVENTS_RESET_LF_PORF                BIT_0

#define TX_PAYLOADDATALENGTH                        32
#define RKE_RFMESSAGELENGTH                         29
#define RKE_RF_MSG_PREAMPLE_LEN                     16
#define RKE_RF_LEARN_MSG_LEN                        24

   
// size of LF PEPS messages
#define PEPS_SIZE_RX_MSG_COMMON   (1+CFG_PEPS_VID_LENGTH)
#define PEPS_SIZE_RX_MSG_ID       (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_RD_ID_TS))
#define PEPS_SIZE_RX_MSG_PARAM_RD (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_RD_PARAM_TS))
#define PEPS_SIZE_RX_MSG_PARAM_WR (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_WR_PARAM_TS))
#define PEPS_SIZE_RX_MSG_2WAY     (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_RD_ID_TS))
#define PEPS_SIZE_RX_MSG_AUTH_UNI (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_AUTH_UNI_TS))
#define PEPS_SIZE_RX_MSG_AUTH_BI  (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_AUTH_BI_TS))
#define PEPS_SIZE_RX_MSG_LF_TST   (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_LF_TST_TS))
#define PEPS_SIZE_RX_MSG_LF_PARAM (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_LF_PARAM_TS))
#define PEPS_SIZE_RX_MSG_SWID     (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_SWID_TS))
#define PEPS_SIZE_RX_MSG_MODE     (PEPS_SIZE_RX_MSG_COMMON+sizeof(RX_MSG_PEPS_MODE_TS))


// dynamic parameters for configuration 0
#define CFG_RF0_POWER         12.5
#define CFG_RF0_MODULATION    ASK
#define CFG_RF0_BITRATE       1000
#define CFG_RF0_CODING        CODING_MAN
#define CFG_RF0_DEVIATION     0
#define CFG_RF0_FREQUENCY     CFG_RF_FREQUENCY

// dynamic parameters for configuration 1
#define CFG_RF1_POWER         12.5
#define CFG_RF1_MODULATION    FSK
#define CFG_RF1_BITRATE       4000//9600
#define CFG_RF1_CODING        CODING_MAN
#define CFG_RF1_DEVIATION     CFG_RF_DEVIATION
#define CFG_RF1_FREQUENCY     CFG_RF_FREQUENCY


// type of rf config
#define CFG_RF_LONG_RANGE     0
#define CFG_RF_MEDIUM_RANGE   1

//#define DIAG_OFF  0
//#define DIAG_OEM  1
//#define DIAG_AS   2

   
// Config mode OEM / AS
#define NTE_DIAG_MODE     (CRAM_DATA(te_diag_mode))

/* ------------------------------------------------------------------------- */
/* g_ApplEEPROMVars                                                          */
/* ------------------------------------------------------------------------- */
#define MARGIN_EEADR            0x007F           
#define eul_key_id              0x8B0           //key unique ID addr
#define eaub_param              0x0D0           //start address of parameter block // 0x090

#define AES_PADD_PATTERN  0x00
#define PARAMBLOCK_SIZE   15
/*
#define LF_AXIS_X       0
#define LF_AXIS_Y       1
#define LF_AXIS_Z       2
*/
#define LF_AXIS_X       2
#define LF_AXIS_Y       1
#define LF_AXIS_Z       0

#define CFG_APP_OEM_CODE    0x0E0EC0DE
#define CFG_APP_AS_CODE     0xA5A5C0DE
// RF Frequency (in Hz)
#define CFG_RF_FREQUENCY   433920000
// RF Deviation in Hz (in case of FSK emission)
// FSK_low_freq = CFG_RF_FREQUENCY-CFG_RF_DEVIATION
// FSK_high_freq = CFG_RF_FREQUENCY+CFG_RF_DEVIATION
#define CFG_RF_DEVIATION   25000
// dynamic parameters for configuration 0
#define CFG_RF0_POWER         12.5
#define CFG_RF0_MODULATION    ASK
#define CFG_RF0_BITRATE       1000
#define CFG_RF0_CODING        CODING_MAN
#define CFG_RF0_DEVIATION     0
#define CFG_RF0_FREQUENCY     CFG_RF_FREQUENCY
// dynamic parameters for configuration 1
#define CFG_RF1_POWER         12.5
#define CFG_RF1_MODULATION    FSK
#define CFG_RF1_CODING        CODING_MAN
#define CFG_RF1_DEVIATION     CFG_RF_DEVIATION
#define CFG_RF1_FREQUENCY     CFG_RF_FREQUENCY
// type of rf config
#define CFG_RF_LONG_RANGE     0
#define CFG_RF_MEDIUM_RANGE   1
// list of PEPS LF/RF command ID
#define PEPS_CID_RD_ID        0
#define PEPS_CID_RD_PARAM     1
#define PEPS_CID_WR_PARAM     2
#define PEPS_CID_2WAY         4
#define PEPS_CID_UNI_AUTH     5
#define PEPS_CID_BI_AUTH      6
#define PEPS_CID_UNI_AUTH_SINGLE  7
#define PEPS_CID_BI_AUTH_SINGLE   8
#define PEPS_CID_LF_TST      10
#define PEPS_CID_LF_PARAM    11
#define PEPS_CID_SWID        14
#define PEPS_CID_MODE        15

// Number of Fobs managed in system (max 8)
#ifdef CFG_APP_2WAYRF
#define PEPS_NB_FOB_MAX        8
#else
#define PEPS_NB_FOB_MAX        4
#endif

// number of channels max 
#define PEPS_NB_CHANNEL_MAX    3

// pointer on RX buffer (either LF or RF buffer)
#define MSG_RX_DATA (g_MsgRXbuffer.aub_data)
// data count received
#define MSG_RX_CNT (g_MsgRXbuffer.ub_size)

#define MSG_TX_DATA  (*(TX_MSG_TU*)(g_MsgTXbuffer.aub_data))

#define SLOT_COMMON   0x01
#define SLOT_SINGLES  0x02

//#define INTERFRAME    4.0//GeRu Was 3.0

#define PEPS_CID_RD_ID        0
#define PEPS_CID_RD_PARAM     1
#define PEPS_CID_WR_PARAM     2
#define PEPS_CID_2WAY         4
#define PEPS_CID_UNI_AUTH     5
#define PEPS_CID_BI_AUTH      6
#define PEPS_CID_UNI_AUTH_SINGLE  7
#define PEPS_CID_BI_AUTH_SINGLE   8
#define PEPS_CID_LF_TST      10
#define PEPS_CID_LF_PARAM    11
#define PEPS_CID_SWID        14
#define PEPS_CID_MODE        15

#define TRANSPARENT_TX_DURATION_MS 500  //Added for Amazon protoype with NXP chip providing TX modulating baseband signal on TMDI (PD4). 

/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------- */
typedef struct {
   
    /** \brief <b>events_system</b>
        contains the event flags with ATA5700 system application information
        \li Bit 7:    rfu
        \li Bit 6:    LFactive
        \li Bit 5:    RFactive
        \li Bit 4:    rfu
        \li Bit 3:    rfu
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    rfu
     */
    uint8_t System;
    
    /** \brief <b>events_wakeup</b>
        contains the event flags with powerOn nPowerOn information
        \li Bit 7:    MsgTimerACtive
        \li Bit 6:    RkeTImerActive
        \li Bit 5:    PepsRFtimeractive
        \li Bit 4:    RFTXcrcActive
        \li Bit 3:    RFTXstopactive
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    SPIreceived
     */
    uint8_t State;
    
    /** \brief <b>events_rf_flags_0</b>
        contains the event flags with ATA5700 RF TX information 
        (see rfTx component)
        \li Bit 7:    
        \li Bit 6:    rfu
        \li Bit 5:    rfu
        \li Bit 4:    rfu
        \li Bit 3:    rfu
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    rfu
    */
    uint8_t Ports;
    
    /** \brief <b>events_rf_flags_1</b>
        contains the event flags with ATA5700 RF TX information 
        (see rfTx component)
        \li Bit 7:    NEWCMNDVALID
        \li Bit 6:    BUTTONFILTERON
        \li Bit 5:    BUTTONPROCESSINGACTIVE
        \li Bit 4:    BUTTONDATAVALID
        \li Bit 3:    INT0ACTIVE
        \li Bit 2:    BUTTON1ACTIVE
        \li Bit 1:    BUTTON2ACTIVE
        \li Bit 0:    BUTTON2ACTIVE
    */
    uint8_t Buttons;
    
}sFlashApplState;

typedef struct {
  
    uint8_t System;   
    uint8_t RKEcommand;  
    uint8_t SPIcount;
    //uint8_t RfTxbuffer[TX_PAYLOADDATALENGTH];
    
}sFlashApplVars;

//Sign Detect register variables
typedef struct {
  
    uint8_t SD12result;   
    uint8_t SD13result;  
    uint8_t SD23result;
    uint8_t SD360result;
    
}sFlashApplignSignDetect;

// CRAM data structure
typedef struct
{
  uint8_t ub_bat;
  uint32_t ul_rolling_code;
  DIAG_MODE_TE te_diag_mode;
  uint8_t ub_btn;
} CRAM_DATA_TS;

// struture containing CRAM variables content and its checksums
typedef struct
{
  CRAM_DATA_TS data;
  uint8_t ub_cks;
  uint8_t ub_cks2;
} CRAM_TS;


// CRAM data
//NO_INIT_DATA CRAM_TS rts_cram;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
uint8_t ATA_Flash_RKEbuttonfilter(uint8_t ButtonState, uint8_t Cnt);
void ATA_RKEtimerProcess(void);
void ATA_RKEtimerStart(void);
void ATA_RKEtimer4Start();

void ATA_transTXtimerStart(uint16_t mscount);
uint8_t ATA_transTXtimerProcess();
void ATA_Flash_RKEbuttonProcessOut(void);
void ATA_FLashAppTimer5Start_C(uint16_t Timer5CompareCnt, uint8_t Timer5Config);
void ATA_FLashAppTimer4Start_C(uint16_t Timer4CompareCnt, uint8_t Timer4Config, uint8_t timer4ModeA);
void ATA_FlashApplTimer5Process_C(void);
void ATA_FlashApplTimer4Process_C(void);
void ATA_PEPStimerStart(uint8_t mscount);
void ATA_PEPStimerProcess(void);
//void ATA_rfTx_PEPSbuildmsg_flash_C(void);
void ATA_rfTx_PEPSmsg_flash_C(void);
void ATA_rfTx_PEPSrftimingprocess_flash_C(void);
void CRCenable(void);
void CRCdisable(void);
void Stopbyteenable(void);
UINT8FUNC ATA_spiOpenSlaveFlash_C(uint8_t bSpcr, uint8_t bSpsr);
VOIDFUNC ATA_spiCloseSlaveFlash_C(void);
VOIDFUNC ATA_spiReceveDataFlash_C();
#endif /* FlashApplVars.h */

