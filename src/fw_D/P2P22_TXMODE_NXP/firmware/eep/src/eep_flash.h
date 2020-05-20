//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/eep/src/eep_flash.h $
  $LastChangedRevision: 460605 $
  $LastChangedDate: 2017-05-19 04:36:10 -0600 (Fri, 19 May 2017) $
  $LastChangedBy: krishna.balan $
-------------------------------------------------------------------------------
  Project:      ATA5700
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
/** \file eep_flash.h
*/

//lint -restore

#ifndef EEP_FLASH_H
#define EEP_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../globals/src/globals.h"
#include "../../lfrssi/src/lfrssi.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/* */
#define ATA5700_EVENTS_CONFIG_EVENTPIN_POL          BIT_0

#define BM_ATA5700_EVENTS_CONFIG_EVENTPIN_POL       BIT_MASK_0

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
// structure for eeprom section .eep_FlashApp_EventHandling
typedef struct {
    /** \brief <b>bConfig</b>
        \li Bit 7:  rfu
        \li Bit 6:  rfu
        \li Bit 5:  rfu
        \li Bit 4:  rfu
        \li Bit 3:  rfu
        \li Bit 2:  rfu
        \li Bit 1:  rfu
        \li Bit 0:  IRQ_POL
    */
    uint8_t bConfig;

    /** \brief <b>bSystem</b>
        \li Bit 7:  SYS_ERR
        \li Bit 6:  SYS_RDY
        \li Bit 5:  AVCC_LOW
        \li Bit 4:  LOWBATT
        \li Bit 3:  rfu
        \li Bit 2:  rfu
        \li Bit 1:  rfu
        \li Bit 0:  T0F
    */
    uint8_t bSystem;

    /** \brief <b>bWakeup</b>
        \li Bit 7:  PWON
        \li Bit 6:  NPWON6
        \li Bit 5:  NPWON5
        \li Bit 4:  NPWON4
        \li Bit 3:  NPWON3
        \li Bit 2:  NPWON2
        \li Bit 1:  NPWON1
        \li Bit 0:  NPWON0
    */
    uint8_t bWakeup;

    /** \brief <b>bRfTx0</b>
        \li Bit 7:  TX_ERR
        \li Bit 6:  RX_RDY
        \li Bit 5:  rfu
        \li Bit 4:  DFIFO_ERR
        \li Bit 3:  SFIFO_ERR
        \li Bit 2:  DFL_BUF_FILLLVL
        \li Bit 1:  SFL_BUF_FILLLVL
        \li Bit 0:  EOT
    */
    uint8_t bRfTx0;

    /** \brief <b>bRfTx1</b>
        \li Bit 7:  ANT_TUNE_RDY
        \li Bit 6:  VCO_TUNE_RDY
        \li Bit 5:  rfu
        \li Bit 4:  rfu
        \li Bit 3:  rfu
        \li Bit 2:  rfu
        \li Bit 1:  rfu
        \li Bit 0:  rfu
    */
    uint8_t bRfTx1;

    /** \brief <b>bPhRxTx0</b>
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: PHID1F
        \li Bit 4: PHID0F
        \li Bit 3: PHIDFF
        \li Bit 2: PHDFF
        \li Bit 1: PHTBLF
        \li Bit 0: CRCEF
    */
    uint8_t bPhRxTx0;

    /** \brief <b>bPhRxTx1</b>
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: rfu
        \li Bit 2: OFLF
        \li Bit 1: UFLF
        \li Bit 0: FLRF
    */
    uint8_t bPhRxTx1;

    /** \brief <b>bLf3dRx</b>
        \li Bit 7: RSSI_ERR
        \li Bit 6: RSSI_RDY
        \li Bit 5: rfu
        \li Bit 4: LF_RDY
        \li Bit 3: LFTOF
        \li Bit 2: LFEOF
        \li Bit 1: LFDEF
        \li Bit 0: LFSYDF
    */
    uint8_t bLf3dRx;

    /** \brief <b>tpRxTx</b>
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: TPBERF
        \li Bit 2: TPNFTF
        \li Bit 1: TPFTF
        \li Bit 0: TPF
    */
    uint8_t bTpRxTx;

    /** \brief <b>bComponents</b>
        contains the component event flags
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: AES_ERR
        \li Bit 2: AES_RDY
        \li Bit 1: I2C_ERR
        \li Bit 0: I2C_RDY
    */
    uint8_t bComponents;

    /** \brief <b>bReset</b>
        contains the reset information of register MCUSR after power-up
        \li Bit 7: LFCALRY
        \li Bit 6: rfu
        \li Bit 5: TPRF
        \li Bit 4: JTRF
        \li Bit 3: WDRF
        \li Bit 2: rfu
        \li Bit 1: EXTRF
        \li Bit 0: PORF
    */
    uint8_t bReset;

    /** \brief <b>bPinChangePortB</b>
        contains the pin change port B event pin configuration
        \li Bit 7: PB7
        \li Bit 6: PB6
        \li Bit 5: PB5
        \li Bit 4: PB4
        \li Bit 3: PB3
        \li Bit 2: PB2
        \li Bit 1: PB1
        \li Bit 0: PB0
    */
    uint8_t bPinChangePortB;

    /** \brief <b>bPinChangePortD</b>
        contains the pin change port D event pin configuration
        \li Bit 7: PD7
        \li Bit 6: PD6
        \li Bit 5: PD5
        \li Bit 4: PD4
        \li Bit 3: PD3
        \li Bit 2: PD2
        \li Bit 1: PD1
        \li Bit 0: PD0
    */
    uint8_t bPinChangePortD;

}sEepFlashAppEventHandling;

// structure for eeprom section .eep_FlashApp_Aes

// structure for eeprom section .eep_FlashApp_Chflt

// structure for eeprom section .eep_FlashApp_Clk
typedef struct{
    uint8_t PRR0;
    uint8_t PRR1;
    uint8_t PRR2;
    uint8_t CLKOD;
    uint8_t CLKOCR;
    uint8_t CMOCR;
}sEepFlashAppClk;

// structure for eeprom section .eep_FlashApp_Cpu
typedef struct {
    uint8_t MCUCR;
    uint8_t SMCR;
}sEepFlashAppCpu;

// structure for eeprom section .eep_FlashApp_Crc

// structure for eeprom section .eep_FlashApp_Debounce
typedef struct {
    uint8_t DBCR;
    uint8_t DBTC;
    uint8_t DBENB;
    uint8_t DBENC;
    uint8_t DBEND;
}sEepFlashAppDebounce;

// structure for eeprom section .eep_FlashApp_Debug
typedef struct {
    uint8_t bTrace;
    uint8_t bDBGSW;
}sEepFlashAppDebug;

// structure for eeprom section .eep_FlashApp_Demod

// structure for eeprom section .eep_FlashApp_Dfifo

// structure for eeprom section .eep_FlashApp_Eeprom

// structure for eeprom section .eep_FlashApp_Fe

// structure for eeprom section .eep_FlashApp_Frsync

// structure for eeprom section .eep_FlashApp_Gpioregs

// structure for eeprom section .eep_FlashApp_I2c

// structure for eeprom section .eep_FlashApp_Int
typedef struct {
    uint8_t PCICR;
    uint8_t EIMSK;
    uint8_t EICRA;
    uint8_t PCMSK0;
    uint8_t PCMSK1;
}sEepFlashAppInt;

// structure for eeprom section .eep_FlashApp_Led

// structure for eeprom section .eep_FlashApp_Lf3d

// structure for eeprom section .eep_FlashApp_LfProtocolHandler

// structure for eeprom section .eep_FlashApp_LfRssi
typedef struct {
    /** \brief <b>bMargin</b>
        The RSSInorm measurement is used to calculate a reference value corresponding
        to the magnetic metric, e.g. B. To avoid negative exponents for the log to
        lin conversion during the RSSI measurement, this reference value is
        calculated to a negative RSSI value. This value is named MARGIN here.
    */
    uint8_t  bMargin;

    /** \brief <b>wRssiRef</b>
        RSSI measurement results for x,y,z using the internal current source
        at end of line calibration.
        \li wRssiRef[0]:    RSSIref_x
        \li wRssiRef[1]:    RSSIref_y
        \li wRssiRef[2]:    RSSIref_z
    */
    uint16_t wRssiRef[3];

    /** \brief <b>wRssiNorm</b>
        RSSI measurement results for x,y,z with defined magnetic field
        at end of line calibration.
        \li wRssiNorm[0]:    RSSINorm_x
        \li wRssiNorm[1]:    RSSINorm_y
        \li wRssiNorm[2]:    RSSINorm_z
    */
    uint16_t wRssiNorm[3];

    /** \brief <b>wBref</b>
        Bref is the magnetic flux density that would result in (-MARGIN)
        as the result of a RSSI measurement.
    */
    uint16_t wBref;

}sEepFlashAppLfRssiEndOfLineCalibrationSettings;

// structure for eeprom section .eep_FlashApp_Mem

// structure for eeprom section .eep_FlashApp_PortB
// structure for eeprom section .eep_FlashApp_PortC
// structure for eeprom section .eep_FlashApp_PortD
typedef struct {
    uint8_t DDR;
    uint8_t PORT;
}sEepFlashAppPort;

// structure for eeprom section .eep_FlashApp_RxBuf

// structure for eeprom section .eep_FlashApp_RxDsp

// structure for eeprom section .eep_FlashApp_Sfifo

// structure for eeprom section .eep_FlashApp_Spi
typedef struct {
    uint8_t SPCR;
    uint8_t SPSR;
}sEepFlashAppSpi;

// structure for eeprom section .eep_FlashApp_Ssm


// structure for eeprom section .eep_FlashApp_Sup
typedef struct {
    uint8_t SUPCR;
    uint8_t VMCR;
}sEepFlashAppSup;

// structure for eeprom section .eep_FlashApp_Symch

// structure for eeprom section .eep_FlashApp_Temper


// structure for eeprom section .eep_FlashApp_Timer0Wdt
typedef struct {
    uint8_t WDTCR;
}sEepFlashAppTimer0Wdt;

// structure for eeprom section .eep_FlashApp_Timer1

// structure for eeprom section .eep_FlashApp_Timer2

// structure for eeprom section .eep_FlashApp_Timer3

// structure for eeprom section .eep_FlashApp_Timer4

// structure for eeprom section .eep_FlashApp_Timer5

// structure for eeprom section .eep_FlashApp_Tmo

// structure for eeprom section .eep_FlashApp_TplfCal

// structure for eeprom section .eep_FlashApp_Transponder

// structure for eeprom section .eep_FlashApp_TxDsp

// structure for eeprom section .eep_FlashApp_Txm

#define EEPDATA_NMBR_SAVE 3
#define LF_AXIS_NB 3
typedef struct
{
  uint8_t aub_aeskey2[3][16];
  uint8_t aub_res[16];
  uint8_t aub_aeskey1[3][16];
  uint8_t aub_res2[16];
} sEepFlashApp_AESKey;

typedef struct
{
  uint32_t ul_rolling_code[EEPDATA_NMBR_SAVE];
  uint8_t aub_vid[EEPDATA_NMBR_SAVE][4];
  uint8_t ub_fidx[EEPDATA_NMBR_SAVE];
  uint8_t aub_rssi_norm[EEPDATA_NMBR_SAVE][LF_AXIS_NB];
  uint8_t aub_rssi_intref[EEPDATA_NMBR_SAVE][LF_AXIS_NB];
} sEepFlashApp_RKEPEPS;

#define EEPDATA_BLOCK_DATA_RKEPEPS(PARAM)  (g_sEepFlashApp_RKEPEPS.##PARAM##)

#define eub_fidx EEPDATA_BLOCK_DATA_RKEPEPS(ub_fidx)
#define eaub_vid EEPDATA_BLOCK_DATA_RKEPEPS(aub_vid)
#define eul_rolling_code EEPDATA_BLOCK_DATA_RKEPEPS(ul_rolling_code)


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
extern sEepFlashAppEventHandling g_sEepFlashAppEventHandling_flash;
extern sEepFlashAppClk g_sEepFlashAppClk_flash;
extern sEepFlashAppCpu g_sEepFlashAppCpu_flash;
extern sEepFlashAppDebounce g_sEepFlashAppDebounce_flash;
extern sEepFlashAppDebug g_sEepFlashAppDebug_flash;
extern sEepFlashAppInt g_sEepFlashAppInt_flash;
extern sEepFlashAppPort g_sEepFlashAppPortB_flash;
extern sEepFlashAppPort g_sEepFlashAppPortC_flash;
extern sEepFlashAppPort g_sEepFlashAppPortD_flash;
extern sEepFlashAppSpi g_sEepFlashAppSpi_flash;
extern sEepFlashAppSup g_sEepFlashAppSup_flash;
extern sEepFlashAppTimer0Wdt g_sEepFlashAppTimer0Wdt_flash;

extern sEepFlashAppLfRssiEndOfLineCalibrationSettings g_sEepFlashAppLfRssiEndOfLineCalibrationSettings_flash;
extern uint8_t g_bEepFlashAppLfRssiSrcCalibrationSetting_flash;

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
/*stopSimExtraction*/
#endif

#endif /* EEP_FLASH_H */