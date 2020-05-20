//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/eep/src/eep_flash.c $
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
/** \file eep_flash.c
*/

//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "eep_flash.h"
#include "eep.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define AESaddresslocate              0x04e0U
/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
#pragma location = ".eep_FlashApp_EventHandling"
__root __no_init sEepFlashAppEventHandling g_sEepFlashAppEventHandling_flash;

//#pragma location = ".eep_FlashApp_Aes"

// #pragma location = ".eep_FlashApp_Chflt"

#pragma location = ".eep_FlashApp_Clk"
__root __no_init sEepFlashAppClk g_sEepFlashAppClk_flash;

#pragma location = ".eep_FlashApp_Cpu"
__root __no_init sEepFlashAppCpu g_sEepFlashAppCpu_flash;

// #pragma location = ".eep_FlashApp_Crc"

#pragma location = ".eep_FlashApp_Debounce"
__root __no_init sEepFlashAppDebounce g_sEepFlashAppDebounce_flash;

#pragma location = ".eep_FlashApp_Debug"
__root __no_init sEepFlashAppDebug g_sEepFlashAppDebug_flash;

// #pragma location = ".eep_FlashApp_Demod"

// #pragma location = ".eep_FlashApp_Dfifo"

// #pragma location = ".eep_FlashApp_Eeprom"

// #pragma location = ".eep_FlashApp_Fe"

// #pragma location = ".eep_FlashApp_Frsync"

// #pragma location = ".eep_FlashApp_Gpioregs"

// #pragma location = ".eep_FlashApp_I2c"

#pragma location = ".eep_FlashApp_Int"
__root __no_init sEepFlashAppInt g_sEepFlashAppInt_flash;

// #pragma location = ".eep_FlashApp_Led"

// #pragma location = ".eep_FlashApp_Lf3d"

// #pragma location = ".eep_FlashApp_LfProtocolHandler"

// #pragma location = ".eep_FlashApp_LfRssi"
#pragma location = ".eep_FlashApp_LfRssiEndOfLineCalibrationSettings"
__root __no_init sEepFlashAppLfRssiEndOfLineCalibrationSettings g_sEepFlashAppLfRssiEndOfLineCalibrationSettings_flash;

#pragma location = ".eep_wEepRfrccAddress"
__root __no_init uint16_t wEepRfrccAddress;

//#pragma location = ".eepSecKeyAddrA"
//__root __no_init uint16_t weepSecKeyAddrA[32];

//#pragma location = ".eepSecKeyAddrB"
//__root __no_init uint16_t weepSecKeyAddrB[32];

#pragma location = ".eep_FlashApp_LfRssiSrcCalibrationSetting"
__root __no_init uint8_t g_bEepFlashAppLfRssiSrcCalibrationSetting_flash;

// #pragma location = ".eep_FlashApp_Mem"

#pragma location = ".eep_FlashApp_PortB"
__root __no_init sEepFlashAppPort g_sEepFlashAppPortB_flash;

#pragma location = ".eep_FlashApp_PortC"
__root __no_init sEepFlashAppPort g_sEepFlashAppPortC_flash;

#pragma location = ".eep_FlashApp_PortD"
__root __no_init sEepFlashAppPort g_sEepFlashAppPortD_flash;

// #pragma location = ".eep_FlashApp_RxBuf"

// #pragma location = ".eep_FlashApp_RxDsp"

// #pragma location = ".eep_FlashApp_Sfifo"

#pragma location = ".eep_FlashApp_Spi"
__root __no_init sEepFlashAppSpi g_sEepFlashAppSpi_flash;

// #pragma location = ".eep_FlashApp_Ssm"

#pragma location = ".eep_FlashApp_Sup"
__root __no_init sEepFlashAppSup g_sEepFlashAppSup_flash;

// #pragma location = ".eep_FlashApp_Symch"

// #pragma location = ".eep_FlashApp_Temper"

#pragma location = ".eep_FlashApp_Timer0Wdt"
__root __no_init sEepFlashAppTimer0Wdt g_sEepFlashAppTimer0Wdt_flash;

// #pragma location = ".eep_FlashApp_Timer1"

// #pragma location = ".eep_FlashApp_Timer2"

// #pragma location = ".eep_FlashApp_Timer3"

// #pragma location = ".eep_FlashApp_Timer4"

// #pragma location = ".eep_FlashApp_Timer5"

// #pragma location = ".eep_FlashApp_Tmo"

// #pragma location = ".eep_FlashApp_TplfCal"

// #pragma location = ".eep_FlashApp_Transponder"

// #pragma location = ".eep_FlashApp_TxDsp"

// #pragma location = ".eep_FlashApp_Txm"
//#pragma location = AESaddresslocate
//__root __no_init sEepFlashApp_AESKey g_sEepFlashApp_AESKey;

//#pragma location = ".eep_sCustomerEEPromSectionAESkey"
//__root __no_init sEepFlashApp_AESKey g_sEepFlashApp_AESKey;

#pragma location = ".eep_sCustomerEEPromSectionRKEPEPS"
__root __no_init sEepFlashApp_RKEPEPS g_sEepFlashApp_RKEPEPS;

#pragma location = ".eep_sCustomerEEPromSectionUSRID"
__root __no_init uint8_t g_EepFlashApp_USRID[4];

//#pragma location = ".eep_sCustomerEEPromSectionFOBindx"
//__root __no_init uint8_t g_EepFlashApp_FOBindx;

#pragma location = ".eep_sCustomerEEPromSectionParameterData"
__root __no_init uint8_t g_EepFlashApp_PARAMdata[32][16];//GR was 3

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
