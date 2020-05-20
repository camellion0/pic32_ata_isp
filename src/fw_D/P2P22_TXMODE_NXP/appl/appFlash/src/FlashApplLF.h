/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2_Gen2_LF/appl/appFlash/src/FlashApplLF.h $
  $LastChangedRevision: 334201 $
  $LastChangedDate: 2015-08-21 08:06:40 -0600 (Fri, 21 Aug 2015) $
  $LastChangedBy: mhahnen $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.3.18.2236
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

/** \file FlashApplLF.h
*/

#ifndef FLASH_APP_LF_H
#define FLASH_APP_LF_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/globals/src/globals.h"
#include "../../../firmware/spi/src/ata5700_command_set_flash.h"
#include "../../../firmware/system/src/system_flash.h"
#include "../../../firmware/init/src/init_flash.h"



/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
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


VOIDFUNC ATA_StartRssi_flash_C(uint8_t bmode);  
VOIDFUNC ATA_lfRssiCalcCorr_C(void);
VOIDFUNC app_rssi_load_factors(void);



#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
/*stopSimExtraction*/
#endif

#endif /* FLASH_APP_LF_H */