//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/init/src/init_flash.h $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
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
/** \file init_flash.h
*/

//lint -restore

#ifndef INIT_FLASH_H
#define INIT_FLASH_H
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../eep/src/eep_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
typedef sEepFlashAppEventHandling sSramFlashAppEventHandling;

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/

extern VOIDFUNC ATA_lfTpInit_flash_ASM(void);

extern UINT8FUNC ATA_initAta5700_flash_C(void);
extern UINT8FUNC ATA_initEventHandling_flash_C(void);
extern UINT8FUNC ATA_initIoModuleClk_flash_C(void);
extern UINT8FUNC ATA_initIoModuleCpu_flash_C(void);
extern UINT8FUNC ATA_initIoModuleDebounce_flash_C(void);
extern UINT8FUNC ATA_initIoModuleDebug_flash_C(void);
extern UINT8FUNC ATA_initIoModuleInt_flash_C(void);
extern UINT8FUNC ATA_initIoModulePortB_flash_C(void);
extern UINT8FUNC ATA_initIoModulePortC_flash_C(void);
extern UINT8FUNC ATA_initIoModulePortD_flash_C(void);
extern UINT8FUNC ATA_initIoModuleSpi_flash_C(void);
extern UINT8FUNC ATA_initIoModuleSup_flash_C(void);
extern UINT8FUNC ATA_initIoModuleTimer0Wdt_flash_C(void);

extern VOIDFUNC ATA_initAvrRegister_flash_C(void);

extern uint8_t g_bSleepModeConfig_flash;
extern sSramFlashAppEventHandling g_sEventHandling_flash;
extern uint8_t m_bTempArray_flash[10];

#endif /* INIT_FLASH_H */