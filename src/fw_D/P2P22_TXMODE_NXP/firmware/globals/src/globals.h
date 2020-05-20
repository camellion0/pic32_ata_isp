//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/globals/src/globals.h $
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
/** \file globals.h Include global firmware structures.
    - sRfData includes the RF data buffer control variables
    - sGlobalSystemFlags includes the global system flags
*/
//lint -restore

#ifndef GLOBALS_H
#define GLOBALS_H
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "globals_types.h"
#include "globals_defs.h"

#ifdef __IAR_SYSTEMS_ICC__
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define ROM_VERSION         0x20U
#define ROMPATCH_VERSION    0x00U

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (variables)                                          */
/*===========================================================================*/
//lint -esym(9003, g_sDebug) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sDebug' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
extern sDebugErrorCodes g_sDebug;

extern __root const prog_char romVersion;
extern __root const prog_char romPatchVersion;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (functions)                                          */
/*===========================================================================*/

extern VOIDFUNC ATA_globalsSetClk_C(uint8_t bClockPrescalerValue);

extern VOIDFUNC ATA_globalsClkSwitchXTO_C(uint8_t bXtoClockSelect);

extern VOIDFUNC ATA_globalsClkSwitchFrc_C(void);

extern VOIDFUNC ATA_globalsClkSwitchFrcWithDelay_C(uint8_t bDelay);

extern VOIDFUNC ATA_globalsClkSwitchMrc_C(void);

extern VOIDFUNC ATA_globalsClkSwitchExt_C(uint8_t fDvccHighEnable);

extern VOIDFUNC ATA_globalsActivateXTO_C(void);

extern VOIDFUNC ATA_globalsDeActivateXTO_C(void);

extern VOIDFUNC ATA_globalsWdtDisable_C(void);

extern VOIDFUNC ATA_globalsWdtEnable_C(uint8_t uConfWDTCR);

extern VOIDFUNC ATA_globalsSleep_C(uint8_t bSleepModeConfig);

extern VOIDFUNC ATA_globalsInitSramSpace_C(uint8_t *pData, uint8_t bLength);

extern VOIDFUNC ATA_globalsCopySramSpace_C(uint8_t *pDestination, uint8_t *pSource, uint8_t bLength);

extern VOIDFUNC ATA_globalsSetVoltageMonitor_C( uint8_t bVmcsrVal );

extern VOIDFUNC ATA_globalsClkSwitchSrc_C(void);

extern VOIDFUNC  ATA_globalsWaitNus_ASM(uint8_t bTime);

extern INT16FUNC ATA_globalsMulS8U8_ASM(int8_t bFac1, uint8_t bFac2);

extern UINT32FUNC ATA_globalsMulU16U16_ASM(uint16_t fac1, uint16_t fac2);

extern VOIDFUNC ATA_globalsInitDebug_C(void);

#elif defined __IAR_SYSTEMS_ASM__

/*startSimExtraction*/
ROM_VERSION         EQU 0x20
ROMPATCH_VERSION    EQU 0x00
/*stopSimExtraction*/

#endif

#endif /* GLOBALS_H */
