//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/extif/src/extif_flash.h $
  $LastChangedRevision: 586334 $
  $LastChangedDate: 2020-01-27 13:26:46 -0700 (Mon, 27 Jan 2020) $
  $LastChangedBy: grueter $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.3.18.0
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
/** \file extif_flash.h
*/

//lint -restore

#ifndef EXTIF_FLASH_H
#define EXTIF_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define PCINT0TRUE              BIT_7
#define PCINT1TRUE              BIT_6
#define BM_PCINT0TRUE           BIT_MASK_7
#define BM_PCINT1TRUE           BIT_MASK_6

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/

/** \brief <b>pcIntHandler</b>
    is used for function pointer definition of pin change ISRs.
*/
typedef void (*pcIntHandler)(void);

typedef uint8_t tPCINTStatus;
 /** \brief <b>Timer5Status</b>
        contains timer5 status flags  
        (see rfTx component)
        \li Bit 7:    PC0INTTRUE
        \li Bit 6:    PC1INTTRUE
        \li Bit 5:    rfu
        \li Bit 4:    rfu
        \li Bit 3:    rfu
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    rfu
    */

/*----------------------------------------------------------------------------- */
/** \brief <b>sIFData</b>
    contains the configuration/variables which are used for SPI communication
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>pcInt0</b>
        is the function pointer to the routine executed, , when an IRQ occures via PCINT0.
    */
    pcIntHandler pcInt0;

    /** \brief <b>pcInt1</b>
        is the function pointer to the routine executed, when an IRQ occures via PCINT1.
    */
    pcIntHandler pcInt1;

    /** \brief <b>pcInt0old</b>
        is the old value of pcint0, to detect which pin caused the IRQ.
    */
    uint8_t pcInt0old;

    /** \brief <b>pcInt1old</b>
        is the old val of pcint1, to detect which pin caused the IRQ.
    */
    uint8_t pcInt1old;

}sIFData;

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/

extern VOIDFUNC ATA_portB_flash_ASM(void);

extern VOIDFUNC ATA_portD_flash_ASM(void);

extern VOIDFUNC ATA_initExtIf_flash_C(pcIntHandler fpPortBisr, pcIntHandler fpPortCisr);

extern sIFData g_sExtIf;

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/

/* sIFData */
IFDATA_PCINT0PTRL             EQU   0x00
IFDATA_PCINT0PTRH             EQU   IFDATA_PCINT0PTRL + 1
IFDATA_PCINT1PTRL             EQU   IFDATA_PCINT0PTRH + 1
IFDATA_PCINT1PTRH             EQU   IFDATA_PCINT1PTRL + 1
IFDATA_PCINT0OLD              EQU   IFDATA_PCINT1PTRH + 1
IFDATA_PCINT1OLD              EQU   IFDATA_PCINT0OLD + 1

/*stopSimExtraction*/

#endif

#endif /* EXTIF_H */