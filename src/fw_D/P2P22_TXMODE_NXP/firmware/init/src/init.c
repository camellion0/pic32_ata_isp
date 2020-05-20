//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/init/src/init.c $
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
/** \file init.c
    this package does the wakeup detection and the system initialization
    according to the EEPROM settings.
*/
//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "init.h"
#include "../../globals/src/globals_defs.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*----------------------------------------------------------------------------- */
/** \brief <b>ATA_initCheckWakeupSource_C</b>
    checks the status of the NPWRON/PWRON pins in order to determine whether a 
    valid system wake-up condition has been detected:
    The wakeupSource is returned as follows:

    \li Bit7:   NPWRON6 (PD7)
    \li Bit6:   NPWRON5 (PD6)
    \li Bit5:   NPWRON4 (PD5)
    \li Bit4:   NPWRON3 (PD4)
    \li Bit3:   NPWRON2 (PD3)
    \li Bit2:   NPWRON1 (PD2)
    \li Bit1:   NPWRON0 (PD1)
    \li Bit0:    PWRON  (PC2)

    \return     Pin status of POWER ON, respectively NPOWER ON, whereas 1 means 
                active and 0 means inactive

    \internal
    \li 010: Read the NPWRON pin and the PWRON pin from PINC and PIND in 
             order to determine the system wake-up condition and build return value

    \Derived{No}

    \Rationale{A means to determine which pin was responsible for the 
               system wakeup is required}

    \Traceability   N/A
    \endinternal
\n
*/
/*----------------------------------------------------------------------------- */
UINT8FUNC ATA_initCheckWakeupSource_C(void)
{
    uint8_t bWakeupSource;

    /* LLR-Ref: 010 */
    bWakeupSource  = (uint8_t)PIND & (uint8_t)0xFE; /* PD7..1 */
    bWakeupSource |= (uint8_t)(((PINC & BM_PINC2) ? BIT_MASK_0 : 0x00U)); /* PC2 */
    bWakeupSource ^= (uint8_t)0xFE; /* Invert all NPWRON except PWRON */

    return bWakeupSource;
}
