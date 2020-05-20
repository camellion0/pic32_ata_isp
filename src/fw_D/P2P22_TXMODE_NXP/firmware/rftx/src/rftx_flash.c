//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Branches/VB_PRIMUS2P_ROM_2.0/firmware/rftx/src/rftx_flash.c $
  $LastChangedRevision: 266519 $
  $LastChangedDate: 2014-06-10 13:48:14 +0200 (Di, 10 Jun 2014) $
  $LastChangedBy: florian.schweidler $
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
/** \file rftx_flash.c
*/

//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "../../eep/src/eep.h"
#include "rftx.h"
#include "rftx_flash.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/** \brief <b>g_sSramRfTxServiceConfig</b>
    contains the service configuration in the SRAM
*/
__root __no_init sRfTxServiceChannelConfig g_sSramRfTxServiceConfig;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

UINT16FUNC ATA_rfTxGetIndirectEEPromServiceConfigAddr_flash_C (uint16_t wAddress)
{
    /* Get indirect address for given RF Tx EEPROM service configuration address */
    uint8_t ary[2];
    ATA_eepReadBytes_C(&ary[0],wAddress,2U);
    return (uint16_t)((ary[1]<<8U|ary[0]));
}
