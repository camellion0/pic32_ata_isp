/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/GR_inwork/appl/appFlash/src/FlashApplVars.h $
  $LastChangedRevision: 336308 $
  $LastChangedDate: 2015-09-09 13:10:18 -0600 (Wed, 09 Sep 2015) $
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
/** \file FlashApplRF.h
 */

//lint -restore

#ifndef FLASHAPPLRF_H
#define FLASHAPPLRF_H

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
//#include "../../stdc/src/stdc.h"
//#include "../../globals/src/globals.h"
//#include "../../rftx/src/rftx_defs.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
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

// Number of Fobs managed in system (max 4)

#define PEPS_NB_FOB_MAX        4
#endif

// number of channels max 
#define PEPS_NB_CHANNEL_MAX    3

// pointer on RX buffer (either LF or RF buffer)
#define MSG_RX_DATA (g_MsgRXbuffer.aub_data)
// data count received
#define MSG_RX_CNT (g_MsgRXbuffer.ub_size)

#define SLOT_COMMON   0x01
#define SLOT_SINGLES  0x02

//#define INTERFRAME    4.0//GeRu Was 3.0

/* ------------------------------------------------------------------------- */
/* g_FlashApplVars                                                           */
/* ------------------------------------------------------------------------- */