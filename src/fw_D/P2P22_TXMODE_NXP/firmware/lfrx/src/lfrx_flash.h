//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrx/src/lfrx_flash.h $
  $LastChangedRevision: 465393 $
  $LastChangedDate: 2017-07-03 08:54:02 -0600 (Mon, 03 Jul 2017) $
  $LastChangedBy: krishna.balan $
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
/** \file lfrx_flash.h
 */

//lint -restore

#ifndef LFRX_FLASH_H
#define LFRX_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

#define ATA_SYSVER_MAX_LF_CONT_EOT_TELEGRAM_DEFS    (0x04)

/* s3dLfRxConfig.lf_flags */
#define LFRXCONFIG_LF_FLAGS_RSSI_ERR            BIT_7
#define LFRXCONFIG_LF_FLAGS_RSSI_RDY            BIT_6
#define LFRXCONFIG_LF_FLAGS_LF_RDY              BIT_4
#define LFRXCONFIG_LF_FLAGS_LFTOF               BIT_3
#define LFRXCONFIG_LF_FLAGS_LFEOF               BIT_2
#define LFRXCONFIG_LF_FLAGS_LFDEF               BIT_1
#define LFRXCONFIG_LF_FLAGS_LFSYDF              BIT_0

#define BM_LFRXCONFIG_LF_FLAGS_RSSI_ERR         BITMASK(LFRXCONFIG_LF_FLAGS_RSSI_ERR)
#define BM_LFRXCONFIG_LF_FLAGS_RSSI_RDY         BITMASK(LFRXCONFIG_LF_FLAGS_RSSI_RDY)
#define BM_LFRXCONFIG_LF_FLAGS_LF_RDY           BITMASK(LFRXCONFIG_LF_FLAGS_LF_RDY)
#define BM_LFRXCONFIG_LF_FLAGS_LFTOF            BITMASK(LFRXCONFIG_LF_FLAGS_LFTOF)
#define BM_LFRXCONFIG_LF_FLAGS_LFEOF            BITMASK(LFRXCONFIG_LF_FLAGS_LFEOF)
#define BM_LFRXCONFIG_LF_FLAGS_LFDEF            BITMASK(LFRXCONFIG_LF_FLAGS_LFDEF)
#define BM_LFRXCONFIG_LF_FLAGS_LFSYDF           BITMASK(LFRXCONFIG_LF_FLAGS_LFSYDF)

/* s3dLfRxConfig.ph_flags_0 */
#define LFRXCONFIG_PH_FLAGS_0_PHCOF             BIT_7
#define LFRXCONFIG_PH_FLAGS_0_PHID1F            BIT_5
#define LFRXCONFIG_PH_FLAGS_0_PHID0F            BIT_4
#define LFRXCONFIG_PH_FLAGS_0_PHIDFF            BIT_3
#define LFRXCONFIG_PH_FLAGS_0_PHDFF             BIT_2
#define LFRXCONFIG_PH_FLAGS_0_PHTBLF            BIT_1
#define LFRXCONFIG_PH_FLAGS_0_CRCEF             BIT_0

#define BM_LFRXCONFIG_PH_FLAGS_0_PHCOF          BITMASK(LFRXCONFIG_PH_FLAGS_0_PHCOF)
#define BM_LFRXCONFIG_PH_FLAGS_0_PHID1F         BITMASK(LFRXCONFIG_PH_FLAGS_0_PHID1F)
#define BM_LFRXCONFIG_PH_FLAGS_0_PHID0F         BITMASK(LFRXCONFIG_PH_FLAGS_0_PHID0F)
#define BM_LFRXCONFIG_PH_FLAGS_0_PHIDFF         BITMASK(LFRXCONFIG_PH_FLAGS_0_PHIDFF)
#define BM_LFRXCONFIG_PH_FLAGS_0_PHDFF          BITMASK(LFRXCONFIG_PH_FLAGS_0_PHDFF)
#define BM_LFRXCONFIG_PH_FLAGS_0_PHTBLF         BITMASK(LFRXCONFIG_PH_FLAGS_0_PHTBLF)
#define BM_LFRXCONFIG_PH_FLAGS_0_CRCEF          BITMASK(LFRXCONFIG_PH_FLAGS_0_CRCEF)

/* s3dLfRxConfig.ph_flags_1 */
#define LFRXCONFIG_PH_FLAGS_1_OUFLF             BIT_1
#define LFRXCONFIG_PH_FLAGS_1_FLRF              BIT_0

#define BM_LFRXCONFIG_PH_FLAGS_1_OUFLF          BITMASK(LFRXCONFIG_PH_FLAGS_1_OUFLF)
#define BM_LFRXCONFIG_PH_FLAGS_1_FLRF           BITMASK(LFRXCONFIG_PH_FLAGS_1_FLRF)


/* COMMANDS */
#define   READ_UID       (uint8_t)0x00U   /* 0x00U */
#define   START_AUTH     (uint8_t)0x01U   /* 0x13U */
#define   TP_ERR_STS     (uint8_t)0x02U   /* 0x26U */
#define   INIT_ENH_MOD   (uint8_t)0x03U   /* 0x35U */
#define   READ_USR_MEM   (uint8_t)0x04U   /* 0x4CU */
#define   WRT_USR_MEM    (uint8_t)0x05U   /* 0x5FU */
#define   WRT_MEM_PRT    (uint8_t)0x06U   /* 0x6AU */
#define   LRN_SKT_K1     (uint8_t)0x07U   /* 0x79U */
#define   LRN_SKT_K2     (uint8_t)0x08U   /* 0x8BU */
#define   START_AC       (uint8_t)0x09U   /* 0x98U */
#define   LEAVE_ENH_MODE (uint8_t)0x0AU   /* 0xADU */
#define   TRA_BIT0       (uint8_t)0x0BU   /* 0xBEU */
#define   TRA_BIT1       (uint8_t)0x0CU   /* 0xC7U */
#define   RPT_LST_RES    (uint8_t)0x0EU   /* 0xE1U */

/* Status byte values */
#define   STATUS_DEFAULT                   (uint8_t)0xFFU
#define   STATUS_SUCCESS                   (uint8_t)0xF0U
#define   STATUS_ADD_LOC_LOCKED            (uint8_t)0xF1U
#define   STATUS_ADD_OUT_RANGE             (uint8_t)0xF2U
#define   STATUS_COMMAND_NOT_SUPPORTED     (uint8_t)0xF3U
#define   STATUS_REQUEST_FRAME_ERROR       (uint8_t)0xF5U
#define   STATUS_CRC_INCORRECT             (uint8_t)0xF4U
#define   STATUS_BILATERAL_AUTHEN_FAILED   (uint8_t)0xF6U
#define   STATUS_AES_BLOCK_ERROR           (uint8_t)0xF7U
#define   STATUS_GENERIC_ERROR             (uint8_t)0xF8U
#define   STATUS_EEPROM_ERROR_CORRECTION   (uint8_t)0xF9U
#define   STATUS_VM_THRESHOLD_VIOLATION    (uint8_t)0xFAU
#define   STATUS_BIT_ERROR                 (uint8_t)0xFBU

#define  BA_FAILED       0
#define  BA_SUCCESS      1

#define  UNILATERAL      0
#define  BILATERAL       1

#define MASK_HIGH_NIBBLE    (uint8_t)0x0FU

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/

typedef struct {

    /** \brief <b>ph_flags_0</b>
        contains the event flags for the Protocol Handler
        (see ph component)
        \li Bit 7: PHCOF
        \li Bit 6: rfu
        \li Bit 5: PHID1F
        \li Bit 4: PHID0F
        \li Bit 3: PHIDFF
        \li Bit 2: PHDFF
        \li Bit 1: PHTBLF
        \li Bit 0: CRCEF
    */
    uint8_t bLDFFLags0;

    /** \brief <b>ph_flags_1</b>
        contains the event flags for the Protocol Handler FIFO
        (see LDFIMfo component)
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: rfu
        \li Bit 2: rfu
        \li Bit 1: OUFLF
        \li Bit 0: FLRF
    */
    uint8_t bLDFFLags1;

    /** \brief <b>flags</b>
        contains the event flags for the 3D LF Receiver
        \li Bit 7: RSSI_ERR
        \li Bit 6: RSSI_RDY
        \li Bit 5: rfu
        \li Bit 4: LF_RDY
        \li Bit 3: LFTOF
        \li Bit 2: LFEOF
        \li Bit 1: LFDEF
        \li Bit 0: LFSYDF
    */
    uint8_t bLfFlags;

    uint8_t bStatus;

    uint8_t bConfig;

}s3dLfRxConfig;


/* System Verification specific */
typedef struct {

    uint8_t bPhidfr;
    uint8_t bPhdfr;
    uint8_t bPhtblr;
    uint8_t bPhcrph; // Clock switch necessary
    uint8_t bPhcrpl; // Clock switch necessary
    uint8_t bPhcsth; // Clock switch necessary
    uint8_t bPhcstl; // Clock switch necessary
    uint8_t bPhcrcr;
    uint8_t bTpecmr;
    uint8_t bTpecr1;
    uint8_t bTpecr2;
    uint8_t bTpecr3;
    uint8_t bTpecr4;
    uint8_t bPhtcr;
    uint8_t bSpare_01; // Reserved for future use
    uint8_t bSpare_02; // Reserved for future use

} sPhShadowRegisters;

//-----------------------------------------------------------------------------
/** \brief <b>configuration</b>
    holds the configuration of the EEPROM region
      \li Bit 7:    (TDH)     detection header transmission (0 = off, 1 = on)
      \li Bit 6:    (SKT)     secure key transfer (0 = off, 1 = on)
      \li Bit 5:    (KS)      key selection (0 = key 1 first, 1 = key 2 first)
      \li Bit 4..3: (DLP1/0)  downlink mode selection (00 = BPLM, 01 = QPLM,
                                                       10 = DPS)
      \li Bit 2:    (CM)      crypto mode (0 = unilateral, 1 = bilateral)
      \li Bit 1:    (MOD)     uplink modulation (0 = manchester, 1 = biphase)
      \li Bit 0:    (DCD)     CRC checking disable bit
*/
//-----------------------------------------------------------------------------
typedef struct configuration {
  uint8_t     DCD : 1;
  uint8_t     MOD : 1;
  uint8_t     CM  : 1;
  uint8_t     DLP : 2;
  uint8_t     KS  : 1;
  uint8_t     SKT : 1;
  uint8_t     TDH : 1;
}sConfiguration;

//-----------------------------------------------------------------------------
/** \brief <b>config</b>
    union to access whole byte or dedicated bit
*/
//-----------------------------------------------------------------------------
typedef union config
{
  uint8_t        bConfigbyte;
  sConfiguration sConfigbits;
}uConfiguration;

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (variables)                                          */
/*===========================================================================*/

/**/
extern sPhShadowRegisters g_sLfRxIdShadowValues_flash;

/**/
extern sPhShadowRegisters g_sLfRxEotShadowValues_flash[ATA_SYSVER_MAX_LF_CONT_EOT_TELEGRAM_DEFS];

/**/
extern uint8_t            g_bLfRxEotShadowIndex_flash;

/**/
extern s3dLfRxConfig      g_sLfRx_flash;

/**/
extern VOIDFUNC ATA_lfRxInit_flash_C(void);

extern uConfiguration g_uTpConfig;
extern uint8_t g_bRxBufferIndex;
extern uint8_t g_bTpStatusByte;
extern uint8_t g_bBitsToReceive;
extern uint8_t g_bReceiveState;
extern uint8_t g_bTxBuffer[36];
extern uint8_t g_bRxBuffer[36];
extern uint8_t g_bResponseDelayReached;
extern uint8_t g_bBytesToTransmit;

extern char ID0_Wake;
extern char ID1_Wake; 
extern char LF_DecErrFlag; 

#elif defined __IAR_SYSTEMS_ASM__

#endif

#endif /* LFRX_FLASH_H */