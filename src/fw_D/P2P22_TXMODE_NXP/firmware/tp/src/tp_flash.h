//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/tp/src/tp_flash.h $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
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
/** \file tp_flash.h
 */

//lint -restore

#ifndef TP_FLASH_H
#define TP_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/* sTpRxTxConfig.tp_flags */
#define LFRXCONFIG_TP_FLAGS_TPBERF              BIT_3
#define LFRXCONFIG_TP_FLAGS_TPNFTF              BIT_2
#define LFRXCONFIG_TP_FLAGS_TPFTF               BIT_1
#define LFRXCONFIG_TP_FLAGS_TPF                 BIT_0

#define BM_LFRXCONFIG_TP_FLAGS_TPBERF           BITMASK(LFRXCONFIG_TP_FLAGS_TPBERF)
#define BM_LFRXCONFIG_TP_FLAGS_TPNFTF           BITMASK(LFRXCONFIG_TP_FLAGS_TPNFTF)
#define BM_LFRXCONFIG_TP_FLAGS_TPFTF            BITMASK(LFRXCONFIG_TP_FLAGS_TPFTF)
#define BM_LFRXCONFIG_TP_FLAGS_TPF              BITMASK(LFRXCONFIG_TP_FLAGS_TPF)


/* EM Transponder Mode Command IDs */
#define EM_CMD_ID_MODE                              0x01U
#define EM_CMD_ID_MODE_PARITY                       0x01U
#define EM_CMD_WRITE_WORD                           0x05U
#define EM_CMD_WRITE_WORD_PARITY                    0x00U
#define EM_CMD_UM_MODE_1                            0x02U
#define EM_CMD_UM_MODE_1_PARITY                     0x01U
#define EM_CMD_UM_MODE_2                            0x07U
#define EM_CMD_UM_MODE_2_PARITY                     0x01U
#define EM_CMD_SEND_PIN                             0x04U
#define EM_CMD_SEND_PIN_PARITY                      0x01U
#define EM_CMD_AUTHENTICATION                       0x03U
#define EM_CMD_AUTHENTICATION_PARITY                0x00U


/* EM Mode state handling */
#define EM_MODE_STATE_INIT                          0x00U
#define EM_MODE_STATE_RECONFIG                      0x01U
#define EM_MODE_STATE_PROCESSING                    0x02U
#define EM_MODE_WAIT_FOR_DATA                       0x03U

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/

typedef struct {
    
    /** \brief <b>flags</b>
        contains the event flags for the Transponder 
        \li Bit 7: rfu
        \li Bit 6: rfu
        \li Bit 5: rfu
        \li Bit 4: rfu
        \li Bit 3: TPBERF
        \li Bit 2: TPNFTF
        \li Bit 1: TPFTF
        \li Bit 0: TPF
    */
    uint8_t bTpFlags;

    uint8_t bStatus;
    
    uint8_t bConfig;
    
}sTpRxTxConfig;

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (variables)                                          */
/*===========================================================================*/
extern sTpRxTxConfig g_sTpRxTx;
extern uint8_t g_bTpEmTransponderStateIndex_flash;

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES (functions)                                          */
/*===========================================================================*/
extern VOIDFUNC ATA_tpRxTxInit_flash_C(void);
extern VOIDFUNC ATA_tpEmModeInit_flash_C(void);
extern VOIDFUNC ATA_tpEmModeCommandReconfiguration_flash_C(void);
extern VOIDFUNC ATA_tpEmModeSingleTelProc_flash_C(void);

#elif defined __IAR_SYSTEMS_ASM__

/* sTpRxTxConfig.tp_flags */
LFRXCONFIG_TP_FLAGS_TPBERF              EQU     BIT_3
LFRXCONFIG_TP_FLAGS_TPNFTF              EQU     BIT_2
LFRXCONFIG_TP_FLAGS_TPFTF               EQU     BIT_1
LFRXCONFIG_TP_FLAGS_TPF                 EQU     BIT_0

BM_LFRXCONFIG_TP_FLAGS_TPBERF           EQU     (0x01 << LFRXCONFIG_TP_FLAGS_TPBERF)
BM_LFRXCONFIG_TP_FLAGS_TPNFTF           EQU     (0x01 << LFRXCONFIG_TP_FLAGS_TPNFTF)
BM_LFRXCONFIG_TP_FLAGS_TPFTF            EQU     (0x01 << LFRXCONFIG_TP_FLAGS_TPFTF)
BM_LFRXCONFIG_TP_FLAGS_TPF              EQU     (0x01 << LFRXCONFIG_TP_FLAGS_TPF)

/* EM Transponder Mode Command IDs */
EM_CMD_ID_MODE                              EQU     0x01
EM_CMD_ID_MODE_PARITY                       EQU     0x01
EM_CMD_WRITE_WORD                           EQU     0x05
EM_CMD_WRITE_WORD_PARITY                    EQU     0x00
EM_CMD_UM_MODE_1                            EQU     0x02
EM_CMD_UM_MODE_1_PARITY                     EQU     0x01
EM_CMD_UM_MODE_2                            EQU     0x07
EM_CMD_UM_MODE_2_PARITY                     EQU     0x01
EM_CMD_SEND_PIN                             EQU     0x04
EM_CMD_SEND_PIN_PARITY                      EQU     0x01
EM_CMD_AUTHENTICATION                       EQU     0x03
EM_CMD_AUTHENTICATION_PARITY                EQU     0x00

/* EM Mode state handling */
EM_MODE_STATE_INIT                          EQU     0x00
EM_MODE_STATE_RECONFIG                      EQU     0x01
EM_MODE_STATE_PROCESSING                    EQU     0x02
EM_MODE_WAIT_FOR_DATA                       EQU     0x03

#endif

#endif /* LFRX_H */