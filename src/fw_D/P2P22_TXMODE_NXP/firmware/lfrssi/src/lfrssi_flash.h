//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrssi/src/lfrssi_flash.h $
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
/** \file rf.h
 */

#ifndef LFRSSI_FLASH_H
#define LFRSSI_FLASH_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../rftx/src/rftx.h"
#include "../../lfrx/src/lfrx_flash.h"
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/* defines for sLfRssiConfig.bFlags */
#define LFRSSICONFIG_FLAGS_RESET                        (uint8_t)0x00
#define LFRSSICONFIG_FLAGS_MEASUREMENT_ENABLE_FLAG      BIT_0
#define LFRSSICONFIG_FLAGS_BM_MEASUREMENT_ENABLE_FLAG   BITMASK(LFRSSICONFIG_FLAGS_MEASUREMENT_ENABLE_FLAG)

/* defines for sLfRssiConfig.bStatus */
#define LFRSSICONFIG_STATUS_RESET                           (uint8_t)0x00
#define LFRSSICONFIG_STATUS_INTERNAL_MEASUREMENT_FLAG       BIT_2
#define LFRSSICONFIG_STATUS_BM_INTERNAL_MEASUREMENT_FLAG    BITMASK(LFRSSICONFIG_STATUS_INTERNAL_MEASUREMENT_FLAG)
#define LFRSSICONFIG_STATUS_LASTSTATECHANGE_FLAG            BIT_1
#define LFRSSICONFIG_STATUS_BM_LASTSTATECHANGE_FLAG         BITMASK(LFRSSICONFIG_STATUS_LASTSTATECHANGE_FLAG)
#define LFRSSICONFIG_STATUS_MEASUREMENT_SERIAL_FLAG         BIT_0
#define LFRSSICONFIG_STATUS_BM_MEASUREMENT_SERIAL_FLAG      BITMASK(LFRSSICONFIG_STATUS_MEASUREMENT_SERIAL_FLAG)

#define SERIAL_LFRSSI_MEASUREMENT   (uint8_t)0x02
#define PARALLEL_LFRSSI_MEASUREMENT (uint8_t)0x04

/* define channel timeout window range */
#define APPL_LOWER_THRESHOLD_RAW_RSSI_VALUE (uint16_t)5       // lower threshold border indicating no channel timeout
#define APPL_UPPER_THRESHOLD_RAW_RSSI_VALUE (uint16_t)507     // upper threshold border indicating no channel timeout

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
/** \brief <b>sysFlowStateMachineFunc_t</b>
    is used for function pointer definition of system flow state machines.
*/
typedef void (*lfRssiFlowStateMachineFunc_t)(void);

/** \brief <b>sysFlowStateMachineFuncLut_t</b>
    is used for function pointer definition of system flow state machine
    look up tables in code section.
*/
typedef const lfRssiFlowStateMachineFunc_t __flash lfRssiFlowStateMachineFuncLut_t;

/*----------------------------------------------------------------------------- */
/** \brief <b>sLfRssiAppFlowCtrl</b>
    contains the information to control the flow of the software state machines.
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** brief <b>bIndex</b>
        contains the current state machine index.
    */
    volatile uint8_t bIndex;

    /** \brief <b>fpLut</b>
        is used as pointer to currently used state machine look up table.
    */
    sysFlowStateMachineFuncLut_t *fpLut;

    /** \brief <b>bLastRfTxStateIndex</b>
        contains the last state machine index.
    */
    uint8_t bLastRfTxStateIndex;

}sLfRssiAppFlowCtrl;

/*----------------------------------------------------------------------------- */
/** \brief <b>sLfRssiAppCtrl</b>
    holds internal states and flags to signalize LF RSSI measurement state.
*/
/*----------------------------------------------------------------------------- */
typedef struct{
    /** brief <b>bFlags</b>
        contains information to be used outside LF RSSI module.
    */
    uint8_t  bFlags;

    /** \brief <b>bStatus</b>
        contains information to used only inside LF RSSI module.
    */
    uint8_t  bStatus;

}sLfRssiAppCtrl;

/*----------------------------------------------------------------------------- */
/** \brief <b>sLfRssiAppRegConfig</b>
    holds the specific register settings to be used for the LF RSSI measurement.
*/
/*----------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  bRscr;
    uint8_t  bRsms1r;
    uint8_t  bRsms2r;
    uint8_t  bRsdlyr;
    uint8_t  bRssrcr;

}sLfRssiAppRegConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sLfRssiAppCustConfig</b>
    holds the customer application settings for the LF RSSI measurement.
*/
/*----------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  bNumMeas;
    uint8_t  bOptions;
    uint16_t wCycleTime;

}sLfRssiAppCustConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sLfRssiAppResult</b>
    holds the LF RSSI measurement data and related calculation results.
*/
/*----------------------------------------------------------------------------- */
typedef struct
{
    // add additional flags and internal status for each measurement??
    uint16_t wRawLfRssi[3];
    uint8_t  bSignDetect[4];
    uint16_t wCorrLfRssi[3];
    uint16_t w3dVecVal;
    uint16_t wLinearVal;

}sLfRssiAppResult;



/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
extern sLfRssiAppFlowCtrl   g_sLfRssiFlowCtrl_flash;
extern sLfRssiAppCtrl       g_sLfRssiCtrl_flash;
extern sLfRssiAppRegConfig  g_sLfRssiRegConfig_flash;
extern sLfRssiAppCustConfig g_sLfRssiCustConfig_flash;

/* applications commonly used functions */
extern VOIDFUNC ATA_lfRssiMeasInit_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasProcessing_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasEnableLfReceiver_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasExecCalc_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasClose_flash_C(void);


/* BMW application related section */
extern sLfRssiAppResult g_sLfRssiBmwResult_flash[3];

extern VOIDFUNC ATA_lfRssiMeasConfig_flash_C(uint8_t);
extern VOIDFUNC ATA_lfRssiMeasOpen_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasStartInt_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasStartExt_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasWaitReady_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasPrepSendResults_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasSendResult_flash_C(void);


/* HFM application related section */
extern sLfRssiAppResult g_sLfRssiHfmResult_flash[9];
extern uint16_t         g_wLfRssiMeasSamples[48];

extern VOIDFUNC ATA_lfRssiMeasConfigHfm_flash_C(uint8_t);
extern VOIDFUNC ATA_lfRssiMeasProcessingHfm_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasOpenHfm_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasStartIntHfm_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasStartExtHfm_flash_C(void);
extern VOIDFUNC ATA_lfRssiMeasWaitReadyHfm_flash_C(void);


#elif defined __IAR_SYSTEMS_ASM__

#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* LFRSSI_FLASH_H */