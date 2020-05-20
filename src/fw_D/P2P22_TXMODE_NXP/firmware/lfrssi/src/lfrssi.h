//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrssi/src/lfrssi.h $
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
/** \file lfrssi.h
 */
//lint -restore

#ifndef LFRSSI_H
#define LFRSSI_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#undef DEBUG_LFRSSI_LUT

#define LFRSSI_NUM_COILS        (uint8_t)3
#define LFRSSI_COIL_1_DATA      (uint8_t)0
#define LFRSSI_COIL_2_DATA      (uint8_t)1
#define LFRSSI_COIL_3_DATA      (uint8_t)2

#define BM_RSMODE_MEASUREMENT   (uint8_t)0
#define BM_RSMODE_CORRCALC      BM_RSMODE0
#define BM_RSMODE_3DVECTOR      BM_RSMODE1
#define BM_RSMODE_LOG2LIN       (BM_RSMODE1 | BM_RSMODE0)

#define LFRSSI_LOW_BYTE         (uint8_t)0
#define LFRSSI_HIGH_BYTE        (uint8_t)1

/* defines for sLfRssi.bFlags */
#define LFRSSI_FLAGS_RESET                              (uint8_t)0x00
#define LFRSSI_FLAGS_ERROR_FLAG                         BIT_7
#define LFRSSI_FLAGS_BM_ERROR_FLAG                      BITMASK(LFRSSI_FLAGS_ERROR_FLAG)
#define LFRSSI_FLAGS_MEASUREMENT_READY_FLAG             BIT_6
#define LFRSSI_FLAGS_BM_MEASUREMENT_READY_FLAG          BITMASK(LFRSSI_FLAGS_MEASUREMENT_READY_FLAG)
#define LFRSSI_FLAGS_MEASUREMENT_ABORTED_FLAG           BIT_5
#define LFRSSI_FLAGS_BM_MEASUREMENT_ABORTED_FLAG        BITMASK(LFRSSI_FLAGS_MEASUREMENT_ABORTED_FLAG)
#define LFRSSI_FLAGS_CHANNEL_ENABLE_MISMATCH_FLAG       BIT_4
#define LFRSSI_FLAGS_BM_CHANNEL_ENABLE_MISMATCH_FLAG    BITMASK(LFRSSI_FLAGS_CHANNEL_ENABLE_MISMATCH_FLAG)

/* defines for sLfRssi.bStatus */
#define LFRSSI_STATUS_RESET                             (uint8_t)0x00
#define LFRSSI_STATUS_MEAS_DATA_AVAILABLE_FLAG          BIT_7
#define LFRSSI_STATUS_BM_MEAS_DATA_AVAILABLE_FLAG       BITMASK(LFRSSI_STATUS_MEAS_DATA_AVAILABLE_FLAG)
#define LFRSSI_STATUS_CHANCORR_DATA_AVAILABLE_FLAG      BIT_6
#define LFRSSI_STATUS_BM_CHANCORR_DATA_AVAILABLE_FLAG   BITMASK(LFRSSI_STATUS_CHANCORR_DATA_AVAILABLE_FLAG)
#define LFRSSI_STATUS_3DVEC_DATA_AVAILABLE_FLAG         BIT_5
#define LFRSSI_STATUS_BM_3DVEC_DATA_AVAILABLE_FLAG      BITMASK(LFRSSI_STATUS_3DVEC_DATA_AVAILABLE_FLAG)
#define LFRSSI_STATUS_LINEAR_DATA_AVAILABLE_FLAG        BIT_4
#define LFRSSI_STATUS_BM_LINEAR_DATA_AVAILABLE_FLAG     BITMASK(LFRSSI_STATUS_LINEAR_DATA_AVAILABLE_FLAG)
#define LFRSSI_STATUS_INTERNAL_MEASUREMENT_FLAG         BIT_3
#define LFRSSI_STATUS_BM_INTERNAL_MEASUREMENT_FLAG      BITMASK(LFRSSI_STATUS_INTERNAL_MEASUREMENT_FLAG)
#define LFRSSI_STATUS_CHANNELS_OUTOFRANGE_FLAG          BIT_2
#define LFRSSI_STATUS_BM_CHANNELS_OUTOFRANGE_FLAG       BITMASK(LFRSSI_STATUS_CHANNELS_OUTOFRANGE_FLAG)
#define LFRSSI_STATUS_SIGNDETECT_ENA_FLAG               BIT_1
#define LFRSSI_STATUS_BM_SIGNDETECT_ENA_FLAG            BITMASK(LFRSSI_STATUS_SIGNDETECT_ENA_FLAG)
#define LFRSSI_STATUS_OPERATION_ACTIVE_FLAG             BIT_0
#define LFRSSI_STATUS_BM_OPERATION_ACTIVE_FLAG          BITMASK(LFRSSI_STATUS_OPERATION_ACTIVE_FLAG)


/* defines for register configuration structure, must look like this:
    typedef struct
    {
        uint8_t  bRscr;
        uint8_t  bRsms1r;
        uint8_t  bRsms2r;
        uint8_t  bRsdlyr;
        uint8_t  bRssrcr;

    }sLfRssiRegConfig;
*/
/* offsets to structure elements for bytewise access */
#define LFRSSI_BYTEOFFSET_REGCONFIG_RSCR            0U                                          // 00<->00
#define LFRSSI_BYTEOFFSET_REGCONFIG_RSMS1R          LFRSSI_BYTEOFFSET_REGCONFIG_RSCR  + 1U      // 01<->01
#define LFRSSI_BYTEOFFSET_REGCONFIG_RSMS2R          LFRSSI_BYTEOFFSET_REGCONFIG_RSMS1R + 1U     // 02<->02
#define LFRSSI_BYTEOFFSET_REGCONFIG_RSDLYR          LFRSSI_BYTEOFFSET_REGCONFIG_RSMS2R + 1U     // 03<->03
#define LFRSSI_BYTEOFFSET_REGCONFIG_RSSRCR          LFRSSI_BYTEOFFSET_REGCONFIG_RSDLYR + 1U     // 04<->04


/* defines for result structure, must look like this without padding bytes:
   typedef struct
   {
        uint16_t wRawLfRssi[3];
        uint8_t  bSignDetect[4];
        uint16_t wCorrLfRssi[3];
        uint16_t w3dVecVal;
        uint16_t wLinearVal;
    }sLfRssiResult;
*/

/* offsets to structure elements of for bytewise access */
#define LFRSSI_BYTEOFFSET_RESULT_RSC1L              0U                                      // 00<->00
#define LFRSSI_BYTEOFFSET_RESULT_RSC1H              LFRSSI_BYTEOFFSET_RESULT_RSC1L + 1U     // 01<->01
#define LFRSSI_BYTEOFFSET_RESULT_RSC2L              LFRSSI_BYTEOFFSET_RESULT_RSC1H + 1U     // 02<->02
#define LFRSSI_BYTEOFFSET_RESULT_RSC2H              LFRSSI_BYTEOFFSET_RESULT_RSC2L + 1U     // 03<->03
#define LFRSSI_BYTEOFFSET_RESULT_RSC3L              LFRSSI_BYTEOFFSET_RESULT_RSC2H + 1U     // 04<->04
#define LFRSSI_BYTEOFFSET_RESULT_RSC3H              LFRSSI_BYTEOFFSET_RESULT_RSC3L + 1U     // 05<->05

/* offsets to structure elements to access sign detection variables */
#define LFRSSI_BYTEOFFSET_RESULT_SD12RR             0U                                      // 00<->00
#define LFRSSI_BYTEOFFSET_RESULT_SD13RR             LFRSSI_BYTEOFFSET_RESULT_SD12RR + 1U    // 01<->01
#define LFRSSI_BYTEOFFSET_RESULT_SD23RR             LFRSSI_BYTEOFFSET_RESULT_SD13RR + 1U    // 02<->02
#define LFRSSI_BYTEOFFSET_RESULT_SD360R             LFRSSI_BYTEOFFSET_RESULT_SD23RR + 1U    // 03<->03


#define LFRSSI_INTOFFSET_REF_VAL_CHAN1              0U                                      // 00<->00
#define LFRSSI_INTOFFSET_REF_VAL_CHAN2              LFRSSI_INTOFFSET_REF_VAL_CHAN1 + 1U     // 01<->01
#define LFRSSI_INTOFFSET_REF_VAL_CHAN3              LFRSSI_INTOFFSET_REF_VAL_CHAN2 + 1U     // 02<->02

#define LFRSSI_INTOFFSET_NORM_VAL_CHAN1             0U                                      // 00<->00
#define LFRSSI_INTOFFSET_NORM_VAL_CHAN2             LFRSSI_INTOFFSET_NORM_VAL_CHAN1 + 1U    // 01<->01
#define LFRSSI_INTOFFSET_NORM_VAL_CHAN3             LFRSSI_INTOFFSET_NORM_VAL_CHAN2 + 1U    // 02<->02

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
typedef struct
{
    uint8_t  bRscr;
    uint8_t  bRsms1r;
    uint8_t  bRsms2r;
    uint8_t  bRsdlyr;
    uint8_t  bRssrcr;

}sLfRssiRegConfig;

typedef struct
{
    /** \brief <b>bFlags</b>
        is used to signalize events outside to the LF RSSI module occurred during LF RSSI measurement.
        \li Bit 7:      LF RSSI module error
                        0: no errors occurred during LF RSSI measurement
                        1: errors occurred during LF RSSI measurement, see debug.errorCode for closer description
        \li Bit 6:      LF RSSI measurement ready
                        0: LF RSSI measurement not started or in progress
                        1: LF RSSI measurement done
        \li Bit 5:      LF RSSI measurement aborted
                        0: LF RSSI measurement not aborted
                        1: LF RSSI measurement aborted
        \li Bit 4:      LF channels enabled for LF RSSI measurement including sign detection
                        0: All LF channels are enabled for LF RSSI measurement
                        1: Not all LF channels are enabled for LF RSSI measurement
        \li Bit 3..0:   rfu
    */
    uint8_t bFlags;

    /** \brief <b>bStatus</b>
        is used as signalize events inside the LF RSSI module occurred during LF RSSI measurement.
        \li Bit 7:      LF RSSI measurement data available
                        0: no LF RSSI measurement data available
                        1: new LF RSSI measurement data available
        \li Bit 6:      LF RSSI channel correction data available
                        0: no LF RSSI channel correction data available
                        1: new LF RSSI channel correction data available
        \li Bit 5:      LF RSSI 3d vector data available
                        0: no LF RSSI 3d vector available
                        1: new LF RSSI 3d vector available
        \li Bit 4:      LF RSSI linear data available
                        0: no LF RSSI linear data available
                        1: new LF RSSI linear data available
        \li Bit 3:      LF RSSI measurement mode
                        0: external LF RSSI measurement
                        1: internal LF RSSI measurement
        \li Bit 2:      Channel out of range warning
                        0: no timeout(s) on participated measurement channel(s) detected
                        1: timeout(s) on participated measurement channel(s) detected
        \li Bit 1:      LF RSSI measurement sign detection flag
                        0: LF RSSI measurement without sign detection
                        1: LF RSSI measurement with sign detection
        \li Bit 0:      LF RSSI operation active
                        0: no LF RSSI operation active
                        1: LF RSSI operation in progress
    */
    uint8_t bStatus;

    /** \brief <b>bPrr1</b>
        is used to store the PRR1 register value temporarily as long as the LF RSSI measurement
        has been enabled.
    */
    uint8_t bPrr1;

    /** \brief <b>bTpcr2</b>
        is used to store the TPCR2 register value temporarily as long as the LF RSSI measurement
        has been enabled.
    */
    uint8_t bTpcr2;

    /** \brief <b>bSrcCal</b>
        is used to store the global SRCCAL register value temporarily. In case an internal LF RSSI measurement
        has to be done, it is possible to calibrate the system SRC clock, e.g. to compensate
        temperature drifts, just prior the LF field is determined. After measurement has been finished,
        the srcCal value is restored to the SRCCAL register.
    */
    uint8_t bSrcCal;

    /** \brief <b>bLfcr1</b>
        is used to store the LFCR1 register value temporarily as long as the LF RSSI measurement
        has been enabled.
    */
    uint8_t bLfcr1;

    /** \brief <b>bChanCalibVal[6]</b>
        is used to store the calculated channel correction values RSSIcal.
        \li bChanCalibVal[1..0] contains the calculated correction values for LF channel 1.
        \li bChanCalibVal[3..2] contains the calculated correction values for LF channel 2.
        \li bChanCalibVal[5..4] contains the calculated correction values for LF channel 3.
    */
    uint8_t bChanCalibVal[6];

    /** \brief <b>bOutOfRangeMask</b>
        is used to store the LF channel out of range signalization after LF RSSI measurement
        signals availability of data. The mask is an AND combination of RSFR and LFCR0
        register.
        \li Bit 2: LF receiver channel 3 signals out of range
        \li Bit 1: LF receiver channel 2 signals out of range
        \li Bit 0: LF receiver channel 1 signals out of range
    */
    uint8_t bOutOfRangeMask;

}sLfRssi;

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
extern sLfRssi g_sLfRssi;

extern VOIDFUNC   ATA_lfRssiInit_C(void);
extern VOIDFUNC   ATA_lfRssiOpen_C(void);
extern VOIDFUNC   ATA_lfRssiClose_C(void);
extern VOIDFUNC   ATA_lfRssiSetEepromConfig_C(uint8_t bEepRssiSrcCalVal);
extern VOIDFUNC   ATA_lfRssiMeasStart_C(void *pConfig, uint8_t bMode, uint8_t bSign);
extern VOIDFUNC   ATA_lfRssiMeasStop_C(void);
extern VOIDFUNC   ATA_lfRssiGetAverageResult_C(uint8_t *pResMeas, uint8_t *pResSignDetect);
extern VOIDFUNC   ATA_lfRssiGetSamplesResult_C(uint8_t *pResMeas, uint8_t bNum, uint8_t bIndex);
extern VOIDFUNC   ATA_lfRssiCalcChanCalibVal_C(uint8_t bMargin, uint16_t *pRefData, uint16_t *pNormData);
extern VOIDFUNC   ATA_lfRssiCalcChanCorr_C(uint8_t *pExtData, uint8_t *pIntData, uint8_t *pResult);
extern VOIDFUNC   ATA_lfRssiCalc3dVec_C(uint8_t *pData, uint8_t *pResult);
extern VOIDFUNC   ATA_lfRssiCalcLog2Lin_C(uint8_t *p3dVec, uint8_t *pResult);
extern UINT16FUNC ATA_lfRssiCalcBappl_C(uint16_t wBref, uint16_t wRssiValue);
extern UINT16FUNC ATA_lfRssiNormalize12q4_C(uint32_t dwValue, int8_t exponent);

#elif defined __IAR_SYSTEMS_ASM__

#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* LFRSSI_H */
