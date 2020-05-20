//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/calib/src/calib.h $
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

/** \file calib.h
 */

//lint -restore

#ifndef CALIB_H
#define CALIB_H

#ifdef __IAR_SYSTEMS_ICC__
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
#include "../../rftx/src/rftx_defs.h"
#include "../../globals/src/globals.h"
#include "../../timer2/src/timer2.h"


/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

// sCalibConfig
// sCalibConfig.flags
#define CALIB_CONFIG_FLAGS_ERROR     BIT_7
#define BM_CALIB_CONFIG_FLAGS_ERROR  BITMASK(CALIB_CONFIG_FLAGS_ERROR)
// sCalibConfig.status

// sCalibConfig.config
// bit 7:  FRC calibration enable/disable flag
//             0: FRC calibration disabled
//             1: FRC calibration enabled
#define CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC  BIT_7
// bit 6:  SRC calibration enable/disable flag
//             0: SRC calibration disabled
//             1: SRC calibration enabled
#define CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC  BIT_6
// bit 1:  XTO Active after SRC/FRC calibration flag
//             0: XTO is deactivated after SRC/FRC calibration
//             1: XTO is still activated after SRC/FRC calibration
#define CALIB_CONFIG_SRC_FRC_CALIBRATION_XTO     BIT_1
// bit 0:  SRC/FRC calibration clock
//             0: SRC/FRC calibration via XTO_CLK
//             1: SRC/FRC calibration via EXT_CLK
#define CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK   BIT_0

#define BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC   BITMASK(CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_FRC)
#define BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC   BITMASK(CALIB_CONFIG_SRC_FRC_CALIBRATION_ENABLE_SRC)
#define BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_XTO          BITMASK(CALIB_CONFIG_SRC_FRC_CALIBRATION_XTO)
#define BM_CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK        BITMASK(CALIB_CONFIG_SRC_FRC_CALIBRATION_CLOCK)

/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/

/*----------------------------------------------------------------------------- */
/** \brief <b>sEEPromSrcCalibrationConfig</b>
    \brief <b>sSrcCalibrationConfig</b>
    contains the configuration data for SRC Calibration
 */
/*----------------------------------------------------------------------------- */
typedef struct {
    /** \brief <b>xtoCyclesPerStep</b>
        contains the number of XTO/4(XTO/8) cycles for SRC/FRC calibration
        measurement gate
     */
    uint16_t wXtoCyclesPerMeasurement;
    /** \brief <b>gradient</b>
        contains the gradient in unsigned fractional format q2.6
     */
    uint8_t bGradient;
}sSrcCalibrationConfig, sEEPromSrcCalibrationConfig;

/*----------------------------------------------------------------------------- */
/** \brief <b>sEEPromFrcCalibrationConfig</b>
    \brief <b>sFrcCalibrationConfig</b>
    contains the configuration data for FRC Calibration
 */
/*----------------------------------------------------------------------------- */
typedef struct {
    /** \brief <b>T3CORL</b>
     */
    uint8_t bT3CORL;
    /** \brief <b>T3CORH</b>
     */
    uint8_t bT3CORH;
}sFrcCalibrationConfig, sEEPromFrcCalibrationConfig;
/*----------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------- */
typedef struct{
    /*
        bit 7:      calib Module error flag
        bit 6..0:   tbd
     */
    uint8_t bFlags;
    /*
        bit 7..0:   tbd
     */
    uint8_t bStatus;
    /*
        bit 7:  FRC calibration enable/disable flag
                    0: FRC calibration disabled
                    1: FRC calibration enabled
        bit 6:  SRC calibration enable/disable flag
                    0: SRC calibration disabled
                    1: SRC calibration enabled
        bit 6:  rfu
        bit 5:  rfu
        bit 4:  rfu
        bit 2:  rfu
        bit 1:  XTO Active after SRC/FRC calibration flag
                    0: XTO is deactivated after SRC/FRC calibration
                    1: XTO is still activated after SRC/FRC calibration
        bit 0:  SRC/FRC calibration clock
                    0: SRC/FRC calibration via XTO_CLK
                    1: SRC/FRC calibration via EXT_CLK
     */
    uint8_t bConfig;
    
    /** \brief <b>wSrcResult</b>
        contains the measured number of XTO/4(XTO/8) cycles for SRC 
        calibration
     */
    uint16_t wSrcResult;
    
    /** \brief <b>sSrcCalibrationConfig</b>
        contains the EEPROM SRC calibration configuration
     */
    sSrcCalibrationConfig sSrcCalibrationConfig;
    
    /** \brief <b>wFrcResult</b>
        contains the measured number of XTO/4(XTO/8) cycles for FRC 
        calibration
     */
    uint16_t wFrcResult;
    
    /** \brief <b>sFrcCalibrationConfig</b>
        contains the EEPROM FRC calibration configuration
     */
    sFrcCalibrationConfig sFrcCalibrationConfig;

}sCalibConfig;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
extern sCalibConfig g_sCalibConfig;
extern sEEPromSrcCalibrationConfig g_sEepCalibSrcCalibrationConfig;
extern sEEPromFrcCalibrationConfig g_sEepCalibFrcCalibrationConfig;


extern VOIDFUNC ATA_calibInit_C(void);
extern VOIDFUNC ATA_calibClose_C(void);
extern VOIDFUNC ATA_calibStartCalibration_C(uint8_t config);
extern VOIDFUNC ATA_calibStartSrcCalibration_C(void);
extern VOIDFUNC ATA_calibStartFrcCalibration_C(void);

#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
/*stopSimExtraction*/
#endif

#endif /* CALIB_H */
