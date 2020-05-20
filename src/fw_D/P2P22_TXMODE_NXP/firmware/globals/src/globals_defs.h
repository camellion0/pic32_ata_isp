//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/globals/src/globals_defs.h $
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
//lint -restore

#ifndef GLOBALS_DEFS_H
#define GLOBALS_DEFS_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
#define FUNCTION_TRACE_C
// #undef FUNCTION_TRACE_C
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_SETTRACEPOINT_C</b>
    sets a trace point at the current address
    \param[in]      id      Trace ID
    \param[in]      data    Trace data
 */
/*---------------------------------------------------------------------------*/
#ifdef FUNCTION_TRACE_C
#define ATA_SET_FUNCTION_TRACE_POINT_C(id, data) {          \
        uint8_t bSregTrace = SREG;                          \
        __disable_interrupt();                              \
        ATA_SET_FUNCTION_TRACE_POINT_ISR_C(id, data)        \
        SREG = bSregTrace;                                  \
    }
#define ATA_SET_FUNCTION_TRACE_POINT_ISR_C(id, data) {      \
        TRCDR  = data;                                      \
        TRCIDL = (uint8_t) ((uint16_t)id >> 0);             \
        TRCIDH = (uint8_t) ((uint16_t)id >> 8);             \
    }
#else
#define ATA_SET_FUNCTION_TRACE_POINT_C(id, data)
#define ATA_SET_FUNCTION_TRACE_POINT_ISR_C(id, data)
#endif



/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_SETBIT_C</b>
    sets the bit bit_number in variable data
 */
/*---------------------------------------------------------------------------*/
#define ATA_SETBIT_C(data, bit_number) {  \
    data |= (1 << bit_number);          \
    }

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_CLEARBIT_C</b>
    clears the bit bit_number in variable data
 */
/*---------------------------------------------------------------------------*/
#define ATA_CLEARBIT_C(data, bit_number) { \
    data &= (uint8_t)~(1<<bit_number);     \
    }

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_SETBIT_C</b>
    sets the bits of bit_mask in variable data
 */
/*---------------------------------------------------------------------------*/
#define ATA_SETBITMASK_C(data, bit_mask) {  \
    data |= (bit_mask);                 \
    }

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_CLEARBITMASK_C</b>
    clears the bits of bit_mask in variable data
 */
/*---------------------------------------------------------------------------*/
#define ATA_CLEARBITMASK_C(data, bit_mask) { \
    data &= (uint8_t)~(bit_mask);        \
    }
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_POWERON_C</b>
    enables the Power via PRR register
 */
/*---------------------------------------------------------------------------*/
#define ATA_POWERON_C(data, bit_number)   ATA_CLEARBIT_C(data, bit_number)
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_POWERON_C</b>
    disables the Power via PRR register
 */
/*---------------------------------------------------------------------------*/
#define ATA_POWEROFF_C(data, bit_number)  ATA_SETBIT_C(data, bit_number)

#define BITMASK(name)   (uint8_t)(1U << name)

/* bits and masks */
#define BIT_7      7
#define BIT_6      6
#define BIT_5      5
#define BIT_4      4
#define BIT_3      3
#define BIT_2      2
#define BIT_1      1
#define BIT_0      0

#define BIT_MASK_7     BITMASK(BIT_7)
#define BIT_MASK_6     BITMASK(BIT_6)
#define BIT_MASK_5     BITMASK(BIT_5)
#define BIT_MASK_4     BITMASK(BIT_4)
#define BIT_MASK_3     BITMASK(BIT_3)
#define BIT_MASK_2     BITMASK(BIT_2)
#define BIT_MASK_1     BITMASK(BIT_1)
#define BIT_MASK_0     BITMASK(BIT_0)

/* Bits and Byte masking */
#define MASK_HIGH_NIBBLE            (uint8_t)0x0FU
#define MASK_LOW_NIBBLE             (uint8_t)0xF0U
#define MASK_HIGH_BYTE              (uint16_t)0x00FFU
#define MASK_LOW_BYTE               (uint16_t)0xFF00U

/** \brief <b>SHIFT_HIGH_TO_LOW_BYTE</b>
    defines the number of bits to shift a high byte to low byte position
    (Integer with 16-Bit)
*/
#define SHIFT_HIGH_TO_LOW_BYTE       (uint8_t)0x08U

/** \brief <b>SHIFT_LOW_TO_HIGH_BYTE</b>
    defines the number of bits to shift a low byte to high byte position
    (Integer with 16-Bit)
*/
#define SHIFT_LOW_TO_HIGH_BYTE      (uint8_t)0x08U

/** \brief <b>NUM_EEPROM_SERVICES</b>
    is the number of services located in EEPROM
*/
#define NUM_EEPROM_SERVICES          3U

/** \brief <b>NUM_SRAM_SERVICES</b>
    is the number of services located in SRAM
*/
#define NUM_SRAM_SERVICES            2U

/** \brief <b>NUM_SERVICES</b>
    is the total number of services in the application
*/
#define NUM_SERVICES                (NUM_EEPROM_SERVICES + NUM_SRAM_SERVICES)

/** \brief <b>NUM_CHANNELS_PER_SERVICE</b>
    is the number of channels located in one service
*/
#define NUM_CHANNELS_PER_SERVICE    3U

/** \brief <b>NUM_PATHES_PER_SERVICE</b>
    is the number of pathes located in one service
*/
#define NUM_PATHES_PER_SERVICE      1U

/** \brief <b>DEBUG_ERROR_CODE_xxx</b>
    are the errorCodes for ATA5700 SW
*/
#define DEBUG_ERROR_CODE_SYSTEM_ERROR_EEPROM_NOT_VALID                  0U                                                                  // 000<->00
#define DEBUG_ERROR_CODE_SETIDLEMODE_MISS_SSMRDY                        DEBUG_ERROR_CODE_SYSTEM_ERROR_EEPROM_NOT_VALID + 1U                 // 001<->01
#define DEBUG_ERROR_CODE_SETIDLEMODE_TIMER_LOCKED                       DEBUG_ERROR_CODE_SETIDLEMODE_MISS_SSMRDY + 1U                       // 002<->02
#define DEBUG_ERROR_CODE_RX_STATESTARTSSM_TIMER_LOCKED                  DEBUG_ERROR_CODE_SETIDLEMODE_TIMER_LOCKED + 1U                      // 003<->03
#define DEBUG_ERROR_CODE_RX_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT      DEBUG_ERROR_CODE_RX_STATESTARTSSM_TIMER_LOCKED + 1U                 // 004<->04
#define DEBUG_ERROR_CODE_TX_STATESTARTSSM_TIMER_LOCKED                  DEBUG_ERROR_CODE_RX_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT + 1U     // 005<->05
#define DEBUG_ERROR_CODE_TX_STATEWAIT4SSMRDY_TIMEOUT                    DEBUG_ERROR_CODE_TX_STATESTARTSSM_TIMER_LOCKED + 1U                 // 006<->06
#define DEBUG_ERROR_CODE_POLL_STATESTARTSSM_TIMER_LOCKED                DEBUG_ERROR_CODE_TX_STATEWAIT4SSMRDY_TIMEOUT + 1U                   // 007<->07
#define DEBUG_ERROR_CODE_POLL_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT    DEBUG_ERROR_CODE_POLL_STATESTARTSSM_TIMER_LOCKED + 1U               // 008<->08
#define DEBUG_ERROR_CODE_ANTTUNE_STATESTARTSSM_TIMER_LOCKED             DEBUG_ERROR_CODE_POLL_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT + 1U   // 009<->09
#define DEBUG_ERROR_CODE_ANTTUNE_STATEWAIT4SSMRDY_TIMEOUT               DEBUG_ERROR_CODE_ANTTUNE_STATESTARTSSM_TIMER_LOCKED + 1U            // 010<->0A
#define DEBUG_ERROR_CODE_VCOCAL_STATESTARTSSM_TIMER_LOCKED              DEBUG_ERROR_CODE_ANTTUNE_STATEWAIT4SSMRDY_TIMEOUT + 1U              // 011<->0B
#define DEBUG_ERROR_CODE_VCOCAL_STATEWAIT4SSMRDY_TIMEOUT                DEBUG_ERROR_CODE_VCOCAL_STATESTARTSSM_TIMER_LOCKED + 1U             // 012<->0C
#define DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMER_LOCKED                       DEBUG_ERROR_CODE_VCOCAL_STATEWAIT4SSMRDY_TIMEOUT + 1U               // 013<->0D
#define DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMEOUT                            DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMER_LOCKED + 1U                      // 014<->0E
#define DEBUG_ERROR_CODE_POLLING_TIMER1_LOCKED                          DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMEOUT + 1U                           // 015<->0F
#define DEBUG_ERROR_CODE_INVALID_OPM_SWITCHING                          DEBUG_ERROR_CODE_POLLING_TIMER1_LOCKED + 1U                         // 016<->10
#define DEBUG_ERROR_CODE_INVALID_OPM_MODE_DURING_TUNE_AND_CHECK         DEBUG_ERROR_CODE_INVALID_OPM_SWITCHING + 1U                         // 017<->11
#define DEBUG_ERROR_CODE_AVCCLOW_DURING_TX                              DEBUG_ERROR_CODE_INVALID_OPM_MODE_DURING_TUNE_AND_CHECK + 1U        // 018<->12
#define DEBUG_ERROR_CODE_SERVICE_INIT_FAILURE                           DEBUG_ERROR_CODE_AVCCLOW_DURING_TX + 1U                             // 019<->13
#define DEBUG_ERROR_CODE_DFIFO_OVER_UNDER_FLOW                          DEBUG_ERROR_CODE_SERVICE_INIT_FAILURE + 1U                          // 020<->14
#define DEBUG_ERROR_CODE_SFIFO_OVER_UNDER_FLOW                          DEBUG_ERROR_CODE_DFIFO_OVER_UNDER_FLOW + 1U                         // 021<->15
#define DEBUG_ERROR_CODE_RSSI_STATESTARTSSM_TIMER_LOCKED                DEBUG_ERROR_CODE_SFIFO_OVER_UNDER_FLOW + 1U                         // 022<->16
#define DEBUG_ERROR_CODE_SRC_FRC_CALIB_FAILED                           DEBUG_ERROR_CODE_RSSI_STATESTARTSSM_TIMER_LOCKED + 1U               // 023<->17
#define DEBUG_ERROR_CODE_GETRXTELEGRAM_SSM_ERROR                        DEBUG_ERROR_CODE_SRC_FRC_CALIB_FAILED + 1U                          // 024<->18
#define DEBUG_ERROR_CODE_TEMPMEAS_STATESTARTSSM_TIMER_LOCKED            DEBUG_ERROR_CODE_GETRXTELEGRAM_SSM_ERROR + 1U                       // 025<->19
#define DEBUG_ERROR_CODE_TEMPMEAS_STATEWAIT4SSMRDY_TIMEOUT              DEBUG_ERROR_CODE_TEMPMEAS_STATESTARTSSM_TIMER_LOCKED + 1U           // 026<->1A
#define DEBUG_ERROR_CODE_AVCCLOW_TIMEOUT                                DEBUG_ERROR_CODE_TEMPMEAS_STATEWAIT4SSMRDY_TIMEOUT + 1U             // 027<->1B
// error codes for module SPI
#define DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED                            DEBUG_ERROR_CODE_AVCCLOW_TIMEOUT + 1U                               // 028<->1C
#define DEBUG_ERROR_CODE_SPI_ALREADY_OPENED                             DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED + 1U                           // 029<->1D
// error codes for module ATA5831
#define DEBUG_ERROR_CODE_ATA5831_SPI_CHECKSUM_ERROR                     DEBUG_ERROR_CODE_SPI_ALREADY_OPENED + 1U                            // 030<->1E
// error codes for module CALIB
#define DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED                      DEBUG_ERROR_CODE_ATA5831_SPI_CHECKSUM_ERROR + 1U                    // 031<->1F
#define DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED                      DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED + 1U                     // 032<->20
#define DEBUG_ERROR_CODE_CALIB_EXTERNAL_CLOCK_FAILURE                   DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED + 1U                     // 033<->21
// error codes for module RFTX
#define DEBUG_ERROR_CODE_RFTX_STARTSSM_TIMER_LOCKED                     DEBUG_ERROR_CODE_CALIB_EXTERNAL_CLOCK_FAILURE + 1U                  // 034<->22
#define DEBUG_ERROR_CODE_RFTX_SHUTDOWN_ERROR                            DEBUG_ERROR_CODE_RFTX_STARTSSM_TIMER_LOCKED + 1U                    // 035<->23
#define DEBUG_ERROR_CODE_RFTX_WAIT4SSMRDY_TIMEOUT                       DEBUG_ERROR_CODE_RFTX_SHUTDOWN_ERROR + 1U                           // 036<->24
#define DEBUG_ERROR_CODE_RFTX_SFIFO_ERROR                               DEBUG_ERROR_CODE_RFTX_WAIT4SSMRDY_TIMEOUT + 1U                      // 037<->25
#define DEBUG_ERROR_CODE_RFTX_DFIFO_ERROR                               DEBUG_ERROR_CODE_RFTX_SFIFO_ERROR + 1U                              // 038<->26
#define DEBUG_ERROR_CODE_RFTX_WAIT4VCOSSMRDY_TIMEOUT                    DEBUG_ERROR_CODE_RFTX_DFIFO_ERROR + 1U                              // 039<->27
#define DEBUG_ERROR_CODE_RFTX_WAIT4ANTSSMRDY_TIMEOUT                    DEBUG_ERROR_CODE_RFTX_WAIT4VCOSSMRDY_TIMEOUT + 1U                   // 040<->28
// error codes for module LF RSSI
#define DEBUG_ERROR_CODE_LFRSSI_EEPROM_READ_ERROR                       DEBUG_ERROR_CODE_RFTX_WAIT4ANTSSMRDY_TIMEOUT + 1U                   // 041<->29
#define DEBUG_ERROR_CODE_LFRSSI_EEPROM_ACCESS_ERROR                     DEBUG_ERROR_CODE_LFRSSI_EEPROM_READ_ERROR + 1U                      // 042<->2A
#define DEBUG_ERROR_CODE_LFRSSI_ANALOGPART_NOTREADY                     DEBUG_ERROR_CODE_LFRSSI_EEPROM_ACCESS_ERROR + 1U                    // 043<->2B
#define DEBUG_ERROR_CODE_LFRSSI_INVALID_MEASUREMENT_CHANNELS            DEBUG_ERROR_CODE_LFRSSI_ANALOGPART_NOTREADY + 1U                    // 044<->2C
#define DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED                     DEBUG_ERROR_CODE_LFRSSI_INVALID_MEASUREMENT_CHANNELS + 1U           // 045<->2D
#define DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE                        DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED + 1U                    // 046<->2E
#define DEBUG_ERROR_CODE_LFRSSI_CHANNELS_OUTOFRANGE                     DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE + 1U                       // 047<->2F
// error codes for module LF RX
#define DEBUG_ERROR_CODE_LFRX_EEPROM_READ_ERROR                         DEBUG_ERROR_CODE_LFRSSI_CHANNELS_OUTOFRANGE + 1U                    // 048<->30
#define DEBUG_ERROR_CODE_LFRX_EEPROM_ACCESS_ERROR                       DEBUG_ERROR_CODE_LFRX_EEPROM_READ_ERROR + 1U                        // 049<->31
// error codes for module AES
#define DEBUG_ERROR_CODE_AES_ALREADY_IN_USE_ERROR                       DEBUG_ERROR_CODE_LFRX_EEPROM_ACCESS_ERROR + 1U                      // 050<->32
#define DEBUG_ERROR_CODE_AES_INVALID_KEY_ERROR                          DEBUG_ERROR_CODE_AES_ALREADY_IN_USE_ERROR + 1U                      // 051<->33
#define DEBUG_ERROR_CODE_AES_RUN_ERROR                                  DEBUG_ERROR_CODE_AES_INVALID_KEY_ERROR + 1U                         // 052<->34
// error codes for module RFRCC
#define DEBUG_ERROR_CODE_RFRCC_EEP_READ_ERROR                           DEBUG_ERROR_CODE_AES_RUN_ERROR + 1U                                 // 053<->35
#define DEBUG_ERROR_CODE_RFRCC_EEP_WRITE_ACCESS_ERROR                   DEBUG_ERROR_CODE_RFRCC_EEP_READ_ERROR + 1U                          // 054<->36
#define DEBUG_ERROR_CODE_RFRCC_MAC_ERROR                                DEBUG_ERROR_CODE_RFRCC_EEP_WRITE_ACCESS_ERROR + 1U                  // 055<->37
#define DEBUG_ERROR_CODE_RFRCC_SUBKEY_ERROR                             DEBUG_ERROR_CODE_RFRCC_MAC_ERROR + 1U                               // 056<->38
// error codes for module TWI
#define DEBUG_ERROR_CODE_TWI_NACK_SLAW_ERROR                            DEBUG_ERROR_CODE_RFRCC_SUBKEY_ERROR + 1U                            // 057<->39
#define DEBUG_ERROR_CODE_TWI_NACK_DATA_TX_ERROR                         DEBUG_ERROR_CODE_TWI_NACK_SLAW_ERROR + 1U                           // 058<->3A
#define DEBUG_ERROR_CODE_TWI_STATUS_CODE_ERROR                          DEBUG_ERROR_CODE_TWI_NACK_DATA_TX_ERROR + 1U                        // 059<->3B
#define DEBUG_ERROR_CODE_TWI_NACK_SLAR_ERROR                            DEBUG_ERROR_CODE_TWI_STATUS_CODE_ERROR + 1U                         // 060<->3C
#define DEBUG_ERROR_CODE_TWI_INVALID_REQUEST_ERROR                      DEBUG_ERROR_CODE_TWI_NACK_SLAR_ERROR + 1U                           // 061<->3D

// error codes for module CALIB (extended)
#define DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR                        DEBUG_ERROR_CODE_TWI_INVALID_REQUEST_ERROR + 1U                     // 062<->3E

// error codes for module RFTX (extended)
#define DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR                         DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR + 1U                       // 063<->3F

// error codes for module Globals
#define DEBUG_ERROR_CODE_GLOBALS_EEPROM_READ_ERROR                      DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR + 1U                        // 064<->40

// error code indicating NOT USED
#define DEBUG_ERROR_CODE_SYSTEM_ERROR_NOT_USED                          0xFFU                                                               // 255<->FF

/* ------------------------------------------------------------------------- */
/* tuneCheckConfig_t                                                 */
/* ------------------------------------------------------------------------- */
#define TUNE_CHECK_CONFIG_ANTENNA_TUNING    BIT_7
#define TUNE_CHECK_CONFIG_TEMP_MEASUREMENT  BIT_6
#define TUNE_CHECK_CONFIG_SRC_CALIB         BIT_5
#define TUNE_CHECK_CONFIG_FRC_CALIB         BIT_4
#define TUNE_CHECK_CONFIG_VCO_CALIB         BIT_3
#define TUNE_CHECK_CONFIG_RF_CALIB          BIT_2
#define TUNE_CHECK_CONFIG_SELF_CHECK        BIT_1
#define TUNE_CHECK_CONFIG_REG_REFRESH       BIT_0

#define BM_TUNE_CHECK_CONFIG_ANTENNA_TUNING    BIT_MASK_7
#define BM_TUNE_CHECK_CONFIG_TEMP_MEASUREMENT  BIT_MASK_6
#define BM_TUNE_CHECK_CONFIG_SRC_CALIB         BIT_MASK_5
#define BM_TUNE_CHECK_CONFIG_FRC_CALIB         BIT_MASK_4
#define BM_TUNE_CHECK_CONFIG_VCO_CALIB         BIT_MASK_3
#define BM_TUNE_CHECK_CONFIG_RF_CALIB          BIT_MASK_2
#define BM_TUNE_CHECK_CONFIG_SELF_CHECK        BIT_MASK_1
#define BM_TUNE_CHECK_CONFIG_REG_REFRESH       BIT_MASK_0
/* ------------------------------------------------------------------------- */
/* sysModeConfig_t                                                           */
/* ------------------------------------------------------------------------- */
#define SYS_MODE_CONFIG_RF_CALIBRATION          BIT_7
#define SYS_MODE_CONFIG_ANTENNA_TUNING          BIT_6
#define SYS_MODE_CONFIG_VCO_TUNING              BIT_5
#define SYS_MODE_CONFIG_IDLE_MODE_SELECTOR      BIT_4
#define SYS_MODE_CONFIG_DIRECT_SWITCH           BIT_3
#define SYS_MODE_CONFIG_TRANSPARENT_MODE        BIT_2
#define SYS_MODE_CONFIG_OPM1                    BIT_1
#define SYS_MODE_CONFIG_OPM0                    BIT_0

#define BM_SYS_MODE_CONFIG_RF_CALIBRATION       BIT_MASK_7
#define BM_SYS_MODE_CONFIG_ANTENNA_TUNING       BIT_MASK_6
#define BM_SYS_MODE_CONFIG_VCO_TUNING           BIT_MASK_5
#define BM_SYS_MODE_CONFIG_IDLE_MODE_SELECTOR   BIT_MASK_4
#define BM_SYS_MODE_CONFIG_DIRECT_SWITCH        BIT_MASK_3
#define BM_SYS_MODE_CONFIG_TRANSPARENT_MODE     BIT_MASK_2

#define BM_SYS_MODE_CONFIG_OPM                  ( BIT_MASK_1 | BIT_MASK_0 )
#define BM_SYS_MODE_CONFIG_OPM1                 BIT_MASK_1
#define BM_SYS_MODE_CONFIG_OPM0                 BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* System clock selections                                                   */
/* ------------------------------------------------------------------------- */
#define BM_SYS_CLOCK_CLOCK_SELECT_XTO4          (0x07U)
#define BM_SYS_CLOCK_CLOCK_SELECT_XTO6          (0x03U)

/* ------------------------------------------------------------------------- */
/* Operation modes                                                           */
/* ------------------------------------------------------------------------- */
#define OPM_IDLE        0x00U
#define OPM_TX          0x01U

/* ------------------------------------------------------------------------- */
/* TRXCONF_SERVICE_CHANNEL_CONFIG                                            */
/* ------------------------------------------------------------------------- */
#define SVC_CH_CONFIG_PATHB         BIT_7
#define SVC_CH_CONFIG_PATHA         BIT_6
#define SVC_CH_CONFIG_CH1           BIT_5
#define SVC_CH_CONFIG_CH0           BIT_4

#define SVC_CH_CONFIG_SER2          BIT_2
#define SVC_CH_CONFIG_SER1          BIT_1
#define SVC_CH_CONFIG_SER0          BIT_0

#define BM_SVC_CH_CONFIG_PATHB      BIT_MASK_7
#define BM_SVC_CH_CONFIG_PATHA      BIT_MASK_6

#define BM_SVC_CH_CONFIG_CH         ( BIT_MASK_5 | BIT_MASK_4 )
#define BM_SVC_CH_CONFIG_CH1        BIT_MASK_5
#define BM_SVC_CH_CONFIG_CH0        BIT_MASK_4

#define BM_SVC_CH_CONFIG_SER        ( BIT_MASK_2 | BIT_MASK_1 | BIT_MASK_0 )
#define BM_SVC_CH_CONFIG_SER2       BIT_MASK_2
#define BM_SVC_CH_CONFIG_SER1       BIT_MASK_1
#define BM_SVC_CH_CONFIG_SER0       BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* pinEventConfig_t                                                          */
/* ------------------------------------------------------------------------- */
#define PIN_EVENT_CONFIG_POWERON    BIT_7
#define PIN_EVENT_CONFIG_DEBUG      BIT_6
#define PIN_EVENT_CONFIG_NPWR6      BIT_5
#define PIN_EVENT_CONFIG_NPWR5      BIT_4
#define PIN_EVENT_CONFIG_NPWR4      BIT_3
#define PIN_EVENT_CONFIG_NPWR3      BIT_2
#define PIN_EVENT_CONFIG_NPWR2      BIT_1
#define PIN_EVENT_CONFIG_NPWR1      BIT_0

#define BM_PIN_EVENT_CONFIG_POWERON    BIT_MASK_7
#define BM_PIN_EVENT_CONFIG_DEBUG      BIT_MASK_6
#define BM_PIN_EVENT_CONFIG_NPWR6      BIT_MASK_5
#define BM_PIN_EVENT_CONFIG_NPWR5      BIT_MASK_4
#define BM_PIN_EVENT_CONFIG_NPWR4      BIT_MASK_3
#define BM_PIN_EVENT_CONFIG_NPWR3      BIT_MASK_2
#define BM_PIN_EVENT_CONFIG_NPWR2      BIT_MASK_1
#define BM_PIN_EVENT_CONFIG_NPWR1      BIT_MASK_0


/* ------------------------------------------------------------------------- */
/* sysEventConfig_t                                                          */
/* ------------------------------------------------------------------------- */
#define SYS_EVENT_CONFIG_SYS_ERR        BIT_7
#define SYS_EVENT_CONFIG_CMD_RDY        BIT_6
#define SYS_EVENT_CONFIG_SYS_RDY        BIT_5
#define SYS_EVENT_CONFIG_AVCCLOW        BIT_4
#define SYS_EVENT_CONFIG_LOWBATT        BIT_3
#define SYS_EVENT_CONFIG_EVENTPIN_POL   BIT_0

#define BM_SYS_EVENT_CONFIG_SYS_ERR         BIT_MASK_7
#define BM_SYS_EVENT_CONFIG_CMD_RDY         BIT_MASK_6
#define BM_SYS_EVENT_CONFIG_SYS_RDY         BIT_MASK_5
#define BM_SYS_EVENT_CONFIG_AVCCLOW         BIT_MASK_4
#define BM_SYS_EVENT_CONFIG_LOWBATT         BIT_MASK_3
#define BM_SYS_EVENT_CONFIG_EVENTPIN_POL    BIT_MASK_0


/* ------------------------------------------------------------------------- */
/* function pointer types                                                    */
/* ------------------------------------------------------------------------- */

/** \brief <b>timerIRQHandler</b>
    is used for function pointer definition of all Timer ISRs
*/
typedef void (*timerIRQHandler)(void);



#elif defined __IAR_SYSTEMS_ASM__

#define FUNCTION_TRACE_ASM
// #undef  FUNCTION_TRACE_ASM

/* ----------------------------------------------------------------------------- */
/** \brief <b>ATA_SET_FUNCTION_TRACE_POINT_ASM</b>
    sets a trace point at the current address
    \param  [in]    trace id
    \param  [in]    trace data
    \return none
*/
/* ----------------------------------------------------------------------------- */
ATA_SET_FUNCTION_TRACE_POINT_ASM MACRO   id, data
#ifdef FUNCTION_TRACE_ASM
    PUSH    R30
    CLI
    ; TRCDR = data;
    LDI     R30 , data
    STS     TRCDR , R30

    ; TRCIDL = (uint8_t) ((uint16_t)id >> 0);
    LDI     R30 , low(id/2)
    STS     TRCIDL , R30

    ; TRCIDH = (uint8_t) ((uint16_t)id >> 8);
    LDI     R30 , high(id/2)
    STS     TRCIDH , R30

    SEI
    POP     R30
#endif
    ENDM

/* ----------------------------------------------------------------------------- */
/** \brief <b>ATA_SET_FUNCTION_TRACE_POINT_ISR_ASM</b>
    sets a trace point at the current address
    \param  [in]    trace id
    \param  [in]    trace data
    \return none
*/
/* ----------------------------------------------------------------------------- */
ATA_SET_FUNCTION_TRACE_POINT_ISR_ASM MACRO   id, data
#ifdef FUNCTION_TRACE_ASM
    PUSH    R30

    ; TRCDR = data;
    LDI     R30 , data
    STS     TRCDR , R30

    ; TRCIDL = (uint8_t) ((uint16_t)id >> 0);
    LDI     R30 , low(id/2)
    STS     TRCIDL , R30

    ; TRCIDH = (uint8_t) ((uint16_t)id >> 8);
    LDI     R30 , high(id/2)
    STS     TRCIDH , R30

    POP     R30
#endif
    ENDM

/*startSimExtraction*/
/* ------------------------------------------------------------------------- */
/* bits and masks                                                            */
/* ------------------------------------------------------------------------- */

BIT_7  EQU 7
BIT_6  EQU 6
BIT_5  EQU 5
BIT_4  EQU 4
BIT_3  EQU 3
BIT_2  EQU 2
BIT_1  EQU 1
BIT_0  EQU 0

BIT_MASK_7 EQU ( 0x01 << BIT_7 )
BIT_MASK_6 EQU ( 0x01 << BIT_6 )
BIT_MASK_5 EQU ( 0x01 << BIT_5 )
BIT_MASK_4 EQU ( 0x01 << BIT_4 )
BIT_MASK_3 EQU ( 0x01 << BIT_3 )
BIT_MASK_2 EQU ( 0x01 << BIT_2 )
BIT_MASK_1 EQU ( 0x01 << BIT_1 )
BIT_MASK_0 EQU ( 0x01 << BIT_0 )
/* bits and masks */

NUM_EEPROM_SERVICES         EQU 3
NUM_RFTX_EEPROM_SERVICES    EQU 2


NUM_SRAM_SERVICES           EQU 2
NUM_SERVICES                EQU ( NUM_EEPROM_SERVICES + NUM_SRAM_SERVICES )
NUM_CHANNELS_PER_SERVICE    EQU 3
NUM_PATHES_PER_SERVICE      EQU 1

RSSIBUFMASK  EQU 0x1F
RSSIBUFSIZE  EQU 32
RFBUFSIZE    EQU 32
RFBUFMASK    EQU 0x1F
RDPTR        EQU 0x00
WRPTR        EQU 0x01
FILLLEVEL    EQU 0x02
THD          EQU 0x03

SETAND       EQU 0x0A
SETOR        EQU 0x0B
CLRAND       EQU 0x0C
CLROR        EQU 0x0D

SSISTATE     EQU 0x00
TMR1STATE    EQU 0x00
TMR2STATE    EQU 0x00
TMR3STATE    EQU 0x00
TMR4STATE    EQU 0x00
TMR5STATE    EQU 0x00

/* ------------------------------------------------------------------------- */
/* svcChConfig_t                                                             */
/* ------------------------------------------------------------------------- */
SVC_CH_CONFIG_ENA_PATHB      EQU BIT_7
SVC_CH_CONFIG_ENA_PATHA      EQU BIT_6
SVC_CH_CONFIG_CH_1           EQU BIT_5
SVC_CH_CONFIG_CH_0           EQU BIT_4
SVC_CH_CONFIG_SER_2          EQU BIT_2
SVC_CH_CONFIG_SER_1          EQU BIT_1
SVC_CH_CONFIG_SER_0          EQU BIT_0

BM_SVC_CH_CONFIG_ENA_PATHB    EQU BIT_MASK_7
BM_SVC_CH_CONFIG_ENA_PATHA    EQU BIT_MASK_6
BM_SVC_CH_CONFIG_CH           EQU BIT_MASK_5 | BIT_MASK_4
BM_SVC_CH_CONFIG_SER          EQU BIT_MASK_2 | BIT_MASK_1 | BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* sysModeConfig_t                                                           */
/* ------------------------------------------------------------------------- */
SYS_MODE_CONFIG_RF_CALIBRATION          EQU BIT_7
SYS_MODE_CONFIG_ANTENNA_TUNING          EQU BIT_6
SYS_MODE_CONFIG_VCO_TUNING              EQU BIT_5
SYS_MODE_CONFIG_IDLE_MODE_SELECTOR      EQU BIT_4
SYS_MODE_CONFIG_DIRECT_SWITCH           EQU BIT_3
SYS_MODE_CONFIG_TRANSPARENT_MODE        EQU BIT_2
SYS_MODE_CONFIG_OPM1                    EQU BIT_1
SYS_MODE_CONFIG_OPM0                    EQU BIT_0

BM_SYS_MODE_CONFIG_RF_CALIBRATION       EQU BIT_MASK_7
BM_SYS_MODE_CONFIG_ANTENNA_TUNING       EQU BIT_MASK_6
BM_SYS_MODE_CONFIG_VCO_TUNING           EQU BIT_MASK_5
BM_SYS_MODE_CONFIG_IDLE_MODE_SELECTOR   EQU BIT_MASK_4
BM_SYS_MODE_CONFIG_DIRECT_SWITCH        EQU BIT_MASK_3
BM_SYS_MODE_CONFIG_TRANSPARENT_MODE     EQU BIT_MASK_2

BM_SYS_MODE_CONFIG_OPM                  EQU ( BIT_MASK_1 | BIT_MASK_0 )
BM_SYS_MODE_CONFIG_OPM1                 EQU BIT_MASK_1
BM_SYS_MODE_CONFIG_OPM0                 EQU BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* pinEventConfig_t                                                          */
/* ------------------------------------------------------------------------- */
PIN_EVENT_CONFIG_POWERON    EQU BIT_7
PIN_EVENT_CONFIG_DEBUG      EQU BIT_6
PIN_EVENT_CONFIG_NPWR6      EQU BIT_5
PIN_EVENT_CONFIG_NPWR5      EQU BIT_4
PIN_EVENT_CONFIG_NPWR4      EQU BIT_3
PIN_EVENT_CONFIG_NPWR3      EQU BIT_2
PIN_EVENT_CONFIG_NPWR2      EQU BIT_1
PIN_EVENT_CONFIG_NPWR1      EQU BIT_0

BM_PIN_EVENT_CONFIG_POWERON    EQU  BIT_MASK_7
BM_PIN_EVENT_CONFIG_DEBUG      EQU  BIT_MASK_6
BM_PIN_EVENT_CONFIG_NPWR6      EQU  BIT_MASK_5
BM_PIN_EVENT_CONFIG_NPWR5      EQU  BIT_MASK_4
BM_PIN_EVENT_CONFIG_NPWR4      EQU  BIT_MASK_3
BM_PIN_EVENT_CONFIG_NPWR3      EQU  BIT_MASK_2
BM_PIN_EVENT_CONFIG_NPWR2      EQU  BIT_MASK_1
BM_PIN_EVENT_CONFIG_NPWR1      EQU  BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* sysEventConfig_t                                                          */
/* ------------------------------------------------------------------------- */
SYS_EVENT_CONFIG_SYS_ERR        EQU BIT_7
SYS_EVENT_CONFIG_CMD_RDY        EQU BIT_6
SYS_EVENT_CONFIG_SYS_RDY        EQU BIT_5
SYS_EVENT_CONFIG_AVCCLOW        EQU BIT_4
SYS_EVENT_CONFIG_LOWBATT        EQU BIT_3
SYS_EVENT_CONFIG_EVENTPIN_POL   EQU BIT_0

BM_SYS_EVENT_CONFIG_SYS_ERR         EQU  BIT_MASK_7
BM_SYS_EVENT_CONFIG_CMD_RDY         EQU  BIT_MASK_6
BM_SYS_EVENT_CONFIG_SYS_RDY         EQU  BIT_MASK_5
BM_SYS_EVENT_CONFIG_AVCCLOW         EQU  BIT_MASK_4
BM_SYS_EVENT_CONFIG_LOWBATT         EQU  BIT_MASK_3
BM_SYS_EVENT_CONFIG_EVENTPIN_POL    EQU  BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* event_power                                                               */
/* ------------------------------------------------------------------------- */
EVENTS_POWER_PWRON                  EQU BIT_7
EVENTS_POWER_DEBUG                  EQU BIT_6
EVENTS_POWER_NPWRON6                EQU BIT_5
EVENTS_POWER_NPWRON5                EQU BIT_4
EVENTS_POWER_NPWRON4                EQU BIT_3
EVENTS_POWER_NPWRON3                EQU BIT_2
EVENTS_POWER_NPWRON2                EQU BIT_1
EVENTS_POWER_NPWRON1                EQU BIT_0

BM_EVENTS_POWER_PWRON               EQU BIT_MASK_7
BM_EVENTS_POWER_DEBUG               EQU BIT_MASK_6
BM_EVENTS_POWER_NPWRON6             EQU BIT_MASK_5
BM_EVENTS_POWER_NPWRON5             EQU BIT_MASK_4
BM_EVENTS_POWER_NPWRON4             EQU BIT_MASK_3
BM_EVENTS_POWER_NPWRON3             EQU BIT_MASK_2
BM_EVENTS_POWER_NPWRON2             EQU BIT_MASK_1
BM_EVENTS_POWER_NPWRON1             EQU BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* events_eventCtrl                                                          */
/* ------------------------------------------------------------------------- */
EVENTS_EVENTCTRL_IRQ_PENDING        EQU     BIT_0

BM_EVENTS_EVENTCTRL_IRQ_PENDING     EQU     BIT_MASK_0

/* ------------------------------------------------------------------------- */
/* Debug Error Codes                                                         */
/* ------------------------------------------------------------------------- */
DEBUG_ERROR_CODE_SYSTEM_ERROR_EEPROM_NOT_VALID                 EQU  0
DEBUG_ERROR_CODE_SETIDLEMODE_MISS_SSMRDY                       EQU  DEBUG_ERROR_CODE_SYSTEM_ERROR_EEPROM_NOT_VALID + 1                  // 001<->01
DEBUG_ERROR_CODE_SETIDLEMODE_TIMER_LOCKED                      EQU  DEBUG_ERROR_CODE_SETIDLEMODE_MISS_SSMRDY + 1                        // 002<->02
DEBUG_ERROR_CODE_RX_STATESTARTSSM_TIMER_LOCKED                 EQU  DEBUG_ERROR_CODE_SETIDLEMODE_TIMER_LOCKED + 1                       // 003<->03
DEBUG_ERROR_CODE_RX_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT     EQU  DEBUG_ERROR_CODE_RX_STATESTARTSSM_TIMER_LOCKED + 1                  // 004<->04
DEBUG_ERROR_CODE_TX_STATESTARTSSM_TIMER_LOCKED                 EQU  DEBUG_ERROR_CODE_RX_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT + 1      // 005<->05
DEBUG_ERROR_CODE_TX_STATEWAIT4SSMRDY_TIMEOUT                   EQU  DEBUG_ERROR_CODE_TX_STATESTARTSSM_TIMER_LOCKED + 1                  // 006<->06
DEBUG_ERROR_CODE_POLL_STATESTARTSSM_TIMER_LOCKED               EQU  DEBUG_ERROR_CODE_TX_STATEWAIT4SSMRDY_TIMEOUT + 1                    // 007<->07
DEBUG_ERROR_CODE_POLL_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT   EQU  DEBUG_ERROR_CODE_POLL_STATESTARTSSM_TIMER_LOCKED + 1                // 008<->08
DEBUG_ERROR_CODE_ANTTUNE_STATESTARTSSM_TIMER_LOCKED            EQU  DEBUG_ERROR_CODE_POLL_STATEWAIT4SSMSTATE_GETTELEGRAM_TIMEOUT + 1    // 009<->09
DEBUG_ERROR_CODE_ANTTUNE_STATEWAIT4SSMRDY_TIMEOUT              EQU  DEBUG_ERROR_CODE_ANTTUNE_STATESTARTSSM_TIMER_LOCKED + 1             // 010<->0A
DEBUG_ERROR_CODE_VCOCAL_STATESTARTSSM_TIMER_LOCKED             EQU  DEBUG_ERROR_CODE_ANTTUNE_STATEWAIT4SSMRDY_TIMEOUT + 1               // 011<->0B
DEBUG_ERROR_CODE_VCOCAL_STATEWAIT4SSMRDY_TIMEOUT               EQU  DEBUG_ERROR_CODE_VCOCAL_STATESTARTSSM_TIMER_LOCKED + 1              // 012<->0C
DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMER_LOCKED                      EQU  DEBUG_ERROR_CODE_VCOCAL_STATEWAIT4SSMRDY_TIMEOUT + 1                // 013<->0D
DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMEOUT                           EQU  DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMER_LOCKED + 1                       // 014<->0E
DEBUG_ERROR_CODE_POLLING_TIMER1_LOCKED                         EQU  DEBUG_ERROR_CODE_SHUTDOWNTRX_TIMEOUT + 1                            // 015<->0F
DEBUG_ERROR_CODE_INVALID_OPM_SWITCHING                         EQU  DEBUG_ERROR_CODE_POLLING_TIMER1_LOCKED + 1                          // 016<->10
DEBUG_ERROR_CODE_INVALID_OPM_MODE_DURING_TUNE_AND_CHECK        EQU  DEBUG_ERROR_CODE_INVALID_OPM_SWITCHING + 1                          // 017<->11
DEBUG_ERROR_CODE_AVCCLOW_DURING_TX                             EQU  DEBUG_ERROR_CODE_INVALID_OPM_MODE_DURING_TUNE_AND_CHECK + 1         // 018<->12
DEBUG_ERROR_CODE_SERVICE_INIT_FAILURE                          EQU  DEBUG_ERROR_CODE_AVCCLOW_DURING_TX + 1                              // 019<->13
DEBUG_ERROR_CODE_DFIFO_OVER_UNDER_FLOW                         EQU  DEBUG_ERROR_CODE_SERVICE_INIT_FAILURE + 1                           // 020<->14
DEBUG_ERROR_CODE_SFIFO_OVER_UNDER_FLOW                         EQU  DEBUG_ERROR_CODE_DFIFO_OVER_UNDER_FLOW + 1                          // 021<->15
DEBUG_ERROR_CODE_RSSI_STATESTARTSSM_TIMER_LOCKED               EQU  DEBUG_ERROR_CODE_SFIFO_OVER_UNDER_FLOW + 1                          // 022<->16
DEBUG_ERROR_CODE_SRC_FRC_CALIB_FAILED                          EQU  DEBUG_ERROR_CODE_RSSI_STATESTARTSSM_TIMER_LOCKED + 1                // 023<->17
DEBUG_ERROR_CODE_GETRXTELEGRAM_SSM_ERROR                       EQU  DEBUG_ERROR_CODE_SRC_FRC_CALIB_FAILED + 1                           // 024<->18
DEBUG_ERROR_CODE_TEMPMEAS_STATESTARTSSM_TIMER_LOCKED           EQU  DEBUG_ERROR_CODE_GETRXTELEGRAM_SSM_ERROR + 1                        // 025<->19
DEBUG_ERROR_CODE_TEMPMEAS_STATEWAIT4SSMRDY_TIMEOUT             EQU  DEBUG_ERROR_CODE_TEMPMEAS_STATESTARTSSM_TIMER_LOCKED + 1            // 026<->1A
DEBUG_ERROR_CODE_AVCCLOW_TIMEOUT                               EQU  DEBUG_ERROR_CODE_TEMPMEAS_STATEWAIT4SSMRDY_TIMEOUT + 1              // 027<->1B
// error codes for module SPI
DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED                           EQU  DEBUG_ERROR_CODE_AVCCLOW_TIMEOUT + 1                                // 028<->1C
DEBUG_ERROR_CODE_SPI_ALREADY_OPENED                            EQU  DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED + 1                            // 029<->1D
// error codes for module ATA5831
DEBUG_ERROR_CODE_ATA5831_SPI_CHECKSUM_ERROR                    EQU  DEBUG_ERROR_CODE_SPI_ALREADY_OPENED + 1                             // 030<->1E
// error codes for module CALIB
DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED                     EQU  DEBUG_ERROR_CODE_ATA5831_SPI_CHECKSUM_ERROR + 1                     // 031<->1F
DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED                     EQU  DEBUG_ERROR_CODE_CALIB_TIMER_SRCCAL_LOCKED + 1                      // 032<->20
DEBUG_ERROR_CODE_CALIB_EXTERNAL_CLOCK_FAILURE                  EQU  DEBUG_ERROR_CODE_CALIB_TIMER_FRCCAL_LOCKED + 1                      // 033<->21
// error codes for module RFTX
DEBUG_ERROR_CODE_RFTX_STARTSSM_TIMER_LOCKED                    EQU  DEBUG_ERROR_CODE_CALIB_EXTERNAL_CLOCK_FAILURE + 1                   // 034<->22
DEBUG_ERROR_CODE_RFTX_SHUTDOWN_ERROR                           EQU  DEBUG_ERROR_CODE_RFTX_STARTSSM_TIMER_LOCKED + 1                     // 035<->23
DEBUG_ERROR_CODE_RFTX_WAIT4SSMRDY_TIMEOUT                      EQU  DEBUG_ERROR_CODE_RFTX_SHUTDOWN_ERROR + 1                            // 036<->24
DEBUG_ERROR_CODE_RFTX_SFIFO_ERROR                              EQU  DEBUG_ERROR_CODE_RFTX_WAIT4SSMRDY_TIMEOUT + 1                       // 037<->25
DEBUG_ERROR_CODE_RFTX_DFIFO_ERROR                              EQU  DEBUG_ERROR_CODE_RFTX_SFIFO_ERROR + 1                               // 038<->26
DEBUG_ERROR_CODE_RFTX_WAIT4VCOSSMRDY_TIMEOUT                   EQU  DEBUG_ERROR_CODE_RFTX_DFIFO_ERROR + 1                               // 039<->27
DEBUG_ERROR_CODE_RFTX_WAIT4ANTSSMRDY_TIMEOUT                   EQU  DEBUG_ERROR_CODE_RFTX_WAIT4VCOSSMRDY_TIMEOUT + 1                    // 040<->28

// error codes for module LF RSSI
DEBUG_ERROR_CODE_LFRSSI_EEPROM_READ_ERROR                      EQU  DEBUG_ERROR_CODE_RFTX_WAIT4ANTSSMRDY_TIMEOUT + 1                    // 041<->29
DEBUG_ERROR_CODE_LFRSSI_EEPROM_ACCESS_ERROR                    EQU  DEBUG_ERROR_CODE_LFRSSI_EEPROM_READ_ERROR + 1                       // 042<->2A
DEBUG_ERROR_CODE_LFRSSI_ANALOGPART_NOTREADY                    EQU  DEBUG_ERROR_CODE_LFRSSI_EEPROM_ACCESS_ERROR + 1                     // 043<->2B
DEBUG_ERROR_CODE_LFRSSI_INVALID_MEASUREMENT_CHANNELS           EQU  DEBUG_ERROR_CODE_LFRSSI_ANALOGPART_NOTREADY + 1                     // 044<->2C
DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED                    EQU  DEBUG_ERROR_CODE_LFRSSI_INVALID_MEASUREMENT_CHANNELS + 1            // 045<->2D
DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE                       EQU  DEBUG_ERROR_CODE_LFRSSI_NULL_POINTER_PASSED + 1                     // 046<->2E
DEBUG_ERROR_CODE_LFRSSI_CHANNELS_OUTOFRANGE                    EQU  DEBUG_ERROR_CODE_LFRSSI_OPERATION_ACTIVE + 1                        // 047<->2F

// error codes for module LF RX
DEBUG_ERROR_CODE_LFRX_EEPROM_READ_ERROR                        EQU  DEBUG_ERROR_CODE_LFRSSI_CHANNELS_OUTOFRANGE + 1                     // 048<->30
DEBUG_ERROR_CODE_LFRX_EEPROM_ACCESS_ERROR                      EQU  DEBUG_ERROR_CODE_LFRX_EEPROM_READ_ERROR + 1                         // 049<->31

// error codes for module AES
DEBUG_ERROR_CODE_AES_ALREADY_IN_USE_ERROR                      EQU  DEBUG_ERROR_CODE_LFRX_EEPROM_ACCESS_ERROR + 1                       // 050<->32
DEBUG_ERROR_CODE_AES_INVALID_KEY_ERROR                         EQU  DEBUG_ERROR_CODE_AES_ALREADY_IN_USE_ERROR + 1                       // 051<->33
DEBUG_ERROR_CODE_AES_RUN_ERROR                                 EQU  DEBUG_ERROR_CODE_AES_INVALID_KEY_ERROR + 1                          // 052<->34

// error codes for module RFRCC
DEBUG_ERROR_CODE_RFRCC_EEP_READ_ERROR                          EQU  DEBUG_ERROR_CODE_AES_RUN_ERROR + 1                                  // 053<->35
DEBUG_ERROR_CODE_RFRCC_EEP_WRITE_ACCESS_ERROR                  EQU  DEBUG_ERROR_CODE_RFRCC_EEP_READ_ERROR + 1                           // 054<->36
DEBUG_ERROR_CODE_RFRCC_MAC_ERROR                               EQU  DEBUG_ERROR_CODE_RFRCC_EEP_WRITE_ACCESS_ERROR + 1                   // 055<->37
DEBUG_ERROR_CODE_RFRCC_SUBKEY_ERROR                            EQU  DEBUG_ERROR_CODE_RFRCC_MAC_ERROR + 1                                // 056<->38

// error codes for module I2C
DEBUG_ERROR_CODE_I2C_NACK_SLAW_ERROR                           EQU  DEBUG_ERROR_CODE_RFRCC_SUBKEY_ERROR + 1                             // 057<->39
DEBUG_ERROR_CODE_I2C_NACK_DATA_TX_ERROR                        EQU  DEBUG_ERROR_CODE_I2C_NACK_SLAW_ERROR + 1                            // 058<->3A
DEBUG_ERROR_CODE_I2C_STATUS_CODE_ERROR                         EQU  DEBUG_ERROR_CODE_I2C_NACK_DATA_TX_ERROR + 1                         // 059<->3B
DEBUG_ERROR_CODE_I2C_NACK_SLAR_ERROR                           EQU  DEBUG_ERROR_CODE_I2C_STATUS_CODE_ERROR + 1                          // 060<->3C
DEBUG_ERROR_CODE_I2C_INVALID_REQUEST_ERROR                     EQU  DEBUG_ERROR_CODE_I2C_NACK_SLAR_ERROR + 1                            // 061<->3D

// error codes for module CALIB (extended)
DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR                       EQU  DEBUG_ERROR_CODE_I2C_INVALID_REQUEST_ERROR + 1                      // 062<->3E

// error codes for module RFTX (extended)
DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR                        EQU  DEBUG_ERROR_CODE_CALIB_EEPROM_READ_ERROR + 1                        // 063<->3F

// error codes for module Globals
DEBUG_ERROR_CODE_GLOBALS_EEPROM_READ_ERROR                     EQU  DEBUG_ERROR_CODE_RFTX_EEPROM_READ_ERROR + 1                         // 064<->40

// error code indicating NOT USED
DEBUG_ERROR_CODE_SYSTEM_ERROR_NOT_USED                         EQU  0xFF                                                                // 255<->FF

/* ------------------------------------------------------------------------- */
/* Operation modes                                                           */
/* ------------------------------------------------------------------------- */
OPM_IDLE        EQU 0x00
OPM_TX          EQU 0x01

/* ------------------------------------------------------------------------- */
/* facLock                                                                   */
/* ------------------------------------------------------------------------- */
FACLOCK_CONF_FEBIA          EQU 0
FACLOCK_CONF_FEBT           EQU FACLOCK_CONF_FEBIA  + 1
FACLOCK_CONF_FELNA          EQU FACLOCK_CONF_FEBT   + 1
FACLOCK_CONF_FELNA2         EQU FACLOCK_CONF_FELNA  + 1
FACLOCK_CONF_FETN4          EQU FACLOCK_CONF_FELNA2 + 1
FACLOCK_CONF_FEVCO_OFFS     EQU FACLOCK_CONF_FETN4  + 1

/* ------------------------------------------------------------------------- */
/* sDebug                                                                    */
/* ------------------------------------------------------------------------- */
DEBUG_ERROR_CODE         EQU 0
DEBUG_SSM_ERROR_CODE     EQU DEBUG_ERROR_CODE + 1


/*stopSimExtraction*/
#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* GLOBALS_DEFS_H */
