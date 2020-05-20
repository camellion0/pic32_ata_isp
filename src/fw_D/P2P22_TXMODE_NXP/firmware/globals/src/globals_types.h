//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/globals/src/globals_types.h $
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

#ifndef GLOBALS_TYPES_H
#define GLOBALS_TYPES_H

#ifdef __IAR_SYSTEMS_ICC__

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "globals_defs.h"
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
/** \brief <b>svcChConfig_t</b>
    is a typedef for the Service/Channel configuration
    \li Bit 7: enaPathB
    \li Bit 6: enaPathA
    \li Bit 5: ch[1]
    \li Bit 4: ch[0]
    \li Bit 3: rfu
    \li Bit 2: ser[2]
    \li Bit 1: ser[1]
    \li Bit 0: ser[0]
 */
typedef uint8_t         svcChConfig_t;

/** \brief <b>tuneCheckConfig_t</b>
    is a typedef for the Tune and Check configuration
    \li Bit 7: Antenna Tuning
    \li Bit 6: Temperature Measurement
    \li Bit 5: SRC calibration
    \li Bit 4: FRC calibration
    \li Bit 3: VCO calibration
    \li Bit 2: RF calibration
    \li Bit 1: Self check
    \li Bit 0: Register Refresh during Self check
 */
typedef uint8_t         tuneCheckConfig_t;

/** \brief <b>sysModeConfig_t</b>
    is a typedef for system Mode configuration
    \li Bit 7:         RF calibration flag (0:disabled / 1:enabled)
    \li Bit 6:         Antenna tuning flag (0:disabled / 1:enabled)
    \li Bit 5:         VCO tuning flag (0:disabled / 1:enabled)
    \li Bit 4:         next system IDLEMode selector 0:RC / 1:XTO
    \li Bit 3:         direct (1)/ normal (0) switching
    \li Bit 2:         TMDEN: Transparent Mode Data Enable
    \li Bit 1..0:      OPM[0..1]: Operation Mode
        - 0: Idle Mode (SPI or LIN)
        - 1: TX Mode
 */
typedef uint8_t         sysModeConfig_t;

/** \brief <b>pinEventConfig_t</b>
    is a typedef for pin event configuration
    \li Bit 7:    PowerOn
    \li Bit 6:    Debug
    \li Bit 5:    nPWR6
    \li Bit 4:    nPWR5
    \li Bit 3:    nPWR4
    \li Bit 2:    nPWR3
    \li Bit 1:    nPWR2
    \li Bit 0:    nPWR1
 */
typedef uint8_t     pinEventConfig_t;

/** \brief <b>sysEventConfig_t</b>
    is a typedef for system event configuration
    \li Bit 7:    SystemErrorFlag
    \li Bit 6:    CMD_RDY
    \li Bit 5:    SYS_RDY
    \li Bit 4:    AVCCLOWM
    \li Bit 3:    LOWBATTM
    \li Bit 2:    rfu
    \li Bit 1:    rfu
    \li Bit 0:    EventPinPolarity
 */
typedef uint8_t     sysEventConfig_t;

/*----------------------------------------------------------------------------- */
/** \brief <b>sBuildInformation</b>
    contains the build information */
/*----------------------------------------------------------------------------- */
typedef struct {
    uint32_t dwRevisionNumber;
    uint8_t  sRevisionDate[20];
    uint8_t  sBuildTime[20];
}sBuildInformation;

/*----------------------------------------------------------------------------- */
/** \brief <b>sDebugErrorCodes</b>
    contains the bytes for the last occured system error and the last occured
    ssm errror
 */
/*----------------------------------------------------------------------------- */
typedef struct{
    /** \brief <b>bErrorCode</b>
        contains the last occured system error
     */
    uint8_t bErrorCode;
    /** \brief <b>bSsmErrorCode</b>
        contains the last occured ssm error
     */
    uint8_t bSsmErrorCode;
}sDebugErrorCodes;

/*----------------------------------------------------------------------------- */
/** \brief <b>sTimerAsyn8BitParams</b>
    contains the variables used to configure an 8-Bit asynchronous Timer
*/
/*----------------------------------------------------------------------------- */
typedef struct {
    /** \brief <b>ctrl</b>
        contains the configuration of the Timer control register.
    */
    uint8_t ctrl;

    /** \brief <b>mode</b>
        contains the configuration of the Timer mode register.
    */
    uint8_t mode;

    /** \brief <b>comp</b>
        contains the configuration of the Timer compare register.
    */
    uint8_t comp;
    
    /** \brief <b>irqMask</b>
        contains the configuration of the Timer interrupt mask register.
    */
    uint8_t irqMask;

    /** \brief <b>ovfIsr</b>
        contains the function pointer for the Timer overflow ISR.
    */
    timerIRQHandler ovfIsr;
    
    /** \brief <b>compIsr</b>
        contains the function pointer for the Timer compare ISR.
    */
    timerIRQHandler compIsr;
}sTimerAsyn8BitParams;


/*----------------------------------------------------------------------------- */
/** \brief <b>sTimerAsyn16BitParams</b>
    contains the variables used to configure an 16-Bit asynchronous Timer
*/
/*----------------------------------------------------------------------------- */
typedef struct {
    /** \brief <b>ctrl</b>
        contains the configuration of the Timer control register.
    */
    uint8_t ctrl;

    /** \brief <b>modeA</b>
        contains the configuration of the Timer mode A register.
    */
    uint8_t modeA;

    /** \brief <b>modeB</b>
        contains the configuration of the Timer mode B register.
    */
    uint8_t modeB;
    
    /** \brief <b>compL</b>
        contains the configuration of the Timer compare (low byte) register.
    */
    uint8_t compL;
    
    /** \brief <b>compH</b>
        contains the configuration of the Timer compare (high byte) register.
    */
    uint8_t compH;
    
    /** \brief <b>irqMask</b>
        contains the configuration of the Timer interrupt mask register.
    */
    uint8_t irqMask;

    /** \brief <b>ovfIsr</b>
        contains the function pointer for the Timer overflow ISR.
    */
    timerIRQHandler ovfIsr;
    
    /** \brief <b>compIsr</b>
        contains the function pointer for the Timer compare ISR.
    */
    timerIRQHandler compIsr;
    
    /** \brief <b>capIsr</b>
        contains the function pointer for the Timer capture ISR.
    */
    timerIRQHandler capIsr;
}sTimerAsyn16BitParams;


/*----------------------------------------------------------------------------- */
/** \brief <b>sTimerSyn16BitParams</b>
    contains the variables used to configure an 16-Bit synchronous Timer
*/
/*----------------------------------------------------------------------------- */
typedef struct {
    /** \brief <b>ctrl</b>
        contains the configuration of the Timer control register.
    */
    uint8_t ctrl;
    
    /** \brief <b>compL</b>
         contains the configuration of the Timer compare (low byte) register.
    */
    uint8_t compL;
    
    /** \brief <b>compH</b>
        contains the configuration of the Timer compare (high byte) register.
    */
    uint8_t compH;
    
    /** \brief <b>countL</b>
        contains the configuration of the Timer count (low byte) register.
    */
    uint8_t countL;

    /** \brief <b>countH</b>
        contains the configuration of the Timer compare (high byte) register.
    */
    uint8_t countH;
    
    /** \brief <b>irqMask</b>
        contains the configuration of the Timer interrupt mask register.
    */
    uint8_t irqMask;

    /** \brief <b>ovfIsr</b>
        contains the function pointer for the Timer overflow ISR.
    */
    timerIRQHandler ovfIsr;
    
    /** \brief <b>compIsr</b>
        contains the function pointer for the Timer compare ISR.
    */
    timerIRQHandler compIsr;
    
}sTimerSyn16BitParams;


#elif defined __IAR_SYSTEMS_ASM__
#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* GLOBALS_TYPES_H */
