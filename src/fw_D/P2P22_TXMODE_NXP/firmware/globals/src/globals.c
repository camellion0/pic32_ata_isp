//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/globals/src/globals.c $
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

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "globals.h"
#include "../../timer1/src/timer1.h"
#include "../../eep/src/eep.h"
#include "../../calib/src/calib.h"


/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/** \brief <b>COPY_SERVICE_AND_CHANNEL_FROM_EEPROM</b> is used as define to
    copy complete service and channel information from EEPROM
 */
#define COPY_SERVICE_AND_CHANNEL_FROM_EEPROM    (uint8_t)0x81U
/** \brief <b>COPY_CHANNEL_ONLY_FROM_EEPROM</b> is used as define to
    copy only channel information from EEPROM
 */
#define COPY_CHANNEL_ONLY_FROM_EEPROM           (uint8_t)0x41U
/** \brief <b>COPY_SERVICE_AND_CHANNEL_FROM_SRAM</b> is used as define to
    copy complete service and channel information from SRAM
 */
#define COPY_SERVICE_AND_CHANNEL_FROM_SRAM      (uint8_t)0x80U
/** \brief <b>COPY_CHANNEL_ONLY_FROM_SRAM</b> is used as define to
    copy only channel information from SRAM
 */
#define COPY_CHANNEL_ONLY_FROM_SRAM             (uint8_t)0x40U


#define VOLTAGE_MONITOR_MRC_WAIT_CYCLES       4U
#define VOLTAGE_MONITOR_SRC_WAIT_CYCLES       1U
#define VOLTAGE_MONITOR_FRC_WAIT_CYCLES      35U
#define VOLTAGE_MONITOR_XTO4_WAIT_CYCLES     32U

/*===========================================================================*/
/*  Modul Globals (Variables)                                                */
/*===========================================================================*/

/** \brief <b>g_sDebug</b>
    contains the last error codes for system error and ssm error.
*/
#pragma location = ".debug"
__no_init sDebugErrorCodes g_sDebug;

/** \brief <b>romVersion</b>
    contains the ROM version
*/
#pragma location = ".romversion"
__root const prog_char romVersion = ROM_VERSION;

/** \brief <b>romPatchVersion</b>
    contains the ROM patch version
*/
#pragma location = ".rompatchversion"
__root const prog_char romPatchVersion = ROMPATCH_VERSION;

/*===========================================================================*/
/*  Modul Globals (Functions)                                                */
/*===========================================================================*/

static __root uint8_t ATA_globalsReadMvccSettlingTime_C(void);

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsSetClk_C</b>
    sets the timer and system clock prescaler value.

    \param[in]  bClockPrescalerValue    System clock and timer prescaler value

    \image html ATA_globalsSetClk_C.png

    \internal
    \li 010: Save global SREG register content and disable the global interrupt flag
             by calling function "_CLI" to be able to consistently
             update register WDRCR.

    \li 020: Set bit "CLPCE" in register CLPR to 1 to be able to change any of
             the CLTPS or CLKPS bits.

    \li 030: Set register CLPR to input parameter "uClockPrescalerValue" to apply
             the given timer and system clock prescaler settings.

    \Derived{Yes}

    \Rationale{This function is necessary in order for the Application SW to
               change the system clock prescaler as well as the system timer
               clock prescaler in a performant manner}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsSetClk_C(uint8_t bClockPrescalerValue)
{
    uint8_t bSreg;

    /* LLR-Ref: 010 */
    bSreg = SREG;
    _CLI;

    /* LLR-Ref: 020 */
    CLPR = BM_CLPCE;
    CLPR = bClockPrescalerValue;

    /* LLR-Ref: 030 */
    SREG = bSreg;
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsWdtDisable_C</b>
    disables the watchdog timer.

    \image html ATA_globalsWdtDisable_C.png

    \internal
    \li 010: Save global SREG register content and disable the global interrupt flag
             by calling function "_CLI" to be able to consistently
             update register WDRCR.\n\n
             Note:
             WDE is overwritten by WDRF, thus to clear WDE, WDRF has to be cleared first.

    \li 020: Set bit "WDRF" in register MCUSR to 0 to be able to set bit "WDE" in register WDTCR
             to 0.

    \li 030: Set bits "WDCE" and "WDE" in register WDTCR to 1 to be able to reset WDE and
             withing 4 cycles set WDE to 0 in order to disable the Watchdog timer.
             The Watchdog prescaler value is set to its maximum value per default.

    \li 040: Restore global SREG register with the previously saved content.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-878}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsWdtDisable_C(void)
{
    uint8_t bSreg;

    /* LLR-Ref: 010 */
    bSreg = SREG;
    _CLI;

    /* LLR-Ref: 020 */
    MCUSR &= (uint8_t)(~BM_WDRF);

    /* LLR-Ref: 030 */
    WDTCR = (BM_WDCE | BM_WDE);
    WDTCR = (BM_WDPS2 | BM_WDPS1 | BM_WDPS0);

    /* LLR-Ref: 040 */
    SREG = bSreg;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsWdtEnable_C</b>
    enables the watchdog timer.

    \param[in]  uConfWDTCR      Configuration value for register WDTCR
    
    \image html ATA_globalsWdtEnable_C.png

    \internal
    \li 010: Save global SREG register content and disable the global interrupt flag
             by calling macro "_CLI" to be able to consistently update register
             WDRCR.

    \li 020: Set bit WDCE in register WDTCR in order to be able to change any of the
             WDTCR content.\n\n
             Note:
             Enabling the Watchdog and setting the prescaler at the same time is not
             possible from a HW point of view.

    \li 030: Set register WDTCR to the given parameter "confWDTCR" two
             times in order to enable the Watchdog and to set the Watchdog prescaler
             value.

    \li 040: Restore global SREG register with the previously saved content.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-877}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsWdtEnable_C(uint8_t uConfWDTCR)
{
    uint8_t bSreg = SREG;

    /* LLR-Ref: 010 */
    _CLI;

    /* LLR-Ref: 020 */
    WDTCR |= (BM_WDCE | BM_WDE);

    /* LLR-Ref: 030 */
    WDTCR = uConfWDTCR;
    WDTCR = uConfWDTCR;

    /* LLR-Ref: 040 */
    SREG = bSreg;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsSleep_C</b>
    activates the given AVR sleep mode.

    \param[in]  bSleepModeConfig    Sleep mode to be applied

    \image html ATA_globalsSleep_C.png

    \internal
    \li 010: Disable the global interrupt flag by calling macro "_CLI" to be able to
             consistently update register SMCR.

    \li 020: Apply the given sleep mode via parameter "bSleepModeConfig" to register
              SMCR, AND
             set the "Sleep enable" bit in register SMCR to 1 in order for the AVR
              to enter the selected sleep mode when the SLEEP instruction is
              executed.

    \li 030: Enable the global interrupt flag by calling macro "_SEI" to allow
              any interrupt handling during sleep in order for the AVR to wake up
              on dedicated events,
             AND
              enter the sleep mode by calling macro "_SLEEP".\n\n
             Note:
             If the MCU wakes up again, execution starts after the _SLEEP
             instruction!

    \li 040: Set the "Sleep enable" bit in register SMCR to 0 in order to avoid
             any unintended entering of the MCU into sleep mode.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-900}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsSleep_C(uint8_t bSleepModeConfig)
{
    /* LLR-Ref: 010 */
    _CLI;

    /* LLR-Ref: 020 */
    SMCR = bSleepModeConfig & (BM_SM2 | BM_SM1 | BM_SM0 );
    SMCR |= BM_SE;

    /* LLR-Ref: 030 */
    _SEI;
    _SLEEP;

    /* LLR-Ref: 040 */
    SMCR &= (uint8_t)~BM_SE;
}

/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsClkSwitchFrc_C</b>
    switches the AVR core to be clocked with the FRC (FRC activation included
    and DVCC High Enable included).
    Keep prescaler settings in mind.\n\n
    HW considerations:
    The CCS bit and CMM2..0 bits in register CMCR need to be set separately in
    order to switch from XTO to FRC correctly (both clocks require bit CCS to be
    set to 1).
    The following code snippet "CMCR = BM_CMCCE" does not set bit CCS to 0, as
    only the CMCCE is enabled. Any change has to occur one cycle after CMCCE
    is set to 1.

    \image html ATA_globalsClkSwitchFrc_C.png

    \internal
    \li 010: Clear temporary clock value for register CMCR without changing bits
             4 and 5 AND store CMOCR register settings at function entry.

    \li 012: Enable "DVCC high enable" in order to provide enough current for the
             AVR core when running with FRC.\n
             Set AVR clock to FRC:

    \li 015: Start the FRC manually by setting bit FRCAO in register CMOCR to 1 AND
             wait until the FRC is active.

    \li 020: Disable all interrupts to allow for an atomic execution of a register
             CMCR change and
             enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, assign the temporary clock value to
              register CMCR with bit CCS set to 0 and bits CMM2..0 set to FRC
              in order to be able to change bits CMM2..0 to FRC.

    \li 030: Enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, assign the temporary clock value to
              register CMCR with bit CCS set to 1 in order to activate the value of
              CMM2..0 bits.\n\n
             End of AVR clock change to FRC

    \li 040: Set the timer clock prescaler value to 1 by first enabling to change
              register CLPR by setting bit CLPCE in register CLPR to 1 and, within
              four cycles, set register CLPR to the value CLTPS0.

    \li 050: Restore register CMOCR settings regarding the FRC always on flag 
              before activation of the FRC in order to reflect the overall FRC 
              always on status after FRC activation and
             restore global interrupt configuration, since the critical section is
             finished.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-871,Primus2P-873}
    \endinternal
\n
*/
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsClkSwitchFrc_C(void)
{
    /* LLR-Ref: 010 */
    uint8_t bSreg = SREG;
    uint8_t cmcr = CMCR & 0x30U;
    uint8_t bCmocrFrcao = CMOCR & BM_FRCAO;
    
    /* LLR-Ref: 012 */
    SUPCR |= BM_DVHEN;

    /* LLR-Ref: 015 */
    CMOCR |= BM_FRCAO;
    do
    {
        _NOP;
    } while((CMOCR & BM_FRCACT) == 0U);

    /* LLR-Ref: 020 */
    cmcr |= BM_CMM0;

    _CLI;
    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 030 */
    cmcr |= BM_CCS;

    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 040 */
    CLPR = BM_CLPCE;
    CLPR = BM_CLTPS0;

    /* LLR-Ref: 050 */
    if ( bCmocrFrcao == 0x00U )
    {
        CMOCR &= ~BM_FRCAO;
    }

    SREG = bSreg;
}

/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsClkSwitchFrcWithDelay_C</b>
    switches the AVR core to be clocked with the FRC (FRC activation included
    and DVCC High Enable included). After clock switching a programmable wait
    time is executed
    \param[in] bDelay wait time which is executed after clock switching
    
    \internal
        \Traceability{Primus2P-3750}
    \endinternal
 */
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsClkSwitchFrcWithDelay_C(uint8_t bDelay)
{
    ATA_globalsClkSwitchFrc_C();
    for(uint8_t i = 0U; i < bDelay; i++) {
      _NOP;
    }    
}

/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsClkSwitchMrc_C</b>
    switches the AVR core to be clocked with the MRC. Keep prescaler
    settings in mind.\n\n
    HW considerations:
    The MRC is always on when DVCC is on, i.e. when the AVR is running, the MRC
    is on and can only be turned off via dedicated sleep modes.

    \image html ATA_globalsClkSwitchMrc_C.png

    \internal
    \li 010: Disable all interrupts to allow for an atomic execution of an AVR clock
             switch to the MRC and
             set AVR clock to MRC by first enabling to change register CMCR by
              setting bit CMCCE in register CMCR to 1 and, within four cycles, set
              bit CCS to 0.

    \li 015: Disable "DVCC high enable" as it is no longer required, since the AVR
             core has been switched to the MRC.

    \li 020: Set the timer clock prescaler value to 1 by first enabling to change
              register CLPR by setting bit CLPCE in register CLPR to 1 and, within
              four cycles, set register CLPR to the value CLTPS0 and
             restore the previous interrupt setting, since the critical section is
              finished.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-872,Primus2P-874}
    \endinternal
\n
*/
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsClkSwitchMrc_C(void)
{
    uint8_t bSreg = SREG;
    uint8_t cmcr = 0x00U;

    /* LLR-Ref: 010 */
    _CLI;
    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 015 */
    SUPCR &= ~BM_DVHEN;

    /* LLR-Ref: 020 */
    CLPR = BM_CLPCE;
    CLPR = BM_CLTPS0;
    SREG = bSreg;
}

/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsClkSwitchSrc_C</b>
    switches the AVR core to be clocked with the SRC. Keep prescaler
    settings in mind.\n\n
    HW considerations:
    The SRC is always on when DVCC is on, i.e. when the AVR is running, the SRC
    is on and can only be turned off via dedicated sleep modes.

    \image html ATA_globalsClkSwitchSrc_C.png

    \internal
             Beginning of AVR clock change to SRC

    \li 010: Disable all interrupts to allow for an atomic execution of an AVR clock
             switch to the SRC and
             enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, set bit CCS to 0 and bits CMM2..0 set
              to 0 (SRC) in order to be able to change bits CMM2..0 to SRC.

    \li 020: Enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, set bit CCS set to 1 in order to
              activate the value of CMM2..0 bits.\n\n
             End of AVR clock change to SRC

    \li 030: Disable "DVCC high enable" as it is no longer required, since the AVR
             core has been switched to the SRC.

    \li 040: Set the timer clock prescaler value to 1 by first enabling to change
              register CLPR by setting bit CLPCE in register CLPR to 1 and, within
              four cycles, set register CLPR to the value CLTPS0 and
             restore the previous interrupt setting, since the critical section is
              finished.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Pimus2P-2527}
    \endinternal
\n
*/
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsClkSwitchSrc_C(void)
{
    uint8_t bSreg = SREG;
    uint8_t cmcr = CMCR & 0x30U;

    /* LLR-Ref: 010 */
    _CLI;
    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 020 */
    cmcr |= BM_CCS;

    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 030 */
    SUPCR &= ~BM_DVHEN;

    /* LLR-Ref: 040 */
    CLPR = BM_CLPCE;
    CLPR = BM_CLTPS0;
    SREG = bSreg;
}


/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsClkSwitchExt_C</b>
    switches the AVR core to be clocked with the an external clock source.
    Keep prescaler settings in mind.\n\n
    HW considerations:
    If the external clock fails while external clock monitoring is still
    active, it is up to the Application SW whether or not the corresponding
    interrupt routine is executed (depends on the configuration of CMIMR).
    This function is not enabling the external clock monitoring fail interrupt.

    \param[in]  fDvccHighEnable         Indication whether to enable DVCC High enable

    \image html ATA_globalsClkSwitchExt_C.png

    \internal
             Note:
             To allow for any "high priority" interrupt to be executed faster,
             interrupts are enabled and disabled again.\n\n
             Beginning of AVR clock change to external clock source

    \li 010: IF parameter "fDvccHighEnable" is set to TRUE (indicating an external
             clock greater 1 MHZ, THEN
               Enable "DVCC high enable" by setting bit DVHEN in register
               SUPCR to 1, since the AVR system clock is greater than 1 MHZ.
             ELSE
               Disable "DVCC high enable" by setting bit DVHEN in register SUPCR to
               0, since the AVR system clock is smaller than or equal to 1 MHZ.
             ENDIF

    \li 015: Clear any previous External Clock error indication via bit ECF in
             register CMSR
              AND
             enable the External Clock Monitoring error interrupt via bit ECIE
             in register CMIMR

    \li 020: Disable all interrupts to allow for an atomic execution of an AVR clock
             switch to the external clock source and
             enable changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, set bit CCS to 0 and bit CMM1 to 1
              (External Clock) and bit CMOEN to 1 to enable the external clock
               monitoring.

    \li 030: Enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, set bit CCS set to 1 in order to
              activate the value of CMM2..0 bits.\n\n
             End of AVR clock change to the external clock

    \li 040: Set the timer clock prescaler value to 1 by first enabling to change
              register CLPR by setting bit CLPCE in register CLPR to 1 and, within
              four cycles, set register CLPR to the value CLTPS0 and
             Restore global interrupt configuration, since the critical section is
             finished.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1994}
    \endinternal
\n
*/
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsClkSwitchExt_C(uint8_t fDvccHighEnable)
{
    uint8_t bSreg = SREG;
    uint8_t cmcr = CMCR & 0x30U;

    /* LLR-Ref: 010 */
    if (fDvccHighEnable == TRUE)
    {
        SUPCR |= BM_DVHEN;
    }
    else
    {
        SUPCR &= ~BM_DVHEN;
    }

    /* LLR-Ref: 015 */
    CMSR  |= BM_ECF;
    CMIMR |= BM_ECIE;

    /* LLR-Ref: 020 */
    cmcr |= (BM_CMONEN | BM_CMM1);

    _CLI;
    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 030 */
    cmcr |= BM_CCS;

    CMCR = BM_CMCCE;
    CMCR = cmcr;

    /* LLR-Ref: 040 */
    CLPR = BM_CLPCE;
    CLPR = BM_CLTPS0;
    SREG = bSreg;
}

/* ---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsClkSwitchXTO_C</b>
    switches the AVR core to be clocked with the given XTO value (including DVCC
    high enable activation)
    Keep prescaler settings in mind.\n\n
    Precondition:
    In order for this function to perform its intended functionality
    successfully, the XTO must have been activated already.\n\n
    HW considerations:
    The CCS bit and CMM2..0 bits in register CMCR need to be set separately in
    order to switch from XTO to FRC correctly (both clocks require bit CCS to be
    set to 1).
    The following code snippet "CMCR = BM_CMCCE" does not set bit CCS to 0, as
    only the CMCCE is enabled. Any change has to occur one cycle after CMCCE
    is set to 1.

    \param[in]  bXtoClockSelect     Indication whether to select XTO4 or XTO6 as system clock

    \image html ATA_globalsClkSwitchXTO_C.png

    \internal
    \li 010: Keep upper 4 MSB bits of register CMCR for future use, since these
             bits are not to be altered by this function.

    \li 012: Enable "DVCC high enable" in order to provide enough current for the
             AVR core when running with FRC.\n\n
             Set AVR clock to selected XTO value:

    \li 020: Disable all interrupts to allow for an atomic execution of a register
             CMCR change and
             enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, assign the temporary clock value to
              register CMCR, bit CCS set to 0 in order to be able to change
              bits CMM2..0 to the given XTO value "bXtoClockSelect".\n\n
             Note:
             To allow for any "high priority" interrupt to be executed faster,
             interrupts are enabled and disabled shortly after.

    \li 030: Enabling changing register CMCR by setting bit CMCCE in register CMCR
              to 1 and, within four cycles, assign the temporary clock value to
              register CMCR, bit CCS set to 1 in order to activate the value of
              CMM2..0 bits.\n\n
             End of AVR clock change to the given XTO value

    \li 040: Set the timer clock prescaler value to 1 by first enabling to change
              register CLPR by setting bit CLPCE in register CLPR to 1 and, within
              four cycles, set register CLPR to the value CLTPS0 and
             restore global interrupt configuration, since the critical section is
              finished.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1793,Primus2P-1794}
    \endinternal
\n
*/
/* ---------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsClkSwitchXTO_C(uint8_t bXtoClockSelect)
{
    /* LLR-Ref: 010 */
    uint8_t bSreg = SREG;
    uint8_t bCmcr = CMCR & 0x30U;

    /* LLR-Ref: 012 */
    SUPCR |= BM_DVHEN;

    /* LLR-Ref: 020 */
    bCmcr |= (bXtoClockSelect & 0x07U);

    _CLI;
    CMCR = BM_CMCCE;
    CMCR = bCmcr;

    /* LLR-Ref: 030 */
    bCmcr |= BM_CCS;

    CMCR = BM_CMCCE;
    CMCR = bCmcr;

    /* LLR-Ref: 040 */
    CLPR = BM_CLPCE;
    CLPR = BM_CLTPS0;
    SREG = bSreg;
}


/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsDeActivateXTO_C</b>
    deactivates the XTO.

    \image html ATA_globalsDeActivateXTO_C.png

    \internal
    \li 010: Deactivate by clearing FEEN1 register and SUPCR AVCC related bits
             if AVCC is enabled

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-2524}
    \endinternal
\n
*/
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsDeActivateXTO_C(void)
{
    /* LLR-Ref: 010 */
    if (SUPCR & BM_AVEN) {
        FEEN1 = 0U;
        SUPCR &= (uint8_t)~(BM_AVEN | BM_AVCCRM | BM_AVCCLM | BM_AVDIC);
    }
}

/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsActivateXTO_C</b>
    activates AVCC as a precondition for the XTO and the XTO itself.

    Variable Usage:
    \li [out] ::g_sTimer1 Global Timer 1 component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_globalsActivateXTO_C.png

    \internal
    \li 010:    Enable AVCC via SUPCR.AVEN
    \li 035:    Load FETN4 value from EEPROM, since this value is mandatory for the
                 XTO to work correctly. In case of an EEPROM read error, set the 
                 ::g_sDebug to DEBUG_ERROR_CODE_GLOBALS_EEPROM_READ_ERROR

                Note:
                FETN4 is loaded while waiting for AVCC and MVCC settling time
                 to be finished to not extend the overall execution time of this 
                 function
                Loading of register FETN4 has to be done after AVCC is up and 
                 running in order for the register assigned value to be stored 
                 correctly.

    \li 040:    Wait until AVCC is stable (SUPFR.AVVCLF and SUPFR.AVCCRF 
                 are cleared)
    \li 070:    Store loaded EEPROM value for FETN4 to register FETN4 and enable 
                 XTO
    \li 080:    Wait until XTO is ready via polling of bit FESR.XRDY

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-865,Primus2P-867}
    \endinternal
\n
*/
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsActivateXTO_C(void)
{
    uint8_t bFetn4Val = 0x00;
    
    /* LLR-Ref: 010 */
    SUPCR |= BM_AVEN;

    /* LLR-Ref: 035 */
    eEepErrorCode sEepErrCode = ATA_eepReadBytes_C(&bFetn4Val, (uint16_t)&g_sAtmelEEPromSection.eepFETN4, 1U);
    if(sEepErrCode != EEC_NO_ERROR)
    {
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_GLOBALS_EEPROM_READ_ERROR;
    }
    
    /* LLR-Ref: 040 */
    do
    {
        SUPFR |= (BM_AVCCLF | BM_AVCCRF);

    } while ( SUPFR&(BM_AVCCLF|BM_AVCCRF) );

    /* LLR-Ref: 070 */
    FETN4 = bFetn4Val;
    FEEN1 |= BM_XTOEN;

    /* LLR-Ref: 080 */
    while ( !(FESR & BM_XRDY))
    {}
}

/*-----------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsInitSramSpace_C</b>
    initializes length bytes of the sram starting from address pData

    \param[out] pData       start address of initialization
    \param[in]  bLength     number of bytes to initialize

    \image html ATA_globalsInitSramSpace_C.png

    \internal
    \li 010:    Check if length is greater 0 otherwise skip initialization
    \li 020:    Initialize length bytes "bLength" of sram space starting from
                "pData"

    \Derived{Yes}

    \Rationale{A means is to be provided to efficiently and consistently
               initialize SRAM data of arbitrary length}

    \Traceability   N/A
    \endinternal
\n
 */
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsInitSramSpace_C(uint8_t *pData, uint8_t bLength)
{
    uint8_t *ptr = pData;

    /* LLR-Ref: 010 */
    if (bLength) {

        /* LLR-Ref: 020 */
        do{
            *ptr++ = 0x00U;
        }while (--bLength);
    }
}

/*-----------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsCopySramSpace_C</b>
    initializes length bytes of the sram starting from address pData

    \param[out] pDestination    destination address
    \param[in]  pSource         source address
    \param[in]  bLength         number of bytes to copy

    \image html ATA_globalsCopySramSpace_C.png

    \internal
    \li 010:    Check if length is greater 0 otherwise skip initialization
    \li 020:    Copy length bytes from source to destination

    \Derived{Yes}

    \Rationale{A means is to be provided to efficiently and consistently
               copy SRAM data of arbitrary length}

    \Traceability   N/A
    \endinternal
\n
 */
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsCopySramSpace_C(uint8_t *pDestination, uint8_t *pSource, uint8_t bLength)
{
    /* LLR-Ref: 010 */
    if (bLength) {
        /* LLR-Ref: 020 */
        do {
            *pDestination++ = *pSource++;
        } while(--bLength);
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsSetVoltageMonitor_C</b>
    set a new value for the voltage monitor control register VMCR.

    \param[in]  bVmcrVal  New value for VMCR

    \image html ATA_globalsSetVoltageMonitor_C.png

    \internal
    \li 010:    Calculate content of variable bWaitVmCycles dependent from AVR core clock
                - AVR core clock XTO4 bWaitVmCycles = VOLTAGE_MONITOR_XTO4_WAIT_CYCLES
                - AVR core clock SRC  bWaitVmCycles = VOLTAGE_MONITOR_SRC_WAIT_CYCLES
                - AVR core clock FRC  bWaitVmCycles = VOLTAGE_MONITOR_FRC_WAIT_CYCLES
                - AVR core clock MRC  bWaitVmCycles = VOLTAGE_MONITOR_MRC_WAIT_CYCLES
    \li 020:    Disable the Voltage Monitor via PRR0.PRVM = 1
    \li 030:    Check if Voltage Monitor shall be enabled via variable bVmcrVal[3..0]
                If Voltage Monitor shall be enabled
    \li 040:    Power up voltage monitor via PRR0.PRVM = 0
    \li 050:    Set VMCR register configuration with bVmcrVal except VMCR.VMIM setting
    \li 060:    Wait ~25us
    \li 070:    Clear the interrupt flag VMSR.VMF by writing a 1 in order to not trigger an
                interrupt when the mask flag is set.
    \li 080:    Wait ~25us
    \li 090:    Enable VMCR.VMIM if set in bVmcrVal

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1473}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsSetVoltageMonitor_C( uint8_t bVmcrVal )
{
    uint8_t bWaitVmCycles = 0U;

    /* LLR-Ref: 010 */
    if (CMCR & BM_CCS) {
        if (CMCR & BM_CMM2) {                                   // AVR core running with XTO4
            bWaitVmCycles = VOLTAGE_MONITOR_XTO4_WAIT_CYCLES;
        } else if ( (CMCR & (BM_CMM2|BM_CMM1|BM_CMM0)) == 0) {  // AVR core running with SRC
            bWaitVmCycles = VOLTAGE_MONITOR_SRC_WAIT_CYCLES;
        } else {                                                // AVR core running with FRC/EXT
            bWaitVmCycles = VOLTAGE_MONITOR_FRC_WAIT_CYCLES;
        }
    } else {                                                    // AVR core running with MRC
        bWaitVmCycles = VOLTAGE_MONITOR_MRC_WAIT_CYCLES;
    }

    /* LLR-Ref: 020 */
    PRR0 |= BM_PRVM;    /* disable VM anyway */

    /* LLR-Ref: 030 */
    if ( (bVmcrVal & (BM_VMLS3|BM_VMLS2|BM_VMLS1|BM_VMLS0)) != 0U) {
        /* LLR-Ref: 040 */
        PRR0 &= (uint8_t)~BM_PRVM;       /* VM enabled */
        /* LLR-Ref: 050 */
        VMCR = (bVmcrVal & ~BM_VMIM);    /* Set VMCR   */
        /* LLR-Ref: 060 */
        for(uint8_t i = 0U; i < bWaitVmCycles; i++) {
          _NOP;
        }
        /* LLR-Ref: 070 */
        VMSCR |= BM_VMF;
        /* LLR-Ref: 080 */
        for(uint8_t i = 0U; i < bWaitVmCycles; i++) {
          _NOP;
        }
        /* LLR-Ref: 090 */
        if (bVmcrVal & BM_VMIM) {
            VMCR |= BM_VMIM;
        }
    }
}

/*-----------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsInitDebug_C</b>
    initializes variable g_sDebug

    Variable Usage:
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_globalsInitDebug_C.png

    \internal
    \li 010: Set ::g_sDebug attributes to default values

    \Derived{Yes}

    \Rationale{In order provide a consistent and maintainable error handling
               approach, a dedicated DEBUG component has been introduced
               during the SW design process}

    \Traceability   N/A
    \endinternal
\n
 */
/*-----------------------------------------------------------------------------*/
VOIDFUNC ATA_globalsInitDebug_C(void)
{
    g_sDebug.bErrorCode     = DEBUG_ERROR_CODE_SYSTEM_ERROR_NOT_USED;
    g_sDebug.bSsmErrorCode  = 0x00U;
}
