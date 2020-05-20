//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/globals/src/globals_flash.c $
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
/** \file globals_flash.c
 */

//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "globals_flash.h"
#include "../../spi/src/ata5700_command_set_flash.h"
#include "../../init/src/init_flash.h"
#include "../../system/src/system_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

//-----------------------------------------------------------------------------
/** \brief <b>ATA_globalsSwitchAvrPhClock_flash_C</b>
    Shall handle the clock switch for the PH_CRC and PH_FIFO from the AVR clock
    to PH clock and vice versa.

    010: Clocks are switched as defined by 'uSwitchValue' (see register
         description of LDFCKSW). Afterwards the function waits till the status
         register has the value of 'uStatusValue'.

    \param[in]  uSwitchValue    value for clock switch

    \return none

    \Derived yes

    \Rationale In order to provide access to the Protocol Handler(PH) FIFO for
               both the AVR and the Protocol Handler itself, a means to switch
               the clocks is necessary.

    \Traceability None

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
/*---------------------------------------------------------------------------*/
void ATA_globalsSwitchAvrPhClock_flash_C(uint8_t bSwitchValue)
{
  /* LLR-Ref: 010 */
  LDFCKSW = bSwitchValue;
  while(LDFCKSW != bSwitchValue);
}

/*-----------------------------------------------------------------------------*/
/**  \brief <b>ATA_globalsExtClockMonitoringInterrupt_ISR_C</b>
    this interrupt service routine is triggered to indicate that
    external clock input is lost.

    HW considerations:
    Clock monitoring and the corresponding interrupt must be enabled
    Clock is automatically switched to MRC, i.e. CCS bit is set to 0.

    \param[out]  g_sCalibConfig     Calibration component data

    \return     VOIDFUNC

    \Derived    No

    \Rationale  N/A

    \Traceability   Primus2P-???,
                    Primus2P-???

    \StackUsage     SU_XXX bytes

    \image html ATA_globalsExtClockMonitoringInterrupt_ISR_C.png
    \image rtf ATA_globalsExtClockMonitoringInterrupt_ISR_C.png
    \n

*/
/*-----------------------------------------------------------------------------*/
#pragma vector=EXCM_vect
__interrupt VOIDFUNC ATA_globalsExtClockMonitoringInterrupt_ISR_flash_C(void)
{
    uint8_t cmcr = CMCR & (~BM_CMONEN);

    /* Disable DVCC High Enable */
    SUPCR &= ~BM_DVHEN;

    /* Disable Clock monitor, since ext. clock is broken to avoid multiple
       execution of this interrupt. */
    CMCR = BM_CMCCE;
    CMCR = cmcr;

    CMIMR &= ~BM_ECIE;

    /* It has to be decided whether switching off MVCC (including the defined
       EEPROM wait time is to be done */

    /* Set event flag */
    g_sAta5700_flash.events_system |= BM_ATA5700_EVENTS_SYSTEM_EXTCLOCK_ERR;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bSystem & BM_ATA5700_EVENTS_SYSTEM_EXTCLOCK_ERR )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_globalsTimer0Interrupt_ISR_flash_C</b>
    shall handle the Timer 0 interrupt.

    \return VOIDFUNC

    \Derived no

    \Rationale none

    \Traceability

    \StackUsage SU_XXX bytes

    \image html ATA_globalsTimer0Interrupt_ISR_flash_C.png
    \image rtf ATA_globalsTimer0Interrupt_ISR_flash_C.png
    \n
*/
/*---------------------------------------------------------------------------*/
#pragma vector=T0INT_vect
__interrupt VOIDFUNC ATA_globalsTimer0Interrupt_ISR_flash_C(void)
{
    /* Check if EOT has been detected */
    g_sAta5700_flash.events_system |= BM_ATA5700_EVENTS_SYSTEM_TIMER_0;

    /* Do event pin handling */
    if ( g_sEventHandling_flash.bSystem & BM_ATA5700_EVENTS_SYSTEM_TIMER_0 )
    {
        ATA_systemSetEventPin_flash_ASM();
    }
}
