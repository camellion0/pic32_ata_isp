//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrx/src/lfrx.c $
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
/** \file lfrx.c
*/
//lint -restore

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "lfrx.h"
#include "../../eep/src/eep.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Globals                                                                  */
/*===========================================================================*/
/** \brief <b>g_sLfRx</b>
    contains the status information for module LFRX.
*/
//lint -esym(9003, g_sLfRx) FlSc (10.06.2014)
/* disable lint note 9003 - could define variable 'g_sLfRx' at block scope
 * variable shall be accessible from outside via flash software or other library
 * modules
 */
#pragma location = ".lfRx"
__no_init sLfRx g_sLfRx;


/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_lfRxInit_C</b>
    shall initialize the LF Receiver calibration registers with the EEPROM
    configured data.

    Variable Usage:
    \li [out] ::g_sLfRx Global LF Rx component data
    \li [out] ::g_sDebug Global Debug component data

    \image html ATA_lfRxInit_C.png

    \internal
    \li 010: Reset flags of ::g_sLfRx since the default state before starting the
              LF initialization is ok.

    \li 020: Enable the LF Receiver clock by setting bit PRLFR in register PRR1 to
              0 in order to be able to update LF related registers
             AND
             Enable the Protocol Handler clock by setting bit PRLFPH in register 
              PRR1 to 0 in order to be able to update SRC related registers.

    \li 025: Wait for any previous EEPROM access (Write or AES) to be finished by
              checking the NVMBSY bit in register EECR being 0.

    \li 030: Set address of "SRCTCAL" EEPROM location to EEARH, EEARL registers
              in order to subsequently load the values for SRCTCAL and SRCCAL,
             AND
             Enable the EEPROM burst read mode by setting bit EEBRE in register
              EECR2 to 1.

    \li 040: Load SRC temperature compensation value (SRCTCAL) from its dedicated
              EEPROM location to register SRCTCAL,
             Load SRC calibration value (SRCCAL) from its dedicated EEPROM location
              to register SRCCAL.

    \li 050: Set starting address of LF calibration EEPROM location to EEARH, EEARL
              registers in order to subsequently load the LF calibration values.

    \li 060: Load all LF calibration values (LFCALR1 to LFCALR53) from their
              dedicated EEPROM location to registers LFCALR1 to LFCALR53.

    \li 065: Load Transponder calibration value (TPCALR11) from its dedicated
              EEPROM location to register TPCALR11,
             Load Transponder calibration value (TPCALR12) from its dedicated
              EEPROM location to register TPCALR12.
             Load Transponder calibration value (TPCALR13) from its dedicated
              EEPROM location to register TPCALR13.

    \li 070: Load all LF decoder configuration values (LFDSR1 to LFDSR11) from their
              dedicated EEPROM location to registers LFDSR1 to LFDSR11.

    \li 080: IF after the EEPROM read accesses to retrieve the requested data, an
              uncorrectable EEPROM error was detected, indicated by
              bit "E2FF" in register EECR2 being set 1,
             THEN
               Indicate an LF initialization error by setting the error flag in
                ::g_sLfRx and setting the global error code in ::g_sDebug to
                the LF Rx EEPROM read error.
             ENDIF

    \li 090: Clear EEPROM burst read mode flag by setting bit "EEBRE" in register
              EECR2 to 0 and clear both EEPROM error flags by setting bits "E2CF"
              and "E2FF" to 1.

    \li 100: Lock and activate all LF calibration registers by setting bits
              "LFCALRY" and "LFCALP" in register LFCPR to 1.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-1989,Primus2P-2152}
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_lfRxInit_C(void)
{
    uint8_t bSreg = SREG;
    uint8_t* pDestValue;

    /* LLR-Ref: 010 */
    g_sLfRx.bFlags = LFRX_FLAGS_RESET;

    /* LLR-Ref: 020 */
    ATA_POWERON_C(PRR1, PRLFR);
    ATA_POWERON_C(PRR1, PRLFPH);

    /* LLR-Ref: 025 */
    while(EECR & BM_NVMBSY){}

    /* lint -e923 GeWi (23nov2011)*/
    /* disable lint error 923 - Cast between pointer type and an integral type.
     * Misra Rule 11.3 says: Casting between a pointer and an integer type should be avoided where possible,
     * but may be unavoidable when addressing memory mapped registers or other hardware specific features.
     */
    /* LLR-Ref: 030 */
    EEARH = (uint8_t)((uint16_t)&g_sAtmelEEPromSection.eepSRCTCAL >> 8U);
    EEARL = (uint8_t)((uint16_t)&g_sAtmelEEPromSection.eepSRCTCAL >> 0U);
    /*lint -restore */

    EECR2 |= BM_EEBRE;

    /* LLR-Ref: 040 */
    SRCTCAL = EEDR;
    SRCCAL  = EEDR;

    /* LLR-Ref: 050 */
    EEARH = (uint8_t)((uint16_t)&g_sAtmelEEPromSection.eepLFCALR[0] >> 8U);
    EEARL = (uint8_t)((uint16_t)&g_sAtmelEEPromSection.eepLFCALR[0] >> 0U);
    
    /* LLR-Ref: 060 */
    pDestValue = (uint8_t*)&LFCALR1;
    for( uint8_t i = 0x00U; i < EEP_NUM_LF_CALIB_REG; i++)
    {
        *pDestValue++ = EEDR;
    }
    
    /* LLR-Ref: 065 */
    TPCALR11 = EEDR;
    TPCALR12 = EEDR;
    TPCALR13 = EEDR;
    
    /* LLR-Ref: 070 */
    pDestValue = (uint8_t*)&LFDSR1;
    for( uint8_t i = 0x00U; i < EEP_NUM_LF_DECODER_REG; i++)
    {
        *pDestValue++ = EEDR;
    }
    
    /* LLR-Ref: 080 */
    if ( EECR2 & BM_E2FF )
    {
        g_sLfRx.bFlags |= LFRX_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRX_EEPROM_READ_ERROR;
        EECR2 |= BM_E2FF;
    }
    
    /* LLR-Ref: 090 */
    if(EECR2 & BM_E2AVF)
    {
        g_sLfRx.bFlags |= LFRX_FLAGS_BM_ERROR_FLAG;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_LFRX_EEPROM_ACCESS_ERROR;
        EECR2 |= BM_E2AVF;
    }

    /* LLR-Ref: 090 */
    EECR2 &= ~BM_EEBRE;

    /* LLR-Ref: 100 */
    __disable_interrupt();
    LFCPR = BM_LFCPCE;
    LFCPR = (BM_LFCALRY | BM_LFCALP);
    SREG = bSreg;
}
