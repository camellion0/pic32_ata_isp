//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/02_AutoRF/Primus2pSW/Branches/RB_PRIMUS2P_ROM_1.3/firmware/spi/src/spi_flash.c $
  $LastChangedRevision: 244572 $
  $LastChangedDate: 2014-02-12 14:37:22 +0100 (Mi, 12 Feb 2014) $
  $LastChangedBy: florian.schweidler $
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
/** \file spi_flash.c
*/

//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "spi_flash.h"
#include "spi.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiSlaveOpen_flash_C</b>
    shall setup SPI interface either for master or slave mode.

\param[out] ::g_sSpiConfig  Global SPI component data
\param[out] ::g_sDebug      Global Debug component data
\param[in]  bSpcr           SPCR register setting
\param[in]  bSpsr           SPSR register setting

\return     FAIL if SPI is already opened, otherwise OK

\StackUsageInBytes{XXX}

\image html ATA_spiOpen_C.png

\internal
\li 005: Activate SPI clock domain to access corresponding registers
         IF the SPI interface is already in use as indicated by bit SPE in
         register SPCR being set to 1, THEN
\li 008: Set ::g_sDebug .bErrorCode to DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED\n
         ELSE
\li 010:   Power up SPI interface via PRR0.PRSPI=0
\li 020:   Configure the ports as follows
            - PD1/NSS  - SPI Not Slave Select    (input)
            - PB5/MISO - SPI Master In Slave Out (output)
            - PB6/MOSI - SPI Master Out Slave In (input)
            - PB7/SCK  - SPI Clock               (input)

\li 040:   Configure SPCR register setting according to function parameters
            - SPSR = spsr
            - SPCR = spcr

\li 050:   Set SPI status to "INITIALIZED" in ::g_sSpiConfig .bStatus\n
         ENDIF

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-788}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_spiSlaveOpen_flash_C(uint8_t bSpcr, uint8_t bSpsr)
{
    uint8_t fRetVal = FAIL;
    uint8_t bPrr1 = PRR1;
    
    /* LLR-Ref: 005 */
    ATA_POWERON_C(PRR0, PRSPI)
    
    if (SPCR & BM_SPE)
    {
        /* LLR-Ref: 008 */
        PRR1 = bPrr1;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_ALREADY_OPENED;
    }
    else
    {
        /* LLR-Ref: 020 */
        DDRB &= ~(BM_SPI_MOSI_PIN | BM_SPI_SCK_PIN);
        DDRB |=  BM_SPI_MISO_PIN;
        DDRD &= ~(BM_SPI_NSS_PIN);
        
        /* INT0 for NSS interrupt. Generate an interrupt on any INT0
         * change and leave INT1 unchanged
         */
        EICRA |= BM_ISC00;
        EICRA &= (uint8_t)~(BM_ISC01);
        EIMSK |= BM_INT0;

        /* LLR-Ref: 040 */
        SPSR  = bSpsr;
        SPCR  = bSpcr;

        /* LLR-Ref: 050 */
        g_sSpiConfig.bStatus = BM_SPICONFIG_STATUS_INITIALIZATION_STATUS;
        fRetVal = OK;
    }
    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiSlaveClose_flash_C</b>
    shall disable the SPI interface.

\param[out] ::g_sSpiConfig  Global SPI component data

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_spiClose_C.png

\internal
\li 005: Disable INT0, since SPI slave interface is to be closed

\li 010: Reset SPCR register setting

\li 020: Power down SPI interface via PRR0.PRSPI=1

\li 030: Set output back pins to input

\li 040: Set SPI status to "NOT INITIALIZED" in ::g_sSpiConfig .bStatus

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-958}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_spiSlaveClose_flash_C(void)
{
    /* LLR-Ref: 005 */
    EIMSK &= ~BM_INT0;
  
    /* LLR-Ref: 010 */
    SPCR = 0U;

    /* LLR-Ref: 020 */
    ATA_POWEROFF_C(PRR0, PRSPI)

    /* LLR-Ref: 030 */
    DDRB &= (uint8_t)~(BM_SPI_MOSI_PIN | BM_SPI_MISO_PIN | BM_SPI_SCK_PIN);
    DDRD &= (uint8_t)~(BM_SPI_NSS_PIN);
    
    /* LLR-Ref: 040 */
    g_sSpiConfig.bStatus &= (uint8_t)~BM_SPICONFIG_STATUS_INITIALIZATION_STATUS;
}