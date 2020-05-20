//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/spi/src/spi.c $
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
/** \file spi.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "spi.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  Modul Globals                                                            */
/*---------------------------------------------------------------------------*/
/** \brief <b>g_sSpiConfig</b>
    contains the configuration and status information for module SPI.
*/
#pragma location = ".spiConfig"
__no_init sSpiConfig g_sSpiConfig;

/*---------------------------------------------------------------------------*/
/*  IMPLEMENTATION                                                           */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiOpen_C</b>
    shall setup a SPI master interface. The PRR0.PRSPI clock domain remains 
    active after functions is left.

    \param[in]  bSpcr           SPCR register setting
    \param[in]  bSpsr           SPSR register setting
    \return     FAIL if SPI is already opened, otherwise OK

    Variable Usage:
    \li [out] ::g_sSpiConfig  Global SPI component data
    \li [out] ::g_sDebug      Global Debug component data
    
    \image html ATA_spiOpen_C.png

    \internal
    \li 005: Power up SPI interface via PRR0.PRSPI=0
    \li 010: IF the SPI interface is already in use as indicated by bit SPE in
             register SPCR being set to 1, THEN
               Set ::g_sDebug .bErrorCode to DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED\n
             ELSE
               Disable INT0 via register EIMSK

    \li 020:   Configure the ports as follows
                - PD1/NSS  - SPI Not Slave Select    (output) + Pull up
                - PB5/MISO - SPI Master In Slave Out (input)
                - PB6/MOSI - SPI Master Out Slave In (output)
                - PB7/SCK  - SPI Clock               (output)

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
UINT8FUNC ATA_spiOpen_C(uint8_t bSpcr, uint8_t bSpsr)
{
    int fRetVal = FAIL;

    /* LLR-Ref: 005 */
    ATA_POWERON_C(PRR0, PRSPI)
    
    /* LLR-Ref: 010 */
    if (SPCR & BM_SPE)
    {
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_ALREADY_OPENED;
    }
    else
    {
        EIMSK &= ~BM_INT0;
          
        /* LLR-Ref: 020 */
        DDRB |= (BM_SPI_MOSI_PIN | BM_SPI_SCK_PIN);
        DDRB &= (uint8_t)~BM_SPI_MISO_PIN;
        DDRD |= BM_SPI_NSS_PIN;
        PORTD|= BM_SPI_NSS_PIN;

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
/** \brief <b>ATA_spiClose_C</b>
    shall disable the SPI interface.

    Variable Usage:
    \li [out] ::g_sSpiConfig Global SPI component data

    \image html ATA_spiClose_C.png

    \internal
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
VOIDFUNC ATA_spiClose_C(void)
{
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


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiDataExchange_C</b>
    sends a byte via SPI interface and returns the response.

    \param[in]  data    Data byte to be sent
    \return     Received data byte

    \image html ATA_spiDataExchange_C.png

    \internal
    \li 010: Write data byte to SPDR register

    \li 020: Wait for SPSR.SPIF flag which indicates that serial transfer is complete

    \li 030: Return SPDR register content

    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the SPI
               specific data handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_spiDataExchange_C(uint8_t data)
{
    /* LLR-Ref: 010 */
    SPDR = data;
    
    /* LLR-Ref: 020 */
    while(!(SPSR & BM_SPIF));
    
    /* LLR-Ref: 030 */
    return SPDR;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiTransfer_C</b>
    does the data transfer via SPI interface.

    \param[in]      pTxData         Pointer to transmit buffer
    \param[in]      pRxData         Pointer to receive buffer
    \param[in]      bLen            Data bytes to transmit
    \return     FAIL if SPI is not initialized, otherwise OK

    Variable Usage:
    \li [out] ::g_sDebug Global Debug component data
    \li [in] ::g_sSpiConfig Global SPI component data
    
    \image html ATA_spiTransfer_C.png

    \internal
             IF SPI interface is initialized as indicated by ::g_sSpiConfig 
             .bStatus[7] being set to 1, THEN
    \li 010: Select SPI interface by calling function ::ATA_spiSelect_C

    \li 020: Write given number of data bytes to the SPI interface and read data 
             the same amount of data from the SPI interface

    \li 030: Deselect SPI interface by calling function ::ATA_spiDeselect_C\n
             ELSE
    \li 040: Set ::g_sDebug .bErrorCode to DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED\n
             ENDIF

    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the SPI
               specific data handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_spiTransfer_C(uint8_t *pTxData, uint8_t *pRxData, uint8_t bLen)
{
    uint8_t fRetVal = OK;

    if (g_sSpiConfig.bStatus & BM_SPICONFIG_STATUS_INITIALIZATION_STATUS)
    {
        /* LLR-Ref: 010 */
        ATA_spiSelect_C();

        /* LLR-Ref: 020 */
        for(uint8_t i=0; i< bLen; i++)
        {
            // *pRxData++ = ATA_spi_data_exchange(*pTxData++);
            SPDR = *pTxData++;
            while(!(SPSR & BM_SPIF));
            *pRxData++ = SPDR;
        }

        /* LLR-Ref: 030 */
        ATA_spiDeselect_C();
    }
    else
    {
        /* LLR-Ref: 040 */
        fRetVal = FAIL;
        g_sDebug.bErrorCode = DEBUG_ERROR_CODE_SPI_NOT_INITIALIZED;
    }

    return fRetVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiSelect_C</b>
    selects the SPI interface via NSS pin.

    \image html ATA_spiSelect_C.png

    \internal
    \li 010: Set the NSS pin to low to indicate the start of a SPI communication

    \li 020: Clear the SPSR.SPIF bit by first reading the SPI Status Register with 
             SPIF set, then accessing the SPI Data Register (SPDR).
        
    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the SPI
               specific data handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_spiSelect_C(void)
{
    /* LLR-Ref: 010 */
    PORTD &= (uint8_t)~BM_SPI_NSS_PIN;
    
    /* LLR-Ref: 020 */
    uint8_t bTmp = SPSR;
    bTmp = SPDR;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiDeselect_C</b>
    deselects the SPI interface via NSS pin.

    \image html ATA_spiDeselect_C.png

    \internal
    \li 010: Set the NSS pin to high to indicate the end of a SPI communication

    \Derived{Yes}

    \Rationale{This function is a helper function in order to implement the SPI
               specific data handling}

    \Traceability   N/A
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_spiDeselect_C(void)
{
    /* LLR-Ref: 010 */
    PORTD |= BM_SPI_NSS_PIN;
}
