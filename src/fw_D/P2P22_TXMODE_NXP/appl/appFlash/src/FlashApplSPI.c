/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/FlashApplSPI.c $
  $LastChangedRevision: 591974 $
  $LastChangedDate: 2020-03-16 09:23:12 -0600 (Mon, 16 Mar 2020) $
  $LastChangedBy: grueter $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 5.51.0
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

/** \file FlashSPI.c
    this file contains an ATA5700 SPI application software
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../../firmware/init/src/init.h"
#include "../../../firmware/rftx/src/rftx.h"
#include "../../../firmware/lfrx/src/lfrx.h"
#include "../../../firmware/spi/src/ata5700_command_set_flash.h"
#include "../../../firmware/init/src/init_flash.h"
#include "../../../firmware/system/src/system_flash.h"
#include "../../../firmware/timer1/src/timer1.h"
#include "../../../firmware/timer5/src/timer5_flash.h"
#include "../../../firmware/timer4/src/timer4_flash.h"
#include "../../../firmware/globals/src/globals.h"
#include "../../../firmware/lfrx/src/lfrx_flash.h"
#include "../../../firmware/tp/src/tp_flash.h"
#include "../../../firmware/extif/src/extif_flash.h"
#include "../../../firmware/lfrssi/src/lfrssi.h"
#include "../../../firmware/lfrssi/src/lfrssi_flash.h"
#include "../../../firmware/calib/src/calib.h"
#include "../../../firmware/aes/src/aes.h"
#include "../../../firmware/spi/src/spi.h"
#include "../src/micro.h"
#include "rfrcc_flash.h"
#include "FlashApplVars.h"
#include "../src/FlashApplPEPS.h"
#include "FlashApplMSG.h"
#include "../src/FlashApplLF.h" 


/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
extern sFlashApplState gFlashApplState;
extern sFlashApplVars gFlashApplVars;
extern tTimer5Status gTimer5Status;
//uint8_t m_bTempArray_flash[10];
//extern m_bTempArray_flash[10];
#pragma location = ".sram_SPI_RXbuffer"
__no_init uint8_t g_SPI_RXbuffer[10];


//uint8_t *pTxData = &g_bModuleTestBuffer_flash[0];
//uint8_t *pRxData = &g_bModuleTestBuffer_flash[BASIC_SPI_FUNCTION_TELEGRAM_LENGTH];

/*===========================================================================*/
/*  Modul Prototypes                                                         */
/*===========================================================================*/
//void SendShortCommand(void);
//void ATA_Flash_IOinit(void);
/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_spiOpenSlaveFlash_C</b>
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
                - PD1/NSS  - SPI Not Slave Select    (input) + Pull up
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
UINT8FUNC ATA_spiOpenSlaveFlash_C(uint8_t bSpcr, uint8_t bSpsr)
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
        EIMSK |= BM_INT0;  //Enable PD1 NSS interrupt
          
        /* LLR-Ref: 020 */
        DDRB &= ~(BM_SPI_MOSI_PIN | BM_SPI_SCK_PIN);
        DDRB |= (uint8_t)BM_SPI_MISO_PIN;
        DDRD &= (uint8_t)~BM_SPI_NSS_PIN;
        PORTD|= BM_SPI_NSS_PIN;  //pull up active

        SFIR |= (1<<RIL0);

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
/** \brief <b>ATA_spiCloseSlaveFlash_C</b>
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
VOIDFUNC ATA_spiCloseSlaveFlash_C(void)
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
/*----------------------------------------------------------------------------- */
/*brief <b>PCINTO Interrupt handler</b>*/
            
/*    The function contains the interrupt vector for the Pin Change interrupts 0-7.
    This routine determins the specefic pin that generated the interupt and sets
    variables to alert the main loop that a new button press has been detected.*/
#pragma vector=INT0_vect 
__interrupt VOIDFUNC ATA_Int0Handler_ISR_flash_C(void)
{
 // ATA_globalsClkSwitchFrc_C();
  MCUCR |= (1<<SPIIO); //Disable all but SPI interrupts
  EIMSK &= ~(BM_INT0); //Disable PD1 NSS interrupt
  ATA_spiReceveDataFlash_C();
  
  EIFR = 0x01; //Clear INT0 flag
  MCUCR &= ~(1<<SPIIO); //Disable all but SPI interrupts
  EIMSK |= BM_INT0;  //Enable PD1 NSS interrupt
}
VOIDFUNC ATA_spiReceveDataFlash_C(void)
{
  //bit_set(LED1);
  uint8_t SpiByteCounter = 0;
  
while (  (PIND & BM_SPI_NSS_PIN) != BM_SPI_NSS_PIN )
     {  
        if (  (SPSR & (1<<SPIF))== (1<<SPIF)   ){
        g_SPI_RXbuffer[SpiByteCounter] = SPDR;
        SpiByteCounter++;
        if (SpiByteCounter>8) SpiByteCounter=0;
        }
     }
gFlashApplState.State |= BM_SPI_received_flag;
gFlashApplVars.SPIcount = SpiByteCounter;
return;  
}

