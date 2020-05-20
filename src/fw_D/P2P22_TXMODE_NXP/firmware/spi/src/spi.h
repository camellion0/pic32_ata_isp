//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/spi/src/spi.h $
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
/** \file spi.h
 */
//lint -restore

#ifndef SPI_H
#define SPI_H

#ifdef __IAR_SYSTEMS_ICC__
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"

/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
/* create alias for the different SPI chip pins - code assumes all on port B */
#define SPI_NSS_PIN  PORTD1
#define SPI_MISO_PIN PORTB5
#define SPI_MOSI_PIN PORTB6
#define SPI_SCK_PIN  PORTB7

#define BM_SPI_NSS_PIN  BM_PORTD1
#define BM_SPI_MISO_PIN BM_PORTB5
#define BM_SPI_MOSI_PIN BM_PORTB6
#define BM_SPI_SCK_PIN  BM_PORTB7

/* SPI clock modes */
#define SPI_MODE_0 0x00 /* Sample (Rising) Setup (Falling) CPOL=0, CPHA=0 */
#define SPI_MODE_1 0x01 /* Setup (Rising) Sample (Falling) CPOL=0, CPHA=1 */
#define SPI_MODE_2 0x02 /* Sample (Falling) Setup (Rising) CPOL=1, CPHA=0 */
#define SPI_MODE_3 0x03 /* Setup (Falling) Sample (Rising) CPOL=1, CPHA=1 */

/* data direction */
#define SPI_LSB 1 /* send least significant bit (bit 0) first */
#define SPI_MSB 0 /* send most significant bit (bit 7) first */

/* whether to raise interrupt when data received (SPIF bit received) */
#define SPI_NO_INTERRUPT 0
#define SPI_INTERRUPT    1

/* slave or master with clock diviser */
#define SPI_SLAVE       0xF0
#define SPI_MSTR_CLK4   0x00 /* chip clock/4 */
#define SPI_MSTR_CLK16  0x01 /* chip clock/16 */
#define SPI_MSTR_CLK64  0x02 /* chip clock/64 */
#define SPI_MSTR_CLK128 0x03 /* chip clock/128 */
#define SPI_MSTR_CLK2   0x04 /* chip clock/2 */
#define SPI_MSTR_CLK8   0x05 /* chip clock/8 */
#define SPI_MSTR_CLK32  0x06 /* chip clock/32 */
// #define SPI_MSTR_CLK64 0x07 /* chip clock/64 */ --> already defined and therefore obsolete

/*
    sSpiConfig
 */
#define SPICONFIG_STATUS_INITIALIZATION_STATUS          BIT_7
#define BM_SPICONFIG_STATUS_INITIALIZATION_STATUS       BIT_MASK_7

/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/
typedef struct {
    /** \brief <b>bStatus</b>
        enables and disables available Init Functions.
        \li Bit 7:       SPI initialization status 0:NOT initialized / 1:initialized
        \li Bit 6:       rfu
        \li Bit 5:       rfu
        \li Bit 4:       rfu
        \li Bit 3:       rfu
        \li Bit 2:       rfu
        \li Bit 1:       rfu
        \li Bit 0:       rfu
    */
    uint8_t bStatus;

    /** \brief <b>pTmp</b>
        used as temporary registers for storing address information.
    */
    volatile uint8_t *pTmp;

    /** \brief <b>pAddress</b>
        used as address pointer for storing addresses of block read/write.
    */
    volatile uint8_t *pAddress;

    /** \brief <b>bLength</b>
        used as length indicator for buffer read/write commands.
    */
    volatile uint8_t bLength;

}sSpiConfig;
/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/
extern sSpiConfig g_sSpiConfig;
extern UINT8FUNC ATA_spiOpen_C(uint8_t, uint8_t);
extern VOIDFUNC ATA_spiClose_C(void);
extern UINT8FUNC ATA_spiDataExchange_C(uint8_t);
extern UINT8FUNC ATA_spiTransfer_C(uint8_t *, uint8_t *, uint8_t);

extern VOIDFUNC ATA_spiSelect_C(void);
extern VOIDFUNC ATA_spiDeselect_C(void);

extern VOIDFUNC ATA_spiRxBufferHandler_ISR_ASM(void);
extern VOIDFUNC ATA_spiRxBufferDisableInt_ASM(void);
extern VOIDFUNC ATA_spiRxBufferEnd_ASM(void);
extern VOIDFUNC ATA_spiTxBufferHandler_ISR_ASM(void);
extern VOIDFUNC ATA_spiTxBufferDisableInt_ASM(void);
extern VOIDFUNC ATA_spiTxBufferEnd_ASM(void);

#elif defined __IAR_SYSTEMS_ASM__

/* startDoxyExclude */
/* ----------------------------------------------------------------------------- */
/** \brief <b>ENABLE_SPIRXBUFFER</b>
    This macro enables the IRQ for the SPI Rx Fifo and sets the correct value for
    the Fifo Level. This macro also stores the address for the fucntion which shall
    be called after the fill level IRQ (extIf-Structure).

    \param address: Address of the function which is called after the fill level IRQ.
    num_bytes: Count of Bytes which is set to the value of the Fifo Level.

    \return none
*/
/* ----------------------------------------------------------------------------- */
ENABLE_SPIRXBUFFER MACRO    address, num_bytes
    ; SFIR = BM_SRIE | num_bytes -> generate interrupt if num_bytes bytes in rx fifo
    LDI  R30 , (BM_SRIE | num_bytes)
    STS  SFIR , R30

    LDI  R30 , low(address/2)
    STS  g_sSpiConfig + SPICONFIG_PTMP , R30

    LDI  R30 , high(address/2)
    STS  g_sSpiConfig + SPICONFIG_PTMP + 1 , R30
    ENDM
/* stopDoxyExclude */

/* startDoxyExclude */
/* ----------------------------------------------------------------------------- */
/** \brief <b>ENABLE_SPITXBUFFER</b>
    This macro enables the IRQ for the SPI Tx Fifo and sets the correct value for
    the Fifo Level. This macro also stores the address for the fucntion which shall
    be called after the fill level IRQ (extIf-Structure).

    \param address: Address of the function which is called after the fill level IRQ.
    num_bytes: Count of Bytes which is set to the value of the Fifo Level.

    \return none
*/
/* ----------------------------------------------------------------------------- */
ENABLE_SPITXBUFFER MACRO    address, num_bytes
    ; SFIR = BM_STIE | num_bytes -> generate interrupt if num_bytes bytes in tx fifo
    LDI  R30 , (BM_STIE | num_bytes << 4)
    STS  SFIR , R30

    LDI  R30 , low(address/2)
    STS  g_sSpiConfig + SPICONFIG_PTMP , R30

    LDI  R30 , high(address/2)
    STS  g_sSpiConfig + SPICONFIG_PTMP + 1 , R30
    ENDM
/* stopDoxyExclude */


/* create alias for the different SPI chip pins - code assumes all on port B */
SPI_NSS_PIN  EQU PORTD1
SPI_MISO_PIN EQU PORTB5
SPI_MOSI_PIN EQU PORTB6
SPI_SCK_PIN  EQU PORTB7

BM_SPI_NSS_PIN  EQU BM_PORTD1
BM_SPI_MISO_PIN EQU BM_PORTB5
BM_SPI_MOSI_PIN EQU BM_PORTB6
BM_SPI_SCK_PIN  EQU BM_PORTB7

/* SPI clock modes */
SPI_MODE_0 EQU 0x00 /* Sample (Rising) Setup (Falling) CPOL=0, CPHA=0 */
SPI_MODE_1 EQU 0x01 /* Setup (Rising) Sample (Falling) CPOL=0, CPHA=1 */
SPI_MODE_2 EQU 0x02 /* Sample (Falling) Setup (Rising) CPOL=1, CPHA=0 */
SPI_MODE_3 EQU 0x03 /* Setup (Falling) Sample (Rising) CPOL=1, CPHA=1 */

/* data direction */
SPI_LSB EQU 1 /* send least significant bit (bit 0) first */
SPI_MSB EQU 0 /* send most significant bit (bit 7) first */

/* whether to raise interrupt when data received (SPIF bit received) */
SPI_NO_INTERRUPT EQU 0
SPI_INTERRUPT    EQU 1

/* slave or master with clock diviser */
SPI_SLAVE       EQU 0xF0
SPI_MSTR_CLK4   EQU 0x00 /* chip clock/4 */
SPI_MSTR_CLK16  EQU 0x01 /* chip clock/16 */
SPI_MSTR_CLK64  EQU 0x02 /* chip clock/64 */
SPI_MSTR_CLK128 EQU 0x03 /* chip clock/128 */
SPI_MSTR_CLK2   EQU 0x04 /* chip clock/2 */
SPI_MSTR_CLK8   EQU 0x05 /* chip clock/8 */
SPI_MSTR_CLK32  EQU 0x06 /* chip clock/32 */
// #define SPI_MSTR_CLK64 0x07 /* chip clock/64 */ --> already defined and therefore obsolete

/* sIFData */
SPICONFIG_STATUS                EQU     0x00
SPICONFIG_PTMP                  EQU     SPICONFIG_STATUS + 1
SPICONFIG_PADDRESS              EQU     SPICONFIG_PTMP + 2
SPICONFIG_LENGTH                EQU     SPICONFIG_PADDRESS + 2

TX_BUFFER_FIFO_SIZE             EQU     0x04
RX_BUFFER_FIFO_SIZE             EQU     TX_BUFFER_FIFO_SIZE

#endif

#endif /* SPI_H */
