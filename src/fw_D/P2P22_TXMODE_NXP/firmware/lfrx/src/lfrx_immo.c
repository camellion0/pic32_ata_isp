/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/lfrx/src/lfrx_immo.c $
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
/** \file lfrx_immo.c
*/
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "lfrx_immo.h"
#include "../../lfrx/src/lfrx_flash.h"
#include "../../rftx/src/rftx.h"
#include "../../spi/src/ata5700_command_set.h"
#include "../../eep/src/eep.h"
#include "../../aes/src/aes.h"
#include "../../globals/src/globals_flash.h"
#include "../../lfrssi/src/lfrssi_flash.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
__no_init static uint8_t m_bTempArray[40];
__no_init static uint8_t m_bChallengeLengthBits_flash;
__no_init static uint8_t m_bResponseLengthBits_flash;


/*===========================================================================*/
/*  LOCAL PROTOTYPES                                                         */
/*===========================================================================*/
static void ATA_lfRxInitImmo_flash_C(void);
static uint8_t ATA_lfRxParseCmdImmo_flash_C(void);
static void ATA_lfRxAuthenticateImmo_flash_C(void);

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxRfTxProcessImmo_flash_C</b>
    does the process of one command reception via LF, command processing via AOI
    protocol and response via RF

    010:

    020:

    030:


    \param none

    \return none

    \Derived no

    \Rationale none

    \Traceability none

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
//-----------------------------------------------------------------------------
void ATA_lfRxRfTxProcessImmo_flash_C(void)
//-----------------------------------------------------------------------------
{
  uint8_t bRftxConfig = 0x48;

  /* read the configuration byte present in EEPROM */
  ATA_eepReadMultipleBytes_C(0x0815, 0x01, &g_uTpConfig.bConfigbyte);

  /* initialize for LF reception */
  ATA_lfRxInitImmo_flash_C();
  /* configure sleep mode power down */
  SMCR = 0x05;

  /* sleep till complete frame is received */
  while(TRUE) {
    __disable_interrupt();
    if(g_bReceiveState > 2) {
      break;
    }
    __enable_interrupt();
    __sleep();
  }

  g_bReceiveState = 0;
  __enable_interrupt();

  g_bBytesToTransmit = ATA_lfRxParseCmdImmo_flash_C();

  if(g_bBytesToTransmit > 0) {

    /* check if LF RSSI measurement has been done */
    if( extReq.lfrxRftxConfig & 0x06 )
    {
        ATA_lfRssiMeasConfig_flash_C( extReq.lfrxRftxConfig & 0x06 );
        bRftxConfig |= 0x04;  /* stay in TX after transmission */
    }

    /* start RF TX statemachine */
    ATA_rfTxStartTx_C(bRftxConfig, (uint8_t *) &g_sEepRfTxServiceConfig1);
  }
  /* delete external request flag */
  extReq.lfrxRftxConfig &= ~BIT_MASK_0;
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxInitImmo_flash_C</b>
    shall configure the transponder for AOIP command reception

    010:

    020:

    030:


    \param none

    \return none

    \Derived no

    \Rationale none

    \Traceability Primus2P-916, Primus2P-917, Primus2P-1226

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
//-----------------------------------------------------------------------------
static void ATA_lfRxInitImmo_flash_C(void)
//-----------------------------------------------------------------------------
{
  /* LLR-Ref: Disable power reduction to serial peripheral interface. */
  PRR0 &= ~BM_PRCO;
  /* LLR-Ref: Disable power reduction to LF receiver. */
  PRR1 &= ~BM_PRLFR;

  /* LF synchronization stop bit */
  LFSTOP = 0x4F; // symbols length = 4, symbols = b1111

  /* LF calibration protect */
  m_bTempArray[0] = 0x03;
  LFCPR = BM_LFCPCE;    // change enable
  LFCPR = m_bTempArray[0]; // calibration ready, calibration protect

  /* LLR-Ref: Switch FIFO and CRC clock to AVR clock to do configuration. */
  ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x01, (uint8_t)0x01);

  /* LLR-Ref: CRC settings. */
  PHCRCR = 0x00;
  PHCSTH = 0x00;
  PHCSTL = 0x00;
  PHCRPH = 0x00;
  PHCRPL = 0x02;
  PHCRCR = 0xC8;      /* CRC enable, strobe enable, CRC4, MSB, ID frame excluded */

  /* LLR-Ref: FIFO settings. */
  PHFC = 0x00;        /* DRA = 0: AVR read access to FIFO, set fill level configuration to 0 */
  PHFL = BM_PHCLR;    /* clear RdPtr, WrPtr, fill-level, status and bit length */
  PHFR = 0x03;        /* clear bit length and CRC error flags */

  /* LLR-Ref: Switch clocks back to protocol handler clock. */
  ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x00, (uint8_t)0x00);

  /* LLR-Ref: Interrupt settings. */
  LFIMR = 0x00;       /* disable LF Interrupts */
  PHIMR = BM_PHDFIM;  /* enable data frame interrupt (LFBCR_vect) */
  PHFI  = 0x00;       /* disable FIFO interrupts */

  /* LLR-Ref: Clear the reception buffer index for LF reception. */
  g_bRxBufferIndex = 0x00;

  /* LLR-Ref: Update the number of bits to be received. */
  g_bBitsToReceive = 0x08;

  /* LLR-Ref: Initialize receive state */
  g_bReceiveState = 0;

  /* LLR-Ref: Enable response delay timer. */
  PHTCOR = 58;        /* 62 * 32ms = 1984ms */
  PHTEMR = BM_TBLEM;  /* bit length reached triggers PH-timer event */
  PHTCMR = 0x7A;      /* start timer on event, reset counter, enable
                         interrupt, compare reset, prescaler value = 4 */
  g_bResponseDelayReached = FALSE; /* Reset response delay flag. */
  PHTCMR |= BM_PHTE;

  /* LLR-Ref: Start reception. */
  LFCR1 |= BM_LFRE;
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxReinitImmo_flash_C</b>
    Function is called to reconfigure the LF receiver while receiving data.
    The reconfiguration is needed for commands which includes additional data
    after the command byte.

    010: Parse commands for reconfiguration.

    020: Reconfiguration of CRC (no clock switch has to be done cause data is
         stored in shadow registers).


    \param none

    \return none

    \Derived no

    \Rationale none

    \Traceability Primus2P-1222

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
//-----------------------------------------------------------------------------
void ATA_lfRxReinitImmo_flash_C(uint8_t bCmd4Bit)
//-----------------------------------------------------------------------------
{
  /* LLR-Ref: 010 */
  if(bCmd4Bit == START_AUTH) {
    /* read out challenge and response length from EEPROM */
    ATA_eepReadMultipleBytes_C(0x0819, 0x02, &m_bTempArray[0]);
    m_bChallengeLengthBits_flash = m_bTempArray[0];
    m_bResponseLengthBits_flash = m_bTempArray[1];

    if(g_uTpConfig.sConfigbits.CM == UNILATERAL) {
      g_bBitsToReceive = m_bChallengeLengthBits_flash;
    }
    else {
      g_bBitsToReceive = m_bChallengeLengthBits_flash + m_bResponseLengthBits_flash;
    }
  }

  PHID0L = 0;               /* disable ID0 */
  PHID1L = 0;               /* disable ID1 */
  PHIDFR = 0;               /* disable ID frame length */
  PHCKCR |= BM_CSM;         /* enable continue receive mode to transfer shadow register after telegram bit length reached */
  PHDFR = g_bBitsToReceive; /* data frame length */
  PHIMR = BM_PHTBLIM;       /* enable telegram bit length reached interrupt (LFBCR_vect) */

  /* LLR-Ref: 020 */
  if(g_uTpConfig.sConfigbits.DCD == 0) {  /* CRC enabled */
    PHCRPH = 0;                   /* CRC polynomial */
    PHCRPL = 0xCA;
    PHCSTH = 0;
    PHCSTL = 0x00;
    PHCRCR = 0xD8;                 /* CRC enable, strobe enable, CRC8, MSB, ID frame excluded */
    PHTBLR = g_bBitsToReceive + 8; /* telegram bit length (+8 bit CRC) */
  }
  else {                                /* CRC disabled */
    PHTBLR = g_bBitsToReceive;     /* telegram bit length */
    PHCRCR = 0;                    /* disable CRC */
  }
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxParseCmdImmo_flash_C</b>
    Function checks received command and prepares data for transmission.

    010:

    020:

    030:


    \param none

    \return [out] bBytesToTransmit  number of bytes to be send out

    \Derived no

    \Rationale none

    \Traceability Primus2P-922, Primus2P-924, Primus2P-925, Primus2P-927,
                  Primus2P-929, Primus2P-930, Primus2P-932, Primus2P-933,
                  Primus2P-934, Primus2P-936, Primus2P-938, Primus2P-939,
                  Primus2P-942, Primus2P-943, Primus2P-946, Primus2P-950,
                  Primus2P-952, Primus2P-956, Primus2P-957

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
//-----------------------------------------------------------------------------
static uint8_t ATA_lfRxParseCmdImmo_flash_C(void)
//-----------------------------------------------------------------------------
{
  /* variable to hold the command without CRC4 */
  uint8_t bCmd4Bit;
  /* variable to hold the byte count to be transmitted */
  uint8_t bBytesToTransmit = 1;

  /* extract the commmand from the received frame */
  bCmd4Bit = g_bRxBuffer[0];

  if(g_bTpStatusByte == STATUS_SUCCESS) {
    g_bTxBuffer[0] = 0xFE;

    /* if the command is to read the UID */
    if(bCmd4Bit == READ_UID) {
      /* load the transmission buffer with UID present in EEPROM */
      ATA_eepReadMultipleBytes_C(0x08B0, 0x04, &g_bTxBuffer[1]);
      /* update the number of bytes to be transmitted */
      bBytesToTransmit = 5; // UID_SIZE + 1;
    }

    /* if the command is to start the authentication process */
    else if(bCmd4Bit == START_AUTH) {

      /* call the authentication processing function */
      ATA_lfRxAuthenticateImmo_flash_C();

      /* update the number of bytes to be transmitted */
      bBytesToTransmit = (m_bResponseLengthBits_flash >> 3) + 1;
    }

    else {
      /* any other cases nothing to be transmitted */
      g_bTpStatusByte = STATUS_COMMAND_NOT_SUPPORTED;
    }
  }

  if(g_bTpStatusByte != STATUS_SUCCESS) {
    bBytesToTransmit = 0;
  }

  /* update the status byte with the command */
  g_bTpStatusByte = ((bCmd4Bit << 4) | MASK_HIGH_NIBBLE) & g_bTpStatusByte;

  if(   (bCmd4Bit > TP_ERR_STS)
     && (bCmd4Bit < RPT_LST_RES)) {
    /* update the transmission buffer with existing status byte */
    g_bTxBuffer[0] = g_bTpStatusByte;
  }

  return bBytesToTransmit;
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxAuthenticateImmo_flash_C</b>
    Function handles the authentication process using the EEPROM data and the
    AES block whether unilateral or bilateral authentication is configured.

    010:

    020:

    030:


    \param none

    \return none

    \Derived no

    \Rationale none

    \Traceability Primus2P-924, Primus2P-925, Primus2P-956, Primus2P-1372,
                  Primus2P-1406

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
//-----------------------------------------------------------------------------
static void ATA_lfRxAuthenticateImmo_flash_C(void)
//-----------------------------------------------------------------------------
{
    /* local value for calculation */
    uint8_t bIndex, i;
    /* value to hold the number of the actual secret key */
    uint8_t bKeySelect = 0;
    /* value to hold the number of challenge bytes */
    uint8_t bChallengeLengthBytes;
    /* value to hold the number of response bytes */
    uint8_t bResponseLengthBytes;

    /* check configuration which key schould be used first */
    if(g_uTpConfig.sConfigbits.KS == 1) {
        bKeySelect = 1;
    }

    ATA_aesSetConfig_C(bKeySelect, 0x02);

    bChallengeLengthBytes = m_bChallengeLengthBits_flash >> 3;
    for(bIndex = 0; bIndex < bChallengeLengthBytes; bIndex++) {
        g_sAesComponentData.bDataBuffer[bIndex] = g_bRxBuffer[1 + bIndex];
    }
    /* Padding done manually cause different to ROM library */
    g_sAesComponentData.bDataLength = 128;
    m_bTempArray[ 0] = 0x12;
    m_bTempArray[ 1] = 0x34;
    m_bTempArray[ 2] = 0x56;
    m_bTempArray[ 3] = 0x78;
    m_bTempArray[ 4] = 0x00;
    m_bTempArray[ 5] = 0x00;
    m_bTempArray[ 6] = 0x00;
    m_bTempArray[ 7] = 0x00;
    m_bTempArray[ 8] = 0x00;
    m_bTempArray[ 9] = 0x00;
    m_bTempArray[10] = 0x00;
    m_bTempArray[11] = 0x00;
    for(i = 0; bIndex < 16; bIndex++, i++) {
        g_sAesComponentData.bDataBuffer[bIndex] = m_bTempArray[i];
    }

    ATA_aesEncryptData_C();

    bResponseLengthBytes = m_bResponseLengthBits_flash >> 3;

    if(g_uTpConfig.sConfigbits.CM == BILATERAL) {
        /* check if encrypted challenge correlate with received MAC */
        i = bChallengeLengthBytes + 1;
        for(bIndex = 0; bIndex < bResponseLengthBytes; bIndex++, i++) {
            if(g_bRxBuffer[i] != g_sAesComponentData.bDataBuffer[bIndex]) {
            g_bTpStatusByte = STATUS_BILATERAL_AUTHEN_FAILED;
            }
        }
        if(g_bTpStatusByte == STATUS_SUCCESS) {
            /* check configuration which key schould be used second */
            if(g_uTpConfig.sConfigbits.KS == 1) {
                bKeySelect = 0;
            }
            else {
                bKeySelect = 1;
            }
            ATA_aesSetConfig_C(bKeySelect, 0x02);

            ATA_aesEncryptData_C();
        }
    }

    /* copy encrypted data to TX buffer */
    for(bIndex = 0; bIndex < bResponseLengthBytes; bIndex++) {
        g_bTxBuffer[1 + bIndex] = g_sAesComponentData.bDataBuffer[bIndex];
    }
}

//-----------------------------------------------------------------------------
/** \brief <b>ATA_lfRxBitCntReachedImmo_flash_C</b>
    This function is serviced in after/while reception if the defined bit count
    is reached to reconfigure or stop the ongoing reception.

    010:

    020:

    030:


    \param none

    \return none

    \Derived no

    \Rationale none

    \Traceability none

    \StackUsage SU_XXX bytes

    \image none
    \n
*/
//-----------------------------------------------------------------------------
void ATA_lfRxBitCntReachedImmo_flash_C(void)
//-----------------------------------------------------------------------------
{
  /* variable to hold the command without CRC4 */
  uint8_t bCmd4Bit;
  /* variable to hold count of bytes to be received */
  uint8_t bBytesToReceive;

  /* LLR-Ref: 040 */
  g_bReceiveState++;

  /* LLR-Ref: 045 */
  PHFR = (BM_PHTBLF | BM_PHDFF);

  /* LLR-Ref: 050 */
  if(g_bReceiveState > 1) {
    PHCKCR &= ~BM_CSM;                   /* disable continue receive mode */
    if(TPFR & BM_TPBERF) {               /* TP bit error */
      g_bTpStatusByte = STATUS_BIT_ERROR;
      TPFR = BM_TPBERF;
    }
    else if(PHFR & BM_CRCEF) {           /* PH CRC error */
      g_bTpStatusByte = STATUS_CRC_INCORRECT;
      PHFR = BM_CRCEF;
    }
    else {                               /* no error */
      g_bTpStatusByte = STATUS_SUCCESS;
    }
  }

  /* LLR-Ref: 060 */
  if(   (g_bReceiveState == 1)
     || (g_bReceiveState == 3))
  {
    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x01, (uint8_t)0x01);

    bBytesToReceive = g_bBitsToReceive >> 3;
    while(bBytesToReceive > 0) {
      /* read data from protocol handler FIFO data register */
      g_bRxBuffer[g_bRxBufferIndex] = PHFD;
      g_bRxBufferIndex += 1;
      bBytesToReceive--;
    }

    PHFL = 0x80;            /* clear RdPtr, WrPtr, fill-level, status and bit length */

    ATA_globalsSwitchAvrPhClock_flash_C((uint8_t)0x00, (uint8_t)0x00);
  }

  /* LLR-Ref: 070 */
  bCmd4Bit = g_bRxBuffer[0];

  /* LLR-Ref: 080 */
  if(   (g_bReceiveState == 1)
     && (   (bCmd4Bit == START_AUTH)
         || ((bCmd4Bit >= READ_USR_MEM) && (bCmd4Bit <= LRN_SKT_K2)) ) ) {
    ATA_lfRxReinitImmo_flash_C(bCmd4Bit);
  }
  /* LLR-Ref: 090 */
  else if(g_bReceiveState == 1) {
    /* enable telegram bit length reached interrupt (LFBCR_vect) */
    PHIMR = BM_PHTBLIM;
    g_bReceiveState++;
  }
  /* LLR-Ref: 100 */
  else if(   (g_bReceiveState == 2)
          && (g_bTpStatusByte == STATUS_SUCCESS)) {
    if(bCmd4Bit == WRT_USR_MEM) {
      PHIMR = 0;          /* disable bit count reached interrupt (LFBCR_vect) */
      TPIMR = BM_TPFTIM;  /* enable transponder field timeout interrupt (TPTOERR_vect) */
    }
    if(PHTEMR & BM_TBLEM) {
      PHTCMR |= BM_PHRES;
      g_bResponseDelayReached = FALSE; /* Reset response delay flag. */
      PHTCMR |= BM_PHTE;
    }
  }
  /* LLR-Ref: 105 */
  else if(   (g_bReceiveState > 1)
          && (g_bTpStatusByte != STATUS_SUCCESS)) {
    /* comparator value for time out detection */
    TPDCR5 = 0x2D;
    /* disable protocol handler interrupts */
    PHIMR = 0;
    /* enable transponder field timeout interrupt (TPTOERR_vect) */
    TPIMR = BM_TPFTIM;
    /* set receive state to value '2' to hold sleep in main function till
       field timeout is detected */
    g_bReceiveState = 2;
  }
  /* LLR-Ref: 110 */
  else {
    /* LLR-Ref: 120 */
    PHIMR = 0;
    /* LLR-Ref: 130 */
    TPCR3 &= ~BM_TPRD;
    /* LLR-Ref: 140 */
    PHCRCR = 0x00;
  }
}
