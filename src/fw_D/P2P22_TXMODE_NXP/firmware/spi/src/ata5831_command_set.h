//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/spi/src/ata5831_command_set.h $
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
/** \file ata5831_command_set.h
 */
//lint -restore

#ifndef ATA5831_COMMAND_SET_H
#define ATA5831_COMMAND_SET_H

#ifdef __IAR_SYSTEMS_ICC__
/*---------------------------------------------------------------------------*/
/*  INCLUDES                                                                 */
/*---------------------------------------------------------------------------*/
#include "../../stdc/src/stdc.h"
#include "../../globals/src/globals.h"
/*---------------------------------------------------------------------------*/
/*  DEFINES                                                                  */
/*---------------------------------------------------------------------------*/
#define ATA5831_ID_READ_FILL_LEVEL_RX_FIFO             (0x01U)
#define ATA5831_ID_READ_FILL_LEVEL_TX_FIFO             (0x02U)
#define ATA5831_ID_READ_FILL_LEVEL_RSSI_FIFO           (0x03U)
#define ATA5831_ID_GET_EVENT_BYTES                     (0x04U)
#define ATA5831_ID_READ_RSSI_FIFO                      (0x05U)
#define ATA5831_ID_READ_RX_FIFO                        (0x06U)
#define ATA5831_ID_WRITE_REGISTER_SRAM                 (0x07U)
#define ATA5831_ID_READ_REGISTER_SRAM                  (0x08U)
#define ATA5831_ID_WRITE_EEPROM                        (0x09U)
#define ATA5831_ID_READ_EEPROM                         (0x0AU)
#define ATA5831_ID_WRITE_TX_FIFO                       (0x0BU)
#define ATA5831_ID_WRITE_TX_PREAMBLE_FIFO              (0x0CU)
#define ATA5831_ID_SET_SYSTEM_MODE                     (0x0DU)
#define ATA5831_ID_CALIBRATE_AND_CHECK                 (0x0EU)
#define ATA5831_ID_PATCH_SPI                           (0x0FU)

#define ATA5831_ID_GET_ROM_VERSION                     (0x12U)
#define ATA5831_ID_GET_FLASH_VERSION                   (0x13U)
#define ATA5831_ID_CUSTOM_COMMAND                      (0x14U)
#define ATA5831_ID_SYSTEM_RESET                        (0x15U)
#define ATA5831_ID_TRIGGER_EEP_SECURE_WRITE            (0x16U)
#define ATA5831_ID_SET_VOLTAGE_MONITOR                 (0x17U)
#define ATA5831_ID_OFF_COMMAND                         (0x18U)
#define ATA5831_ID_READ_TEMPERATURE                    (0x19U)
#define ATA5831_ID_INIT_SRAM_SERVICE                   (0x1AU)
#define ATA5831_ID_START_RSSI_MEAS                     (0x1BU)
#define ATA5831_ID_GET_RSSI_VALUE                      (0x1CU)

#define SATA5831_CONFIG_READ_WRITE_INDICATOR    BIT_7
#define SATA5831_CONFIG_LENGTH_INDICATOR        BIT_6
#define SATA5831_CONFIG_ADDRESS_INDICATOR       BIT_5
#define SATA5831_CONFIG_DUMMY_INDICATOR         BIT_4
#define SATA5831_CONFIG_EVENT_BYTES_READ_FLAG   BIT_3

#define SATA5831_CONFIG_TELEGRAM_MODE_FLAG    BIT_0

#define BM_SATA5831_CONFIG_READ_WRITE_INDICATOR     BIT_MASK_7
#define BM_SATA5831_CONFIG_LENGTH_INDICATOR         BIT_MASK_6
#define BM_SATA5831_CONFIG_ADDRESS_INDICATOR        BIT_MASK_5
#define BM_SATA5831_CONFIG_DUMMY_INDICATOR          BIT_MASK_4
#define BM_SATA5831_CONFIG_EVENT_BYTES_READ_FLAG    BIT_MASK_3

#define BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG      BIT_MASK_0

#define SATA5831_STATUS_ERROR_FLAG                  BIT_7
#define BM_SATA5831_STATUS_ERROR_FLAG               BIT_MASK_7


/*---------------------------------------------------------------------------*/
/*  TYPE DEFINITIONS                                                         */
/*---------------------------------------------------------------------------*/
typedef struct{
    /** \brief <b>dataPtr</b>
        is used as pointer for data transmission and/or reception
    */
    uint8_t *pDataPtr;
    
    /** \brief <b>bNssLowSckDelay</b>
        is used as delay counter from NSS low to first SCK
    */
    uint8_t bNssLowSckDelay;
    /** \brief <b>bSckNssHighDelay</b>
        is used as delay counter from last SCK to NSS high
    */
    uint8_t bSckNssHighDelay;

    /** \brief <b>bEventsSystem</b>
        contains the event flags with ATA5831 system information
        \li Bit 7:    SYS_ERR
        \li Bit 6:    CMD_RDY
        \li Bit 5:    SYS_RDY
        \li Bit 4:    AVCCLOW
        \li Bit 3:    LOWBATT
        \li Bit 2:    RSSI_BUF
        \li Bit 1:    RX_BUF
        \li Bit 0:    TX_BUF
    */
    uint8_t bEventsSystem;

    /** \brief <b>bEventsEvent</b>
        contains the event flags with ATA5831 RX/TX/RSSI information
        \li Bit 7:    ID_CHECK
        \li Bit 6:    WCOA
        \li Bit 5:    SOTA
        \li Bit 4:    EOTA
        \li Bit 3:    ID_CHECKB
        \li Bit 2:    WCOB
        \li Bit 1:    SOTB
        \li Bit 0:    EOTB
    */
    uint8_t bEventsEvent;

    /** \brief <b>bEventsPower</b>
        contains the event flags with powerOn nPowerOn information
        \li Bit 7: PWRON
        \li Bit 6: NPWRON6
        \li Bit 5: NPWRON5
        \li Bit 4: NPWRON4
        \li Bit 3: NPWRON3
        \li Bit 2: NPWRON2
        \li Bit 1: NPWRON1
        \li Bit 0: NPWRON0
     */
    uint8_t bEventsPower;

    /** \brief <b>bEventsConfig</b>
        holds the current selected service channel configuration
     */
    uint8_t bEventsConfig;

    /** \brief <b>bConfig</b>
        \li Bit 7:    read/write command indicator
                        0: read command
                        1: write command
        \li Bit 6:    length field valid indicator
        \li Bit 5:    address field valid indicator
        \li Bit 4:    dummy bytes inserted
        \li Bit 3:    event bytes read indicator
        \li Bit 2:    rfu
        \li Bit 1:    rfu
        \li Bit 0:    telegram mode
                        0: checksum mode
                        1: normal mode (interrupt usage SPIRX/SPITX fifo interrupt)
    */
    uint8_t     bConfig;
    /** \brief <b>bId</b>
        holds the id parameter of current command
     */
    uint8_t     bId;
    /** \brief <b>wAddress</b>
        holds the address information of current command
     */
    uint16_t    wAddress;
    /** \brief <b>bLength</b>
        holds the length information of current command
     */
    uint8_t     bLength;
    /** \brief <b>bRxLength</b>
        holds the number of bytes which have to be received before command completion.
        This variable is decremented at each byte reception in SPI Rx/Tx FIFO ISR
     */
    uint8_t     bRxLength;
    /** \brief <b>bTelegramLength</b>
        holds the number of bytes which have to be received before command completion
        This variable is decremented at each byte transmission
     */
    uint8_t     bTelegramLength;

    /** \brief <b>bStatus</b>
        contains the status of module ATA5831
        bit 7:      ATA5831 Module error flag
        bit 6..0:   rfu
     */
    uint8_t     bStatus;
    /** \brief <b>bChecksumMiso</b>
        contains the checksum on MISO for feature secure SPI
     */
    uint8_t     bChecksumMiso;
    /** \brief <b>bChecksumMosi</b>
        contains the checksum on MOSI for feature secure SPI
     */
    uint8_t     bChecksumMosi;
}sAta5831CommandSet;

/** \brief <b>ata5831CommandHandler</b>
    is used for function pointer definition of ATA5831 command set
*/
typedef void (*ata5831CommandHandler)(void);

/*---------------------------------------------------------------------------*/
/*  EXTERNAL PROTOTYPES                                                      */
/*---------------------------------------------------------------------------*/

/* ATA5831_ID_READ_FILL_LEVEL_RX_FIFO 0x01 */
extern VOIDFUNC ATA_5831ReadFillLevelRxFifo_C(uint8_t *);

/* ATA5831_ID_READ_FILL_LEVEL_TX_FIFO 0x02 */
extern VOIDFUNC ATA_5831ReadFillLevelTxFifo_C(uint8_t *);

/* ATA5831_ID_READ_FILL_LEVEL_RSSI_FIFO 0x03 */
extern VOIDFUNC ATA_5831ReadFillLevelRssiFifo_C(uint8_t *);

/* ATA5831_ID_GET_EVENT_BYTES 0x04*/
extern VOIDFUNC  ATA_5831GetEventBytes_C(void);

/* ATA5831_ID_READ_RSSI_FIFO 0x05 */
extern VOIDFUNC  ATA_5831ReadRssiFifo_C(uint8_t, uint8_t *);

/* ATA5831_ID_READ_RX_FIFO 0x06 */
extern VOIDFUNC ATA_5831ReadRxFifo_C(uint8_t, uint8_t *);

/* ATA5831_ID_WRITE_REGISTER_SRAM 0x07 */
extern VOIDFUNC ATA_5831WriteSramRegister_C(uint16_t, uint8_t, uint8_t *);

/* ATA5831_ID_READ_REGISTER_SRAM 0x08 */
extern VOIDFUNC ATA_5831ReadSramRegister_C(uint16_t, uint8_t, uint8_t *);

/* ATA5831_ID_WRITE_EEPROM 0x09 */
extern VOIDFUNC ATA_5831WriteEeprom_C(uint16_t, uint8_t);

/* ATA5831_ID_READ_EEPROM 0x0A */
extern VOIDFUNC ATA_5831ReadEeprom_C(uint16_t, uint8_t *);

/* ATA5831_ID_WRITE_TX_FIFO 0x0B */
extern VOIDFUNC ATA_5831WriteTxFifo_C(uint8_t, uint8_t *);

/* ATA5831_ID_WRITE_TX_PREAMBLE_FIFO 0x0C */
extern VOIDFUNC ATA_5831WriteTxPreambleFifo_C(uint8_t, uint8_t *);

/* ATA5831_ID_SET_SYSTEM_MODE 0x0D */
extern VOIDFUNC ATA_5831SetSystemMode_C(sysModeConfig_t, svcChConfig_t);

/* ATA5831_ID_CALIBRATE_AND_CHECK 0x0E */
extern VOIDFUNC ATA_5831CalibrateAndCheck_C(tuneCheckConfig_t, svcChConfig_t);

/* ATA5831_ID_PATCH_SPI 0x0F */
extern VOIDFUNC ATA_5831PatchSpi_C(uint8_t);

/* ATA5831_ID_GET_ROM_VERSION 0x12 */
extern VOIDFUNC ATA_5831GetRomVersion_C(uint8_t *);

/* ATA5831_ID_GET_FLASH_VERSION 0x13 */
extern VOIDFUNC ATA_5831GetFlashVersion_C(uint8_t *);

/* ATA5831_ID_CUSTOM_COMMAND 0x14 */
extern VOIDFUNC ATA_5831CustomerConfigurableCommand_C(void);

/* ATA5831_ID_SYSTEM_RESET 0x15 */
extern VOIDFUNC ATA_5831SystemReset_C(void);

/* ATA5831_ID_TRIGGER_EEP_SECURE_WRITE 0x16 */
extern VOIDFUNC ATA_5831TriggerEepromSecureWrite_C(void);

/* ATA5831_ID_SET_VOLTAGE_MONITOR 0x17 */
extern VOIDFUNC ATA_5831SetVoltageMonitor_C(uint8_t);

/* ATA5831_ID_OFF_COMMAND 0x18 */
extern VOIDFUNC ATA_5831OffCommand_C(void);

/* ATA5831_ID_READ_TEMPERATURE 0x19 */
extern VOIDFUNC ATA_5831ReadTemperatureValue_C(uint8_t *);

/* ATA5831_ID_INIT_SRAM_SERVICE 0x1A */
extern VOIDFUNC ATA_5831InitSramService_C(uint8_t, uint8_t);

/* ATA5831_ID_START_RSSI_MEAS 0x1B */
extern VOIDFUNC ATA_5831StartRssiMeasurement_C(svcChConfig_t);

/* ATA5831_ID_GET_RSSI_VALUE 0x1C */
extern VOIDFUNC ATA_5831GetRssiValue_C(uint8_t *);

extern VOIDFUNC ATA_5831Init_C(uint8_t, uint8_t, uint8_t);

extern VOIDFUNC ATA_5831Close_C(void);

extern VOIDFUNC ATA_5831Start_C(void);

extern VOIDFUNC ATA_5831StartSckNssTimer_ASM(void);
extern VOIDFUNC ATA_5831CommandReady_ASM(void);
extern VOIDFUNC ATA_5831CommandTimer1Isr_ASM(void);
extern VOIDFUNC ATA_5831ReadCommandFillTxFifo_ASM(void);
extern VOIDFUNC ATA_5831WriteCommandFillTxFifo_ASM(void);
extern VOIDFUNC ATA_5831CommandSpiFifoIsr_ASM(void);
extern VOIDFUNC ATA_5831ReadCommandReadRxFifo_ASM(void);
extern VOIDFUNC ATA_5831ReadCommandReadDummyRxFifo_ASM(void);
extern VOIDFUNC ATA_5831WriteCommandReadRxFifo_ASM(void);

extern sAta5831CommandSet g_sAta5831;
extern uint8_t g_bAta5831Buffer[3];

#elif defined __IAR_SYSTEMS_ASM__
EXTERN  g_sAta5831

/*startSimExtraction*/

/* sAta5831 */
SATA5831_DATAPTR            EQU     0x00
SATA5831_NSS_LOW_SCK_DELAY  EQU     SATA5831_DATAPTR + 2
SATA5831_SCK_NSS_HIGH_DELAY EQU     SATA5831_NSS_LOW_SCK_DELAY + 1
SATA5831_EVENTS_SYSTEM      EQU     SATA5831_SCK_NSS_HIGH_DELAY + 1
SATA5831_EVENTS_EVENT       EQU     SATA5831_EVENTS_SYSTEM + 1
SATA5831_EVENTS_POWER       EQU     SATA5831_EVENTS_EVENT + 1
SATA5831_EVENTS_CONFIG      EQU     SATA5831_EVENTS_POWER + 1
SATA5831_CONFIG             EQU     SATA5831_EVENTS_CONFIG + 1
SATA5831_ID                 EQU     SATA5831_CONFIG + 1
SATA5831_ADDRESS            EQU     SATA5831_ID + 1
SATA5831_LENGTH             EQU     SATA5831_ADDRESS + 2
SATA5831_RX_LENGTH          EQU     SATA5831_LENGTH + 1
SATA5831_TELEGRAM_LENGTH    EQU     SATA5831_RX_LENGTH + 1
SATA5831_STATUS             EQU     SATA5831_TELEGRAM_LENGTH + 1
SATA5831_CHECKSUM_MISO      EQU     SATA5831_STATUS + 1
SATA5831_CHECKSUM_MOSI      EQU     SATA5831_CHECKSUM_MISO + 1

/* sAta5831.config */
SATA5831_CONFIG_READ_WRITE_INDICATOR    EQU     BIT_7
SATA5831_CONFIG_LENGTH_INDICATOR        EQU     BIT_6
SATA5831_CONFIG_ADDRESS_INDICATOR       EQU     BIT_5
SATA5831_CONFIG_DUMMY_INDICATOR         EQU     BIT_4
SATA5831_CONFIG_EVENT_BYTES_READ_FLAG   EQU     BIT_3

SATA5831_CONFIG_TELEGRAM_MODE_FLAG    EQU     BIT_0

BM_SATA5831_CONFIG_READ_WRITE_INDICATOR     EQU     BIT_MASK_7
BM_SATA5831_CONFIG_LENGTH_INDICATOR         EQU     BIT_MASK_6
BM_SATA5831_CONFIG_ADDRESS_INDICATOR        EQU     BIT_MASK_5
BM_SATA5831_CONFIG_DUMMY_INDICATOR          EQU     BIT_MASK_4
BM_SATA5831_CONFIG_EVENT_BYTES_READ_FLAG    EQU     BIT_MASK_3

BM_SATA5831_CONFIG_TELEGRAM_MODE_FLAG      EQU     BIT_MASK_0

/* sAta5831.config */
SATA5831_STATUS_ERROR_FLAG                  EQU     BIT_7
BM_SATA5831_STATUS_ERROR_FLAG               EQU     BIT_MASK_7


/* ATA5831 Command IDs */
ATA5831_ID_READ_FILL_LEVEL_RX_FIFO             EQU 0x01
ATA5831_ID_READ_FILL_LEVEL_TX_FIFO             EQU 0x02
ATA5831_ID_READ_FILL_LEVEL_RSSI_FIFO           EQU 0x03
ATA5831_ID_GET_EVENT_BYTES                     EQU 0x04
ATA5831_ID_READ_RSSI_FIFO                      EQU 0x05
ATA5831_ID_READ_RX_FIFO                        EQU 0x06
ATA5831_ID_WRITE_REGISTER_SRAM                 EQU 0x07
ATA5831_ID_READ_REGISTER_SRAM                  EQU 0x08
ATA5831_ID_WRITE_EEPROM                        EQU 0x09
ATA5831_ID_READ_EEPROM                         EQU 0x0A
ATA5831_ID_WRITE_TX_FIFO                       EQU 0x0B
ATA5831_ID_WRITE_TX_PREAMBLE_FIFO              EQU 0x0C
ATA5831_ID_SET_SYSTEM_MODE                     EQU 0x0D
ATA5831_ID_CALIBRATE_AND_CHECK                 EQU 0x0E
ATA5831_ID_PATCH_SPI                           EQU 0x0F
// ATA5831_ID_SYSTEM_RESET_ROM                    EQU 0x10
// ATA5831_ID_SYSTEM_RESET_FLASH                  EQU 0x11
ATA5831_ID_GET_ROM_VERSION                     EQU 0x12
ATA5831_ID_GET_FLASH_VERSION                   EQU 0x13
ATA5831_ID_CUSTOM_COMMAND                      EQU 0x14
ATA5831_ID_SYSTEM_RESET                        EQU 0x15
ATA5831_ID_TRIGGER_EEP_SECURE_WRITE            EQU 0x16
ATA5831_ID_SET_VOLTAGE_MONITOR                 EQU 0x17
ATA5831_ID_OFF_COMMAND                         EQU 0x18
ATA5831_ID_READ_TEMPERATURE                    EQU 0x19
ATA5831_ID_INIT_SRAM_SERVICE                   EQU 0x1A
ATA5831_ID_START_RSSI_MEAS                     EQU 0x1B
ATA5831_ID_GET_RSSI_VALUE                      EQU 0x1C

/*stopSimExtraction*/

#endif
#endif /* ATA5831_COMMAND_SET_H */
