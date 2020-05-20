/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/iic/src/iic.h $
  $LastChangedRevision: 458065 $
  $LastChangedDate: 2017-05-02 04:55:50 -0600 (Tue, 02 May 2017) $
  $LastChangedBy: krishna.balan $
-------------------------------------------------------------------------------
  Project:      ATA5700
  Target MCU:   ATA5700
  Compiler:     IAR C/C++ Compiler for AVR 6.3.18.2236
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

/** \file iic.h
*/

#ifndef IIC_H
#define IIC_H

#ifdef __IAR_SYSTEMS_ICC__
/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "../../stdc/src/stdc.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/** \brief <b>BM_IICLOCK</b>
    is the set-mask for iicStatus lock bit.
*/
#define BM_IIC2LOCK                (uint8_t)0x80

/** \brief <b>BM_IICTXDATAPENDING</b>
    is the set-mask for iicStatus.
*/
#define BM_IICTXDATAPENDING        (uint8_t)0x40

/** \brief <b>BM_IICTRXERROR</b>
    is the set-mask for iicStatus.
*/
#define BM_IICTRXERROR             (uint8_t)0x20

/** \brief <b>BM_IICRXDATAPEDING</b>
    is the set-mask for iicStatus.
*/
#define BM_IICRXDATAPEDING         (uint8_t)0x10

/** \brief <b>BM_IICTRANSACTIONONGOING</b>
    is the set-mask for iicStatus iic transaction on going indication.
*/
#define BM_IICTRANSACTIONONGOING (uint8_t)0x08

/** \brief <b>BM_CONFERRORIIC</b>
    is the set-mask for iicStatus lock bit.
*/
#define BM_CONFERRORIIC    (uint8_t)0x04

/** \brief <b>BM_IICTRXSTATE</b>
    is the set-mask for iicStatus.
*/
#define BM_IICTRXSTATE (uint8_t)0x03

/** \brief <b>IICMASTERTX</b>
    is the set-mask for iic transaction indication.
*/
#define IICMASTERTX (uint8_t)0x00
#define IICMASTERRX (uint8_t)0x01
#define IICSLAVETX (uint8_t)0x02
#define IICSLAVERX (uint8_t)0x03

/** \brief <b>BM_PTRVALID</b>
    is the (set)mask for iicConfig.
*/
#define BM_PTRVALID    (uint8_t)0x80

/** \brief <b>BM_IICIRQ</b>
    is the (set)mask for BM_IICIRQ.
*/
#define BM_IICIRQ    (uint8_t)0x40

/** \brief <b>BM_IICTRX</b>
    is the (set)mask for BM_IICTRX.
*/
#define BM_IICTRX    (uint8_t)0x01


/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
typedef struct
{
    /** \brief <b>iicAddr</b>
        contains the address of the IIC peripheral. Direct copied to the I2AR
        Register.
    */
    uint8_t iicAddr;
    /** \brief <b>iicAddrMsk</b>
        contains the address of the IIC peripheral. Direct copied to the I2AMR
        Register.
    */
    uint8_t iicAddrMsk;    
    /** \brief <b>iicBaudRate</b>
        contains the baudrate configuration in case the IIC is used as master.
        Direct copied to the I2BR Register.
    */    
    uint8_t iicBaudRate;
    /** \brief <b>iicPrescaler</b>
        contains the 2 prescaler 
        \li Bit 7..2: rfu        
        \li Bit 1..0: Prescaler BIts for BaudRate generation       
    */    
    uint8_t iicPrescaler;
    /** \brief <b>iicConfig</b>
        contains the IIC interface configuration bits.
        \li Bit 7:    Bit used to ctrl the setup of the IIC buffers pointer
                        '1' -> setup of pointer enabled, cleared automatically
                        after the setup
        \li Bit 6:    '1' -> IIC is IRQ driven / '0' -> IIC is polled via MainLoop
        \li Bit 5..0:    rfu
    */    
    uint8_t iicConfig;    
    /** \brief <b>iicStatus</b>
        contains the status information of the IIC component
        \li Bit 7:    IIC lock indication ('1' -> locked)
        \li Bit 6:    Data pending to be TXed
        \li Bit 5:    IIC transaction error
        \li Bit 4:    RXed data pending
        \li Bit 3:    IIC transaction ongoing
        \li Bit 2:    Configuration Error
        \li Bit 1..0: IIC mode (00 .. 11 MASTER RX/TX .. SLAVE RX/TX)
    */        
    uint8_t iicStatus;
    /** \brief <b>iicByteCount</b>
        used for counting when serving the IIC irq or data handling.
    */       
    uint8_t iicByteCount;
    /** \brief <b>iicDataTx</b>
        amount of data to be TXed.
    */        
    uint8_t iicDataTx;
    /** \brief <b>iicBufLen</b>
        IIC data buffer size of linked via the *iicBuffer pointer.
    */        
    uint8_t iicBufLen;
    /** \brief <b>iicBuffer</b>
        pointer to the IIC data buffer.
    */        
    uint8_t* iicBuffer;    
}sIicCtrlBlock;


/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
extern sIicCtrlBlock iicCtrlBlock;

extern UINT8FUNC ATA_iicUse_C(sIicCtrlBlock* ctrlBlock, uint8_t iicAddr, uint8_t iicAddrMsk, uint8_t iicBaudRate, uint8_t iicPrescaler, uint8_t bufLen, uint8_t* ptr, uint8_t iicConfig);
extern VOIDFUNC  ATA_iicReset_C(sIicCtrlBlock* ctrlBlock);
extern VOIDFUNC  ATA_iicClrError_C(sIicCtrlBlock* ctrlBlock);
extern VOIDFUNC  ATA_iicClrRxDataPend_C(sIicCtrlBlock* ctrlBlock);
extern UINT8FUNC ATA_iicHwSetup_C(sIicCtrlBlock* ctrlBlock, uint8_t iicAddr, uint8_t iicAddrMsk, uint8_t iicBaudRate, uint8_t iicPrescaler);
extern UINT8FUNC ATA_iicBufferSetup_C(uint8_t len, uint8_t* ptr, sIicCtrlBlock* ctrlBlock);
extern UINT8FUNC ATA_iicOpen_C(sIicCtrlBlock* ctrlBlock);
extern UINT8FUNC ATA_iicMasterTrigger_C(sIicCtrlBlock* ctrlBlock);
extern UINT8FUNC ATA_iicSetData_C(uint8_t index, uint8_t data, sIicCtrlBlock* ctrlBlock, uint8_t len);
extern UINT8FUNC ATA_iicGetData_C(uint8_t index, sIicCtrlBlock* ctrlBlock, uint8_t *data);
extern UINT8FUNC ATA_iicSetBuffer_C(uint8_t* ptrSrc, uint8_t len, sIicCtrlBlock* ctrlBlock);
extern UINT8FUNC ATA_iicGetBuffer_C(uint8_t* ptrDes, uint8_t len, sIicCtrlBlock* ctrlBlock);
extern UINT8FUNC ATA_iicServeIsr_C(void);

extern VOIDFUNC twiByteCompleteHandler_ASM(void);
#elif defined __IAR_SYSTEMS_ASM__
/*startSimExtraction*/
IICCTRLBLOCK_IICADR     EQU 0
IICCTRLBLOCK_IICAMR     EQU IICCTRLBLOCK_IICADR + 1
IICCTRLBLOCK_IICBAUD    EQU IICCTRLBLOCK_IICAMR + 1
IICCTRLBLOCK_IICPRESC   EQU IICCTRLBLOCK_IICBAUD + 1 
IICCTRLBLOCK_IICCONFIG  EQU IICCTRLBLOCK_IICPRESC + 1 
IICCTRLBLOCK_IICSTATUS  EQU IICCTRLBLOCK_IICCONFIG + 1
IICCTRLBLOCK_IICBYTCNT  EQU IICCTRLBLOCK_IICSTATUS + 1
IICCTRLBLOCK_IICDATATX  EQU IICCTRLBLOCK_IICBYTCNT + 1
IICCTRLBLOCK_IICBUFLEN  EQU IICCTRLBLOCK_IICDATATX + 1
IICCTRLBLOCK_IICBUFFER  EQU IICCTRLBLOCK_IICBUFLEN + 1

/** \brief <b>BM_IICLOCK</b>
    is the set-mask for iicStatus lock bit.
*/
BM_IIC2LOCK         EQU 0x80

/** \brief <b>BM_IICTXDATAPENDING</b>
    is the set-mask for iicStatus.
*/
BM_IICTXDATAPENDING         EQU 0x40
IICTXDATAPENDING            EQU 6
/** \brief <b>BM_IICTRXERROR</b>
    is the set-mask for iicStatus.
*/
BM_IICTRXERROR         EQU 0x20

/** \brief <b>BM_IICRXDATAPEDING</b>
    is the set-mask for iicStatus.
*/
BM_IICRXDATAPEDING         EQU 0x10
IICRXDATAPEDING            EQU 4

/** \brief <b>BM_IICTRANSACTIONONGOING</b>
    is the set-mask for iicStatus iic transaction on going indication.
*/
BM_IICTRANSACTIONONGOING         EQU 0x08

/** \brief <b>BM_CONFERRORIIC</b>
    is the set-mask for iicStatus lock bit.
*/
BM_CONFERRORIIC         EQU 0x04

/** \brief <b>BM_IICTRXSTATE</b>
    is the set-mask for iicStatus.
*/
BM_IICTRXSTATE         EQU 0x03

/** \brief <b>IICMASTERTX</b>
    is the set-mask for iic transaction indication.
*/
IICMASTERTX         EQU 0x00
IICMASTERRX         EQU 0x01
IICSLAVETX         EQU 0x02
IICSLAVERX         EQU 0x03

/** \brief <b>BM_IICSTATUSERROR</b>
    mask for all IIC Error indicators.
*/
BM_IICSTATUSERROR  EQU 0x24

BM_PTRVALID        EQU 0x80

/*stopSimExtraction*/
#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* IIC_H */