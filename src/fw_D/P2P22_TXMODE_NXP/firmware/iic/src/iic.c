/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/iic/src/iic.c $
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

/** \file iic.c
*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "iic.h"
#include "../../globals/src/globals.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/


/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
/** \brief <b>iicCtrlBlock</b>
    contains IIC component data.
*/
#pragma location = ".i2cpara"
__no_init sIicCtrlBlock iicCtrlBlock;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicReset_C</b>
    This routine performs the basic reset/init of the iicStatus and
    iicConfig. The IIC peripheral is also brought back into its default state.

\param[out]     ctrlBlock       Pointer to an IIC ctrl structure

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_iicReset_C.png

\internal
\li 010: Reset all IIC - SRAM - Variables to 0x00
\li 020: Disable IIC peripherals with IRQs Disabled globally

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2435}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_iicReset_C(sIicCtrlBlock* ctrlBlock)
{   
    uint8_t tmp; 
    // LLR-Ref: 010 
    // Set all member variables to 0x00 in order to get a well defined state.
    ctrlBlock->iicAddr = 0x00U;
    ctrlBlock->iicAddrMsk = 0x00U;
    ctrlBlock->iicBaudRate = 0x00U;
    ctrlBlock->iicPrescaler = 0x00U;    
    ctrlBlock->iicStatus = 0x00U;
    ctrlBlock->iicConfig = 0x00U;   
    ctrlBlock->iicBufLen = 0x00U;
    ctrlBlock->iicDataTx = 0x00U;   
    ctrlBlock->iicByteCount = 0x00U;          
    // LLR-Ref: 020 
    // Bring the IIC peripheral into a defined state, IRQs are disabled while
    // serving this task.
    tmp = SREG;
    _CLI;
    PRR0 &= (uint8_t)~(BM_PRI2C);
    I2CR = BM_I2INT;
    PRR0 |= (uint8_t)(BM_PRI2C);
    SREG = tmp;
    
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicClrError_C</b>
    This routine clears pending IIC errors and re enables the peripheral.

\param[out]     ctrlBlock       Pointer to an IIC ctrl structure

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_iicClrError_C.png

\internal
\li 010: Reset both available IIC module error flags
\li 020: Re-Enable the IIC to generate an ACK on the Bus

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2477,Primus2P-2518}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_iicClrError_C(sIicCtrlBlock* ctrlBlock)
{              
    // LLR-Ref: 010 
    // Clear the error flags of the IIC module.
    ctrlBlock->iicStatus &= ~(BM_CONFERRORIIC|BM_IICTRXERROR);
    // LLR-Ref: 020 
    // Enable the ACK of data in the IIC module.
    I2CR |= BM_I2EA;         
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicClrRxDataPend_C</b>
    This routine clears the RX data pending IIC flag and re enables
    the peripheral to generate an IIC ACK.

\param[out]     ctrlBlock       Pointer to an IIC ctrl structure

\return     N/A

\StackUsageInBytes{XXX}

\image html ATA_iicClrRxDataPend_C.png

\internal
\li 010: Reset the RX data pending Flag
\li 020: Re-Enable the IIC to generate an ACK on the Bus

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2477,Primus2P-2518}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
VOIDFUNC ATA_iicClrRxDataPend_C(sIicCtrlBlock* ctrlBlock)
{           
    // LLR-Ref: 010 
    // Reset the RX data pending Flag.
    ctrlBlock->iicStatus &= ~(BM_IICRXDATAPEDING);
    // LLR-Ref: 020 
    // Re-Enable the IIC to generate an ACK on the Bus.
    I2CR |= BM_I2EA;         
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicHwSetup_C</b>
    This routine performs the setup of the HW configuration.
    The configuration is based on the IIC settings located 
    inside the iicCtrlBlock structure.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure
\param[in]      iicAddr         IIC address
\param[in]      iicAddrMsk      IIC address Mask
\param[in]      iicBaudRate     IIC baudrate setting
\param[in]      iicPrescaler    IIC prescaler setting

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicHwSetup_C.png

\internal
\li 010: Check for a ongoing IIC transaction\n
         IF set Error Flag IIC configuration error and do not touch the IIC
         configuration nor its SRAM based variables
\li 020: ELSE Configure the IIC SRAM block with the given parameters and return the success

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2434,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicHwSetup_C(sIicCtrlBlock* ctrlBlock, uint8_t iicAddr, uint8_t iicAddrMsk, uint8_t iicBaudRate, uint8_t iicPrescaler)
{   
    uint8_t retVal;
    retVal = FAIL;
    // LLR-Ref: 010 
    // The SRAM based IIC ctrl struct is only updated in case it is inactive.
    if(!(ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING)){
        ctrlBlock->iicAddr = iicAddr;
        ctrlBlock->iicAddrMsk = iicAddrMsk;      
        ctrlBlock->iicBaudRate = iicBaudRate;
        ctrlBlock->iicPrescaler = iicPrescaler;
        retVal = OK;
    } 
    // LLR-Ref: 020     
    // Updating not possible, as IIC is active exchanging data. 
    // Therfore an error flag is raised.
    else{
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }    
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicBufferSetup_C</b>
    This routine performs the setup of the IIC related buffer holding the data
    to be TXed or RXed.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure
\param[in]      len             Length of the linked buffer array
\param[in]      ptr             Address linked to the IIC buffer   

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicBufferSetup_C.png

\internal
\li 010: Check for a buffer len bigger than 0,\n
         Check the ptr valid flag for a valid pointer\n
         Check that no IIC transaction is in progress\n
         IF Configure the IIC SRAM block with the given parameters and return the successset          
\li 020: ELSE raise Error Flag IIC configuration error and do not touch the IIC
         configuration nor its SRAM based variables

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2433,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicBufferSetup_C(uint8_t len, uint8_t* ptr, sIicCtrlBlock* ctrlBlock)
{   
    uint8_t retVal;
    retVal = FAIL;
    // LLR-Ref: 010   
    // The SRAM based IIC ctrl struct is only updated in case it is inactive 
    // and the ptr changing is validated via a flag.
    if((len>0U)&&(ctrlBlock->iicConfig&BM_PTRVALID)&&(!(ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING))){
        ctrlBlock->iicConfig &= ~BM_PTRVALID;
        ctrlBlock->iicBufLen = len;
        ctrlBlock->iicBuffer = ptr;
        retVal = OK;
    }
    // LLR-Ref: 020  
    // Updating not possible, as IIC is active exchanging data or the 
    // validation falg is not set. Therfore an error flag is raised.    
    else{
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }
    
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicOpen_C</b>
    This routine performs the setup of the HW configuration of the
    IIC settings located inside the iicCtrlBlock structure.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicOpen_C.png

\internal
\li 010: Check if an opening of the IIC interface is possible          
\li 020: IF possible configure the IIC HW registers with the contents 
         of the ctrl structure. Switch the clocking to the peripheral
         and mark it as locked.
\li 025: Polling or IRQ based IIC usage. IF IRQ based IIC IRQ is enabled, ELSE
         IIC IRQ is not enabled, but disabeld.
\li 030: ELSE raise Error Flag IIC configuration error and do not touch the IIC
         configuration nor its SRAM based variables 
\li 040: Return FAIL/OK         

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2432,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicOpen_C(sIicCtrlBlock* ctrlBlock)
{   
    uint8_t retVal;
    retVal = FAIL;
    // LLR-Ref: 010  
    // The IIC module is only opened in case it is not opened/locked already
    // and there is no IIC transaction ongoing.
    if(!((ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING)||(ctrlBlock->iicStatus&BM_IIC2LOCK))){
        // LLR-Ref: 020     
        PRR0 &= (uint8_t)~(BM_PRI2C);
        I2AR = ctrlBlock->iicAddr;
        I2AMR = ctrlBlock->iicAddrMsk;
        I2BR = ctrlBlock->iicBaudRate;
        I2SR = ctrlBlock->iicPrescaler;
        ctrlBlock->iicStatus |= BM_IIC2LOCK;
        // LLR-Ref: 025
        // Check for IRQ based IIC operation or Polling based operation.
        // In case IRQ based -> enable IIC IRQ 
        // In case POLLING -> disable IIC IRQ
        if(ctrlBlock->iicConfig&BM_IICIRQ){
            I2CR = (BM_I2EN | BM_I2IE | BM_I2EA);
        }
        else{
            I2CR = (BM_I2EN | BM_I2EA);
        }
        retVal = OK;
    }    
    else{
        // LLR-Ref: 030    
        // Raise an Error Flag and do not touch the IIC
        // configuration nor its SRAM based variables. 
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }
    // LLR-Ref: 040            
    // Return the status of the opening attemp.
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicUse_C</b>
    This routine performs the all needed steps to have the IIC 
    available for usage.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure
\param[in]      iicAddr         IIC address
\param[in]      iicAddrMsk      IIC address mask
\param[in]      iicBaudRate     IIC baudrate setting
\param[in]      iicPrescaler    IIC prescaler setting    
\param[in]      bufLen          Length of the linked buffer array
\param[in]      ptr             Address linked to the IIC buffer
\param[in]      iicConfig       IIC configuration

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicUse_C.png

\internal
\li 010: Trigger the HW tracer 
\li 020: Reset the IIC SRAM contents and the IIC HW by calling function 
         ::ATA_iicReset_C
\li 030: Setup the buffer linked to the IIC module by calling function
         ::ATA_iicBufferSetup_C
\li 040: Setup the IIC related HW by calling function ::ATA_iicHwSetup_C
\li 050: Open the IIC peripheral by calling function ::ATA_iicOpen_C
\li 060: Return the status of the operations

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2432,Primus2P-2476,Primus2P-2477,Primus2P-2501}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicUse_C(sIicCtrlBlock* ctrlBlock, uint8_t iicAddr, uint8_t iicAddrMsk, uint8_t iicBaudRate, uint8_t iicPrescaler, uint8_t bufLen, uint8_t* ptr, uint8_t iicConfig)
{
    uint8_t retVal;
    // LLR-Ref: 010   
    // Trigger the HW tracer. 
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_iicUse_C, 0x00U);
    retVal = FAIL;
    // LLR-Ref: 020  
    // Reset the IIC SRAM contents and the IIC HW.
    ATA_iicReset_C(ctrlBlock);
    ctrlBlock->iicConfig = iicConfig;
    // LLR-Ref: 030   
    // Setup the buffer linked to the IIC module.
    if(ATA_iicBufferSetup_C(bufLen,ptr,ctrlBlock)){
        return retVal;
    }
    // LLR-Ref: 040    
    // Setup the IIC related HW.
    if(ATA_iicHwSetup_C(ctrlBlock,iicAddr,iicAddrMsk,iicBaudRate,iicPrescaler)){
        return retVal;
    }
    // LLR-Ref: 050    
    // Open the IIC peripheral.
    ATA_iicOpen_C(ctrlBlock);
    // LLR-Ref: 060  
    // Return the status of the operations.    
    retVal = OK;
    return retVal;    
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicMasterTrigger_C</b>
    This routine Trigger the start of an IIC master transaction.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicMasterTrigger_C.png

\internal
\li 010: Trigger the HW tracer 
\li 020: Check if triggering the transaction is valid. 
         IF NOT, raise an error condition
\li 030: IF VALID, display it via the iicStatus variable
\li 040: Setup a Master RX transaction
\li 050: Setup a Master TX transaction
\li 060: Trigger the start of the IIC transaction
\li 070: Return the status of the operation

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2426,Primus2P-2427,Primus2P-2428,Primus2P-2429,Primus2P-2431,Primus2P-2477,Primus2P-2501}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicMasterTrigger_C(sIicCtrlBlock* ctrlBlock)
{   
    uint8_t retVal;
    retVal = FAIL;
    // LLR-Ref: 010    
    // Trigger the HW tracer 
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_iicMasterTrigger_C, *(ctrlBlock->iicBuffer));
    // LLR-Ref: 020    
    // Check if an IIC transaction is in progress or the setup data length is 0x00 
    // or the setup data length is larger than the IIC buffer or the IIC is not locked.
    // if one condition is true, an error flag is raised.
    if(((ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING)||(ctrlBlock->iicDataTx==0x00U)||(ctrlBlock->iicDataTx>ctrlBlock->iicBufLen)||(!ctrlBlock->iicStatus&BM_IIC2LOCK))){       
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }    
    // LLR-Ref: 030  
    // Triggering an IIC trnasaction is valid. Check what mode is needed. (Master TX / Master RX)
    // Setup the IIC status byte to display the internal IIC state.
    else{
        ctrlBlock->iicStatus &= ~(BM_IICTRANSACTIONONGOING|BM_IICRXDATAPEDING|BM_IICTXDATAPENDING|BM_IICTRXSTATE);
        // LLR-Ref: 040  
        if(*(ctrlBlock->iicBuffer)&BM_IICTRX){//Master Read IIC transaction.
            ctrlBlock->iicDataTx--;
            if(ctrlBlock->iicDataTx<0x01U){
                ctrlBlock->iicStatus |= BM_CONFERRORIIC;
                return retVal;
            }
            ctrlBlock->iicStatus |= (IICMASTERRX|BM_IICTRANSACTIONONGOING);
        }
        // LLR-Ref: 050  
        else{//Masater Write IIC transaction.
            ctrlBlock->iicStatus |= (IICMASTERTX|BM_IICTRANSACTIONONGOING);        
        }        
        // LLR-Ref: 060   
        // Reset the IIC transaction byte counter.
        ctrlBlock->iicByteCount = 0x00U; 
        I2CR |= (BM_I2STA | BM_I2INT);           
        retVal = OK;        
    }  
    // LLR-Ref: 070   
    // return the status of the IIC master triggering.
    return retVal;
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicSetData_C</b>
    This routine performs the setup of the IIC related buffer holding the data
    to be TXed or RXed.

\param[in,out]  ctrlBlock        Pointer to an IIC ctrl structure
\param[in]      index            Index of place data will put into the pointed buffer
\param[in]      data             Pattern put into the index location
\param[in]      len              Length of data to be sent

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicSetData_C.png

\internal
\li 010: Check if setup of data is valid, IF NOT raise an error condition
\li 020: IF VALID setup the data at the desired location

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2431,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicSetData_C(uint8_t index, uint8_t data, sIicCtrlBlock* ctrlBlock, uint8_t len)
{   
    uint8_t retVal;
    retVal = FAIL;
    // LLR-Ref: 010   
    // Check if setup of data is valid, IF NOT raise an error condition
    // Error is raised in case of an ongoing IIC transaction or the index is out of 
    // the range of the linked array.
    if(((ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING))||(index>=ctrlBlock->iicBufLen)){        
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }
    // LLR-Ref: 020  
    // setup the data at the desired location.
    else{
        *(ctrlBlock->iicBuffer+index) = data;
        ctrlBlock->iicDataTx = len;
        retVal = OK;        
    }    
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicGetData_C</b>
    This routine performs the setup of the IIC related buffer holding the data
    to be TXed or RXed.

\param[in,out]  ctrlBlock        Pointer to an IIC ctrl structure
\param[out]     data             Received data
\param[in]      index            Index of place data will put into the pointed buffer

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicGetData_C.png

\internal
\li 010: Check if read out of data is valid, IF NOT raise an error condition
\li 020: IF VALID read the data at the desired location    

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2431,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicGetData_C(uint8_t index, sIicCtrlBlock* ctrlBlock, uint8_t* data)
{   
    uint8_t retVal;
    retVal = FAIL;
    *data = *(ctrlBlock->iicBuffer+index);
    // LLR-Ref: 010 
    // Check if reading out of data is valid, IF NOT raise an error condition
    // Error is raised in case of an ongoing IIC transaction or the index is out of 
    // the range of the linked array.    
    if(((ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING))||(index>=ctrlBlock->iicBufLen)){                        
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }
    // LLR-Ref: 020  
    // Read the data at the desired location.
    else{
        retVal = OK;
    }    
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicSetBuffer_C</b>
    This routine performs the setup of the IIC related buffer holding the data
    to be TXed or RXed.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure
\param[in]      ptrSrc          Source data buffer
\param[in]      len             Amount of data to be copied

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicSetBuffer_C.png

\internal
\li 010: Check if setup of the buffer data is valid, 
         IF NOT raise an error condition
\li 015: Data copying loop
\li 020: IF VALID setup the buffer data     

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2431,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicSetBuffer_C(uint8_t* ptrSrc, uint8_t len, sIicCtrlBlock* ctrlBlock)
{   
    uint8_t retVal;
    uint8_t i;
    retVal = FAIL;
    // LLR-Ref: 010   
    // Check if setup of buffer data is valid, IF NOT raise an error condition
    // Data is setup in case of no ongoing IIC transaction or the index is in 
    // the range of the linked array.    
    if(!((ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING)||(len>ctrlBlock->iicBufLen))){        
        // LLR-Ref: 015
        // Copy Loop for the desired data.
        for(i=0;i<len;i++){
            *(ctrlBlock->iicBuffer+i) = *ptrSrc++;
        }        
        ctrlBlock->iicDataTx = len;
        retVal = OK;
    }
    // LLR-Ref: 020
    // Raise the Error flag.
    else{
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }    
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicGetBuffer_C</b>
    This routine performs the setup of the IIC related buffer holding the data
    to be TXed or RXed.

\param[in,out]  ctrlBlock       Pointer to an IIC ctrl structure
\param[out]     ptrDes          Destination data buffer
\param[in]      len             Amount of data to be copied

\return     OK on success, FAIL on failure

\StackUsageInBytes{XXX}

\image html ATA_iicGetBuffer_C.png

\internal
\li 010: Check if read out of the buffer is valid, 
         IF NOT raise an error condition
\li 015: Data copying loop
\li 020: IF VALID read the given amount of data of the buffer    

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2431,Primus2P-2477}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicGetBuffer_C(uint8_t* ptrDes, uint8_t len, sIicCtrlBlock* ctrlBlock)
{   
    uint8_t retVal;
    uint8_t i;
    retVal = FAIL;
    // LLR-Ref: 010 
    // Check if reading of buffer data is valid, IF NOT raise an error condition
    // Data is read in case of no ongoing IIC transaction or the index is in 
    // the range of the linked array.    
    if(!((ctrlBlock->iicStatus&BM_IICTRANSACTIONONGOING)||(len>ctrlBlock->iicBufLen))){        
        // LLR-Ref: 015 
        // Copy Loop for the desired data.
        for(i=0;i<len;i++){
            *ptrDes++ = *(ctrlBlock->iicBuffer+i);
        }        
        retVal = OK;
    }
    // LLR-Ref: 020 
    // Raise the Error flag.
    else{
        ctrlBlock->iicStatus |= BM_CONFERRORIIC;
    }    
    return retVal;
}

/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_iicServeIsr_C</b>
    This routine performs the polling of the IIC irq flag in case the IIC 
    is used in non IRQ mode.

\param      N/A

\return     OK in case IRQ was pending, FAIL in case no IRQ was pending

\StackUsageInBytes{XXX}

\image html ATA_iicServeIsr_C.png

\internal
\li 010: Check if IIC - IRQ flag is set, IF serve it by calling function
         ::twiByteCompleteHandler_ASM
\li 020: ELSE return, indicating nothing has been done

\Derived{No}

\Rationale{N/A}

\Traceability{Primus2P-2426,Primus2P-2427,Primus2P-2428,Primus2P-2429,Primus2P-2430,Primus2P-2477,Primus2P-2501}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/
UINT8FUNC ATA_iicServeIsr_C(void)
{  
    uint8_t retVal;
    // LLR-Ref: 010 
    // Check if IIC - IRQ flag is set, IF serve it!
    if(I2CR&BM_I2INT){
        twiByteCompleteHandler_ASM();
        retVal = OK;
    }
    // LLR-Ref: 020  
    // ELSE return, indicating nothing has been done.
    else{
        retVal = FAIL;
    }
    return retVal;
}