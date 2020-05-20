//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/eep/src/eep.c $
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
/** \file firmware/eep/src/eep.c
*/
//lint -restore

/*---------------------------------------------------------------------------*/
/** \brief <b>Module EEP</b>
    This modules provides basic EEProm access functions as well as access to 
    the FLASH based fuse bytes (Low, High and Fix). Since the fuse write access
    is FLASH based, the FLASH write times do apply and not the EEProm write 
    times. However, the error handling regarding the fuse functions is 
    implemented with EEProm error codes.
    The definition of the EEPromlayout for the Atmel EEProm section, and the 
    Customer EEProm section is also contained within this module.

\internal
\Traceability{Primus2P-3143, Primus2P-3144, Primus2P-3146, Primus2P-3147,\
              Primus2P-3150, Primus2P-3151, Primus2P-3153, Primus2P-3154,\
              Primus2P-3159, Primus2P-3162, Primus2P-3165, Primus2P-3184,\
              Primus2P-3185, Primus2P-3187}
\endinternal
\n
*/
/*---------------------------------------------------------------------------*/

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
#include "eep.h"

/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/

/** \brief <b>ADDR_OF_R16</b>
    defines the memory address of register R16 (used as first parameter 
    within a function call by the IAR compiler)
*/
#define MEM_ADDR_R16        0x10U

/*===========================================================================*/
/*  Modul Globals                                                            */
/*===========================================================================*/
//lint -esym(9003, g_sAtmelEEPromSection) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sAtmelEEPromSection' at block scope
 * variable shall be accessed from outside via flash software or other library
 * modules
 */
/** \brief <b>sAtmelEEPromSection</b>
    contains the Atmel specific EEProm section.
*/
#pragma location = ".eep_sAtmelEEPromSection"
__root __no_init sAtmelEEPromSection g_sAtmelEEPromSection;

//lint -esym(9003, g_sCustomerEEPromSection) FlSc (26.05.2014)
/* disable lint note 9003 - could define variable 'g_sCustomerEEPromSection' at block scope
 * variable shall be accessed from outside via flash software or other library
 * modules
 */
/** \brief <b>sCustomerEEPromSection</b>
    contains the Customer specific EEProm section.
*/
#pragma location = ".eep_sCustomerEEPromSection"
__root __no_init sCustomerEEPromSection g_sCustomerEEPromSection;

//#pragma location = ".eep_sCustomerEEPromSectionAESkey"
//__root __no_init sEepFlashApp_AESKey g_sEepFlashApp_AESKey;

/*===========================================================================*/
/*  IMPLEMENTATION                                                           */
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_eepFuseRead_C</b>
    This function is used to read out one of the three fuse bytes handling
    the access rights control to the generic EEProm sections.

    \param[in]  bCtrl Value to control, which of the three available fuse bytes will be read
    \return     Value of the read fuse byte (::uint8_t)
    
    Technical background:F
    16 Bit are used to control all the available 8 generic EEProm section
    This results in two bytes. The third byte is used to monitor the Fuse
    Fix setting. Which byte is read depends on the input parameter.

    \internal
    \li 001: Store SREG state to be able to restore it at the end of the function 
              call
    \li 005: Update HW Trace Unit with specific function information
    \li 008: Wait for the EEProm to be able to respond to the Fuse read request.
    \li 010: Disable interrupts via global interrupt flag in SREG
    \li 020: Check which fuse byte is desired and setup SPMCSR according to this.
             The SPMCSR bits to ctrl the LPM instruction are reset with the following LPM.
    \li 030: Trigger the LPM instruction.
    \li 040: Restore SREG to state before function call
    \li 050: Return the read fuse byte value

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3150}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
__root uint8_t ATA_eepFuseRead_C(uint8_t bCtrl)
/*---------------------------------------------------------------------------*/
{
    /* LLR-Ref: 001 */
    uint8_t bSreg = SREG;  
  
    uint8_t bFuseVal;
    uint8_t *regPtr;

    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_eepFuseRead_C, bCtrl);

    /* LLR-Ref: 008 */
    while(EECR&(BITMASK(NVMBSY)));
    
    /* LLR-Ref: 010 */
    __disable_interrupt();

    /* Dummy write, as the inline ASM will overwrite this! 
       Just needed to supress a warning */
    regPtr = (unsigned char *)MEM_ADDR_R16;
    
    /* LLR-Ref: 020 */                
    if(bCtrl==FUSE_EP_L){
        SPMCSR = BM_GET_FUSE_EP_L;
    }
    else if(bCtrl==FUSE_EP_H){
        SPMCSR = BM_GET_FUSE_EP_H;
    }
    else{
        SPMCSR = BM_GET_FUSE_SECF;
    }
    
    /* LLR-Ref: 030 */
    __asm ("LPM");
    __asm ("MOV R16 , R0");
    //bFuseVal = __load_program_memory((const unsigned char __flash *)0x0000U);
    
    /* LLR-Ref: 040 */
    SREG = bSreg;
    
    /* LLR-Ref: 050 */
    bFuseVal = *regPtr;//needed to supress a compiler warning

    return bFuseVal;
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_eepFuseWrite_C</b>
    This function is used to write one of the two fuse bytes handling
    the access rights control to the generic EEProm sections.

    \param[in]  bCtrl             Value to control, which of the three available fuse bytes will be read
    \param[in]  bVal              Value to be written to the selected fuse byte
    \return     Status of the performed EEPROM operation (::eEepErrorCode)

    \internal
    \li 001: Store SREG state to be able to restore it at the end of the function 
             call
    \li 010: Wait for the EEProm to be able responding to the Fuse write request.
    \li 020: Disable interrupts via global interrupt flag in SREG
    \li 025: Reset watchdog
    \li 030: Prepare data to be written via SPM, R17 contains the second passed
             parameter (IAR calling convention), this value is written to R0, the
             register used by the SPM instruction
    \li 040: Update HW Trace Unit with specific function information         
             Not to be placed at the start cause of the inline assembly!
    \li 050: Setup SPMCSR according to the fuse byte intended to be written
    \li 060: Enable the Fuse byte write by setting the enable bit and execute the
             write via the SPM instruction
    \li 070: Restore SREG to state before function call.
    \li 080: Check written fuse with the desired value and return the status of the
             comparison

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3150}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
__root eEepErrorCode ATA_eepFuseWrite_C(uint8_t bCtrl, uint8_t bVal)
/*---------------------------------------------------------------------------*/
{
    /* LLR-Ref: 001 */
    uint8_t bSreg = SREG;
    
    uint8_t bFuseVal;
    eEepErrorCode tmp;

    /* LLR-Ref: 010 */
    while(EECR&(BITMASK(NVMBSY)));

    /* LLR-Ref: 020 */
    __disable_interrupt();
    
    /* LLR-Ref: 025 */
    __watchdog_reset();
        
    /* LLR-Ref: 030 */
    __asm ("MOV R0 , R17");
    
    /* LLR-Ref: 040 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_eepFuseWrite_C, bCtrl); //DO not move this line to any other location
    
    /* LLR-Ref: 050 */
    if(bCtrl==FUSE_EP_L){
        SPMCSR = BM_GET_FUSE_EP_L;
    }
    else if(bCtrl==FUSE_EP_H){
        SPMCSR = BM_GET_FUSE_EP_H;
    }
    else{
        SPMCSR = BM_GET_FUSE_SECF;
    }
    /* LLR-Ref: 060 */
    __asm ("SPM");
    
    /* LLR-Ref: 070 */
    SREG = bSreg;
    
    /* LLR-Ref: 080 */
    bFuseVal = ATA_eepFuseRead_C(bCtrl);
    if(bVal != bFuseVal){
        tmp = EEC_ACCESS_FUSE_WR_FAIL;
    }
    else {
        tmp = EEC_NO_ERROR;
    }
	return tmp;
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_eepEEPaccessRightsChange_C</b>
    This function is used to adjust the access rights settings of the EEProm.
    The fuses as well as the mask register will be manipulated according to the
    given values.

    \param[in]  bSectionID          EEPROM section ID for which the access rights 
                                    shall be changed
    \param[in]  bVal                Bit1..0: New access right value for the given 
                                    EEProm section (Bit 0 for WRITE, Bit 1 for READ)
                                     - Bit value "1" for an access right means 
                                       "unlock"
                                     - Bit value "0" for an access right means 
                                       "lock"
                                    Bit7: Indication for fuse or register

    \return     Status of the performed EEPROM operation (::eEepErrorCode)

    \internal
    \li 005: Update HW Trace Unit with specific function information
    \li 010: Check for Fuse settings to be adjusted
    \li 020: Fuse high byte adjustment
    \li 030: Fuse low byte adjustment
    \li 040: Return Error in case Section is not valid to be protected via the Fuses
    \li 050: Adjust access rights via register
    \li 060: Calculate mask and pattern to adjuste the needed EEPR<X> Register

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3150, Primus2P-3153, Primus2P-3187}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
__root eEepErrorCode ATA_eepEEPaccessRightsChange_C(uint8_t bSectionID, uint8_t bVal)
/*---------------------------------------------------------------------------*/
{
    uint8_t tmp;
    uint8_t mask;

    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_eepEEPaccessRightsChange_C, bSectionID);

    /* LLR-Ref: 010 */
    if(bVal&EEP_ACC_FUSE)
    {   
        /* Change access rights via fuses */
    	bVal &= ~EEP_ACC_FUSE;
        tmp = bSectionID - EEP_FIRST_SECTION_FUSE_ID;
    	bSectionID = bSectionID - EEP_FIRST_SECTION_FUSE_ID;
        
        /* In order to handle both FUSE_EP_L and FUSE_EP_H, the mask and value
           byte need to be adjusted for FUSE_EP_H in order to take effect. */
    	if ((bSectionID&EEP_SECTION_ID_MASK) > (EEP_NUM_SECTION_PER_FUSE_REG_BYTE - 1))
        {
            tmp = tmp - EEP_NUM_SECTION_PER_FUSE_REG_BYTE;
        }
        
        /* Mask out the two bits intended to be changed */
        mask = (EEP_ACCESS_RIGHT_MASK <<(tmp<<1U));
        bVal = (bVal <<(tmp<<1U));
            
    	/* LLR-Ref: 020 */
    	if((bSectionID&EEP_SECTION_ID_MASK)<EEP_NUM_SECTION_PER_FUSE_REG_BYTE)
        {
            tmp = ATA_eepFuseRead_C(FUSE_EP_L);
            tmp &= ~mask;
            tmp |= bVal;
            return ATA_eepFuseWrite_C(FUSE_EP_L,tmp);
        }
        /* LLR-Ref: 030 */
        else if((bSectionID&EEP_SECTION_ID_MASK)<(2U*EEP_NUM_SECTION_PER_FUSE_REG_BYTE))
        {
            tmp = ATA_eepFuseRead_C(FUSE_EP_H);
            tmp &= ~mask;
            tmp |= bVal;
            return ATA_eepFuseWrite_C(FUSE_EP_H,tmp);
        }
        /* LLR-Ref: 040 */
        else
        {
            return EEC_SECTION_INVALID;
        }
    }
    /* LLR-Ref: 050 */
    else
    {
        /* LLR-Ref: 060 */
        tmp = (bSectionID>>2U);
        bSectionID &= EEP_ACCESS_RIGHT_MASK;
        
        /* Mask out the two bits intended to be changed */
        mask = (EEP_ACCESS_RIGHT_MASK <<(bSectionID<<1U));
        
        /* Invert the section access rights settings */
        bVal = (bVal^EEP_ACCESS_RIGHT_INVERT_MASK)&EEP_ACCESS_RIGHT_MASK;
        bVal = (bVal <<(bSectionID<<1U));
        
        /* Check which EEPRx register is to be updated with the given access
           rights */
        if(tmp == EEP_SECTION_ID_IN_EEPR0){
            tmp = EEPR0;
            tmp &= ~mask;
            tmp |= bVal;
            EEPR0 = tmp;
        }
        else if(tmp == EEP_SECTION_ID_IN_EEPR1){
            tmp = EEPR1;
            tmp &= ~mask;
            tmp |= bVal;
            EEPR1 = tmp;
        }
        else if(tmp == EEP_SECTION_ID_IN_EEPR2){
            tmp = EEPR2;
            tmp &= ~mask;
            tmp |= bVal;
            EEPR2 = tmp;
        }
        else if(tmp == EEP_SECTION_ID_IN_EEPR3){
            tmp = EEPR3;
            tmp &= ~mask;
            tmp |= bVal;
            EEPR3 = tmp;
        }
        else{
            return EEC_SECTION_INVALID;
        }
        return EEC_NO_ERROR;
	}
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_eepFusesFix_C</b>
    This function is used to fix the EEProm access rights fuses. if fuse is
    programmed the fuses are not eraseable anymore.

    \param[in]  bVal                Value to be written to the fix fuse byte
    \return     Status of the performed EEPROM operation (::eEepErrorCode)

    \internal
    \li 005: Update HW Trace Unit with specific function information
    \li 010: Adjust Fuse settings by calling function ::ATA_eepFuseWrite_C with the
              new fuse fix value and fuse fix as target fuse

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3150, Primus2P-3153}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
__root eEepErrorCode ATA_eepFusesFix_C(uint8_t bVal)
/*---------------------------------------------------------------------------*/
{
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_eepFusesFix_C, bVal);

    /* LLR-Ref: 010 */
    return ATA_eepFuseWrite_C(FUSE_EP_F,bVal);
}
/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_eepReadBytes_C</b>
    The purpose of this function is to read multiple bytes from the EEPROM in
    accordance with the applicable EEPROM Access Right Map.
    To copy the bytes, the EEPROM burst read mode is used.

    \param[out] pDes  Pointer pointing to the destination to store the read data to
    \param[in]  wSrc  Specifying the EEProm address to start reading from
    \param[in]  bCount amount of data to be read from the given EEProm address
    \return     Status of the performed EEPROM operation (::eEepErrorCode)
    
    \internal
    \li 001: Store SREG state to be able to restore it at the end of the function 
              call
    \li 005: Update HW Trace Unit with specific function information
    \li 010: Check for the addresses being within range without overflow protection

             Note 1:
             The HW/SW access right management will prevent any locked/protected 
             data to be read. Address overflow will also be detected.

             Note 2:
             The computation of max. address and the following check is correct,
             since the last valid address to read one byte from is 
             EEP_ADDR_START_OUT_OF_RANGE_SEC - 1. This check approach does not 
             require to subract -1 from max. address, which would be the most
             natural way to compute the real max. address.

    \li 020: Check EEP write feature for being busy. Wait in case until it is able
             to deal with the request.
    \li 030: Disable interrupts via global interrupt flag in SREG
    \li 040: Setup EEProm address for start the reading request.
    \li 050: Enable the EEPROM burst read feature by setting bit "EEBRE" in
             register EECR2 to 1
    \li 060: read desired data from EEProm and store it to the destination address
    \li 070: If EEP FEC (EEP error correction flag) flag is, set the return status 
              to EEProm error correction occured and clear the flag afterwards
    \li 080: If EEP AVF (Access violation Flag) flag is, set the return status to
              EEProm address locked for reading and clear the flag afterwards
    \li 090: Disable the EEPROM burst read mode by setting bit "EEBRE" in register
             EECR2 to 0.
    \li 100: Restore SREG to state before function call.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3143, Primus2P-3146, Primus2P-3150, Primus2P-3153,\
                  Primus2P-3184}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
__root eEepErrorCode ATA_eepReadBytes_C(uint8_t* pDes, uint16_t wSrc, uint8_t bCount)
{
    /* LLR-Ref: 001 */
    uint8_t bSreg = SREG;

    uint16_t maxAddr;
    eEepErrorCode ret = EEC_NO_ERROR;
    
    /* LLR-Ref: 005 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_eepReadBytes_C, bCount);
    
    /* LLR-Ref: 010 */
    maxAddr = wSrc + bCount;
    if((wSrc > EEP_ADDR_END_ATMEL_SEC) ||
       (maxAddr > EEP_ADDR_START_OUT_OF_RANGE_SEC))
    {
        return EEC_ADDR_OUT_OF_RANGE;
    }

    /* LLR-Ref: 020 */
    while(EECR&(BITMASK(NVMBSY)));
    
    /* LLR-Ref: 030 */
    __disable_interrupt();
    
    /* LLR-Ref: 040 */
    EEARH = (uint8_t)(wSrc >> SHIFT_HIGH_TO_LOW_BYTE);
    EEARL = (uint8_t)(wSrc & MASK_HIGH_BYTE);
    
    /* LLR-Ref: 050 */
    EECR2 = BM_EEBRE;
    
    /* LLR-Ref: 060 */
    for(uint8_t i=0;i<bCount;i++){
        *pDes++ = EEDR;
    }
    /* LLR-Ref: 070 */
    if(EECR2 & BM_E2FF) {
        ret = EEC_ERROR_CORRECTION_OCCURED;
        EECR2 |= BM_E2FF;
    }
    
    /* LLR-Ref: 080 */
    if(EECR2 & BM_E2AVF)
    {
        ret = EEC_ADDR_LOCKED_FOR_READING;
        EECR2 |= BM_E2AVF;
    }
    
    /* LLR-Ref: 090 */
    EECR2 &= ~BM_EEBRE;

    /* LLR-Ref: 100 */
    SREG = bSreg;

    return ret;
}


/*---------------------------------------------------------------------------*/
/** \brief <b>ATA_eepWriteBytes_C</b>
    shall write data to the device internal EEProm.

    \param[in] pSrc   Pointer pointing to the source of the written data
    \param[in] wDes   Specifying the EEP address to start writing to
    \param[in] bCount Specifying the amount of data to write to the given EEP address
    \return     Status of the performed EEPROM write access (:.eEepErrorCode)

    \internal
    \li 001: Store SREG state to be able to restore it at the end of the function 
              call
    \li 010: Update HW Trace Unit with specific function information
    \li 020: Check for the addresses being within range with overflow protection

             Note 1:
             The HW/SW access right management will prevent any locked/protected 
             data to be read. Address overflow will also be detected.

             Note 2:
             The computation of max. address and the following check is correct,
             since the last valid address to read one byte from is 
             EEP_ADDR_START_OUT_OF_RANGE_SEC - 1. This check approach does not 
             require to subract -1 from max. address, which would be the most
             natural way to compute the real max. address.

    \li 030: Loop writing the desired data to EEProm by using the page write feature.
    \li 040: Check EEP write feature for being busy. Wait in case untill it is able
              to deal with the request.
    \li 050: Disable interrupts via global interrupt flag in SREG
    \li 060: Reset WDT counter
    \li 070: Setup EEP address and enalbe the page writing feature.
    \li 080: Loop to write the data to the EEP page buffer.
    \li 090: Break in case page border is reached or all bytes have been transfered.
    \li 100: Enable the atomic EEPROM write operation by resetting bits "EEPM1" and
              "EEPM0" in register EECR. Enable the master write mode by setting bit
              "EEMWE" in register EECR. Then Start the EEPROM write by setting bit
              "EEWE" in register EECR.
    \li 110: Restore SREG to state before function call.
    \li 130: Check error flag caused by a wrong access.

    \Derived{No}

    \Rationale{N/A}

    \Traceability{Primus2P-3143, Primus2P-3146, Primus2P-3150, Primus2P-3153\
                  Primus2P-3185}
    \endinternal
\n
*/
/*---------------------------------------------------------------------------*/
__root eEepErrorCode ATA_eepWriteBytes_C(uint8_t *pSrc, uint16_t wDes, uint8_t bCount)
{
    /* LLR-Ref: 001 */
    uint8_t bSreg = SREG;
  
    uint16_t maxAddr;
    eEepErrorCode ret = EEC_NO_ERROR;
    
    /* LLR-Ref: 010 */
    ATA_SET_FUNCTION_TRACE_POINT_C(ATA_eepWriteBytes_C, bCount);
        
    /* LLR-Ref: 020 */
    maxAddr = wDes + bCount;
    if((wDes > EEP_ADDR_END_CUSTO_SEC) || (maxAddr > EEP_ADDR_START_ATMEL_SEC))
    {
        return EEC_ADDR_OUT_OF_RANGE;
    }
    
    /* LLR-Ref: 030 */
    while(bCount>0)
    {
        /* LLR-Ref: 040 */
        while(EECR&(BITMASK(NVMBSY)));
        
        /* LLR-Ref: 050 */
        __disable_interrupt();
        
        /* LLR-Ref: 060 */
        __watchdog_reset();
        
        /* LLR-Ref: 070 */
        EECR |= BM_EEPAGE;
        EEARH = (uint8_t)(wDes >> SHIFT_HIGH_TO_LOW_BYTE);
        
        /* LLR-Ref: 080 */
        do{
            EEARL = (uint8_t)(wDes & MASK_HIGH_BYTE);
            EEDR = *pSrc++;
            wDes++;
            bCount--;

        /* LLR-Ref: 090 */
        }while((bCount>0)&&((EEARL&MASK_HIGH_NIBBLE)!=MASK_HIGH_NIBBLE));
        
        /* LLR-Ref: 100 */
        EECR &= ~(BM_EEPM1 | BM_EEPM0);
        EECR |= BM_EEMWE;
        EECR |= BM_EEWE;
        
        /* LLR-Ref: 110 */
        SREG = bSreg;
    }
    
    /* LLR-Ref: 130 */
    if(EECR2&BM_E2AVF)
    {
        ret = EEC_ADDR_LOCKED_FOR_WRITING;
        EECR2 |= BM_E2AVF;
    }
    while(EECR&(BITMASK(NVMBSY)));
    
    return ret;
}
