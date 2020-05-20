/*******************************************************************************
  ADCHS Peripheral Library Template Implementation

  File Name:
    adchs_CVD_Default.h

  Summary:
    ADCHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CVD
    and its Variant : Default
    For following APIs :
        PLIB_ADCHS_CVDEnable
        PLIB_ADCHS_CVDDisable
        PLIB_ADCHS_CVDSetup
        PLIB_ADCHS_CVDResultGet
        PLIB_ADCHS_ExistsCVD

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

//DOM-IGNORE-END

#ifndef _ADCHS_CVD_DEFAULT_H
#define _ADCHS_CVD_DEFAULT_H



//******************************************************************************
/* Function :  ADCHS_CVDEnable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_CVDEnable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_CVDEnable function.
    Operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_CVDEnable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON1.CVDEN = 1;
}


//******************************************************************************
/* Function :  ADCHS_CVDDisable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_CVDDisable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_CVDDisable function.
    Operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_CVDDisable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON1.CVDEN = 0;
}


//******************************************************************************
/* Function :  ADCHS_CVDSetup_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_CVDSetup 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_CVDSetup function.
    Operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_CVDSetup_Default( ADCHS_MODULE_ID index , ADCHS_CVD_CAPACITOR capSel , bool inBetweenOrEqual , bool greaterEqualHi , bool lessThanHi , bool greaterEqualLo , bool lessThanLo , ADCHS_AN_INPUT_ID inputEnable , int16_t  hiLimit , int16_t  loLimit )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

#ifndef CHECON /* PIC32MZ */
    regs->ADCCON2.CVDCPL = capSel;

    regs->ADCCMPCONx[0].IELOLO = lessThanLo;
    regs->ADCCMPCONx[0].IELOHI = greaterEqualLo;
    regs->ADCCMPCONx[0].IEHILO = lessThanHi;
    regs->ADCCMPCONx[0].IEHIHI = greaterEqualHi;
    regs->ADCCMPCONx[0].IEBTWN = inBetweenOrEqual;

    regs->adccmpx[0].ADCCMPEN |= BIT(inputEnable);
    regs->adccmpx[0].ADCCMP = loLimit | (uint32_t) (hiLimit << 16);
#else /*PIC32MK */
    regs->ADCCON2.CVDCPL = capSel;

    regs->ADCCMPCONx[0].bits.IELOLO = lessThanLo;
    regs->ADCCMPCONx[0].bits.IELOHI = greaterEqualLo;
    regs->ADCCMPCONx[0].bits.IEHILO = lessThanHi;
    regs->ADCCMPCONx[0].bits.IEHIHI = greaterEqualHi;
    regs->ADCCMPCONx[0].bits.IEBTWN = inBetweenOrEqual;

    regs->adccmpx[0].ADCCMPEN.set = BIT(inputEnable);
    regs->adccmpx[0].ADCCMP.bits = loLimit | (uint32_t) (hiLimit << 16);
#endif
}


//******************************************************************************
/* Function :  ADCHS_CVDResultGet_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_CVDResultGet 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_CVDResultGet function.
    Operation is atomic.
*/

PLIB_TEMPLATE int16_t ADCHS_CVDResultGet_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

#ifndef CHECON /* PIC32MZ */
    return (int16_t)regs->ADCCMPCONx[0].CVDDATA;
#else /*PIC32MK */
    return (int16_t)regs->ADCCMPCONx[0].bits.CVDDATA;
#endif
}


//******************************************************************************
/* Function :  ADCHS_ExistsCVD_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_ExistsCVD

  Description:
    This template implements the Default variant of the PLIB_ADCHS_ExistsCVD function.
*/

#define PLIB_ADCHS_ExistsCVD PLIB_ADCHS_ExistsCVD
PLIB_TEMPLATE bool ADCHS_ExistsCVD_Default( ADCHS_MODULE_ID index )
{
    return true;
}


#endif /*_ADCHS_CVD_DEFAULT_H*/

/******************************************************************************
 End of File
*/

