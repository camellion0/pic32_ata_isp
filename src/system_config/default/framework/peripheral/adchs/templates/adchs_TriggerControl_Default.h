/*******************************************************************************
  ADCHS Peripheral Library Template Implementation

  File Name:
    adchs_TriggerControl_Default.h

  Summary:
    ADCHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : TriggerControl
    and its Variant : Default
    For following APIs :
        PLIB_ADCHS_AnalogInputLevelTriggerSet
        PLIB_ADCHS_AnalogInputEdgeTriggerSet
        PLIB_ADCHS_AnalogInputTriggerSourceSelect
        PLIB_ADCHS_GlobalSoftwareTriggerEnable
        PLIB_ADCHS_GlobalLevelSoftwareTriggerEnable
        PLIB_ADCHS_GlobalLevelSoftwareTriggerDisable
        PLIB_ADCHS_TriggerSuspendEnable
        PLIB_ADCHS_TriggerSuspendDisable
        PLIB_ADCHS_ExistsTriggerControl

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _ADCHS_TRIGGERCONTROL_DEFAULT_H
#define _ADCHS_TRIGGERCONTROL_DEFAULT_H



//******************************************************************************
/* Function :  ADCHS_AnalogInputLevelTriggerSet_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_AnalogInputLevelTriggerSet 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_AnalogInputLevelTriggerSet function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_AnalogInputLevelTriggerSet_Default( ADCHS_MODULE_ID index , ADCHS_CLASS12_AN_INPUT_ID analogInput )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;
#ifndef CHECON /* PIC32MZ */
    regs->ADCTRGSNS |= BIT(analogInput);
#else /*PIC32MK */
        regs->ADCTRGSNS.set = BIT(analogInput);
#endif
}


//******************************************************************************
/* Function :  ADCHS_AnalogInputEdgeTriggerSet_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_AnalogInputEdgeTriggerSet 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_AnalogInputEdgeTriggerSet function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_AnalogInputEdgeTriggerSet_Default( ADCHS_MODULE_ID index , ADCHS_CLASS12_AN_INPUT_ID analogInput )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

#ifndef CHECON /* PIC32MZ */
    regs->ADCTRGSNS &= ~BIT(analogInput);
#else /*PIC32MK */
    regs->ADCTRGSNS.clr = BIT(analogInput);
#endif
}


//******************************************************************************
/* Function :  ADCHS_AnalogInputTriggerSourceSelect_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_AnalogInputTriggerSourceSelect 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_AnalogInputTriggerSourceSelect function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_AnalogInputTriggerSourceSelect_Default( ADCHS_MODULE_ID index , ADCHS_CLASS12_AN_INPUT_ID inputId , ADCHS_TRIGGER_SOURCE triggerSource )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

#ifndef CHECON /* PIC32MZ */
    regs->ADCTRGx[inputId >> 2] &= ~(0x1f << ((inputId & 0x03) * 8));
    regs->ADCTRGx[inputId >> 2] |= triggerSource << ((inputId & 0x03) * 8);
#else /*PIC32MK */
    regs->ADCTRGx[inputId >> 2].bits &= ~(0x1f << ((inputId & 0x03) * 8));
    regs->ADCTRGx[inputId >> 2].bits |= triggerSource << ((inputId & 0x03) * 8);
#endif
}


//******************************************************************************
/* Function :  ADCHS_GlobalSoftwareTriggerEnable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_GlobalSoftwareTriggerEnable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_GlobalSoftwareTriggerEnable function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_GlobalSoftwareTriggerEnable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON3.GSWTRG = 1;
}


//******************************************************************************
/* Function :  ADCHS_GlobalLevelSoftwareTriggerEnable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_GlobalLevelSoftwareTriggerEnable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_GlobalLevelSoftwareTriggerEnable function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_GlobalLevelSoftwareTriggerEnable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON3.GLSWTRG = 1;
}


//******************************************************************************
/* Function :  ADCHS_GlobalLevelSoftwareTriggerDisable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_GlobalLevelSoftwareTriggerDisable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_GlobalLevelSoftwareTriggerDisable function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_GlobalLevelSoftwareTriggerDisable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON3.GLSWTRG = 0;
}


//******************************************************************************
/* Function :  ADCHS_TriggerSuspendEnable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_TriggerSuspendEnable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_TriggerSuspendEnable function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_TriggerSuspendEnable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON3.TRGSUSP = 1;
}


//******************************************************************************
/* Function :  ADCHS_TriggerSuspendDisable_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_TriggerSuspendDisable 

  Description:
    This template implements the Default variant of the PLIB_ADCHS_TriggerSuspendDisable function.
    This operation is not atomic.
*/

PLIB_TEMPLATE void ADCHS_TriggerSuspendDisable_Default( ADCHS_MODULE_ID index )
{
    volatile adchs_register_t *regs = (adchs_register_t *)index;

    regs->ADCCON3.TRGSUSP = 0;
}


//******************************************************************************
/* Function :  ADCHS_ExistsTriggerControl_Default

  Summary:
    Implements Default variant of PLIB_ADCHS_ExistsTriggerControl

  Description:
    This template implements the Default variant of the PLIB_ADCHS_ExistsTriggerControl function.
*/

#define PLIB_ADCHS_ExistsTriggerControl PLIB_ADCHS_ExistsTriggerControl
PLIB_TEMPLATE bool ADCHS_ExistsTriggerControl_Default( ADCHS_MODULE_ID index )
{
    return true;
}


#endif /*_ADCHS_TRIGGERCONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

