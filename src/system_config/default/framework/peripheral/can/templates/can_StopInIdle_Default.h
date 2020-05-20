/*******************************************************************************
  CAN Peripheral Library Template Implementation

  File Name:
    can_StopInIdle_Default.h

  Summary:
    CAN PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : StopInIdle
    and its Variant : Default
    For following APIs :
        PLIB_CAN_StopInIdleEnable
        PLIB_CAN_StopInIdleDisable
        PLIB_CAN_ExistsStopInIdle

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _CAN_STOPINIDLE_DEFAULT_H
#define _CAN_STOPINIDLE_DEFAULT_H

#include "../templates/can_registers.h"

//******************************************************************************
/* Function :  CAN_StopInIdleEnable_Default

  Summary:
    Implements Default variant of PLIB_CAN_StopInIdleEnable 

  Description:
    This template implements the Default variant of the PLIB_CAN_StopInIdleEnable function.
*/

PLIB_TEMPLATE void CAN_StopInIdleEnable_Default( CAN_MODULE_ID index )
{
	volatile can_registers_t * can = ((can_registers_t *)(index));
	can->CCONSET = _C1CON_SIDL_MASK;
}


//******************************************************************************
/* Function :  CAN_StopInIdleDisable_Default

  Summary:
    Implements Default variant of PLIB_CAN_StopInIdleDisable 

  Description:
    This template implements the Default variant of the PLIB_CAN_StopInIdleDisable function.
*/

PLIB_TEMPLATE void CAN_StopInIdleDisable_Default( CAN_MODULE_ID index )
{
	volatile can_registers_t * can = ((can_registers_t *)(index));
	can->CCONCLR = _C1CON_SIDL_MASK;
}


//******************************************************************************
/* Function :  CAN_ExistsStopInIdle_Default

  Summary:
    Implements Default variant of PLIB_CAN_ExistsStopInIdle

  Description:
    This template implements the Default variant of the PLIB_CAN_ExistsStopInIdle function.
*/

#define PLIB_CAN_ExistsStopInIdle PLIB_CAN_ExistsStopInIdle
PLIB_TEMPLATE bool CAN_ExistsStopInIdle_Default( CAN_MODULE_ID index )
{
    return true;
}


#endif /*_CAN_STOPINIDLE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

