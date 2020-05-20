/*******************************************************************************
  CAN Peripheral Library Template Implementation

  File Name:
    can_PhaseSegment1Length_Default.h

  Summary:
    CAN PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PhaseSegment1Length
    and its Variant : Default
    For following APIs :
        PLIB_CAN_PhaseSegment1LengthSet
        PLIB_CAN_ExistsPhaseSegment1Length

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

#ifndef _CAN_PHASESEGMENT1LENGTH_DEFAULT_H
#define _CAN_PHASESEGMENT1LENGTH_DEFAULT_H

#include "../templates/can_registers.h"

//******************************************************************************
/* Function :  CAN_PhaseSegment1LengthSet_Default

  Summary:
    Implements Default variant of PLIB_CAN_PhaseSegment1LengthSet 

  Description:
    This template implements the Default variant of the PLIB_CAN_PhaseSegment1LengthSet function.
*/

PLIB_TEMPLATE void CAN_PhaseSegment1LengthSet_Default( CAN_MODULE_ID index , CAN_TIME_SEGMENT_LENGTH length )
{
	volatile can_registers_t * can = ((can_registers_t *)(index));
	can->CCFG.SEG1PH = length;
}


//******************************************************************************
/* Function :  CAN_ExistsPhaseSegment1Length_Default

  Summary:
    Implements Default variant of PLIB_CAN_ExistsPhaseSegment1Length

  Description:
    This template implements the Default variant of the PLIB_CAN_ExistsPhaseSegment1Length function.
*/

#define PLIB_CAN_ExistsPhaseSegment1Length PLIB_CAN_ExistsPhaseSegment1Length
PLIB_TEMPLATE bool CAN_ExistsPhaseSegment1Length_Default( CAN_MODULE_ID index )
{
    return true;
}


#endif /*_CAN_PHASESEGMENT1LENGTH_DEFAULT_H*/

/******************************************************************************
 End of File
*/

