/*******************************************************************************
  EBI Peripheral Library Template Implementation

  File Name:
    ebi_PageReadTime_Default.h

  Summary:
    EBI PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PageReadTime
    and its Variant : Default
    For following APIs :
        PLIB_EBI_PageReadCycleTimeGet
        PLIB_EBI_ExistsPageReadTime

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

#ifndef _EBI_PAGEREADTIME_DEFAULT_H
#define _EBI_PAGEREADTIME_DEFAULT_H

#include "ebi_Registers.h"

//******************************************************************************
/* Function :  EBI_PageReadCycleTimeGet_Default

  Summary:
    Implements Default variant of PLIB_EBI_PageReadCycleTimeGet 

  Description:
    This template implements the Default variant of the PLIB_EBI_PageReadCycleTimeGet function.
*/
PLIB_TEMPLATE int EBI_PageReadCycleTimeGet_Default( EBI_MODULE_ID index , int ChipSelectNumber )
{
    volatile ebi_register_t *ebi = (volatile ebi_register_t *)index;

    return (int)ebi->EBISMTx[ChipSelectNumber].TPRC;
}

//******************************************************************************
/* Function :  EBI_ExistsPageReadTime_Default

  Summary:
    Implements Default variant of PLIB_EBI_ExistsPageReadTime

  Description:
    This template implements the Default variant of the PLIB_EBI_ExistsPageReadTime function.
*/
#define PLIB_EBI_ExistsPageReadTime PLIB_EBI_ExistsPageReadTime
PLIB_TEMPLATE bool EBI_ExistsPageReadTime_Default( EBI_MODULE_ID index )
{
    return true;
}

#endif /*_EBI_PAGEREADTIME_DEFAULT_H*/

/******************************************************************************
 End of File
*/

