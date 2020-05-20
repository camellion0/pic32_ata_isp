/*******************************************************************************
* Copyright (c) 2013 Atmel Corporation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/*******************************************************************************
* File Name    : (utils.h)
* Version      : (2.0)
* Device(s)    : 
* OS           : (if applicable)
* H/W Platform : (if applicable)
* Description  : (brief description of what is in the file)
*
* $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/utils.h $
* $LastChangedRevision: 438548 $
* $LastChangedDate: 2016-12-19 04:28:22 -0700 (Mon, 19 Dec 2016) $
* $LastChangedBy: krishna.balan $
*******************************************************************************/

#ifndef UTILS_H_
#define UTILS_H_

//==============================================================================
// Local Include
//==============================================================================
#include "../../../firmware/globals/src/globals.h"
#include <stdbool.h>


// return CRAM variable
#define CRAM_DATA(NAME) (rts_cram.data.##NAME)


//==============================================================================
// Public functions Declarations
//==============================================================================

/* Memory Tools */
extern void memory_copy(uint8_t*, uint8_t*, uint8_t);
extern void memory_copy_const(uint8_t* lpub_dst,
                              const uint8_t* lpub_src,
                              uint8_t lub_length);
extern void memory_set(uint8_t* lpub_dst,
                       uint8_t lub_value, 
                       uint8_t lub_length);
extern bool memory_compare(uint8_t*,uint8_t*,uint8_t);
extern bool memory_compare_const(uint8_t* lpub_src1,
                                    const uint8_t* lpub_src2,                                   
                                    uint8_t lub_length);

uint16_t calcCRC16(uint8_t const message[], uint8_t length);
#endif /*UTILS_H_*/