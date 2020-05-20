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
* File Name    : (micro.h)
* Version      : (2.0)
* Device(s)    : (ATA5790N)
* OS           : (if applicable)
* H/W Platform : (if applicable)
* Description  : (brief description of what is in the file)
*
* $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2_Gen2_LF/appl/appFlash/src/micro.h $
* $LastChangedRevision: 334201 $
* $LastChangedDate: 2015-08-21 08:06:40 -0600 (Fri, 21 Aug 2015) $
* $LastChangedBy: mhahnen $
*******************************************************************************/

#ifndef MICRO_H_
#define MICRO_H_


#include "../../../firmware/stdc/src/ioATA5702M322.h"
#include <inavr.h>

//==============================================================================
// Local Include
//==============================================================================

//==============================================================================
// Public Defines
//==============================================================================

// Define Format Type
#define MICRO_BIG_ENDIAN
#define MCU_ATA5702


// Define constants in ROM
#define CONST __flash const

// Define variables in NO_INIT section
#define NO_INIT_DATA __no_init


// Define variables in EEprom section
#define EEPROM_SECTION __eeprom __root


//==============================================================================
// Public Macro
//==============================================================================


// Access constants in ROM
#define CONST_UINT8(NAME) (*(CONST UINT8*)&(NAME))
#define CONST_UINT16(NAME) (*(CONST UINT16*)&(NAME))
#define CONST_UINT32(NAME) (*(CONST UINT32*)&(NAME))
#define CONST_PTR(NAME) ((UINT8*)(UINT16)NAME)

// interrupts configuration
#define ENABLE_INTERRUPTS   asm("sei")
#define DISABLE_INTERRUPTS  asm("cli")

/* Interrupt Action */
#define INT_ACTION_MASK 0x0F
#define INT_ACTION_NONE 0x00
#define INT_ACTION_LOW 0x01
#define INT_ACTION_HIGH 0x02
#define INT_ACTION_FALL 0x04
#define INT_ACTION_RISE 0x08

/* Interrupt Interrupt */
#define INT_IT_MASK 0x80
#define INT_IT_ENABLE 0x80


#define NOP asm("nop")

/* SLEEP Mode : */
/* ----------- */
#define IDLE                    0x01
#define POWER_SAVE              0x03
#define EXT_POWER_SAVE          0x06
#define POWER_DOWN              0x05
#define EXT_POWER_DOWN          0x09
#define POWER_OFF               0x0B

/* Voltage Monitor : */
/* ----------- */

#define BM_VM_VBAT              0x00
#define BM_VM_VC                0x20
#define BM_VM_VSW               0x40
#define BM_VM_VMIM              0x01
#define BM_VM_DISABLE           0x00
#define BM_VM_2_0V              0x01
#define BM_VM_2_1V              0x02
#define BM_VM_2_2V              0x03
#define BM_VM_2_3V              0x04
#define BM_VM_2_4V              0x05
#define BM_VM_2_5V              0x06
#define BM_VM_2_6V              0x07
#define BM_VM_2_7V              0x08
#define BM_VM_2_8V              0x09
#define BM_VM_2_9V              0x0A
#define BM_VM_3_0V              0x0B
#define BM_VM_3_1V              0x0C
#define BM_VM_3_2V              0x0D
#define BM_VM_3_3V              0x0E
#define BM_VM_3_4V              0x0F
#define BM_VM_3_0V_H            0x81
#define BM_VM_3_1V_H            0x82
#define BM_VM_3_2V_H            0x83
#define BM_VM_3_3V_H            0x84
#define BM_VM_3_4V_H            0x85
#define BM_VM_3_5V              0x86
#define BM_VM_3_6V              0x87
#define BM_VM_3_7V              0x88
#define BM_VM_3_8V              0x89
#define BM_VM_3_9V              0x8A
#define BM_VM_4_0V              0x8B
#define BM_VM_4_1V              0x8C
#define BM_VM_4_2V              0x8D
#define BM_VM_4_3V              0x8E
#define BM_VM_4_4V              0x8F



/* Watchdog @ 2s*/
#define START_WATCHDOG() \
  {\
    DISABLE_INTERRUPTS; \
    asm("WDR");\
    WDTCR |= (1<<WDCE) | (1<<WDE);\
    WDTCR = (1<<WDE) | (1<<WDPS1) | (1<<WDPS0); \
    WDTCR = (1<<WDE) | (1<<WDPS1) | (1<<WDPS0); \
    ENABLE_INTERRUPTS; \
  }

#define REARM_WATCHDOG() \
  { \
    asm("WDR"); \
  }

#define STOP_WATCHDOG() \
  {\
    asm("WDR");\
    MCUSR &= ~(1<<WDRF);\
    WDTCR |= (1<<WDCE)|(1<<WDE);\
    WDTCR = 0x00;\
  }

// Brown out @ 1.8V (Note: do not change voltage monitor)  // MiHa muss noch prüfen ob dies beim Primus2 auch passt
#define ENABLE_BROWNOUT()  {VMCR = (0<<BODLS) | (1<<BODPD) | (VMCR & 0x3F); } 
#define DISABLE_BROWNOUT() {VMCR = 0;}
  
  // init value
#define CRC_INIT_00   0x00
#define CRC_INIT_FF   0xFF

#define CRC_POLY_CCIT 0x07

#endif


