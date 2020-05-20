//lint -e9059
/******************************************************************************
  Use of this software is subject to Microchip's Software License Agreement.
--------------------------------------------------------------------------------
  $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/firmware/compiler/src/compiler.h $
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

/** \file compiler.h Include compiler specific defines.
*/

//lint -restore

#ifndef COMPILER_H
#define COMPILER_H

/*===========================================================================*/
/*  INCLUDES                                                                 */
/*===========================================================================*/
/*===========================================================================*/
/*  DEFINES                                                                  */
/*===========================================================================*/
/** \brief <b>INLINE_ASM_START</b>
    is the beginning of a inline assembler sequence
*/
#define INLINE_ASM_START    asm (

/** \brief <b>INLINE_ASM_END</b>
    is the end of a inline assembler sequence
*/
#define INLINE_ASM_END          );

/** \brief <b>VOIDFUNC</b>
    should be used for all void functions to ensure that they are located in ROM
*/
#define VOIDFUNC 	__root void

/** \brief <b>UINT8FUNC</b>
    should be used for all uint8_t functions to ensure that they are located in ROM
*/
#define UINT8FUNC 	__root uint8_t

/** \brief <b>INT8FUNC</b>
    should be used for all int8_t functions
*/
#define INT8FUNC 	__root int8_t

/** \brief <b>UINT16FUNC</b>
    should be used for all uint16_t functions to ensure that they are located in ROM
*/
#define UINT16FUNC 	__root uint16_t

/** \brief <b>INT16FUNC</b>
    should be used for all int16_t functions to ensure that they are located in ROM
*/
#define INT16FUNC 	__root int16_t

/** \brief <b>UINT32FUNC</b>
    should be used for all uint32_t functions to ensure that they are located in ROM
*/
#define UINT32FUNC 	__root uint32_t

/** \brief <b>INT32FUNC</b>
    should be used for all int32_t functions to ensure that they are located in ROM
*/
#define INT32FUNC 	__root int32_t


/*===========================================================================*/
/*  TYPE DEFINITIONS                                                         */
/*===========================================================================*/
/* Type definitions */

/** \brief <b>uint8_t</b>
    is a typedef for unsigned char in ATA5700 firmware
*/
typedef unsigned char       uint8_t;

/** \brief <b>uint16_t</b>
    is a typedef for unsigned int in ATA5700 firmware
*/
typedef unsigned int        uint16_t;

/** \brief <b>uint32_t</b>
    is a typedef for unsigned long in ATA5700 firmware
*/
typedef unsigned long       uint32_t;

/** \brief <b>int8_t</b>
    is a typedef for signed char in ATA5700 firmware
*/
typedef signed char         int8_t;

/** \brief <b>int16_t</b>
    is a typedef for signed int in ATA5700 firmware
*/
typedef signed int          int16_t;

/** \brief <b>int32_t</b>
    is a typedef for signed long in ATA5700 firmware
*/
typedef signed long         int32_t;

/** \brief <b>__flash prog_char</b>
    is a typedef for signed char located in program memory in ATA5700 firmware
*/
typedef char __flash prog_char;

/** \brief <b>__flash prog_uchar</b>
    is a typedef for unsigned char located in program memory in ATA5700 firmware
*/
typedef unsigned char __flash prog_uchar;

/** \brief <b>__flash prog_uint8_t</b>
    is a typedef for unsigned int (8bit) located in program memory in ATA5700 firmware
*/
typedef uint8_t __flash prog_uint8_t;

/** \brief <b>__flash prog_int8_t</b>
    is a typedef for signed int (8bit) located in program memory in ATA5700 firmware
*/
typedef int8_t __flash prog_int8_t;

/** \brief <b>__flash prog_uint16_t</b>
    is a typedef for unsigned int (16bit) located in program memory in ATA5700 firmware
*/
typedef uint16_t __flash prog_uint16_t;

/** \brief <b>__flash prog_int16_t</b>
    is a typedef for signed int (16bit) located in program memory in ATA5700 firmware
*/
typedef int16_t __flash prog_int16_t;


/* Type definitions */

/*===========================================================================*/
/*  EXTERNAL PROTOTYPES                                                      */
/*===========================================================================*/
#endif /* COMPILER_H */
