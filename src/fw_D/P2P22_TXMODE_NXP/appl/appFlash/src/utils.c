//==============================================================================
// Function Implementation
//==============================================================================


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
* File Name    : (utils.c)
* Version      : (2.0)
* Device(s)    : 
* OS           : (if applicable)
* H/W Platform : (if applicable)
* Description  : (brief description of what is in the file)
*
* $URL: http://svnservulm.corp.atmel.com/svn/CDB/Apps/SW_Lib/Car_Access/CARS_GEN2/ATAB5702A/Branches/P2P22_TXMODE_NXP/appl/appFlash/src/utils.c $
* $LastChangedRevision: 443672 $
* $LastChangedDate: 2017-01-23 01:58:06 -0700 (Mon, 23 Jan 2017) $
* $LastChangedBy: krishna.balan $
*******************************************************************************/

//==============================================================================
//  Local Include
//==============================================================================
#include "utils.h"

//==============================================================================
//  Local defines
//==============================================================================
//! CRC generator polynomial.
#define POLYNOMIAL      0x1021 // CRC-16 CCITT standard.
#define WIDTH           (8 * 2)
#define TOPBIT          (1 << (WIDTH - 1))
//==============================================================================
// Local Macro
//==============================================================================

//==============================================================================
// Local Types
//==============================================================================

//==============================================================================
// Public Variables
//==============================================================================

/**
 * \brief Memory Copy
 *        Copy a RAM buffer to another
 *
 * \param[in] lpub_dst destination buffer
 * \param[in] lpub_src source buffer
 * \param[in] lub_length number of bytes to copy
 *
 * \return void
 */
void memory_copy(uint8_t* lpub_dst, uint8_t* lpub_src, uint8_t lub_length)
{
  while (lub_length)
  {
    /* Copy byte by byte */
    lub_length--;
    *lpub_dst++ = *lpub_src++;
  }
}


/**
 * \brief Memory Copy const
 *        Copy contetnt stored in Flash to a RAM buffer
 *
 * \param[in] lpub_dst destination buffer
 * \param[in] lpub_src source adress in flash
 * \param[in] lub_length number of bytes to copy
 *
 * \return void
 */
void memory_copy_const(uint8_t* lpub_dst, const uint8_t* lpub_src, uint8_t lub_length)
{
  while (lub_length--)
  {
    /* Copy byte by byte */
    *lpub_dst++ = *lpub_src++;
  }
}

/**
 * \brief Memory Set
 *        Initializa all bytes of a buffer to a desired value
 *
 * \param[in] lpub_dst Buffer to set
 * \param[in] lub_value initialization value
 * \param[in] lub_length Number of bytes to set
 *
 * \return void
 */
void memory_set(uint8_t* lpub_dst, uint8_t lub_value, uint8_t lub_length)
{
  while (lub_length)
  {
    /* Copy byte by byte */
    lub_length--;
    lpub_dst[lub_length] = lub_value;
  }
}


/**
 * \brief Memory Compare
 *        Comapre two buffers
 *
 * \param[in] lpub_src1  address of buffer 1
 * \param[in] lpub_src2  address of buffer 2
 * \param[in] lub_length number of bytes to compare
 *
 * \return TRUE if comapre match, FALSE otherwise
 */
bool memory_compare(uint8_t* lpub_src1, uint8_t* lpub_src2, uint8_t lub_length)
{
  while (lub_length)
  {
    /* Copy byte by byte */
    lub_length--;
    if (lpub_src1[lub_length] != lpub_src2[lub_length])
    {
      return FALSE;
    }
  }
  return TRUE;
}

/**
 * \brief Memory Compare const
 *        Comapre a buffer to values stored in Flash
 *
 * \param[in] lpub_src1  address of RAM buffer
 * \param[in] lpub_src2  address of Falsh data
 * \param[in] lub_length number of bytes to compare
 *
 * \return TRUE if comapre match, FALSE otherwise
 */
bool memory_compare_const(uint8_t* lpub_src1,
                             const uint8_t* lpub_src2,                            
                             uint8_t lub_length)
{
  while (lub_length--)
  {
    /* Copy byte by byte */
    if (*lpub_src1++ != (*lpub_src2++))
    {
      return FALSE;
    }
  }
  return TRUE;
}

/**
 * \brief Calculates the 16-bit CRC of the given value
 *
 * \param[in] data_ptr  address of the data array
 * \param[in] length    number of bytes in the data array
 *
 * \return CRC_VALUE - 16-bit CRC value.
 */
uint16_t calcCRC16(uint8_t const message[], uint8_t length)
{
    uint16_t remainder = 0xFFFF;	
    
    // Perform modulo-2 division, a byte at a time.
    for (uint8_t byte = 0; byte < length; ++byte)
    {
        // Bring the next byte into the remainder.
        remainder ^= (message[byte] << (WIDTH - 8));
        
        // Perform modulo-2 division, a bit at a time.
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            //Try to divide the current data bit.
            if (remainder & (uint16_t)TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    // The final remainder is the CRC result.
    return (remainder);
}