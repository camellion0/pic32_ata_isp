/*                      - ICCLBUTL.H -

   Low-level declarations for non-ANSI functions
   used by the C library.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#ifndef _ICCLBUTL_INCLUDED
#define _ICCLBUTL_INCLUDED

#ifndef _SYSTEM_BUILD
  #pragma system_include
#endif

#include "sysmac.h"
#include "stdarg.h"

    /*-----------------------------------------*/
    /* Formatters used by "scanf" and "sscanf" */
    /*-----------------------------------------*/
#if __IAR_SYSTEMS_ICC__ < 2
#if __TID__ & 0x8000
#pragma function=intrinsic(0)
#endif
#endif

#ifndef MEMORY_ATTRIBUTE
#define MEMORY_ATTRIBUTE
#endif

/* Full ANSI (parameters are line, format, ap) */
__INTRINSIC MEMORY_ATTRIBUTE int _formatted_read(const char **,
                                                 const char **,
                                                 va_list);

/* Without floating point */
__INTRINSIC MEMORY_ATTRIBUTE int _medium_read(const char **,
                                              const char **,
                                              va_list);


    /*-------------------------------------------*/
    /* Formatters used by "printf" and "sprintf" */
    /*-------------------------------------------*/

/* Full ANSI (parameters are format, output-function, secret-pointer, ap) */
__INTRINSIC MEMORY_ATTRIBUTE int _formatted_write(const char *,
                                                  void (*)(char, void *),
                                                  void *,
                                                  va_list);

/* Without floating point */
__INTRINSIC MEMORY_ATTRIBUTE int _medium_write(const char *,
                                               void (*)(char, void *),
                                               void *,
                                               va_list);

/* Very reduced version */
__INTRINSIC MEMORY_ATTRIBUTE int _small_write(const char *,
                                              void (*)(char, void *),
                                              void *,
                                              va_list);

#if __IAR_SYSTEMS_ICC__ < 2
#if __TID__ & 0x8000
#pragma function=default
#endif
#endif

#endif
