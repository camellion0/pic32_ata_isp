/*                      - ICCLBUTL_P.H -

   Low-level declarations for non-ANSI functions that 
   use program address space format strings
   used by the C library.

   Version: 1.03 [IANP] (from icclbutl.h 3.30)

*/

#ifndef _ICCLBUTL_P_INCLUDED
#define _ICCLBUTL_P_INCLUDED

#include "stdarg.h"
#include "pgmspace.h"

#ifdef  __IAR_SYSTEMS_ICC__
#ifndef _SYSTEM_BUILD
#pragma system_include
#endif
#endif

#ifndef MEMORY_ATTRIBUTE
#define MEMORY_ATTRIBUTE
#endif

#pragma language=save
#pragma language=extended

                /*-----------------------------------------*/
                /* Formatters used by "scanf" and "sscanf" */
                /*-----------------------------------------*/

/* Full ANSI formatting */
MEMORY_ATTRIBUTE int _formatted_read_P(const char **__line,
                                       PGM_P*__format,
                                       va_list __ap);

/* Without floating point */
MEMORY_ATTRIBUTE int _medium_read_P(const char **__line, 
                                    PGM_P*__format,
                                    va_list __ap);

                /*-------------------------------------------*/
                /* Formatters used by "printf" and "sprintf" */
                /*-------------------------------------------*/

/* Full ANSI formatting */
MEMORY_ATTRIBUTE int _formatted_write_P(PGM_P __format,
                                        void __outputf(char, void *),
                                        void *__sp,
                                        va_list __ap);
/* Without floating point */
MEMORY_ATTRIBUTE int _medium_write_P(PGM_P __format,    
                                     void __outputf(char, void *),
                                     void *__sp,
                                     va_list __ap);

/* Very reduced version */
MEMORY_ATTRIBUTE int _small_write_P(PGM_P __format,
                                    void __outputf(char, void *),
                                    void *__sp,
                                    va_list __ap);

#pragma language=restore

#endif
