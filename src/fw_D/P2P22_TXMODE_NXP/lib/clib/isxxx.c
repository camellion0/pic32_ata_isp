/*                      - ISXXX.C -

   The ANSI character testing functions.

   Note: 7-bit ASCII or 8-bit ASCII are supported by these functions
         by defining an appropiate macro in ctype.h

   The functions takes 'int' as argument, but the value shall be representable
   by 'unsigned char' [0,255] or the value of the macro EOF, otherwise the
   behavior is undefined (4.3 in ANSI X3.159-1989).

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include <ctype.h>


#ifndef __8_BIT_ASCII_WANTED
#error "#define __8_BIT_ASCII_WANTED, must accept 'unsigned char' arguments; ANSI requirement"
#endif

#if !(defined(COMPILE_ISALPHA) || defined(COMPILE_ISUPPER) || \
      defined(COMPILE_ISLOWER) || defined(COMPILE_ISDIGIT) || \
      defined(COMPILE_ISSPACE) || defined(COMPILE_ISPUNCT) || \
      defined(COMPILE_ISALNUM) || defined(COMPILE_ISPRINT) || \
      defined(COMPILE_ISGRAPH) || defined(COMPILE_ISCNTRL) || \
      defined(COMPILE_ISXDIGIT) )
#error "Must #define what function to compile, e.g. COMPILE_ISALPHA"
#endif

#ifdef COMPILE_ISALPHA
#ifdef isalpha
#undef isalpha
#endif

int isalpha(int c)
{
  return (_Ctype+1)[c] & (_U|_L);
}

#endif

#ifdef COMPILE_ISUPPER
#ifdef isupper
#undef isupper
#endif

int isupper(int c)
{
  return (_Ctype+1)[c] & _U;
}

#endif

#ifdef COMPILE_ISLOWER
#ifdef islower
#undef islower
#endif

int islower(int c)
{
  return (_Ctype+1)[c] & _L;
}

#endif

#ifdef COMPILE_ISDIGIT
#ifdef isdigit
#undef isdigit
#endif

int isdigit(int c)
{
  return (_Ctype+1)[c] & _N;
}

#endif

#ifdef COMPILE_ISXDIGIT
#ifdef isxdigit
#undef isxdigit
#endif

int isxdigit(int c)
{
  return (_Ctype+1)[c] & (_N|_X);
}

#endif

#ifdef COMPILE_ISSPACE
#ifdef isspace
#undef isspace
#endif

int isspace(int c)
{
  return (_Ctype+1)[c] & _S;
}

#endif

#ifdef COMPILE_ISPUNCT
#ifdef ispunct
#undef ispunct
#endif

int ispunct(int c)
{
  return (_Ctype+1)[c] & _P;
}

#endif

#ifdef COMPILE_ISALNUM
#ifdef isalnum
#undef isalnum
#endif

int isalnum(int c)
{
  return (_Ctype+1)[c] & (_U|_L|_N);
}

#endif

#ifdef COMPILE_ISPRINT
#ifdef isprint
#undef isprint
#endif

int isprint(int c)
{
  return (_Ctype+1)[c] & (_P|_U|_L|_N|_B);
}

#endif

#ifdef COMPILE_ISGRAPH
#ifdef isgraph
#undef isgraph
#endif

int isgraph(int c)
{
  return (_Ctype+1)[c] & (_P|_U|_L|_N);
}

#endif

#ifdef COMPILE_ISCNTRL
#ifdef iscntrl
#undef iscntrl
#endif

int iscntrl(int c)
{
  return (_Ctype+1)[c] & _C;
}

#endif
