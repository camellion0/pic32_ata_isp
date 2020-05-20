/*                      - STRNCPY_P.C -

   A variant of the ANSI "strncpy" function.
           
   Version: 1.03 [IANP] (from strncpy 3.20)
           
*/

#include "pgmspace.h"

#pragma language=extended

/* copy char s2[max n] to s1[n] */
__x_z char *(strncpy_P)(char *s1, PGM_P s2, size_t n)
{
  char *s = s1;
  char c;

  /* copy at most n chars from s2[] */
  for(c = 1; n != 0; --n)
  {
    c = c ? *s2++ : 0;
    *s1++ = c;
  }

  return s;
}
