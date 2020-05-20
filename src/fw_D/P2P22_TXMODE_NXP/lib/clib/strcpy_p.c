/*                      - STRCPY_P.C -

   A variant of the ANSI "strcpy" function.
           
   Version: 1.03 [IANP] (from strcpy 3.20)

*/

#include "pgmspace.h"

#pragma language=extended

/* copy char s2[] to s1[] */
__x_z char *(strcpy_P)(char *s1, PGM_P s2)
{
  char *s = s1;
  
  for(; (*s1++ = *s2++) != '\0'; )
    ;
  return s;
}
