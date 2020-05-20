/*                      - STRNCMP_P.C -

   A variant of the ANSI "strncmp" function.
           
   Version: 1.03 [IANP] (from strncmp 3.20)

*/

#include "pgmspace.h"

#pragma language=extended

/* compare unsigned char s1[max n], s2[max n] */
__x_z int (strncmp_P)(const char *s1, PGM_P s2, size_t n)
{
  char c1, c2;

  for(; n != 0; --n)
  {
    c1 = *s1++;
    c2 = *s2++;
    if (c1 != c2)
      return c1 < c2 ? -1 : 1;
    else if (c1 == 0)
      return 0;
  }

  return n;
}
