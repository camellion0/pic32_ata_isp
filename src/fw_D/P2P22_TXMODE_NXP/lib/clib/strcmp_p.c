/*                      - STRCMP_P.C -

   A variant of the ANSI "strcmp" function.
           
   Version: 1.03 [IANP] (from strcmp 3.20)
           
*/

#include "pgmspace.h"

#pragma language=extended

/* compare unsigned char s1[], s2[] */
__x_z int (strcmp_P)(const char *s1, PGM_P s2)
{
  unsigned char c1, c2;
  
  while((c1 = *s1++) == (c2 = *s2++))
    if (c1 == '\0')
      return 0;
    
  return c1 < c2 ? -1 : +1;
}
