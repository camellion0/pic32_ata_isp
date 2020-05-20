/*                      - MEMCPY_P.C -

   A variant of the ANSI "memcpy" function.
           
   Version: 1.03 [IANP] (from memcpy 3.20)

*/

#include "pgmspace.h"

#pragma language=extended
 
/* copy char s2[n] to s1[n] in any order */
__x_z void *(memcpy_P)(void *s1, PGM_VOID_P s2, size_t n)
{
  void * su1 = s1;

  if (n > 0)
    do {
      *(*(unsigned char **)&s1)++ = *(*(PGM_P *)&s2)++;
    } while(--n != 0);
    
  return su1;
}
