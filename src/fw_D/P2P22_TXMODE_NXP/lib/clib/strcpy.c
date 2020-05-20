/*                      - STRCPY.C -

   The ANSI "strcpy" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

char *strcpy (char *s1, const char *s2 )
{
#ifdef _INTRINSIC
  return strcpy(s1, s2);
#else
  char *v = s1;
  while ( *s1++ = *s2++ )             /* Copy s2 to s1 */
    ;
  return v;
#endif
}
