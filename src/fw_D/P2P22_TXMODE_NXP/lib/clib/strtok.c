/*      - STRTOK.C -

   The ANSI "strtok" function.


   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "string.h"

char *strtok(char *s1, const char *s2)
{
  /* find next token in s1[] delimited by s2[] */
  char *sbegin, *send;
  static char *ssave = "";      /* for safety */

  sbegin = s1 ? s1 : ssave;
  sbegin += strspn(sbegin, s2);
  if (*sbegin == '\0')
  {
    /* end of scan */
    ssave = ""; /* for safety */
    return 0;
  }
  send = sbegin + strcspn(sbegin, s2);
  if (*send != '\0')
    *send++ = '\0';
  ssave = send;
  return sbegin;
}
