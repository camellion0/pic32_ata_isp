#include <pgmspace.h>

#pragma language=extended

/* copy char s2[] to end of s1[] */
__x_z char *(strcat_P)(char *s1, PGM_P s2)
{
  char *s = s1;
  
  for (; *s1++ != '\0';)
    ;                   /* find end of s1[] */
  --s1;
  for (; (*s1++ = *s2++) != '\0'; )
    ;                   /* copy s2[] to end */
  return s;
}

