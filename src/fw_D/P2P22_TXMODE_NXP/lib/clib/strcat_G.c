#include <pgmspace.h>

#pragma language=extended

/* copy char s2[] to end of s1[] */
__x_z char *(strcat_G)(char *s1, const char __generic *s2)
{
  char *s = s1;
  
  while(*s1++ != '\0')
    ;                   /* find end of s1[] */
  --s1;
  while((*s1++ = *s2++) != '\0')
    ;                   /* copy s2[] to end */
  return s;
}
