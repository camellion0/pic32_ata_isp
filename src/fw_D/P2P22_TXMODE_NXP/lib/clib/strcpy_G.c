#include <pgmspace.h>

#pragma language=extended

/* copy char s2[] to s1[] */
__x_z char *(strcpy_G)(char *s1, const char __generic *s2)
{
  char *s = s1;
  
  for(; (*s1++ = *s2++) != '\0'; )
    ;
  return s;
}
