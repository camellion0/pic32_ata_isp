#include <pgmspace.h>

#pragma language=extended

/* copy char s2[max n] to end of s1[] */
__x_z char *(strncat_G)(char *s1, const char __generic *s2, size_t n)
{
  char *s = s1;
  char  c;
  
  while(*s1++ != '\0')
    ;                   /* find end of s1[] */
  --s1;
  while(n != 0 && (c = *s2++) != '\0')
  {
    *s1++ = c;
    --n;
  }
  *s1 = '\0';
  return s;
}
