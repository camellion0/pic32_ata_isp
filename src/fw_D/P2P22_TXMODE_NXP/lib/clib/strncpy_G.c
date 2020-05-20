#include <pgmspace.h>

#pragma language=extended

/* copy char s2[max n] to s1[n] */
__x_z char *(strncpy_G)(char *s1, const char __generic *s2, size_t n)
{
  char *s = s1;
  char c;

  /* copy at most n chars from s2[] */
  for(c = 1; n != 0; --n)
  {
    c = c ? *s2++ : 0;
    *s1++ = c;
  }

  return s;
}
