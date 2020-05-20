#include <pgmspace.h>

#pragma language=extended

/* compare unsigned char s1[max n], s2[max n] */
__x_z int (strncmp_G)(const char *s1, const char __generic *s2, size_t n)
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
