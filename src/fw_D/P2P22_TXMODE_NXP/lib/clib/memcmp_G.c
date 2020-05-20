#include <pgmspace.h>

#pragma language=extended
/* compare unsigned char s1[n], s2[n] */
__x_z int (memcmp_G)(const void *s1, const void __generic *s2, size_t n)
{
  unsigned char uc1, uc2;

  if (n != 0)
    do {
      uc1 = *(*(const unsigned char           **)&s1)++;
      uc2 = *(*(const unsigned char __generic **)&s2)++;
      if (uc1 != uc2)
        return uc1 < uc2 ? -1 : 1;
    } while(--n);

  return n;
}
