#include <pgmspace.h>

#pragma language=extended
/* compare unsigned char s1[n], s2[n] */
__x_z int (memcmp_P)(const void *s1, PGM_VOID_P s2, size_t n)
{
  unsigned char uc1, uc2;

  if (n != 0)
    do {
      uc1 = *(*(const unsigned char          **)&s1)++;
      uc2 = *(*(PGM_P*)&s2)++;
      if (uc1 != uc2)
        return uc1 < uc2 ? -1 : 1;
    } while(--n);

  return n;
}
