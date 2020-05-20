#include <pgmspace.h>

#pragma language=extended

/* compare unsigned char s1[], s2[] */
__x_z int (strcmp_G)(const char *s1, const char __generic *s2)
{
  unsigned char c1, c2;
  
  while((c1 = *s1++) == (c2 = *s2++))
    if (c1 == '\0')
      return 0;
    
  return c1 < c2 ? -1 : +1;
}
