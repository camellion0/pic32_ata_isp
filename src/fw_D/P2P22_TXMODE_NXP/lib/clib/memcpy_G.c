#include <pgmspace.h>

#pragma language=extended

/* copy char s2[n] to s1[n] in any order */
__x_z void *(memcpy_G)(void *s1, const void __generic *s2, size_t n)
{
  void * su1 = s1;

  if (n > 0)
    do {
      *(*(unsigned char **)&s1)++ = *(*(const unsigned char __generic **)&s2)++;
    } while(--n != 0);
    
  return su1;
}
