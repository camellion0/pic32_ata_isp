#include <pgmspace.h>

#pragma language=extended

/* find length of s[] */
__z size_t (strlen_G)(const char __generic *s)
{
  const char __generic *sc = s;
  
  while(*s++ != '\0')
    ;
  
  return s - sc - 1;
}
