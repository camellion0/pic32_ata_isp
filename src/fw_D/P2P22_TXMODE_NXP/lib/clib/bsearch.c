/*                      - BSEARCH.C -

   The ANSI "bsearch" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"


typedef int CompareFcn(const void *, const void *);


void *bsearch(const void *key, const void *base, size_t nmemb, size_t size,
              CompareFcn *cmp_fn)
{
#ifdef _INTRINSIC
  return bsearch(key, base, nmemb, size, cmp_fn);
#else
  const char *upper;
  const char *lower = (char *) base;
  size_t i;                           /* Always an unsigned type */
  int j;

  while (nmemb > 0)
  {
    i = nmemb / 2;
    upper = lower + i*size;

    if ( (j = (*cmp_fn)(key, upper)) == 0)
    {
      /*=======*/
      /* Found */
      /*=======*/
      return (void *) upper;
    }
    else if (j < 0)
    {
      /*=============================*/
      /* Search in [lower ... upper] */
      /*=============================*/
      nmemb = i;
    }
    else
    {
      /*========================*/
      /* Search in ]upper, EOA] */
      /*========================*/
      lower = upper + size;
      nmemb = nmemb - i - 1;
    }
  }
  return ( void *) 0;
#endif
}
