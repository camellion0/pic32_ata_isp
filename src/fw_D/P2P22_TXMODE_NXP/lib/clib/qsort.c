/*                      - QSORT.C -

   The ANSI "qsort" function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include <stddef.h>
#include <string.h>

char *_qbuf = NULL;

#define PIVOT     ((i + j) >> 1)

#define Mov_item(dst,src,size)  if ((dst) != (src)) memcpy((dst),(src),(size));

#ifndef QSORT_MAXSIZE
#define QSORT_MAXSIZE 128
#endif

static void _nqsort(char *base,
                    int lo, int hi,
                    int size,
                    int (*cmp)(const void *, const void *) )
{
  int i, j;
  char *p = _qbuf;

  while(hi > lo)
  {
    i = lo;
    j = hi;
    p = base + size * PIVOT;
    Mov_item(_qbuf, p, size);
    Mov_item(p, base + size * i, size);
    Mov_item(base + size * i, _qbuf, size);
    p = _qbuf;
    while (i < j)
    {
      while (((*cmp)(base + size * j, p)) > 0)
        --j;
      Mov_item(base + size * i, base + size * j, size);
      while ((i < j) && (((*cmp)(base + size * i, p)) <= 0))
        ++i;
      Mov_item(base + size * j, base + size * i, size);
    }
    Mov_item(base + size * i, p, size);
    if ((i - lo) < (hi - i))
    {
      _nqsort(base, lo, i - 1, size, cmp);
      lo = i + 1;
    }
    else
    {
      _nqsort(base, i + 1, hi, size, cmp);
      hi = i - 1;
      }
    }
}


/*
 * qsort: Not re-entrant, since it uses global '_qbuf'
 */
void qsort(void *base, size_t nmemb, size_t size,
           int (*compar)(const void *, const void *))
{
  char _qtemp[QSORT_MAXSIZE];

  if (size > sizeof(_qtemp)) /* records too large! */
    return;
  _qbuf = _qtemp;

  _nqsort(base, 0, nmemb-1, size, compar);
}
