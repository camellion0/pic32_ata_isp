/*                      - REALLOC.C -

   The ANSI "realloc" function.
           
   Assumes that storage is reserved in a linear place.
   Should be completely CPU-independent.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"
#include "memstruc.i"           /* Describes memory space objects */
#include "inavr.h"

#pragma segment="HEAP"
#define _heap_of_memory ((char *)__segment_begin("HEAP"))
#define _top_of_heap ((char *)__segment_end("HEAP"))

extern char PTR_ATTRIBUTE *_last_heap_object;

extern void _make_new_mem_hole(_m_header PTR_ATTRIBUTE *head_p,
                               char PTR_ATTRIBUTE *ptr);

#pragma language=extended
#pragma optimize=no_inline
static __x_z void _mem_cpy(char PTR_ATTRIBUTE * dst,
                           char PTR_ATTRIBUTE * src,
                           int                  len)
{
  if (len != 0)
  {
    do
    {
      *dst++ = *src++;
    } while(--len);
  }
}

/*----------------------------------------------------------------------*/
/* Reallocates memory starting at location given as first argument to   */
/* size given as second argument.  The contents of the object is        */
/* unchanged to lesser of the new and old size.  Returns pointer to     */
/* the first byte after block header if request for memory satisfied or */
/* NULL otherwise.                                                      */
/*----------------------------------------------------------------------*/

void *realloc(void *ptr_par, size_t n)
{
  char PTR_ATTRIBUTE *curr = _heap_of_memory;
  char PTR_ATTRIBUTE *new;
  _m_header PTR_ATTRIBUTE *curr_h; 
  _m_header PTR_ATTRIBUTE *next_h;
  void PTR_ATTRIBUTE *ptr = (char PTR_ATTRIBUTE *)ptr_par;
  char PTR_ATTRIBUTE * local_last_heap_object;

  /*----------------------------------------------*/
  /* If nowhere to reallocate, behave like malloc */
  /*----------------------------------------------*/     
  if (!ptr)
  {
    return malloc(n);
  }

  /*----------------------------------------------------*/
  /* If the new size is zero and ptr exists, deallocate */
  /*----------------------------------------------------*/
  else if (n == 0)
  {
    free(ptr);         /* free ignores illegal pointers */
    return 0;
  }
     
  /*---------------------------------------------*/
  /* If alignment required round up n to nearest */
  /* __MAX_ALIGNMENT__ multiple                  */
  /*---------------------------------------------*/

  n += __MAX_ALIGNMENT__ - 1;
  n &= ~(__MAX_ALIGNMENT__ - 1);

  local_last_heap_object = _last_heap_object;
  while (curr < local_last_heap_object)
  {
    if (ptr == curr + sizeof(_m_header))            /* Block found */
    {
      curr_h = (_m_header PTR_ATTRIBUTE *) curr;
      if (curr_h->busy)                             /* Block busy */
      {
        if (curr_h->next - (char PTR_ATTRIBUTE *) ptr > n)
        {                                             /* Make it smaller */
          if (curr_h->next == local_last_heap_object)      /* It is the last one */
          {
            _last_heap_object = curr_h->next = (char PTR_ATTRIBUTE *) ptr + n;
          }
          else if (next_h = (_m_header PTR_ATTRIBUTE *) curr_h->next,
                   !next_h->busy)                     /* Next block is free */
          {
            curr_h->next = (char PTR_ATTRIBUTE *) ptr + n;
            curr_h = (_m_header PTR_ATTRIBUTE *) curr_h->next;
            curr_h->busy = 0;
            curr_h->next = next_h->next;
          }
          else
          {
            _make_new_mem_hole(curr_h, (char PTR_ATTRIBUTE *) ptr + n);
          }
        }
        else if (curr_h->next - (char PTR_ATTRIBUTE *) ptr < n)
        {
          /* Make it bigger */

          /* Is it the last block and do we have room in the free area
             to enlarge it? */
          if((char *) ptr > (char PTR_ATTRIBUTE *) ptr + n)    /* Check wrap around */
          {
            return 0;
          }
          else if (   curr_h->next == local_last_heap_object
                   && (char PTR_ATTRIBUTE *) ptr + n <= _top_of_heap)
          {  /* Move high limit */
            _last_heap_object = curr_h->next =
              (char PTR_ATTRIBUTE *) ptr + n;
          }
          /* Or can we take a bit of memory from the following block? */
          else if (   next_h = (_m_header PTR_ATTRIBUTE *) curr_h->next,
                      !next_h->busy                  /* Next one is free */
                   && next_h->next - (char PTR_ATTRIBUTE *) ptr > n
                   && curr_h->next != local_last_heap_object)
          {
            /* Together is enough */
            curr_h->next = next_h->next;             /* Join them */
            _make_new_mem_hole(curr_h, (char PTR_ATTRIBUTE *) ptr + n);
          }
          /* Else malloc a new block, copy, then free old. */
          else if (new = (char PTR_ATTRIBUTE *) malloc(n))
          {                                   /* If there is space somewhere */
            int count = curr_h->next - (char PTR_ATTRIBUTE *) ptr;
            _mem_cpy(new, ptr, count);
            free(ptr);
            ptr = new;
          }
          else
            return 0;
        }
        return ptr;
      }
      else
      {
        /*--------------------------------------------------*/
        /* Found a deallocated block.  'undefined behavior' */
        /* ANSI says, but we are nice guys so we will give  */
        /* a warning.                                       */
        /*--------------------------------------------------*/
        return 0;
      }
    }
    curr = ((_m_header PTR_ATTRIBUTE *) curr)->next;
  }
  return 0;
}

