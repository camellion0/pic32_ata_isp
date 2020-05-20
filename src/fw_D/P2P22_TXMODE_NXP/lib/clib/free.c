/*                      - FREE.C -

   The ANSI "free" function.
           
   Assumes that storage is reserved in a linear place.
   Should be completely CPU-independent.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/


#include "stdlib.h"
#include "memstruc.i"           /* Describes memory space objects */
#include "inavr.h"

#pragma language=extended

#pragma segment= "HEAP" 
#define _heap_of_memory ((char *)__segment_begin("HEAP"))
#define _top_of_heap ((char *)__segment_end("HEAP"))

/*extern char PTR_ATTRIBUTE *const _heap_of_memory;*/

extern char PTR_ATTRIBUTE *_last_heap_object;

/*extern char PTR_ATTRIBUTE *const _top_of_heap;*/


/*----------------------------------------------------------------------*/
/* Releases block of memory starting at location given as argument and  */
/* if it creates two or three free blocks in the row make them one.     */
/* If there is no block begining at this location does nothing.         */
/*----------------------------------------------------------------------*/

void free(void *ptr)
{
  char PTR_ATTRIBUTE *prev = _heap_of_memory;
  char PTR_ATTRIBUTE *curr = _heap_of_memory;
  char PTR_ATTRIBUTE * local_last_heap_object;

  /* No action for null pointer  */
  if (ptr == 0)
    return;

  local_last_heap_object = _last_heap_object;
  while (curr < local_last_heap_object)
  {
    if (ptr == curr + sizeof(_m_header))        /* Block found */
    {
      _m_header PTR_ATTRIBUTE * curr_h = (_m_header PTR_ATTRIBUTE *) curr;
      if (curr_h->busy)                         /* Block busy */
      {
        _m_header PTR_ATTRIBUTE *prev_h = (_m_header PTR_ATTRIBUTE *) prev;
                                                /* Header of previous block */
        _m_header PTR_ATTRIBUTE * next_h =
                            curr_h->next < local_last_heap_object ?
                            (_m_header PTR_ATTRIBUTE *) curr_h->next : curr_h;
                                                /* If no next exists make next
                                                   equal current */
                                                /* Header of next block */
        if (!prev_h->busy)                      /* If previous block free */
        {
          prev_h->next = next_h->busy ? curr_h->next : next_h->next;
          curr_h = prev_h;
        }
        else
        {
          if (!next_h->busy)
            curr_h->next = next_h->next;
          curr_h->busy = 0;
        }
        if (curr_h->next == local_last_heap_object)
                                        /* Update _last_heap_object pointer */
          _last_heap_object = (char PTR_ATTRIBUTE*) curr_h;
      }
      return;
    }
    prev = curr;
    curr = ((_m_header PTR_ATTRIBUTE*) curr)->next;
  }
}
