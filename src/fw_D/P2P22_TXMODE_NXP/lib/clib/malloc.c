/*                      - MALLOC.C -

   The ANSI "malloc" function.
           
   Assumes that storage is reserved in a linear place.
   Should be completely CPU-independent.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdlib.h"
#include "memstruc.i"           /* Describes memory space objects */
#include "inavr.h"

#pragma language=extended

#pragma segment="HEAP"
#define _heap_of_memory ((char *)__segment_begin("HEAP"))
#define _top_of_heap ((char *)__segment_end("HEAP"))

/*extern char PTR_ATTRIBUTE *const _heap_of_memory;*/    /* Beginning of heap */

extern char PTR_ATTRIBUTE *_last_heap_object;        /* Current heap pointer */

/*extern char PTR_ATTRIBUTE *const _top_of_heap;*/      /* End of heap */

/*----------------------------------------------------------------------*/
/* If there are free bytes at location pointed to by head_p in quantity */
/* bigger then block header creates free block.                         */
/*----------------------------------------------------------------------*/

void _make_new_mem_hole(_m_header PTR_ATTRIBUTE *head_p, 
                        char PTR_ATTRIBUTE *ptr)
{
  char PTR_ATTRIBUTE *p;
    
  if (head_p->next - ptr > sizeof(_m_header))
  {
    p = head_p->next;
    head_p->next = ptr;
    head_p = (_m_header PTR_ATTRIBUTE *) ptr;
    head_p->busy = 0;
    head_p->next = p;
  }
}

/*----------------------------------------------------------------------*/
/* Tries to allocate n bytes of memory.   First looks for memory after  */
/* last busy block and if there is enough space allocates them there.   */
/* If there is not it searches the blocks from begining to find big     */
/* enough free block. First fit is accepted.  Returns pointer to the    */
/* first byte after block header if request for memory satisfied or     */
/* NULL otherwise.                                                      */
/*----------------------------------------------------------------------*/

void *malloc(size_t n)
{
  char PTR_ATTRIBUTE *ptr;
  char PTR_ATTRIBUTE *ret_ptr;
  _m_header PTR_ATTRIBUTE *head_p;
  char PTR_ATTRIBUTE * local_last_heap_object;

  /*==========================================*/
  /* If alignment required round up           */
  /* n to nearest __MAX_ALIGNMENT__ multiple  */
  /*==========================================*/

  n += __MAX_ALIGNMENT__ - 1;
  n &= ~(__MAX_ALIGNMENT__ - 1);

  local_last_heap_object = _last_heap_object;
  if (local_last_heap_object == 0)
  {
    local_last_heap_object = _heap_of_memory;
    _last_heap_object = local_last_heap_object;
  }

  if (n + sizeof(_m_header) <= _top_of_heap - local_last_heap_object)
  {
    /*=================================*/
    /* There is still space at the top */
    /*=================================*/
    head_p = (_m_header PTR_ATTRIBUTE *) local_last_heap_object;
    head_p->busy = 1;           /* Set busy */
    head_p->next = local_last_heap_object += n + sizeof(_m_header);
                                /* Calculate address of next free block */
    _last_heap_object = local_last_heap_object;
    return local_last_heap_object - n;
  }
  else
  {
    /*=================*/
    /* Look for a hole */
    /*=================*/
    ptr = _heap_of_memory;
    while (ptr < local_last_heap_object)
    {
      head_p = (_m_header PTR_ATTRIBUTE *) ptr;
      if (   ! head_p->busy
          && (head_p->next - ptr) - sizeof(_m_header) >= n )
      {
        /*========================*/
        /* Found suitable hole... */
        /*========================*/
        ret_ptr = ptr + sizeof(_m_header);
        head_p->busy = 1;
        /*===================================*/
        /* If there is space left - leave    */
        /* it as a hole so create new header */
        /*===================================*/
        _make_new_mem_hole(head_p, ret_ptr + n);
        return ret_ptr;
      }
      ptr = head_p->next;
    }
    /*=======================*/
    /* Sorry, no memory left */
    /*=======================*/
    return 0;
  }
}


