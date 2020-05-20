/*                               - HEAP.C -

   Storage for "malloc".

   To modify the heap size, see the user documentation

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "memstruc.i"

/* a pointer to still free heap memory, */
char PTR_ATTRIBUTE *_last_heap_object = 0;
