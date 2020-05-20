/*                      - MEMSTRUC.I -

   Include file used by MALLOC.C, FREE.C etc.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#ifndef MEMSTRUC_I
#define MEMSTRUC_I

#ifndef _SYSTEM_BUILD
  #pragma system_include
#endif

#ifndef MEM_ATTRIBUTE
#define MEM_ATTRIBUTE
#endif

#ifndef PTR_ATTRIBUTE
#define PTR_ATTRIBUTE MEM_ATTRIBUTE
#endif

/* The header for each allocated region on the heap */
typedef struct
{
  char busy;
  char PTR_ATTRIBUTE *next;
} _m_header;

/* A union that is here to get the maximum alignment bucket */
typedef union
{
  long l;
  long double ld;
  char *p;
} __align_union__;

/* Get number of align bucket from number of bytes in the heap */
#define __HEAP_SIZE__(x) ((x) / sizeof(__align_union__))

#ifndef __MAX_ALIGNMENT__
/* Note, this macro must be used in run-time code */
#if __IAR_SYSTEMS_ICC__ < 2 || !defined(__ALIGNOF__)
#define __ALIGNOF__(type) (sizeof(struct {type y; char a;}) - sizeof(type))
#endif
#define __MAX_ALIGNMENT__ (__ALIGNOF__(__align_union__))
#endif

#endif /* MEMSTRUC_I */
