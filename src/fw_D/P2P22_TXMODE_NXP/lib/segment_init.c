/**************************************************************
 **             - __segment_init.c -
 **
 **     Segment initialization that must be
 **     performed before main is called.
 **
 **     Used with iccAVR.
 **
 **     Copyright 1999 IAR Systems AB. All rights reserved.
 **
 **     $Revision: 328482 $
 **
 **************************************************************/
#include "segment_init.h"
#include <intrinsics.h>

/* We're using extended keywords in this file. Make sure */
/* that the compiler has IAR extensions enabled.         */
#pragma language=extended

/* If you are using C or EC++ code then you must include  */
/* the segment initialization code below or supply your   */
/* own version of segment initialization code. Otherwise  */
/* we cannot guarantee that any C or EC++ code will work. */
/* If you're only using assembler modules then you don't  */
/* need to initialize any segments at all.                */

#ifdef __cplusplus
extern "C" {
#endif

/* __segment_init */

#pragma optimize=no_inline
static __x void __memclr(DstPtr_Type dst, Counter_Type size)
{
  do {
    *dst++ = 0;
  } while(--size != 0);
}

#pragma optimize=no_inline
static __x_z void __flashcpy(DstPtr_Type dst, SrcPtr_Type src, Counter_Type size)
{
  do {
    *dst++ = *src++;
  } while(--size != 0);
}

__C_task void __segment_init(void)
{
  SegmentInitBlockPtr_Type InitTable;

  /* Initialize the INITTAB pointer. */
  InitTable = __segment_begin("INITTAB");

  /* Loop over all copy/zero blocks in the INITTAB segment */
  do
  {
    /* Fetch initialization data from the INITTAB segment. */
    Counter_Type  size = InitTable->Cnt;
    DstPtr_Type   dst  = InitTable->Dst;
    SrcPtr_Type   src  = InitTable->Src;

    /* If the src is 0 then we should clear a memory */
    /* block, otherwise it's a copy operation.       */
    if (src != 0)
      __flashcpy(dst, src, size);
    else
      __memclr(dst, size);

    /* Advance the INITTAB pointer. */
    ++InitTable;
  }  while(InitTable < __segment_end("INITTAB"));
}

#ifdef __cplusplus
}
#endif
