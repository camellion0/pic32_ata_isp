/*                      - ICCFLOAT.H -

   Low-level float declarations used by the float/double C library.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#ifndef _ICCFLOAT_INCLUDED
#define _ICCFLOAT_INCLUDED

#include "sysmac.h"

#if __IAR_SYSTEMS_ICC__ < 2
#if __TID__ & 0x8000
#pragma function=intrinsic(0)
#endif
#endif


    /*-------------------------------------------*/
    /* Intrinsics used by float/double-functions */
    /*-------------------------------------------*/


    /*----------------------------------------*/
    /* Optimizations on float:                */
    /*                                        */
    /* A common prelude in a float-routine    */
    /* will be:                               */
    /*   sign = arg < 0.0;                    */
    /*  arg = fabs(arg);                      */
    /* therefore to have fabs as intrinsic is */
    /* good (only zero sign-bit), as well as  */
    /* optimize compares on float against     */
    /* zero.                                  */
    /* A few more intrinsic routines needed   */
    /* are listed below. Note that none of    */
    /* the functions below check for zero     */
    /*----------------------------------------*/

#if __FLOAT_SIZE__ == __DOUBLE_SIZE__

__INTRINSIC signed char __dgetexp(double arg);
                                        /* Extract the exponent, so that */
                                        /* 2 raised to the exponent times */
                                        /* the mantissa becomes the */
                                        /* argument, if the mantissa is */
                                        /* [0.5,1.0) */
__INTRINSIC double __dnormexp(double arg);
                                        /* Set the exponent so that the */
                                        /* argument is [0.5,1.0) */
__INTRINSIC double __daddexp(double arg,signed char exp);
                                        /* Add the exp to the one in the */
                                        /* argument, not checking for over- */
                                        /* under-flow */
#else

__INTRINSIC signed short __dgetexp(double arg);
__INTRINSIC double __dnormexp(double arg);
__INTRINSIC double __daddexp(double arg,signed short exp);

#endif



extern double __satan(double);
extern double __sinus(double,unsigned char);



#if __IAR_SYSTEMS_ICC__ < 2
#if __TID__ & 0x8000
#pragma function=default
#endif
#endif

#endif /* _ICCFLOAT_INCLUDED */
