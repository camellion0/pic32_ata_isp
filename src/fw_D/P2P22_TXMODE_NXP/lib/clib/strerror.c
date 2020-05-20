/*                      - STRERROR.C -

   The ANSI "strerror" function.

   Error messages given: 0                 : No error;
                         EDOM              : domain error;
                         ERANGE            : range error
                         all other numbers : error no XX

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "errno.h"

#define Max_err_num 99   /* defines maximal error numbers allowed */

static char buf[] = {"error No.xx"};  /* valid error with no error message */

char *strerror(int errnum)
{
 switch (errnum)
 {
   case EZERO:                 /* errors defined in errno.h */
     return "no error";
   case EDOM:
     return "domain error";
   case ERANGE:
     return "range error";
   default:
     if (errnum < 0 || errnum > Max_err_num)  /* error number out of range */
       return "unknown error";  /* the error message may be changed to NULL */
     else
     {
       buf[10] = errnum % 10 + '0';
       buf[9] = (errnum / 10) % 10 + '0';
       return buf;
       }
   }
}
