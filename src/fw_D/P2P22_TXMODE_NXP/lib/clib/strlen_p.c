/*                      - STRLEN_P.C -

   A variant of the ANSI "strlen" function.
           
   Version: 1.03 [IANP] (from strlen 3.20)
           
*/

#include "pgmspace.h"

#pragma language=extended

size_t strlen_P (PGM_P s)
  {
    PGM_P p = s;

    while (*p)
      p++;
    return (p-s);
  }

