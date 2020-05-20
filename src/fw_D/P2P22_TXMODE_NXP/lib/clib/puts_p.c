/*                      - PUTS_P.C -

   A variant of the ANSI "puts" function.

   Version: 1.03 [IANP] (from puts 3.10)

*/

#include "stdio.h"
#include "pgmspace.h"

#pragma language=extended

int puts_P (PGM_P s)  
  {
    while (*s)
      {
        if (!( putchar(*s++)))
          return(EOF);
      }
    putchar('\n');
    return (1);
  }
