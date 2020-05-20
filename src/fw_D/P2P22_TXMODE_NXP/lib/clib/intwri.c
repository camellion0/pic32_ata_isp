/*                      - INTWRI.C -

   Reduced version of the "printf" function.

   This module can be adopted to user-defined routines that needs formatters
   with special properties like different output channels or new format
   specifiers.

   This reduced version of formatter is suitable when program size is critical
   rather than formatting power.  This routine uses less than 20 bytes of
   stack space which makes it practical even in systems with less than 256
   bytes of user RAM.

   The only formatting specifiers supported are:

            %%  %o  %d  %c  %s  %x

   It means that "long" or real variables are not supported as well as field
   width and precision arguments.  Return value is always zero.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "stdio.h"
#include "stdarg.h"
#include "sysmac.h"

int printf(const char *format, ...)
{
  static const char hex[] = "0123456789ABCDEF";
  char format_flag;
  unsigned int u_val, div_val, base;
  char *ptr;
  va_list ap;

  va_start(ap, format);

  for (;;)    /* Until full format string read */
  {
    while ((format_flag = *format++) != '%')      /* Until '%' or '\0' */
    {
      if (!format_flag)
      {
        va_end(ap);
        return 0;
      }
      putchar(format_flag);
    }

    switch (format_flag = *format++)
    {
    case 'c':
      format_flag = va_arg(ap, int);
    default:
      putchar(format_flag);
      continue;

    case 's':
      ptr = VAPTR(char);
      while (format_flag = *ptr++)
      {
        putchar(format_flag);
      }
      continue;

    case 'o':
      base = 8;
      if (sizeof(int) == 2)
        div_val = 0x8000;
      else
        div_val = 0x40000000;
      goto CONVERSION_LOOP;

    case 'd':
      base = 10;
      if (sizeof(int) == 2)
        div_val = 10000;
      else
        div_val = 1000000000;
      goto CONVERSION_LOOP;

    case 'x':
      base = 16;
      if (sizeof(int) == 2)
        div_val = 0x1000;
      else
        div_val = 0x10000000;

CONVERSION_LOOP:
      u_val = va_arg(ap,int);
      if (format_flag == 'd')
      {
        if (((int)u_val) < 0)
        {
          u_val = - u_val;
          putchar('-');
        }
        while (div_val > 1 && div_val > u_val)
        {
          div_val /= 10;
        }
      }

      do
      {
        putchar(hex[u_val / div_val]);
        u_val %= div_val;
        div_val /= base;
      }
      while (div_val);
    }
  }
}
