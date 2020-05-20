/*            - FRMWRI.C -

   Basic "printf", "sprintf" and "fprintf" formatter.

   This module is 100% reentrant and can be adapted to user-defined routines
   that needs formatters with special properties like different output chann-
   els or new format specifiers.

   To reduce size in applications not using real numbers or long integers
   the formatter may be compiled to exclude certain parts.  This is
   controlled by giving a -D option a compilation time.

      -DFLOAT_SUPPORT        Full ANSI formatter
      -DNO_FLOATS            Full ANSI except floats
      -DREDUCED_SUPPORT        Reduced 'int' type of converter

   The reduced version of formatter is suitable when program size is critical
   rather than formatting power.  This routine uses less than 20 bytes of
   stack space which makes it practical even in systems with less than 256
   bytes of user RAM.

   The only formatting specifiers supported by the reduced formatter are:

         %% %c %s %d %o %x %X and %hd %ho %hx %hX %ld %lo %lx %lX.

   It means that real variables are not supported as well as field
   width and precision arguments.

   The last eight (h and l modifiers) can easily be removed by
   removing the line "#define MODIFIERS_IN_REDUCED".

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#ifndef FLOAT_SUPPORT
#ifndef NO_FLOATS
#ifndef REDUCED_SUPPORT
#error -DFLOAT_SUPPORT, -DNO_FLOATS or -DREDUCED_SUPPORT missing
#endif
#endif
#endif

/* Make it easy for customers to disable h and l
   modifiers in REDUCED_SUPPORT by removing the next #define */
#define MODIFIERS_IN_REDUCED

#include "stdarg.h"
#include "float.h"
#include "icclbutl.h"    /* Low-level declarations */
#include "sysmac.h"

#ifndef FRMWRI_BUFSIZE
#define FRMWRI_BUFSIZE 134
#endif

#ifndef MEM_ATTRIBUTE
#define MEM_ATTRIBUTE
#endif

#ifdef MODIFIERS_IN_REDUCED
#define IS_SHORT (h_modifier || (sizeof(int) == 2 && !l_modifier))
#else
#define IS_SHORT (sizeof(int) == 2)
#endif


#ifdef FLOAT_SUPPORT

static char *float_conversion(MEM_ATTRIBUTE long double value,
                              MEM_ATTRIBUTE short nr_of_digits,
                              MEM_ATTRIBUTE char *buf,
                              MEM_ATTRIBUTE char format_flag,
                              MEM_ATTRIBUTE char g_flag,
                              MEM_ATTRIBUTE char alternate_flag)
{
  MEM_ATTRIBUTE char *cp, *buf_pointer;
  MEM_ATTRIBUTE short n, i, dec_point_pos, integral_10_log;

  buf_pointer = buf;
  integral_10_log = 0;

#ifdef _CLIB_FP_INF_NAN
  if (value == 0.Infinity)
  {
    *buf_pointer++ = 'i';
    *buf_pointer++ = 'n';
    *buf_pointer++ = 'f';
    return buf_pointer;
  }
  else if (value != value)
  {
    *buf_pointer++ = 'n';
    *buf_pointer++ = 'a';
    *buf_pointer++ = 'n';
    return buf_pointer;
  }
#endif

  if (value >= 1)
  {
    while (value >= 1e11)        /* To speed up things a bit */
    {
      value /= 1e10;
      integral_10_log += 10;
    }
    while (value >= 10)
    {
      value /= 10;
      integral_10_log++;
    }
  }
  else if (value)            /* Not just 0.0 */
  {
    while (value <= 1e-10)        /* To speed up things a bit */
    {
      value *= 1e10;
      integral_10_log -= 10;
    }
    while (value < 1)
    {
      value *= 10;
      integral_10_log--;
    }
  }
  if (g_flag)
  {
    if (integral_10_log < nr_of_digits && integral_10_log >= -4)
    {
      format_flag = 0;
      nr_of_digits -= integral_10_log;
    }
    nr_of_digits--;
    if (alternate_flag)
    {
      g_flag = 0;         /* %#G - No removal of trailing zeros */
    }
    else
    {
      alternate_flag = 1;  /* %G - Removal of trailing zeros */
    }
  }

  if (format_flag)        /* %e or %E */
  {
    dec_point_pos = 0;
  }
  else
  {
    if (integral_10_log < 0)        /* Less than one.... */
    {
      *buf_pointer++ = '0';
      if ((n = nr_of_digits) || alternate_flag)
      {
        *buf_pointer++ = '.';
      }
      i = 0;
      while (--i > integral_10_log && nr_of_digits)
      {
        *buf_pointer++ = '0';
        nr_of_digits--;
      }
      if (integral_10_log < (-n - 1))
      {
        goto CLEAN_UP;     /* Nothing more to do */
      }
      dec_point_pos = 1;
    }
    else
    {
      dec_point_pos = - integral_10_log;
    }
  }

  i = dec_point_pos;
  while (i <= nr_of_digits )
  {
    n = (short)value;
    value -= n;          /* n=Digit value=Remainder */
    value *= 10;         /* Prepare for next shot */
    *buf_pointer++ = n + '0';
    if ( ! i++ && (nr_of_digits || alternate_flag))
    {
      *buf_pointer++ = '.';
    }
  }
  if (value >= 5)    /* Rounding possible */
  {
    n = 1;    /* Carry */
    cp = buf_pointer - 1;
    do
    {
      if (*cp != '.')
        if ( (*cp += n) == ('9' + 1) )
        {
          *cp = '0';
          n = 1;
        }
        else
        {
          n = 0;
        }
    } while (cp-- > buf);
    if (n)
    {
      if (format_flag)        /* %e or %E */
      {
        cp = buf_pointer;
        while (cp > buf)
        {
          if (*(cp - 1) == '.')
          {
            *cp = *(cp - 2);
            cp--;
          }
          else
          {
            *cp = *(cp - 1);
          }
          cp--;
        }
        integral_10_log++;
      }
      else
      {
        cp = ++buf_pointer;
        while (cp > buf)
        {
          *cp = *(cp - 1);
          cp--;
        }
      }
      *buf = '1';
    }
  }
CLEAN_UP:
  if (g_flag)            /* %G - Remove trailing zeros */
  {
    while (*(buf_pointer - 1) == '0')
    {
      buf_pointer--;
    }
    if (*(buf_pointer - 1) == '.')
    {
      buf_pointer--;
    }
  }
  if (format_flag)        /* %e or %E */
  {
    *buf_pointer++ = format_flag;
    if (integral_10_log < 0)
    {
      *buf_pointer++ = '-';
      integral_10_log = -integral_10_log;
    }
    else
    {
      *buf_pointer++ = '+';
    }
    n = 0;
    buf_pointer +=10;
    do
    {
      n++;
      *buf_pointer++ = (integral_10_log % 10) + '0';
      integral_10_log /= 10;
    } while ( integral_10_log || n < 2 );
    for ( i = n ; n > 0 ; n-- )
      *(buf_pointer - 11 - i + n) = *(buf_pointer - n);
    buf_pointer -= 10;
  }
  return buf_pointer;
}

#endif


/*----------------------------------------------------------------------*/
/*                                                                      */
/*                        - _formatted_write -                          */
/*                                                                      */
/* This routine forms the core and entry of the formatter.  The con-    */
/* version performed conforms to the ANSI specification for "printf".   */
/*----------------------------------------------------------------------*/

int _formatted_write(const char *format,
                     void put_one_char(char, void *),
                     void *secret_pointer,
                     va_list ap)
#ifndef REDUCED_SUPPORT
{
  MEM_ATTRIBUTE char format_flag;
  MEM_ATTRIBUTE int precision;
  MEM_ATTRIBUTE int n;
  MEM_ATTRIBUTE int field_width, nr_of_chars;
  MEM_ATTRIBUTE char plus_space_flag, left_adjust, l_L_modifier;
  MEM_ATTRIBUTE char h_modifier, alternate_flag;
  MEM_ATTRIBUTE char nonzero_value;
  MEM_ATTRIBUTE unsigned long ulong, div_factor;

#ifdef FLOAT_SUPPORT
  MEM_ATTRIBUTE long double fvalue;
#endif

  MEM_ATTRIBUTE char *buf_pointer;
  MEM_ATTRIBUTE char *ptr, *hex;
  MEM_ATTRIBUTE char zeropad;
  MEM_ATTRIBUTE char buf[FRMWRI_BUFSIZE];

  nr_of_chars = 0;
  for (;;)    /* Until full format string read */
  {
    while ((format_flag = *format++) != '%')    /* Until '%' or '\0' */
    {
      if (!format_flag)
        return nr_of_chars;
      put_one_char(format_flag, secret_pointer);
      nr_of_chars++;
    }
    if (*format == '%')    /* %% prints as % */
    {
      format++;
      put_one_char('%',secret_pointer);
      nr_of_chars++;
      continue;
    }

    plus_space_flag = left_adjust = alternate_flag = zeropad = 0;
    ptr = buf_pointer = &buf[0];
    hex = "0123456789ABCDEF";
    /*===================================================*/
    /* check for leading '-', '+', ' ','#' or '0' flags  */
    /*===================================================*/
    for (;;)
    {
      switch (*format)
      {
      case ' ':
        if (plus_space_flag)
          goto NEXT_FLAG;
      case '+':
        plus_space_flag = *format;
        goto NEXT_FLAG;
      case '-':
        left_adjust++;
        goto NEXT_FLAG;
      case '#':
        alternate_flag++;
        goto NEXT_FLAG;
      case '0':
        zeropad++;
        goto NEXT_FLAG;
      }
      break;
NEXT_FLAG:
      format++;
    }

    /*===================================*/
    /* Optional field width (may be '*') */
    /*===================================*/
    if (*format == '*')
    {
      field_width = va_arg(ap, int);
      if (field_width < 0)
      {
        field_width = -field_width;
        left_adjust++;
      }
      format++;
    }
    else
    {
      field_width = 0;
      while (*format >= '0' && *format <= '9')
        field_width = field_width * 10 + (*format++ - '0');
    }

    if (left_adjust)
      zeropad = 0;
    /*=============================*/
    /* Optional precision (or '*') */
    /*=============================*/
    if (*format=='.')
    {
      if (*++format == '*')
      {
        precision = va_arg(ap, int);
        format++;
      }
      else
      {
        precision = 0;
        while (*format >= '0' && *format <= '9')
          precision = precision * 10 + (*format++ - '0');
      }
    }
    else
      precision = -1;

    /*======================================================*/
    /* At this point, "left_adjust" is nonzero if there was */
    /* a sign, "zeropad" is 1 if there was a leading zero   */
    /* and 0 otherwise, "field_width" and "precision"       */
    /* contain numbers corresponding to the digit strings   */
    /* before and after the decimal point, respectively,    */
    /* and "plus_space_flag" is either 0 (no flag) or con-  */
    /* tains a plus or space character. If there was no     */
    /* decimal point, "precision" will be -1.               */
    /*======================================================*/

    l_L_modifier = h_modifier = 0;
    /*==================================*/
    /* Optional 'l','L' r 'h' modifier? */
    /*==================================*/
    switch (*format)
    {
    case 'l':
    case 'L':
      l_L_modifier++;
      format++;
      break;
    case 'h':
      h_modifier++;
      format++;
      break;
    }

    /*===================================================*/
    /* At exit from the following switch, we will emit   */
    /* the characters starting at "buf_pointer" and      */
    /* ending at "ptr"-1                     */
    /*===================================================*/
    switch (format_flag = *format++)
    {

    case 'n':
      if (sizeof(short) == sizeof(long))
      {
        *VAPTR(int) = nr_of_chars;
      }
      else if (sizeof(short) != sizeof(int))
      {
        if (sizeof(int) != sizeof(long))
        {
          if (h_modifier)
            *VAPTR(short) = nr_of_chars;
          else if (l_L_modifier)
            *VAPTR(long) = nr_of_chars;
          else
            *VAPTR(int) = nr_of_chars;
        }
        else
        {
          if (h_modifier)
            *VAPTR(short) = nr_of_chars;
          else
            *VAPTR(int) = nr_of_chars;
        }
      }
      else
      {
        if (l_L_modifier)
          *VAPTR(long) = nr_of_chars;
        else
          *VAPTR(int) = nr_of_chars;
      }
      continue;

    case 'c':
      buf[0] = va_arg(ap, int);
      ptr++;
      break;

    case 's':
      if ( !(buf_pointer = VAPTR(char)) )
        buf_pointer = "(null pointer)";
      if (precision < 0)
        precision = 10000;
      for (n=0; *buf_pointer++ && n < precision; n++)
        ;
      ptr = --buf_pointer;
      buf_pointer -= n;
      break;

    case 'o':
      if (alternate_flag && !precision)
        precision++;
    case 'x':
      hex = "0123456789abcdef";
    case 'u':
    case 'p':
    case 'X':
      if (format_flag == 'p')
        ulong = (long)VAPTR(char);
      /* short:2, int:2, long:2 */
      /* short:4, int:4, long:4 */
      else if (sizeof(short) == sizeof(long))
        ulong = va_arg(ap,unsigned int);
      /* short:2, int:2, long:4 */
      else if (sizeof(short) == sizeof(int))
        ulong = l_L_modifier ?
          va_arg(ap,long) : va_arg(ap,unsigned int);
      /* short:2, int:4, long:4 */
      else if (sizeof(int) == sizeof(long))
        ulong = h_modifier ?
          (unsigned short) va_arg(ap, int) : va_arg(ap, int);
      /* all other cases, for example: */
      /* short:2, int:3, long:4 */
      else
        ulong =
          h_modifier ?
            (unsigned short) va_arg(ap, int) :
            l_L_modifier ?
              va_arg(ap, long) :
              va_arg(ap, unsigned int);
      div_factor = (format_flag == 'o') ?
        8 : (format_flag == 'u') ? 10 : 16;
      plus_space_flag = 0;
      goto INTEGRAL_CONVERSION;

    case 'd':
    case 'i':
      /* short:2, int:2, long:2 */
      /* short:4, int:4, long:4 */
      if (sizeof(short) == sizeof(long))
      {
        if ( (long)(ulong = va_arg(ap,long)) < 0)
        {
          plus_space_flag = '-';
          ulong = -ulong;
        }
      }
      /* short:2, int:2, long:4 */
      else if (sizeof(short) == sizeof(int))
      {
        if ( (long)(ulong = l_L_modifier ?
                    va_arg(ap,long) : (long) va_arg(ap,int)) < 0)
        {
          plus_space_flag = '-';
          ulong = -ulong;
        }
      }
      /* short:2, int:4, long:4 */
      else if (sizeof(int) == sizeof(long))
      {
        if ( (long)(ulong = (long) (h_modifier ?
            (short) va_arg(ap, int) : va_arg(ap,int))) < 0)
        {
          plus_space_flag = '-';
          ulong = -ulong;
        }
      }
      /* all other cases, for example: */
      /* short:2, int:3, long:4 */
      else
      {
        if ( (long)(ulong = (long)
               (h_modifier ?
                (short) va_arg(ap, int) :
                (l_L_modifier ?
                 va_arg(ap, long) :
                 (long) va_arg(ap,int)))) < 0)
        {
          plus_space_flag = '-';
          ulong = -ulong;
        }
      }
      div_factor = 10;

      /*=======================*/
      /* now convert to digits */
      /*=======================*/
INTEGRAL_CONVERSION:
      ptr = buf_pointer = &buf[FRMWRI_BUFSIZE - 1];
      nonzero_value = (ulong != 0);
      if (precision != 0 || nonzero_value)
                                      /* No char if zero and zero precision */
        do
          *--buf_pointer = hex[ulong % div_factor];
        while (ulong /= div_factor);
      if (precision < 0)            /* "precision" takes precedence */
        if (zeropad)
          precision = field_width - (plus_space_flag != 0);
      while (precision > ptr - buf_pointer)
        *--buf_pointer = '0';
      if (alternate_flag && nonzero_value)
      {
        if (format_flag == 'x' || format_flag == 'X')
        {
          *--buf_pointer = format_flag;
          *--buf_pointer = '0';
        }
        else if ((format_flag == 'o') && (*buf_pointer != '0'))
        {
          *--buf_pointer = '0';
        }
      }
      break;

#ifdef FLOAT_SUPPORT
    case 'g':
    case 'G':
      n = 1;
      format_flag -= 2;
      if (! precision)
      {
        precision = 1;
      }
      goto FLOATING_CONVERSION;
    case 'f':
      format_flag = 0;
    case 'e':
    case 'E':
      n = 0;
FLOATING_CONVERSION:
      if (precision < 0)
      {
        precision = 6;
      }
      if (sizeof(double) != sizeof(long double))
      {
        if ( (fvalue = l_L_modifier ?
              va_arg(ap,long double) : va_arg(ap,double)) < 0)
        {
          plus_space_flag = '-';
          fvalue = -fvalue;
        }
      }
      else if ( (fvalue = va_arg(ap,long double)) < 0)
      {
        plus_space_flag = '-';
        fvalue = -fvalue;
      }
      ptr = float_conversion(fvalue,
                             precision,
                             buf_pointer += field_width,
                             format_flag,
                             n,
                             alternate_flag);
      if (zeropad)
      {
        precision = field_width - (plus_space_flag != 0);
        while (precision > ptr - buf_pointer)
          *--buf_pointer = '0';
      }
#else
    case 'g':
    case 'G':
    case 'f':
    case 'e':
    case 'E':
      ptr = buf_pointer = "FLOATS? wrong formatter installed!";
      while (*ptr)
        ptr++;
#endif
      break;

    case '\0':    /* Really bad place to find NUL in */
      format--;

    default:
      /*=======================*/
      /* Undefined conversion! */
      /*=======================*/
      ptr = buf_pointer = "???";
      ptr += 3;
      break;

    }

    /*===========================================================*/
    /* This part emittes the formatted string to "put_one_char". */
    /*===========================================================*/

    /* If field_width == 0 then nothing should be written. */
    precision = ptr - buf_pointer;

    if (precision > field_width)
    {
      n = 0;
    }
    else
    {
      n = field_width - precision - (plus_space_flag != 0);
    }

    /*=================================*/
    /* emit any leading pad characters */
    /*=================================*/
    if (!left_adjust)
      while (--n >= 0)
      {
        put_one_char(' ', secret_pointer);
        nr_of_chars++;
      }

    /*===============================*/
    /* emit flag characters (if any) */
    /*===============================*/
    if (plus_space_flag)
    {
      put_one_char(plus_space_flag, secret_pointer);
      nr_of_chars++;
    }

    /*========================*/
    /* emit the string itself */
    /*========================*/
    while (--precision >= 0)
    {
      put_one_char(*buf_pointer++, secret_pointer);
      nr_of_chars++;
    }

    /*================================*/
    /* emit trailing space characters */
    /*================================*/
    if (left_adjust)
      while (--n >= 0)
      {
        put_one_char(' ', secret_pointer);
        nr_of_chars++;
      }
  }
}

#else       /* REDUCED_SUPPORT STARTS HERE */

{
#ifdef MODIFIERS_IN_REDUCED
  char l_modifier, h_modifier;
  unsigned long u_val, div_val;
#else
  unsigned int u_val, div_val;
#endif
  char format_flag;
  unsigned int nr_of_chars, base;
  char outChar;
  char *ptr;

  nr_of_chars = 0;
  for (;;)    /* Until full format string read */
  {
    while ((format_flag = *format++) != '%')    /* Until '%' or '\0' */
    {
      if (!format_flag)
        return nr_of_chars;
      put_one_char (format_flag, secret_pointer);
      nr_of_chars++;
    }

#ifdef MODIFIERS_IN_REDUCED
    /*=================================*/
    /* Optional 'l' or 'h' modifiers ? */
    /*=================================*/
    l_modifier = h_modifier = 0;
    switch (*format)
    {
    case 'l':
      l_modifier = 1;
      format++;
      break;
    case 'h':
      h_modifier = 1;
      format++;
      break;
    }
#endif

    switch (format_flag = *format++)
    {
    case 'c':
      format_flag = va_arg(ap, int);
    default:
      put_one_char(format_flag, secret_pointer);
      nr_of_chars++;
      continue;

    case 's':
      ptr = VAPTR(char);
      while (format_flag = *ptr++)
      {
        put_one_char(format_flag, secret_pointer);
        nr_of_chars++;
      }
      continue;

    case 'o':
      base = 8;
      if (IS_SHORT)
        div_val = 0x8000;
      else
        div_val = 0x40000000;
      goto CONVERSION_LOOP;

    case 'd':
      base = 10;
      if (IS_SHORT)
        div_val = 10000;
      else
        div_val = 1000000000;
      goto CONVERSION_LOOP;

    case 'X':
    case 'x':
      base = 16;
      if (IS_SHORT)
        div_val = 0x1000;
      else
        div_val = 0x10000000;

CONVERSION_LOOP:
#ifdef MODIFIERS_IN_REDUCED
      if (h_modifier)
        u_val = (format_flag == 'd') ?
          (unsigned long) (short) va_arg(ap, int) :
          (unsigned long) (unsigned short) va_arg(ap, int);
      else if (l_modifier)
        u_val = va_arg(ap, long);
      else
        u_val = (format_flag == 'd') ?
          (unsigned long) va_arg(ap,int) :
          (unsigned long) va_arg(ap,unsigned int);

      if (format_flag == 'd')
      {
        if (((long)u_val) < 0)
#else
      u_val = va_arg(ap,int);

      if (format_flag == 'd')
      {
        if (((int)u_val) < 0)
#endif
        {
          u_val = - u_val;
          put_one_char('-', secret_pointer);
          nr_of_chars++;
        }
      }
      while (div_val > 1 && div_val > u_val)
      {
        div_val /= base;
      }
      do
      {
        outChar = (u_val / div_val) + '0';
        if (outChar > '9')
          if (format_flag == 'x')
            outChar += 'a'-'9'-1;
          else
            outChar += 'A'-'9'-1;
        put_one_char(outChar, secret_pointer);
        nr_of_chars++;
        u_val %= div_val;
        div_val /= base;
      }
      while (div_val);
    }
  }
}
#endif
