/*                      - SATAN.C -

   Support for the ANSI "atan" and "atan2" functions.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"

#if __FLOAT_SIZE__ == __DOUBLE_SIZE__

#define t0      0.36404851634345e1
#define t1      0.29960993560558e1
#define t2      0.32474160322377
#define t3      -0.1049784199021e-1
#define s0      0.3640485264119e1
#define s1      0.42095844163262e1
#define s2      0.1e1

double __satan(double arg)
{
  double b,ysq;

  arg = fabs(arg);
  if (arg <= 1.0)
  {
    if (arg == 1.0)
      return __PIO4;
    b = 0.0;
  }
  else
  {
    b = __PIO4;
    arg = (arg - 1.0) / (arg + 1.0);
  }
  ysq = arg * arg;
  arg = arg * ((((t3 * ysq + t2) * ysq + t1) * ysq + t0) /
               ((s2 * ysq + s1) * ysq + s0));
  return b + arg;
}

#else

/*
   coefficients are #5077 from Hart & Cheney. (19.56D)
*/

#define   sq2p1      2.414213562373095048802e0
#define   sq2m1       .414213562373095048802e0
#define   pio2       1.570796326794896619231e0
#define   pio4        .785398163397448309615e0
#define   p4          .161536412982230228262e2
#define   p3          .26842548195503973794141e3
#define   p2          .11530293515404850115428136e4
#define   p1          .178040631643319697105464587e4
#define   p0          .89678597403663861959987488e3
#define   q4          .5895697050844462222791e2
#define   q3          .536265374031215315104235e3
#define   q2          .16667838148816337184521798e4
#define   q1          .207933497444540981287275926e4
#define   q0          .89678597403663861962481162e3


/*
        __satan reduces its argument (known to be positive)
        to the range [0,0.414...] and calls xatan.
*/

/*
        xatan evaluates a series valid in the
        range [-0.414...,+0.414...].
*/

static double xatan (double arg)
{
  double argsq;
  double value;

  argsq = arg*arg;
  value = ((((p4*argsq + p3)*argsq + p2)*argsq + p1)*argsq + p0);
  value = value / (((((argsq + q4)*argsq + q3)*argsq + q2)*argsq +
                    q1)*argsq + q0);
  return value * arg;
}


double __satan (double arg)
{
  arg = fabs(arg);

  if (arg < sq2m1)
    return xatan(arg);
  else if (arg > sq2p1)
    return pio2 - xatan(1 / arg);
  else
    return pio4 + xatan((arg - 1.0) / (arg + 1.0));
}

#endif
