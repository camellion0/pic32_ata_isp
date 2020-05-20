/*                      - SINUS.C -

   Internal  function.

   $Revision: 328482 $

   Copyright 1986 - 1999 IAR Systems. All rights reserved.
*/

#include "sysmac.h"
#include "math.h"
#include "iccfloat.h"


#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
#define c0      0.157079631844e1
#define c1      -0.645963710599
#define c2      0.79689678946e-1
#define c3      -0.467376661e-2
#define c4      0.151485129e-3
#else
#define c0       0.15707963267948966188272e1
#define c1      -0.64596409750624619108547
#define c2       0.7969262624616543562977e-1
#define c3      -0.468175413530264260121e-2
#define c4       0.1604411847068220716e-3
#define c5      -0.359884300720869272e-5
#define c6       0.5692134872719023e-7
#define c7      -0.66843217206396e-9
#define c8       0.587061098171e-11
#endif


double __sinus(double x, unsigned char quad)
{
  double e,f,ysq,y;
  unsigned int k;

  quad += 2 * (x < 0.0);
  x = fabs(x);
  x = x * __TWOOPI;   /* underflow? */
  if (x > 65532)
  {
    y = modf (x, &e);
    e += quad;
    modf (0.25 * e, &f);
    quad = (unsigned char)(e - 4 * f);
  }
  else
  {
    k = (unsigned int)x;
    y = x - k;
    quad = (quad + k) & 03;
  }
  if (quad & 01)
    y = 1 - y;
  if (quad > 1)
    y = -y;

  ysq = y * y;
#if __FLOAT_SIZE__ == __DOUBLE_SIZE__
  y = ((((c4 * ysq + c3) * ysq + c2) * ysq + c1) * ysq + c0) * y;
#else
  y = ((((((((c8 * ysq + c7) * ysq + c6) * ysq + c5) * ysq + c4) * ysq +
          c3) * ysq + c2) * ysq + c1) * ysq + c0) * y;
#endif
  return y;
}
