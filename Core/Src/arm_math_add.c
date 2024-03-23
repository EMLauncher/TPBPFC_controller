/*
 * arm_math_add.c
 *
 *  Created on: 2023/03/30
 *      Author: eml_t
 */

#include "arm_math_add.h"

/**
  @addtogroup divide
  @{
 */

/**
  @brief         Fixed point division
  @param[in]     numerator    Numerator
  @param[in]     denominator  Denominator
  @param[out]    quotient     Quotient value normalized between -1.0 and 1.0
  @param[out]    shift        Shift left value to get the unnormalized quotient
  @return        error status

  When dividing by 0, an error ARM_MATH_NANINF is returned. And the quotient is forced
  to the saturated negative or positive value.
 */

arm_status arm_divide_q15(q15_t numerator,
  q15_t denominator,
  q15_t *quotient,
  int16_t *shift)
{
  int16_t sign=0;
  q31_t temp;
  int16_t shiftForNormalizing;

  *shift = 0;

  sign = (numerator>>15) ^ (denominator>>15);

  if (denominator == 0)
  {
     if (sign)
     {
        *quotient = 0x8000;
     }
     else
     {
        *quotient = 0x7FFF;
     }
     return(ARM_MATH_NANINF);
  }

  arm_abs_q15(&numerator,&numerator,1);
  arm_abs_q15(&denominator,&denominator,1);

  temp = ((q31_t)numerator << 15) / ((q31_t)denominator);

  shiftForNormalizing= 17 - __CLZ(temp);
  if (shiftForNormalizing > 0)
  {
     *shift = shiftForNormalizing;
     temp = temp >> shiftForNormalizing;
  }

  if (sign)
  {
    temp = -temp;
  }

  *quotient=temp;

  return(ARM_MATH_SUCCESS);
}

/**
  @brief         Fixed point division
  @param[in]     numerator    Numerator
  @param[in]     denominator  Denominator
  @param[out]    quotient     Quotient value normalized between -1.0 and 1.0
  @param[out]    shift        Shift left value to get the unnormalized quotient
  @return        error status

  When dividing by 0, an error ARM_MATH_NANINF is returned. And the quotient is forced
  to the saturated negative or positive value.
 */

arm_status arm_divide_q31(q31_t numerator,
  q31_t denominator,
  q31_t *quotient,
  int16_t *shift)
{
  int16_t sign=0;
  q63_t temp;
  int16_t shiftForNormalizing;

  *shift = 0;

  sign = (numerator>>31) ^ (denominator>>31);

  if (denominator == 0)
  {
     if (sign)
     {
        *quotient = 0x80000000;
     }
     else
     {
        *quotient = 0x7FFFFFFF;
     }
     return(ARM_MATH_NANINF);
  }

  arm_abs_q31(&numerator,&numerator,1);
  arm_abs_q31(&denominator,&denominator,1);

  temp = ((q63_t)numerator << 31) / ((q63_t)denominator);

  shiftForNormalizing= 32 - __CLZ(temp >> 31);
  if (shiftForNormalizing > 0)
  {
     *shift = shiftForNormalizing;
     temp = temp >> shiftForNormalizing;
  }

  if (sign)
  {
    temp = -temp;
  }

  *quotient=(q31_t)temp;

  return(ARM_MATH_SUCCESS);
}

/**
  @} end of divide group
 */

/*

atan for argument between in [0, 1.0]


*/

#define ATANHALFF32 0.463648f
#define PIHALFF32 1.5707963267948966192313f

#define ATAN2_NB_COEFS_F32 10

static const float32_t atan2_coefs_f32[ATAN2_NB_COEFS_F32]={0.0f
,1.0000001638308195518f
,-0.0000228941363602264f
,-0.3328086544578890873f
,-0.004404814619311061f
,0.2162217461808173258f
,-0.0207504842057097504f
,-0.1745263362250363339f
,0.1340557235283553386f
,-0.0323664125927477625f
};

__STATIC_FORCEINLINE float32_t arm_atan_limited_f32(float32_t x)
{
    float32_t res=atan2_coefs_f32[ATAN2_NB_COEFS_F32-1];
    int i=1;
    for(i=1;i<ATAN2_NB_COEFS_F32;i++)
    {
        res = x*res + atan2_coefs_f32[ATAN2_NB_COEFS_F32-1-i];
    }


    return(res);
}

__STATIC_FORCEINLINE float32_t arm_atan_f32(float32_t x)
{
   int sign=0;
   float32_t res=0.0f;

   if (x < 0.0f)
   {
      sign=1;
      x=-x;
   }

   if (x > 1.0f)
   {
      x = 1.0f / x;
      res = PIHALFF32 - arm_atan_limited_f32(x);
   }
   else
   {
     res += arm_atan_limited_f32(x);
   }


   if (sign)
   {
     res = -res;
   }

   return(res);
}


/**
  @ingroup groupFastMath
 */

/**
  @defgroup atan2 ArcTan2

  Computing Arc tangent only using the ratio y/x is not enough to determine the angle
  since there is an indeterminacy. Opposite quadrants are giving the same ratio.

  ArcTan2 is not using y/x to compute the angle but y and x and use the sign of y and x
  to determine the quadrant.

 */

/**
  @addtogroup atan2
  @{
 */

/**
  @brief       Arc Tangent of y/x using sign of y and x to get right quadrant
  @param[in]   y  y coordinate
  @param[in]   x  x coordinate
  @param[out]  result  Result
  @return  error status.

  @par         Compute the Arc tangent of y/x:
                   The sign of y and x are used to determine the right quadrant
                   and compute the right angle.
*/


arm_status arm_atan2_f32(float32_t y,float32_t x,float32_t *result)
{
    if (x > 0.0f)
    {
        *result=arm_atan_f32(y/x);
        return(ARM_MATH_SUCCESS);
    }
    if (x < 0.0f)
    {
        if (y > 0.0f)
        {
           *result=arm_atan_f32(y/x) + PI;
        }
        else if (y < 0.0f)
        {
           *result=arm_atan_f32(y/x) - PI;
        }
        else
        {
            if (signbit(y))
            {
               *result= -PI;
            }
            else
            {
               *result= PI;
            }
        }
        return(ARM_MATH_SUCCESS);
    }
    if (x == 0.0f)
    {
        if (y > 0.0f)
        {
            *result=PIHALFF32;
            return(ARM_MATH_SUCCESS);
        }
        if (y < 0.0f)
        {
            *result=-PIHALFF32;
            return(ARM_MATH_SUCCESS);
        }
    }


    return(ARM_MATH_NANINF);

}

/*

atan for argument between in [0, 1.0]

*/


/* Q2.13 */
#define ATANHALFQ13 0xed6
#define PIHALFQ13 0x3244
#define PIQ13 0x6488

#define ATAN2_NB_COEFS_Q15 10

static const q15_t atan2_coefs_q15[ATAN2_NB_COEFS_Q15]={0x0000
,0x7fff
,0xffff
,0xd567
,0xff70
,0x1bad
,0xfd58
,0xe9a9
,0x1129
,0xfbdb
};

__STATIC_FORCEINLINE q15_t arm_atan_limited_q15(q15_t x)
{
    q31_t res=(q31_t)atan2_coefs_q15[ATAN2_NB_COEFS_Q15-1];
    int i=1;
    for(i=1;i<ATAN2_NB_COEFS_Q15;i++)
    {
        res = ((q31_t) x * res) >> 15U;
        res = res + ((q31_t) atan2_coefs_q15[ATAN2_NB_COEFS_Q15-1-i]) ;
    }

    res = __SSAT(res>>2,16);


    return(res);
}


__STATIC_FORCEINLINE q15_t arm_atan_q15(q15_t y,q15_t x)
{
   int sign=0;
   q15_t res=0;

   if (y<0)
   {
     arm_negate_q15(&y,&y,1);
     sign=1-sign;
   }

   if (x < 0)
   {
      sign=1 - sign;
      arm_negate_q15(&x,&x,1);
   }

   if (y > x)
   {
    q15_t ratio;
    int16_t shift;

    arm_divide_q15(x,y,&ratio,&shift);

    arm_shift_q15(&ratio,shift,&ratio,1);

    res = PIHALFQ13 - arm_atan_limited_q15(ratio);

   }
   else
   {
    q15_t ratio;
    int16_t shift;

    arm_divide_q15(y,x,&ratio,&shift);

    arm_shift_q15(&ratio,shift,&ratio,1);

    res = arm_atan_limited_q15(ratio);

   }


   if (sign)
   {
     arm_negate_q15(&res,&res,1);
   }

   return(res);
}

/**
  @brief       Arc Tangent of y/x using sign of y and x to get right quadrant
  @param[in]   y  y coordinate
  @param[in]   x  x coordinate
  @param[out]  result  Result in Q2.13
  @return  error status.

  @par         Compute the Arc tangent of y/x:
                   The sign of y and x are used to determine the right quadrant
                   and compute the right angle.
*/


arm_status arm_atan2_q15(q15_t y,q15_t x,q15_t *result)
{
    if (x > 0)
    {
        *result=arm_atan_q15(y,x);
        return(ARM_MATH_SUCCESS);
    }
    if (x < 0)
    {
        if (y > 0)
        {
           *result=arm_atan_q15(y,x) + PIQ13;
        }
        else if (y < 0)
        {
           *result=arm_atan_q15(y,x) - PIQ13;
        }
        else
        {
            if (y<0)
            {
               *result= -PIQ13;
            }
            else
            {
               *result= PIQ13;
            }
        }
        return(ARM_MATH_SUCCESS);
    }
    if (x == 0)
    {
        if (y > 0)
        {
            *result=PIHALFQ13;
            return(ARM_MATH_SUCCESS);
        }
        if (y < 0)
        {
            *result=-PIHALFQ13;
            return(ARM_MATH_SUCCESS);
        }
    }


    return(ARM_MATH_NANINF);

}

/*

atan for argument between in [0, 1.0]

*/


/* Q2.29 */
#define ATANHALF_Q29 0xed63383
#define PIHALF_Q29 0x3243f6a9
#define PIQ29 0x6487ed51

#define ATAN2_NB_COEFS_Q31 13

static const q31_t atan2_coefs_q31[ATAN2_NB_COEFS_Q31]={0x00000000
,0x7ffffffe
,0x000001b6
,0xd555158e
,0x00036463
,0x1985f617
,0x001992ae
,0xeed53a7f
,0xf8f15245
,0x2215a3a4
,0xe0fab004
,0x0cdd4825
,0xfddbc054
};


__STATIC_FORCEINLINE q31_t arm_atan_limited_q31(q31_t x)
{
    q63_t res=(q63_t)atan2_coefs_q31[ATAN2_NB_COEFS_Q31-1];
    int i=1;
    for(i=1;i<ATAN2_NB_COEFS_Q31;i++)
    {
        res = ((q63_t) x * res) >> 31U;
        res = res + ((q63_t) atan2_coefs_q31[ATAN2_NB_COEFS_Q31-1-i]) ;
    }

    return(clip_q63_to_q31(res>>2));
}


__STATIC_FORCEINLINE q31_t arm_atan_q31(q31_t y,q31_t x)
{
   int sign=0;
   q31_t res=0;

   if (y<0)
   {
     arm_negate_q31(&y,&y,1);
     sign=1-sign;
   }

   if (x < 0)
   {
      sign=1 - sign;
      arm_negate_q31(&x,&x,1);
   }

   if (y > x)
   {
    q31_t ratio;
    int16_t shift;

    arm_divide_q31(x,y,&ratio,&shift);

    arm_shift_q31(&ratio,shift,&ratio,1);

    res = PIHALF_Q29 - arm_atan_limited_q31(ratio);

   }
   else
   {
    q31_t ratio;
    int16_t shift;

    arm_divide_q31(y,x,&ratio,&shift);

    arm_shift_q31(&ratio,shift,&ratio,1);

    res = arm_atan_limited_q31(ratio);

   }


   if (sign)
   {
     arm_negate_q31(&res,&res,1);
   }

   return(res);
}

/**
  @brief       Arc Tangent of y/x using sign of y and x to get right quadrant
  @param[in]   y  y coordinate
  @param[in]   x  x coordinate
  @param[out]  result  Result in Q2.29
  @return  error status.

  @par         Compute the Arc tangent of y/x:
                   The sign of y and x are used to determine the right quadrant
                   and compute the right angle.
*/


arm_status arm_atan2_q31(q31_t y,q31_t x,q31_t *result)
{
    if (x > 0)
    {
        *result=arm_atan_q31(y,x);
        return(ARM_MATH_SUCCESS);
    }
    if (x < 0)
    {
        if (y > 0)
        {
           *result=arm_atan_q31(y,x) + PIQ29;
        }
        else if (y < 0)
        {
           *result=arm_atan_q31(y,x) - PIQ29;
        }
        else
        {
            if (y<0)
            {
               *result= -PIQ29;
            }
            else
            {
               *result= PIQ29;
            }
        }
        return(ARM_MATH_SUCCESS);
    }
    if (x == 0)
    {
        if (y > 0)
        {
            *result=PIHALF_Q29;
            return(ARM_MATH_SUCCESS);
        }
        if (y < 0)
        {
            *result=-PIHALF_Q29;
            return(ARM_MATH_SUCCESS);
        }
    }


    return(ARM_MATH_NANINF);

}

/**
  @} end of atan2 group
 */
