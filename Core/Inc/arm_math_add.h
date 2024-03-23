/*
 * arm_math_add.h
 *
 *  Created on: 2023/03/30
 *      Author: eml_t
 */

#ifndef INC_ARM_MATH_ADD_H_
#define INC_ARM_MATH_ADD_H_

#include "arm_math.h"

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
  int16_t *shift);

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
  int16_t *shift);



  /**
     @brief  Arc tangent in radian of y/x using sign of x and y to determine right quadrant.
     @param[in]   y  y coordinate
     @param[in]   x  x coordinate
     @param[out]  result  Result
     @return  error status.
   */
  arm_status arm_atan2_f32(float32_t y,float32_t x,float32_t *result);


  /**
     @brief  Arc tangent in radian of y/x using sign of x and y to determine right quadrant.
     @param[in]   y  y coordinate
     @param[in]   x  x coordinate
     @param[out]  result  Result in Q2.29
     @return  error status.
   */
  arm_status arm_atan2_q31(q31_t y,q31_t x,q31_t *result);

  /**
     @brief  Arc tangent in radian of y/x using sign of x and y to determine right quadrant.
     @param[in]   y  y coordinate
     @param[in]   x  x coordinate
     @param[out]  result  Result in Q2.13
     @return  error status.
   */
  arm_status arm_atan2_q15(q15_t y,q15_t x,q15_t *result);

#endif /* INC_ARM_MATH_ADD_H_ */
