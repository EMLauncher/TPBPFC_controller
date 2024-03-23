/*
 * SOGI_PLL.h
 *
 *  Created on: 2023/04/05
 *      Author: eml_t
 */

#ifndef INC_SOGI_PLL_H_
#define INC_SOGI_PLL_H_

// #include "main.h"
#include "arm_math.h"
#include "PI_controller.h"
#include "my_const_values.h"

#define PI_Q2_13 ((q15_t)25736) // pi Q2.13
#define PI_2_Q2_13 ((q15_t)12868) // pi/2 Q2.13

typedef struct {
	// initialized (Const)
	__IO q15_t Ts; // sampling time, period of calling update function. 2^15==1[ms]
	__IO q15_t k; // Q4.11
	__IO q15_t w_max, w_min; // saturate angular velocity. 2^15==1000*pi[rad/s]
	PI_ctrl_inst_q15 dthetaPI; // Δθを0にするPI制御器. PI_Q2_13=pi[rad]が観測値なのでKp代入するとき注意.

	// Read Only
	__IO q15_t ea, eb, ed, eq; // 2^15==1000[V]
	__IO q15_t theta; // angle. 0 to 2^15. 2^15==2pi[rad] arm_math_sin()は0~2pi形式の引数のため
	__IO q15_t w; // angular velocity. 2^15==1000*pi[rad/s]
	__IO q15_t cos_t, sin_t; // cos(theta), sin(theta)
	__IO q15_t eu; // for debug
	__IO uint8_t state;

	// Do NOT Access
	__IO q15_t wTs; // w * Ts. 2^15==pi[rad]
	__IO q15_t wTs_std; // w * Ts. 2^15==1.0[rad] 電圧に掛けたときにスケーリングしなくていいように規格化
	__IO q31_t wTs_q31; // w * Ts. 2^31==pi[rad]
	__IO q31_t theta_q31; // angle. 2^31==pi[rad] 16bitでは分解能が足りないため
} SOGI_PLL_inst_q15;

void init_SOGI_PLL(SOGI_PLL_inst_q15 *S);
void update_SOGI_PLL(SOGI_PLL_inst_q15 *S, q15_t eu);
// static q15_t atan2_q15(q15_t y, q15_t x);
// static q15_t atan_approx_q15(q15_t x);


#endif /* INC_SOGI_PLL_H_ */
