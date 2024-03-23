/*
 * PI_controller.h
 *
 *  Created on: 2023/04/09
 *      Author: eml_t
 */

#ifndef INC_PI_CONTROLLER_H_
#define INC_PI_CONTROLLER_H_

#include "main.h"
#include "arm_math.h"

/*
 * 操作量u[U], 観測値y[Y]という単位だとする.
 */
typedef struct {
	// initialized
	__IO q15_t Kp; // 2^(Kp_decimal)=1.0[U/Y]. must NOT over 2^14=16384
	__IO uint8_t Kp_decimal; // Kpの小数点以下の桁数(15 ~ 0). decimal==15 => q15_t
	__IO q15_t TsTi; // Ts / Ti. 2^15=1.0[-]. Tsはサンプリング時間, Tiは積分時間

	// Write
	__IO q15_t y_target; // 2^15=1.0[Y]. 制御目標値
	__IO q15_t u_min, u_max; // anti-windup.

	// Read Only
	__IO q15_t u; // 2^15=1.0[U]. 操作量

	// Do NOT Access
	// q15_t a[2]; // 2^(Kp_decimal)=1.0[U/Y], coefficient
	q31_t err[2]; // 2^15=1.0[Y]. 誤差 オーバーフロー対策のため32bit. 0:present, 1:previous
} PI_ctrl_inst_q15;

void init_PI_ctrl(PI_ctrl_inst_q15 *C, q15_t y_init);
void invert_PI_ctrl(PI_ctrl_inst_q15 *C);
void update_PI_ctrl(PI_ctrl_inst_q15 *C, q15_t y_in);

#endif /* INC_PI_CONTROLLER_H_ */
