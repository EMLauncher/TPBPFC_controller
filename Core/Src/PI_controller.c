/*
 * PI_controller.c
 *
 *  Created on: 2023/04/09
 *      Author: eml_t
 */

#include "PI_controller.h"

/*
 * initialize instance
 */
void init_PI_ctrl(PI_ctrl_inst_q15 *C, q15_t y_init) {
	// q31_t tmp_q31;

	// calculate coefficients 乗算回数変わらないので削除
	/*
	tmp_q31 = 32768L + ((C->TsTi + 1) >> 1); // 2^15=1.0[-], 1+Ts/2Ti
	tmp_q31 *= C->Kp; // 2^(15+Kp_decimal)=1.0[U/Y], Kp*(1+Ts/2Ti)
	tmp_q31 += 1L << 14;
	tmp_q31 >>= 15; // 2^(Kp_decimal)=1.0[U/Y]
	tmp_q31 = __SSAT(tmp_q31, 16); // saturate 16bit
	C->a[0] = (q15_t)tmp_q31;
	tmp_q31 = -32768L + ((C->TsTi + 1) >> 1); // 2^15=1.0[-], -1+Ts/2Ti
	tmp_q31 *= C->Kp;
	tmp_q31 += 1L << 14;
	tmp_q31 >>= 15;
	tmp_q31 = __SSAT(tmp_q31, 16);
	C->a[1] = (q15_t)tmp_q31;
	*/

	// initialize error
	C->err[0] = (q31_t)C->y_target - y_init;

	return;
}

void invert_PI_ctrl(PI_ctrl_inst_q15 *C) {
	// 操作量uの増加に対してyが短調減少する場合にこの関数を呼び出す
	// 現状は係数をマイナスにするだけだが、ユーザーが途中で係数を変えても大丈夫な仕様にする
	if (C->Kp > 0) {
		C->Kp = -C->Kp;
	}

	return;
}

void update_PI_ctrl(PI_ctrl_inst_q15 *C, q15_t y_in) {
	q31_t tmp_q31;

	// update error
	C->err[1] = C->err[0];
	C->err[0] = (q31_t)C->y_target - y_in;

	// calculate u
	tmp_q31 = C->err[0] + C->err[1]; // e(n) + e(n-1) :2^15=1.0[Y]
	tmp_q31 *= C->TsTi; // Ts/Ti*(e(n)+e(n-1)) :2^30=1.0[Y]
	tmp_q31 += 1L << 15;
	tmp_q31 >>= 16; // 0.5*Ts/Ti*(e(n)+e(n-1)) :2^15=1.0[Y]
	tmp_q31 += C->err[0] - C->err[1]; // 0.5*Ts/Ti*(e(n)+e(n-1))+e(n)-e(n-1) :2^15=1.0[Y]
	tmp_q31 *= C->Kp; // :2^(15+Kp_decimal)=1.0[U]
	tmp_q31 += 1L << (C->Kp_decimal - 1);
	tmp_q31 >>= C->Kp_decimal; // 2^15=1.0[U]
	tmp_q31 += C->u;
	C->u = __SSAT(tmp_q31, 16); // saturate 16bit
	if (C->u > C->u_max) {
		C->u = C->u_max;
	} else if(C->u < C->u_min) {
		C->u = C->u_min;
	}

	return;
}
