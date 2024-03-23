/*
 * SOGI_PLL.c
 *
 *  Created on: 2023/04/05
 *      Author: eml_t
 */

#include "SOGI_PLL.h"

/*
 * initialize SOGI-PLL instance.
 */
void init_SOGI_PLL(SOGI_PLL_inst_q15 *S) {
	q31_t tmp_q31;

	// set constant values
	S->Ts = SAMPLING_TIME;
	S->w_max = OMEGA_MAX;
	S->w_min = OMEGA_MIN;
	S->k = SOGIPLL_K_Q4_11;
	S->dthetaPI.Kp_decimal = Q15_DECIMAL;
	S->dthetaPI.Kp = OMEGA_PI_KP;
	S->dthetaPI.TsTi = OMEGA_PI_TSTI;
	S->dthetaPI.y_target = 0;

	// initialize w
	S->w = (S->w_max + S->w_min) >> 1; // mean of w_max and w_min

	S->wTs_q31 = (q31_t)S->w * S->Ts; // 2^30==pi[rad]
	S->wTs_q31 <<= 1; // 2^31==pi[rad]
	S->wTs = (q15_t) ((S->wTs_q31 + (1L << 15)) >> 16); // 2^15==pi[rad]
	tmp_q31 = (q31_t)S->wTs;
	tmp_q31 *= PI_Q2_13;
	tmp_q31 += 1L << 12;
	S->wTs_std = (q15_t)(tmp_q31 >> 13); // 2^15==1.0[rad]
	S->ea = 1000;
	S->eb = 0;
	S->theta = 0;
	S->theta_q31 = 0L;

	S->state = SOGI_PLL_SET;

	S->dthetaPI.u_max = S->w_max;
	S->dthetaPI.u_min = S->w_min;
	S->dthetaPI.u = S->w;
	init_PI_ctrl(&S->dthetaPI, 0);
	invert_PI_ctrl(&S->dthetaPI); // 角度が+の時、周波数を下げないといけないので符号を変える

	return;
}

/*
 * update SOGI-PLL
 */
void update_SOGI_PLL(SOGI_PLL_inst_q15 *S, q15_t eu) {
	q15_t tmp;
	q31_t tmpX_q31, tmpY_q31;

	// update eab
	S->eu = eu; // debug
	tmpX_q31 = (q31_t)eu - S->ea;
	tmpX_q31 *= S->k;
	tmpX_q31 += (1L << 10); // 精度向上
	tmpX_q31 >>= 11; // k is Q4.11
	tmpX_q31 -= S->eb;
	tmpX_q31 *= S->wTs_std;
	tmpX_q31 += (1L << 14);
	tmpX_q31 >>= 15;
	tmpX_q31 += S->ea;
	tmpX_q31 = __SSAT(tmpX_q31, 16); // saturate 16bit
	tmpY_q31 = S->ea;
	tmpY_q31 *= S->wTs_std;
	tmpY_q31 += (1L << 14);
	tmpY_q31 >>= 15;
	tmpY_q31 += S->eb;
	tmpY_q31 = __SSAT(tmpY_q31, 16); // saturate 16bit
	S->ea = (q15_t)tmpX_q31;
	S->eb = (q15_t)tmpY_q31;

	// update theta
	S->theta_q31 += S->wTs_q31;
	S->theta = (q15_t)((S->theta_q31 >> 17) + (1L << 14)); // mapping -2^31 : 2^31-1 => 0 : 2^15-1

	// calculate edq
	S->cos_t = arm_cos_q15(S->theta);
	S->sin_t = arm_sin_q15(S->theta);
	tmpX_q31 = (q31_t)S->ea * S->cos_t + (q31_t)S->eb * S->sin_t; // 2^30==1000[V]
	tmpX_q31 += 1L << 14;
	tmpX_q31 >>= 15; // 2^15==1000[V]
	// tmpX_q31 = __SSAT(tmpX_q31, 16); // saturate 16bit
	tmpY_q31 = (q31_t)S->eb * S->cos_t - (q31_t)S->ea * S->sin_t; // 2^30==1000[V]
	tmpY_q31 += 1L << 14;
	tmpY_q31 >>= 15; // 2^15==1000[V]
	// tmpY_q31 = __SSAT(tmpY_q31, 16); // saturate 16bit
	S->ed = (q15_t)tmpX_q31;
	S->eq = (q15_t)tmpY_q31;

	if (S->theta < S->wTs) { // 1周した
		// update w
		S->state = SOGI_PLL_PICTRL;
		arm_atan2_q15(S->eq, S->ed, &tmp);
		// tmp = atan2_q15(S->eq, S->ed);
		update_PI_ctrl(&S->dthetaPI, tmp);
		S->w = S->dthetaPI.u;
		S->wTs_q31 = (q31_t) S->w * S->Ts; // 2^30==pi[rad]
		S->wTs_q31 <<= 1; // 2^31==pi[rad]
		S->wTs = (q15_t) ((S->wTs_q31 + (1L << 15)) >> 16); // 2^15==pi[rad]
		tmpX_q31 = S->wTs;
		tmpX_q31 *= PI_Q2_13;
		tmpX_q31 += 1L << 12;
		S->wTs_std = (q15_t)(tmpX_q31 >> 13); // 2^15==1.0[rad]
	} else {
		S->state = SOGI_PLL_SET;
	}

	return;
}

/*
static q15_t atan2_q15(q15_t y, q15_t x) {
	q31_t tmp_q31;
	q15_t atan;

	if (y == 0) {
		if (x < 0) {
			return PI_Q2_13;
		} else {
			return 0;
		}
	}
	if (x == 0) {
		if (y > 0) {
			return PI_2_Q2_13;
		} else {
			return -PI_2_Q2_13;
		}
	}
	// if not returned, x is not 0 and y is not 0
	if (y > -x) {
		if (y < x) {
			// calculate atan(y/x)
			tmp_q31 = (q31_t) y << 16; // q15 -> q31
			tmp_q31 += 1L << 0;
			tmp_q31 /= x; // q31 -> q16
			tmp_q31 >>= 1;
			tmp_q31 = __SSAT(tmp_q31, 16); // saturate 16bit
			atan = atan_approx_q15((q15_t) tmp_q31);
			return atan;
		} else {
			// calculate atan(x/y)
			tmp_q31 = (q31_t) x << 16; // q15 -> q31
			tmp_q31 += 1L << 0;
			tmp_q31 /= y; // q31 -> q16
			tmp_q31 >>= 1;
			tmp_q31 = __SSAT(tmp_q31, 16); // saturate 16bit
			atan = atan_approx_q15((q15_t) tmp_q31);
			return PI_2_Q2_13 + atan;
		}
	} else {
		if (y > x) {
			// calculate atan(y/x)
			tmp_q31 = (q31_t) y << 16; // q15 -> q31
			tmp_q31 += 1L << 0;
			tmp_q31 /= x; // q31 -> q16
			tmp_q31 >>= 1;
			tmp_q31 = __SSAT(tmp_q31, 16); // saturate 16bit
			atan = atan_approx_q15((q15_t) tmp_q31);
			return atan + (y > 0 ? PI_Q2_13 : -PI_Q2_13);
		} else {
			// calculate atan(x/y)
			tmp_q31 = (q31_t) x << 16; // q15 -> q31
			tmp_q31 += 1L << 0;
			tmp_q31 /= y; // q31 -> q16
			tmp_q31 >>= 1;
			tmp_q31 = __SSAT(tmp_q31, 16); // saturate 16bit
			atan = atan_approx_q15((q15_t) tmp_q31);
			return -PI_2_Q2_13 - atan;
		}
	}
}

// calculate atan(x) ~ x [-pi/2, pi/2]
static q15_t atan_approx_q15(q15_t x) {
	if (x > PI_2_Q2_13) {
		return PI_2_Q2_13;
	} else if (x < -PI_2_Q2_13) {
		return -PI_2_Q2_13;
	} else {
		return x;
	}
}
*/
