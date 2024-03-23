/*
 * PFC_controller.c
 *
 *  Created on: 2023/05/14
 *      Author: eml_t
 */

#include "PFC_controller.h"

// #define DEBUG_FAE_B_1

/*
 * 制御に定数などをセットする。idqの目標値は別途設定すること。
 */
void init_PFC_ctrl(PFC_ctrl_inst_q15 *PC) {
	// set 0
	PC->ia = 0; PC->ib = 0;
	PC->id = 0; PC->iq = 0;
	PC->va = 0; PC->vb = 0;
	PC->vd = 0; PC->vq = 0;

	// initialize FAE
	PC->deb = 0;

	// set constant values to id-controller & iq-controller
	PC->idPI.Kp_decimal = Q15_DECIMAL;
	PC->idPI.Kp = IDQ_PI_KP;
	PC->idPI.TsTi = IDQ_PI_TSTI;
	PC->idPI.u_max = VAB_MAX;
	PC->idPI.u_min = -VAB_MAX;
	invert_PI_ctrl(&PC->idPI); // idを増加させたい場合, vdを下げなければならない
	PC->iqPI.Kp_decimal = Q15_DECIMAL;
	PC->iqPI.Kp = IDQ_PI_KP;
	PC->iqPI.TsTi = IDQ_PI_TSTI;
	PC->iqPI.u_max = VAB_MAX;
	PC->iqPI.u_min = -VAB_MAX;
	invert_PI_ctrl(&PC->iqPI);

	// set constant values to vo-controller
	PC->voPI.Kp_decimal = Q15_DECIMAL;
	PC->voPI.Kp = VO_PI_KP;
	PC->voPI.TsTi = VO_PI_TSTI;
	PC->voPI.u_max = IDQ_MAX;
	PC->voPI.u_min = -IDQ_MAX;

	PC->vo = VO_TARGET; // 目標値で初期化

	return;
}

/*
 * PLLがロック、出力電圧が安定したのち、その測定値で制御器に初期値を設定するための関数
 */
void set_init_PFC_ctrl(PFC_ctrl_inst_q15 *PC, SOGI_PLL_inst_q15 *S, q15_t vo_init) {
	// ---------------- idq ----------------
	PC->idPI.u = vo_init; // 操作量vdを出力電圧で初期化 updateを呼び出すとvdにも代入される
	init_PI_ctrl(&PC->idPI, PC->id);
	PC->iqPI.u = 0; // 操作量vqを0で初期化
	init_PI_ctrl(&PC->iqPI, PC->iq);

	PC->va = S->ea; PC->vb = S->eb; // vab = eabで初期化 いらないかも

	// ---------------- vo ----------------
	PC->voPI.u = 0; // 操作量idを0で初期化
	init_PI_ctrl(&PC->voPI, vo_init);
	PC->vo = vo_init;

	return;
}

void update_PFC_ctrl_iab(PFC_ctrl_inst_q15 *PC, SOGI_PLL_inst_q15 *S, q15_t iin) {
	q31_t tmpX_q31, tmpY_q31;

	// ---------------- FAE ----------------
	// update deb
	tmpY_q31 = (q31_t) PC->deb; // store previous value
	PC->deb = S->eb - PC->vb;
	// FAE
#ifndef DEBUG_FAE_B_1
	tmpX_q31 = PC->deb - tmpY_q31; // Δeb[n] - Δeb[n-1]
	tmpX_q31 *= (q15_t) FAE_A;
	tmpX_q31 += 1L << 14;
	tmpX_q31 >>= 15; // a * (Δeb[n] - Δeb[n-1])
	tmpY_q31 = (q31_t) PC->ib;
	tmpY_q31 *= (q15_t) FAE_B;
	tmpY_q31 += 1L << 14;
	tmpY_q31 >>= 15; // b * ib[n-1]
	tmpX_q31 += tmpY_q31; // ib[n]
	tmpX_q31 = __SSAT(tmpX_q31, 16);
	PC->ib = (q15_t) tmpX_q31; // update ib
#endif
#ifdef DEBUG_FAE_B_1
		tmpX_q31 = PC->deb - tmpY_q31; // Δeb[n] - Δeb[n-1]
		tmpX_q31 *= (q15_t)FAE_A;
		tmpX_q31 += 1L << 14;
		tmpX_q31 >>= 15; // a * (Δeb[n] - Δeb[n-1])
		tmpX_q31 += (q31_t)PC->ib; // ib[n]
		tmpX_q31 = __SSAT(tmpX_q31, 16);
		PC->ib = (q15_t)tmpX_q31; // update ib
	#endif

	// ---------------- Park conversion (current) ----------------
	PC->ia = iin;
	tmpX_q31 = (q31_t) PC->ia * S->cos_t + (q31_t) PC->ib * S->sin_t;
	tmpX_q31 += 1L << 14;
	tmpX_q31 >>= 15;
	tmpY_q31 = (q31_t) PC->ib * S->cos_t - (q31_t) PC->ia * S->sin_t;
	tmpY_q31 += 1L << 14;
	tmpY_q31 >>= 15;
	PC->id = (q15_t) tmpX_q31;
	PC->iq = (q15_t) tmpY_q31;

	// ---------------- current PI ----------------
	PC->idPI.y_target = PC->id_tar;
	PC->iqPI.y_target = PC->iq_tar;
	update_PI_ctrl(&PC->idPI, PC->id);
	update_PI_ctrl(&PC->iqPI, PC->iq);
	PC->vd = PC->idPI.u;
	PC->vq = PC->iqPI.u;

	// ---------------- Inverse Park conversion (voltage) ----------------
	tmpX_q31 = (q31_t) PC->vd * S->cos_t - (q31_t) PC->vq * S->sin_t;
	tmpX_q31 += 1L << 14;
	tmpX_q31 >>= 15;
	tmpY_q31 = (q31_t) PC->vq * S->cos_t + (q31_t) PC->vd * S->sin_t;
	tmpY_q31 += 1L << 14;
	tmpY_q31 >>= 15;
	PC->va = (q15_t) tmpX_q31;
	PC->vb = (q15_t) tmpY_q31;

	return;
}

void update_PFC_ctrl_vo(PFC_ctrl_inst_q15 *PC, q15_t vo) {
	PC->vo = vo;

	// soft start
	if (PC->voPI.y_target < VO_TARGET) {
		PC->voPI.y_target += VO_SOFT_START_RATE;
	} else {
		PC->voPI.y_target = VO_TARGET;
	}

	update_PI_ctrl(&PC->voPI, vo);
	// PC->id_tar = PC->voPI.u; // id目標値の更新

	return;
}

/*
 * PLLの1周期ごとにid目標値を設定する関数。
 * 制御周期ごとに呼び出すこと。
 */
void set_id_synchro(PFC_ctrl_inst_q15 *PC, SOGI_PLL_inst_q15 *S) {
	static q15_t ea_p = 0; // 前ステップのea

	// eaのゼロスロス検出
	if (ea_p < 0 && S->ea > 0) {
		PC->id_tar = PC->voPI.u; // id目標値の更新
	}

	ea_p = S->ea;
	return;
}

/*
 * vaから必要なDUTY比を計算する。DUTY_ONEで規格化された値を返すため、そのまま比較レジスタに代入すればよい。
 * DUTYリミット、デッドタイム補償、あり。
 */
void calc_duty(PFC_ctrl_inst_q15 *PC) {
	q31_t tmp_q31;

	tmp_q31 = (q31_t) PC->va;
	tmp_q31 <<= 15;
	tmp_q31 += 1L << 14;
	if (PC->vo < VO_MIN_FOR_DIV) { // escape dividing by 0
		tmp_q31 /= VO_MIN_FOR_DIV;
	} else {
		tmp_q31 /= PC->vo; // va / vo (2^15==1.0)
	}
	tmp_q31 = __SSAT(tmp_q31, 16);
	tmp_q31 *= DUTY_ONE;
	tmp_q31 += 1L << 14;
	tmp_q31 >>= 15; // DUTY_ONEで規格化 (1.0==DUTY_ONE)
	tmp_q31 += PC->ia > 0 ? -FSW_TD : FSW_TD; // デッドタイム補償
	if (PC->va > 0) {
		if (tmp_q31 < DUTY_MIN) { // va>0の時Duty比が大きいほど昇圧比が小さい
			tmp_q31 = DUTY_MIN;
		}
		PC->sig = DUTY_ZERO;
	} else {
		tmp_q31 += DUTY_ONE; // 1+va/vo
		if (tmp_q31 > DUTY_MAX) {
			tmp_q31 = DUTY_MAX;
		} else if (tmp_q31 < DUTY_ZERO) {
			tmp_q31 = DUTY_ZERO; // 負の数をuintにキャストするとバグるため
		}
		PC->sig = DUTY_ONE;
	}
	PC->duty = (uint16_t)tmp_q31;

	return;
}
