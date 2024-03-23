/*
 * PFC_controller.h
 *
 *  Created on: 2023/05/14
 *      Author: eml_t
 */

#ifndef INC_PFC_CONTROLLER_H_
#define INC_PFC_CONTROLLER_H_

#include "PI_controller.h"
#include "my_const_values.h"
#include "SOGI_PLL.h"

typedef struct {
	// initialized (Const)
	PI_ctrl_inst_q15 idPI, iqPI; // 電流制御器
	PI_ctrl_inst_q15 voPI; // 電圧制御器

	// Read Only
	__IO q15_t va, vb, vd, vq; // 2^15==1000V
	__IO q15_t ia, ib, id, iq; // 2^15==100A
	__IO uint16_t duty, sig; // duty: スイッチングレグのduty比 sig:整流レグのONOFF

	// Read / Write
	__IO q15_t id_tar, iq_tar; // 2^15==100A
	__IO q15_t vo; // 2^15==1000V, output DC voltage

	// Do NOT Access
	q15_t deb; // eb - vb
} PFC_ctrl_inst_q15;

void init_PFC_ctrl(PFC_ctrl_inst_q15 *PC);
void set_init_PFC_ctrl(PFC_ctrl_inst_q15 *PC, SOGI_PLL_inst_q15 *S, q15_t vo_init);
void update_PFC_ctrl_iab(PFC_ctrl_inst_q15 *PC, SOGI_PLL_inst_q15 *S, q15_t iin);
void update_PFC_ctrl_vo(PFC_ctrl_inst_q15 *PC, q15_t vo);
void set_id_synchro(PFC_ctrl_inst_q15 *PC, SOGI_PLL_inst_q15 *S);
void calc_duty(PFC_ctrl_inst_q15 *PC);

#endif /* INC_PFC_CONTROLLER_H_ */
