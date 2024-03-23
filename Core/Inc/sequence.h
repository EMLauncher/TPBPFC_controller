/*
 * sequence.h
 *
 *  Created on: Nov 25, 2023
 *      Author: eml_t
 */

#ifndef INC_SEQUENCE_H_
#define INC_SEQUENCE_H_

#include "main.h"

#include "my_const_values.h"
#include "SOGI_PLL.h"
#include "PFC_controller.h"

typedef struct {
	__IO uint8_t flags;
	__IO uint8_t softOCP_cnt;
	__IO uint8_t UVLO_cnt;
	__IO uint8_t OVP2_cnt;
} status_inst;

void init_status(status_inst *stat);
void detectSoftOCP(status_inst *stat, PFC_ctrl_inst_q15 *PC);
void detectHardOCP(status_inst *stat);
void detectUVLO(status_inst *stat, SOGI_PLL_inst_q15 *sogipll);
void detectOVP(status_inst *stat, PFC_ctrl_inst_q15 *PC);

#endif /* INC_SEQUENCE_H_ */
