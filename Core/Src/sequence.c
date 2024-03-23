/*
 * sequence.c
 *
 *  Created on: Nov 25, 2023
 *      Author: eml_t
 */

#include "sequence.h"

static void stopPWM(void);
static void turnON_S84(void);
static void turnOFF_S84(void);

void init_status(status_inst *stat) {
	stat->flags = ALL_CLEAR;
	stat->softOCP_cnt = 0;
	stat->UVLO_cnt = 0;
	stat->OVP2_cnt = 0;
}

void detectSoftOCP(status_inst *stat, PFC_ctrl_inst_q15 *PC) {
	if (PC->ia > IIN_OCP || PC->ia < -IIN_OCP) {
		stat->softOCP_cnt++;
	} else {
		stat->softOCP_cnt = 0;
	}
	if (stat->softOCP_cnt >= SOFTOCP_CNT_MAX) {
		stopPWM();
		turnOFF_S84();
		stat->flags |= OCP_FLAG;
	}
}

void detectHardOCP(status_inst *stat) {
	// hard OCPは検知するとマイコンの入力ポートがHになる仕様
	if (GPIOB->IDR & PORTB_OCP_Msk) {
		stopPWM();
		turnOFF_S84();
		stat->flags |= OCP_FLAG;
	}
}

void detectUVLO(status_inst *stat, SOGI_PLL_inst_q15 *sogipll) {
	if ((stat->flags & PLL_LOCKED) && (sogipll->ed < VDQ_UVLO)) {
		stat->UVLO_cnt++;
	} else {
		stat->UVLO_cnt = 0;
	}
	if (stat->UVLO_cnt >= UVLO_CNT_MAX) {
		stopPWM();
		turnOFF_S84();
		stat->flags |= UVLO_FLAG;
	}
}

void detectOVP(status_inst *stat, PFC_ctrl_inst_q15 *PC) {
	if (PC->vo > VO_OVP1) {
		PC->voPI.u = OVP1_ID;
	}
	if (PC->vo > VO_OVP2) {
		stat->OVP2_cnt++;
		if (stat->OVP2_cnt >= OVP2_CNT_MAX) {
			stopPWM();
			turnOFF_S84();
			stat->flags |= OVP_FLAG;
		}
	} else {
		stat->OVP2_cnt = 0;
	}
}

static void stopPWM(void) {
	GPIOB->ODR |= PORTB_STOP_Msk; // cut off all PWM ports

	// OC1
	TIM1->CCMR1 &= 0x0003 << TIM_CCMR1_OC1M_Pos;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1; // Set channel 1 to inactive level on match.
	TIM1->CCER |= TIM_CCER_CC1NP; // invert complementary output polarity

	// OC2
	TIM1->CCMR1 &= 0x0003 << TIM_CCMR1_OC2M_Pos;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM1->CCER |= TIM_CCER_CC2NP;

	TIM1->EGR |= TIM_EGR_COMG; // control update event
}

static void turnON_S84(void) {
	GPIOB->ODR |= PORTB_S84_Msk; // turn on 84 relay
}

static void turnOFF_S84(void) {
	GPIOB->ODR &= ~PORTB_S84_Msk; // turn off 84 relay
}
