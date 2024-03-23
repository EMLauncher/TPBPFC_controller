/*
 * my_const_values.h
 *
 *  Created on: 2023/05/19
 *      Author: eml_t
 */

#ifndef INC_MY_CONST_VALUES_H_
#define INC_MY_CONST_VALUES_H_

/*
 * standardized value
 * time: 1ms == 2^15
 * voltage: 1000V == 2^15
 * current: 100A == 2^15
 * angular velocity: 1000pi[rad/s] == 500Hz == 2^15
 * angle: pi[rad] == 2^15
 */

// common
#define SAMPLING_TIME 1638 // 50us
#define Q15_DECIMAL 15

// SOGI-PLL
#define OMEGA_MAX 4588 // 70Hz, maximum frequency for PI controller
#define OMEGA_MIN 2621 // 40Hz
#define SOGIPLL_K_Q4_11 6144 // 5.0
#define OMEGA_PI_KP 1252 // 30.0*(2^15/25736)/1000[rad/s/rad]
#define OMEGA_PI_TSTI 3972 // 1/55s / 0.15s * 2^15
#define VDQ_UVLO 2621 // 80V
#define V_APP0 32 // 1V

#define SOGI_PLL_RESET 0
#define SOGI_PLL_SET 1 // normal state
#define SOGI_PLL_PICTRL 2 // PI controller calculating

// current control
#define FAE_A 16302 // (1000V / 100A) / (0.1Ω + 2 * 500uH / 50us) * 2^15
#define FAE_B 32441 // (-0.1Ω + 2 * 500uH / 50us) / (0.1Ω + 2 * 500uH / 50us) * 2^15
#define IDQ_PI_KP 13107 // 4.0V/A * 100A / 1000V * 2^15
#define IDQ_PI_TSTI 328 // 50us / 5ms * 2^15
#define VAB_MAX 14746 // 450V
#define IIN_OCP 7209 // 22A

#define VIN_GAIN_Q4_11 13233 // ADC to voltage * 2^11[V/LSB] (amp gain: 0.0040858926 VADC/Vin)
#define VIN_OFFSET 2048 // ADC offset (0.5*Vcc)

#define IIN_GAIN_Q4_11 16236 // ADC to current * 2^11[A/LSB] (amp gain: 33.3mV/A)
#define IIN_OFFSET 2048 // ADC offset (1.65V)

// voltage control
#define VO_SAMPLE_CNT 8 // count of measuring Vo for initializing
#define VO_TARGET_START_MERGIN 328 // 10V 目標値の初期値を現在の出力電圧から増加させる電圧
#define VO_SOFT_START_RATE 2 // 出力目標値の増加 per 1ms (ソフトスタートのため)
/* memo
 *  9830 // 300V
 *  11796 // 360V
 *  12452 // 380V
 *  13107 // 400V
 *  13763 // 420V
 *  14746 // 450V (検出限界超える)
 */
#define VO_TARGET 11796 // 360V
#define VO_OVP1 13107 // 400V
#define OVP1_ID -328 // -1A, OVP1を検知した時にId指令値をこの値にする
#define VO_OVP2 13763 // 420V, 即時停止
#define VO_PI_KP 5120 // 0.25A/V * 1000V / 100A * 2^11
#define VO_PI_TSTI 328 // 1ms(=Ts) / 100ms(=Ti) * 2^15
#define IDQ_MAX 3932 // 12A

// #define VO_GAIN_Q4_11 7453 // ADC to voltage * 2^11[V/LSB] (amp gain: 0.007254902 VADC/Vo 実測)
#define VO_GAIN_Q4_11 7191 // ADC to voltage * 2^11[V/LSB] (amp gain: 0.007518797 VADC/Vo 理論)
#define VO_OFFSET 0 // ADC offset (0V)

#define VO_MIN_FOR_DIV 3932 // 120V voで割り算する時用の最小値 0除算を避ける
#define DUTY_ONE 1800 // duty ratio: 1.0
#define DUTY_MAX 1440 // 0.8 duty ratio limit
#define DUTY_MIN 360 // 0.2
#define DUTY_ZERO 0 // duty ratio: 0.0
#define FSW_TD 72 // fsw[Hz]*Td[s]*DUTY_ONE デッドタイム補償用

#define ALL_CLEAR 0x00
#define OCP_FLAG 0x01
#define OVP_FLAG 0x02
#define UVLO_FLAG 0x04
#define PLL_LOCKED 0x08

#define SOFTOCP_CNT_MAX 5
#define UVLO_CNT_MAX 50
#define OVP2_CNT_MAX 3

#define PORTB_S84_Msk 0x0001
#define PORTB_OCP_Msk 0x0002
#define PORTB_STOP_Msk 0x0008

#endif /* INC_MY_CONST_VALUES_H_ */
