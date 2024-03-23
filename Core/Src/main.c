/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_math_add.h"
#include "SOGI_PLL.h"
#include "PFC_controller.h"
#include "sequence.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "my_const_values.h"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN PV */
volatile uint16_t ADC_results[3] = {0};
volatile q15_t eu = 0, Iin = 0, Vo = 32767; // debug
volatile uint32_t timeCnt = 0, subTimeCnt = 0; // debug timeCnt++ per 200us
SOGI_PLL_inst_q15 sogipll;
PFC_ctrl_inst_q15 PFCctrl;
status_inst stat;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
static void init_registers(void);
static void start_sequence(void);
static q15_t ADC2Vin(q15_t ADC);
static q15_t ADC2Iin(q15_t ADC);
static q15_t ADC2Vo(q15_t ADC);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char buf[256];
	uint16_t len = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  init_registers();
  init_status(&stat);
  init_SOGI_PLL(&sogipll);

  PFCctrl.id_tar = 0;
  PFCctrl.iq_tar = 0;
  init_PFC_ctrl(&PFCctrl);
  len = sprintf(buf, "TPB-PFC v010\n\rsoftware v001\n\rdesigned by EMLauncher\n\r");
  CDC_Transmit_FS((uint8_t *)buf, len);
  len = sprintf(buf, "initializing ...");
  CDC_Transmit_FS((uint8_t *)buf, len);
  start_sequence();
  len = sprintf(buf, "OK\n\r");
  CDC_Transmit_FS((uint8_t *)buf, len);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  len = sprintf(buf, "%ld,%d,%d,%d,%d\n\r", timeCnt, sogipll.ea, Iin, Vo, stat.flags);
	  // len = sprintf(buf, "%d\n\r", status);
	  CDC_Transmit_FS((uint8_t *)buf, len);
	  // HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	HAL_RCC_DeInit();
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void init_registers(void) {
	// initialize GPIO
	GPIOB->ODR |= PORTB_STOP_Msk; // cut off all PWM ports
	GPIOB->ODR &= ~PORTB_S84_Msk; // 84 relay OFF

	// initialize ADC1
	ADC1->CR2 &= ~ADC_CR2_ADON; // ADC off
	// ADC1->CR1 |= (ADC_CR1_JEOCIE | ADC_CR1_EOCIE); // end of regular/injected conversion interrupt enable (for DMA)
	ADC1->CR2 &= ~ADC_CR2_CONT; // single conversion mode
	ADC1->CR2 |= (ADC_CR2_EXTTRIG | ADC_CR2_JEXTTRIG); // external trigger mode for regular/injected channels
	ADC1->CR2 |= (7U << ADC_CR2_EXTSEL_Pos); // external trigger for regular channels: SWSTART
	ADC1->CR2 |= (7U << ADC_CR2_JEXTSEL_Pos); // external trigger for injected channels: JSWSTART
	// ADC1->CR2 |= ADC_CR2_DMA; // enable DMA
	ADC1->SMPR2 |= (3U << ADC_SMPR2_SMP2_Pos) | (3U << ADC_SMPR2_SMP0_Pos); // 28.5 cycles ch2, ch0
	ADC1->SQR1 |= (0U << ADC_SQR1_L_Pos); // regular channel sequence length: 1
	ADC1->SQR3 |= (2U << ADC_SQR3_SQ1_Pos); // regular channel sequence: ch2,
	ADC1->JSQR |= (0U << ADC_JSQR_JL_Pos); // injected channel sequence length: 1
	ADC1->JSQR |= (0U << ADC_JSQR_JSQ4_Pos); // injected channel sequence: ch0, (caution: JSQ4 -> JSQ3 ->JSQ2 -> JSQ1)
	ADC1->CR2 |= ADC_CR2_ADON; // ADC on
	ADC1->CR2 |= ADC_CR2_RSTCAL; // reset CAL registers
	while (ADC1->CR2 & ADC_CR2_RSTCAL);
	ADC1->CR2 |= ADC_CR2_CAL; // CAL
	while (ADC1->CR2 & ADC_CR2_CAL);
	// NVIC_EnableIRQ(ADC1_2_IRQn); // enable interrupt for NVIC

	// initialize ADC2 (slave of ADC1)
	ADC2->CR2 &= ~ADC_CR2_ADON; // ADC off
	ADC2->CR1 |= ADC_CR1_DISCEN; // reset CR2
	ADC2->CR2 |= ADC_CR2_EXTTRIG; // external trigger mode for regular channels
	ADC2->CR2 |= (7U << ADC_CR2_EXTSEL_Pos); // external trigger for regular channels: SWSTART
	ADC2->SMPR2 |= (3U << ADC_SMPR2_SMP1_Pos); // 28.5 cycles ch1
	ADC2->SQR1 |= (0U << ADC_SQR1_L_Pos); // regular channel sequence length: 1
	ADC2->SQR3 |= (1U << ADC_SQR3_SQ1_Pos); // regular channel sequence: ch1,
	ADC2->CR2 |= ADC_CR2_ADON; // ADC on
	ADC2->CR2 |= ADC_CR2_RSTCAL; // reset CAL registers
	while (ADC2->CR2 & ADC_CR2_RSTCAL);
	ADC2->CR2 |= ADC_CR2_CAL; // CAL
	while (ADC2->CR2 & ADC_CR2_CAL);

	ADC1->CR1 |= (1U << ADC_CR1_DUALMOD_Pos); // dual mode: Combined regular simultaneous + injected simultaneous mode

	/* memo: read regular channels
	ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion for regular channels
	while(!(ADC1->SR & ADC_SR_EOC)); // wait
	ADC_results[0] = ADC1->DR & ADC_DR_DATA; // result of ADC1 for regular channels
	ADC_results[1] = ADC2->DR; // result of ADC2 for regular channels
	*/

	/* memo: read injected channels
	ADC1->CR2 |= ADC_CR2_JSWSTART; // start conversion for injected channels
	while(!(ADC1->SR & ADC_SR_JEOC)); // wait
	ADC_results[2] = ADC1->JDR1; // result of ADC1 for injected channels (first sequence)
	*/

	// initialize TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // clock enable
	TIM1->CR1 |= TIM_CR1_ARPE; // ARR is buffered.
	TIM1->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR); // edge-aligned mode, upcounter
	TIM1->CR1 |= TIM_CR1_URS; // only overflow generates UEV
	TIM1->DIER |= TIM_DIER_UIE; // interrupt enable.
	TIM1->PSC = 1 - 1; // prescaler 1/1
	TIM1->ARR = DUTY_ONE - 1; // period=(ARR+1)*1/72[us]=25[us]
	TIM1->RCR = 1; // UEV generates by 2 times overflow
	TIM1->BDTR |= (0x0000 + 72) << TIM_BDTR_DTG_Pos; // dead-time=(0+72)*1/72[us]
	// TIM1->BDTR |= TIM_BDTR_BKE; // BRK input enable
	// TIM1->BDTR |= TIM_BDTR_BKP; // BRK is active-high
	// TIM1->DIER |= TIM_DIER_BIE; // BRK interrupt enable
	// TIM1->BDTR |= TIM_BDTR_OSSI;
	TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE; // CC1:PWM mode 1, preload, fast mode
	TIM1->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2FE;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE; // CC1, CC1N:output enable
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;
	TIM1->CCR1 = 1500;
	NVIC_EnableIRQ(TIM1_UP_IRQn); // enable TIM1 update IRQHandler

	// initialize TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // clock enable
	TIM2->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM2->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR); // edge-aligned mode, upcounter
	TIM2->CR1 |= TIM_CR1_URS; // only overflow generates UEV
	TIM2->DIER |= TIM_DIER_UIE; // interrupt enable
	TIM2->PSC = 72 - 1; // prescaler 1/72
	TIM2->ARR = 1000 - 1; // period=1ms
	TIM2->RCR = 0;
	NVIC_EnableIRQ(TIM2_IRQn);
}

static void start_sequence(void) {
	q31_t tmp_q31;

	// 1. start to calculate SOGI-PLL
	ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion for regular channels
	TIM1->CR1 |= TIM_CR1_CEN; // counter enable
	// マイコンの電源だけ先に入っている場合に備えて入力電圧が上昇するまで待機
	while(sogipll.eu < VDQ_UVLO) {}
	HAL_Delay(300); // wait 300ms 15cycles@50Hz for charging main cap.
	GPIOB->ODR |= PORTB_S84_Msk; // turn on 84 relay
	HAL_Delay(15); // 二次突入電流の収束を待つ

	// 2. Voの初期値計測
	tmp_q31 = 0L;
	for (uint8_t i = 0; i < VO_SAMPLE_CNT; i++) {
		ADC1->CR2 |= ADC_CR2_JSWSTART; // start conversion for injected channels
		while(!(ADC1->SR & ADC_SR_JEOC)); // wait
		ADC_results[2] = ADC1->JDR1; // result of ADC1 for injected channels (first sequence)
		tmp_q31 += ADC_results[2];
		HAL_Delay(2);
	}
	tmp_q31 /= VO_SAMPLE_CNT;
	Vo = ADC2Vo((q15_t)tmp_q31);
	PFCctrl.voPI.y_target = Vo; // target of Vo initialized as measured Vo
	PFCctrl.voPI.y_target += (q15_t)VO_TARGET_START_MERGIN;

	// 3. SOGI-PLLの収束判定
	HAL_Delay(1000); // テキトーに1秒待つ
	stat.flags |= PLL_LOCKED;

	// 4. synchronize Line Phase. ea=+0(theta=pi/2)からスタートする
	while(sogipll.ea > -VDQ_UVLO);
	while(sogipll.ea < 0);

	// 5. 制御器の初期化
	TIM1->DIER &= ~TIM_DIER_UIE; // update interrupt disable
	TIM2->DIER &= ~TIM_DIER_UIE;

	// 初期値の設定
	set_init_PFC_ctrl(&PFCctrl, &sogipll, Vo);
	timeCnt = 0L;

	// 電圧制御器起動
	TIM2->CR1 |= TIM_CR1_CEN;

	TIM1->DIER |= TIM_DIER_UIE; // update interrupt enable
	TIM2->DIER |= TIM_DIER_UIE;

	GPIOB->ODR &= ~PORTB_STOP_Msk; // enable all PWM ports
	TIM1->BDTR |= TIM_BDTR_AOE; // MOE after next UEV
}

// TIM1 update interrupt
void TIM1_UP_IRQHandler(void) {
	TIM1->DIER &= ~TIM_DIER_UIE; // update interrupt disable
	// GPIOC->ODR |= GPIO_ODR_ODR13; // PC13 H debug

	while(!(ADC1->SR & ADC_SR_EOC)); // wait
	ADC_results[0] = ADC1->DR & ADC_DR_DATA; // result of ADC1 for regular channels
	ADC_results[1] = ADC2->DR; // result of ADC2 for regular channels
	eu = ADC2Vin(ADC_results[0]);
	update_SOGI_PLL(&sogipll, eu);
	// set_id_synchro(&PFCctrl, &sogipll);
	// debug
	/*
	if (sogipll.state == SOGI_PLL_PICTRL) {
		GPIOC->ODR |= GPIO_ODR_ODR13; // PC13 H debug
	} else {
		GPIOC->ODR &= ~GPIO_ODR_ODR13; // PC13 L debug
	}
	*/

	Iin = ADC2Iin(ADC_results[1]);
	update_PFC_ctrl_iab(&PFCctrl, &sogipll, Iin);

	detectSoftOCP(&stat, &PFCctrl);
	detectHardOCP(&stat);
	detectUVLO(&stat, &sogipll);

	calc_duty(&PFCctrl);
	TIM1->CCR1 = PFCctrl.duty;
	TIM1->CCR2 = PFCctrl.sig;

	ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion for regular channels

	/* debug */
	subTimeCnt++;
	subTimeCnt %= 4;
	timeCnt += subTimeCnt;
	/* debug END */

	// GPIOC->ODR &= ~GPIO_ODR_ODR13; // PC13 L debug
	TIM1->SR &= ~TIM_SR_UIF; // clear update flag
	TIM1->DIER |= TIM_DIER_UIE; // update interrupt enable
}

// TIM2 interrupt
void TIM2_IRQHandler(void) {
	TIM2->DIER &= ~TIM_DIER_UIE;

	if (ADC1->SR & ADC_SR_JEOC) { // 変換完了
		// 普通、変換は完了している
		ADC_results[2] = ADC1->JDR1; // result of ADC1 for injected channels (first sequence)
		Vo = ADC2Vo(ADC_results[2]);
	} else {
		// dot nothing
	}

	// 出力電圧PI制御
	update_PFC_ctrl_vo(&PFCctrl, Vo);
	detectOVP(&stat, &PFCctrl);
	PFCctrl.id_tar = PFCctrl.voPI.u; // id目標値の更新

	ADC1->CR2 |= ADC_CR2_JSWSTART; // start conversion for injected channels

	TIM2->SR &= ~TIM_SR_UIF;
	TIM2->DIER |= TIM_DIER_UIE;
}

static q15_t ADC2Vin(q15_t ADC) {
	q31_t tmp_q31;

	tmp_q31 = (q31_t)ADC - VIN_OFFSET; // 2^12==1.0[V/V]
	tmp_q31 *= VIN_GAIN_Q4_11;
	tmp_q31 += 1L << 10;
	tmp_q31 >>= 11; // 2^15==1000V
	return (q15_t)tmp_q31;
}

static q15_t ADC2Iin(q15_t ADC) {
	q31_t tmp_q31;

	tmp_q31 = (q31_t)ADC - IIN_OFFSET;
	tmp_q31 *= IIN_GAIN_Q4_11;
	tmp_q31 += 1L << 10;
	tmp_q31 >>= 11; // 2^15==100A
	return (q15_t)tmp_q31;
}

static q15_t ADC2Vo(q15_t ADC) {
	q31_t tmp_q31;

	tmp_q31 = (q31_t)ADC - VO_OFFSET;
	tmp_q31 *= VO_GAIN_Q4_11;
	tmp_q31 += 1L << 10;
	tmp_q31 >>= 11;
	return (q15_t)tmp_q31;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
