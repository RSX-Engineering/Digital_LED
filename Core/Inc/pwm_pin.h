/*
 * pwm_pin.h
 *
 *  Created on: Apr 10, 2024
 *      Author: Cosmin
 */

#ifndef SRC_PWM_PIN_H_
#define SRC_PWM_PIN_H_

#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_tim.h"
#include "stm32c0xx_ll_usart.h"
#include "stm32c0xx_ll_gpio.h"


#define PWM_PRESC	1
#define PWM_PERIOD 32767
#define PWM_NUMCH	4

class PwmPin {
public:
	PwmPin(){}
	void Setup() {
	  LL_TIM_InitTypeDef TIM_InitStruct = {0};
	  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  // We will want duty control register from 0 - 32767 , target freq is around 800Hz
	  // SYSCLK is 48Mhz and with a presc = 1 (1+1) =2  timclk = 24MHz => 24Mhz/32767 = 732.4 Hz gen pwm freq

	  // Peripheral clock enable
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	  // TIM3 interrupt Init
	  NVIC_SetPriority(TIM3_IRQn, 0);
	  NVIC_EnableIRQ(TIM3_IRQn);

	  TIM_InitStruct.Prescaler = PWM_PRESC;
	  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	  TIM_InitStruct.Autoreload = PWM_PERIOD;
	  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	  LL_TIM_Init(TIM3, &TIM_InitStruct);
	  LL_TIM_EnableARRPreload(TIM3);

	  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
	  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
	  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
	  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;	// enable
	  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	  TIM_OC_InitStruct.CompareValue = 0; //default on 0
	  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);

	  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
	  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
	  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
	  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH4);

	  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);		 // LL_TIM_TRGO_ENABLE
	  LL_TIM_DisableMasterSlaveMode(TIM3);

	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	  /**TIM3 GPIO Configuration
	  PA6   ------> TIM3_CH1
	  PA7   ------> TIM3_CH2
	  PA8   ------> TIM3_CH3
	  PB7   ------> TIM3_CH4
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  // Enable output channels
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  // Enable counter
	  LL_TIM_EnableCounter(TIM3);

//	  LL_TIM_OC_SetCompareCH1(TIM3, 0);	 // set default on 0
//	  LL_TIM_OC_SetCompareCH2(TIM3, 0);	 // set default on 0
//	  LL_TIM_OC_SetCompareCH3(TIM3, 0);	 // set default on 0
//	  LL_TIM_OC_SetCompareCH4(TIM3, 0);  // set default on 0

	}

	// set duty cycle on the channel ch = 1...4 to a value regValue = 0...32767
	void setChValue(uint8_t ch, uint16_t regValue) {
		if(regValue > PWM_PERIOD) regValue = PWM_PERIOD;
		switch(ch) {
			case 1:LL_TIM_OC_SetCompareCH1(TIM3, regValue); break;
			case 2:LL_TIM_OC_SetCompareCH2(TIM3, regValue); break;
			case 3:LL_TIM_OC_SetCompareCH3(TIM3, regValue); break;
			case 4:LL_TIM_OC_SetCompareCH4(TIM3, regValue); break;
		}
		// Serial.print(" >LED"); Serial.print(ch); Serial.print("="); Serial.print(regValue); Serial.print("< ");
	}

	void setDuty(uint8_t ch,uint8_t val) {
		if(val >= 100) setChValue(ch, PWM_PERIOD);
		else setChValue(ch, val * 327);
	}

};

#endif /* SRC_PWM_PIN_H_ */
