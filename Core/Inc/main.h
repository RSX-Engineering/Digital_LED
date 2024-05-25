/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

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
#include "stm32c0xx_ll_adc.h"


#define BATMON_ENABLE_TEMP
#define UART_SERIAL_ENABLE 
#define LIGHTTEST
#ifdef LIGHTTEST  
  #define DIRECT_LED_DRIVERS        // uncomment this to use direct instead of PWL drivers (for LED charterization only)
  #define SERIAL_BAUD 9600          // need 9600 for automated color tester
#else
  #define SERIAL_BAUD 115200   
#endif

#include <stdio.h>


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//void USART_TXEmpty_Callback(void);
//void USART_CharTransmitComplete_Callback(void);
void USART_CharReception_Callback(void);
void USART_Error_Callback(void);
void HAL_DMA_ChannelX_IRQHandler(uint8_t channelNr);
extern __IO uint32_t SystickTime10us;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
