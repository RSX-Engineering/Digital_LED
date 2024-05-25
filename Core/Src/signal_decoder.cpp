/*
 * signal_decoder.c
 *
 *  Created on: Apr 4, 2024
 *      Author: Cosmin
 */

#include "signal_decoder.h"
#include "stm32c0xx.h"
#include "main.h"
#include "string.h"
#include "UartSerial.h"
#include "BatMon.h"

#define TIM_CCx_ENABLE                     0x00000001U
#define TIM_CHANNEL_1                      0x00000000U                          /*!< Capture/compare channel 1 identifier      */
#define TIM_CHANNEL_2                      0x00000004U                          /*!< Capture/compare channel 2 identifier      */

#define __HAL_DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->CCR |=  DMA_CCR_EN)

/**
  * @brief  Disable the specified DMA Channel.
  * @param __HANDLE__ DMA handle
  * @retval None
  */
#define __HAL_DMA_DISABLE(__HANDLE__)       ((__HANDLE__)->CCR &=  ~DMA_CCR_EN)


/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param __INTERRUPT__ specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval None
  */
#define __HAL_DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->CCR |= (__INTERRUPT__))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  __HANDLE__ DMA handle
  * @param __INTERRUPT__ specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval None
  */

#define __HAL_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->CCR &= ~(__INTERRUPT__))
#define __HAL_DMA_CLEAR_FLAG(__HANDLE__, __FLAG__) (DMA1->IFCR |= (__FLAG__))

#define COLOR_BITS_NR 24
#define NR_COLOR	127
#define LED_COLOR_NR   COLOR_BITS_NR * NR_COLOR + 1
#define DUTY_VALUE_0	30
#define ONE_MS  100  // 10us * 100 = 1000us => 1ms
#define US_50   5    // 10us * 5 = 50 us  

//volatile uint16_t riseData[LED_COLOR_NR];
volatile  uint8_t __attribute__((aligned(32))) fallData[LED_COLOR_NR + 4];
bool isMeasured = false;

/**
  * @brief  Handle DMA interrupt request.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval None
  */
void HAL_DMA_ChannelX_IRQHandler(uint8_t channelNr)
{
  uint32_t flag_it = DMA1->ISR;
  uint32_t source_it = DMA1_Channel1->CCR;
  uint32_t ChannelIndex = 0;
  DMA_Channel_TypeDef *hdma = DMA1_Channel1;

  if(channelNr == 1) {
	  source_it = DMA1_Channel1->CCR;
	  ChannelIndex = 0;
	  hdma = DMA1_Channel1;
  }
  else if(channelNr == 2) {
	  source_it = DMA1_Channel2->CCR;
	  ChannelIndex = 4;
	  hdma = DMA1_Channel2;
  }

  /* Half Transfer Complete Interrupt management ******************************/
  if (((flag_it & (DMA_ISR_HTIF1 << (ChannelIndex & 0x1cU))) != 0U) && ((source_it & DMA_CCR_HTIE) != 0U))
  {
    /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
    if ((hdma->CCR & DMA_CCR_CIRC) == 0U)
    {
      /* Disable the half transfer interrupt */
      __HAL_DMA_DISABLE_IT(hdma, DMA_CCR_HTIE);
    }
    /* Clear the half transfer complete flag */
    __HAL_DMA_CLEAR_FLAG(hdma, (DMA_ISR_HTIF1 << (ChannelIndex & 0x1cU)));

  }

  /* Transfer Complete Interrupt management ***********************************/
  else if ((0U != (flag_it & (DMA_ISR_TCIF1 << (ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_CCR_TCIE)))
  {
    if ((hdma->CCR & DMA_CCR_CIRC) == 0U)
    {
      /* Disable the transfer complete and error interrupt */
      __HAL_DMA_DISABLE_IT(hdma, DMA_CCR_TEIE | DMA_CCR_TCIE);

    }
    /* Clear the transfer complete flag */
    __HAL_DMA_CLEAR_FLAG(hdma, (DMA_ISR_TCIF1 << (ChannelIndex & 0x1cU)));

//    HAL_TIM_IC_CaptureCallback(channelNr);
    if (channelNr == 2)
    {
    	isMeasured = true;
    }

  }

  /* Transfer Error Interrupt management **************************************/
  else if (((flag_it & (DMA_ISR_TEIF1 << (ChannelIndex & 0x1cU))) != 0U) && ((source_it & DMA_CCR_TEIE) != 0U))
  {
    /* When a DMA transfer error occurs */
    /* A hardware clear of its EN bits is performed */
    /* Disable ALL DMA IT */
    __HAL_DMA_DISABLE_IT(hdma, (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE));

    /* Clear all flags */
    __HAL_DMA_CLEAR_FLAG(hdma, (DMA_ISR_GIF1 << (ChannelIndex & 0x1cU)));

  }
  else
  {
    /* Nothing To Do */
  }
  return;
}



void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState)
{
  uint32_t tmp;


  tmp = TIM_CCER_CC1E << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxE Bit */
  TIMx->CCER &= ~tmp;

  /* Set or reset the CCxE Bit */
  TIMx->CCER |= (uint32_t)(ChannelState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}


/**
  * @brief  Starts the TIM Input Capture measurement in DMA mode.
  * @param  htim TIM Input Capture handle
  * @param  Channel TIM Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
int8_t HAL_TIM_IC_Start_DMA(TIM_TypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
  int8_t status = 0;
//  uint32_t tmpsmcr;
  DMA_Channel_TypeDef *hdma = DMA1_Channel1;
  /* Enable the Input Capture channel */
  TIM_CCxChannelCmd(htim, Channel, TIM_CCx_ENABLE);

  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {
    	hdma = DMA1_Channel1;
    	__HAL_DMA_DISABLE(hdma);
        /* Configure DMA Channel source address */
        hdma->CPAR = (uint32_t)&htim->CCR1;

        /* Configure DMA Channel destination address */
        hdma->CMAR = (uint32_t)pData;

        hdma->CNDTR = Length;

        __HAL_DMA_DISABLE_IT(hdma, DMA_CCR_HTIE);
        __HAL_DMA_ENABLE_IT(hdma, (DMA_CCR_TCIE | DMA_CCR_TEIE));

        __HAL_DMA_ENABLE(hdma);
        LL_TIM_EnableDMAReq_CC1(TIM1);

      break;
    }

    case TIM_CHANNEL_2:
    {
    	hdma = DMA1_Channel2;
    	__HAL_DMA_DISABLE(hdma);
        /* Configure DMA Channel source address */
        hdma->CPAR = (uint32_t)&htim->CCR2;

        /* Configure DMA Channel destination address */
        hdma->CMAR = (uint32_t)pData;
        hdma->CNDTR = Length;
      /* Enable the DMA channel */
        __HAL_DMA_DISABLE_IT(hdma, DMA_CCR_HTIE);
        __HAL_DMA_ENABLE_IT(hdma, (DMA_CCR_TCIE | DMA_CCR_TEIE));

        __HAL_DMA_ENABLE(hdma);
        LL_TIM_EnableDMAReq_CC2(TIM1);

      break;
    }
    default:
      status = -1;
      break;
  }
    LL_TIM_EnableCounter(TIM1);

  /* Return function status */
  return status;
}
/**
  * @brief  Init gpio for intended use (reading input or alternate function for timer)
  * @param  true - gpio for input read
  * 	 	false - timer AF
  * @retval void
  */
static void ll_gpioInit(bool forRead)
{
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	  /**TIM1 GPIO Configuration
	  PA0   ------> TIM1_CH1
	  */
	  if(forRead) {
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
		  GPIO_InitStruct.Alternate = 0;
	  } else {
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	  }
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;

	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void ll_init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**TIM1 GPIO Configuration
//  PA0   ------> TIM1_CH1
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  ll_gpioInit(false);
  /* TIM1 DMA Init */

  /* TIM1_CH1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_TIM1_CH1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* TIM1_CH2 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_TIM1_CH2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);	 // LL_DMA_PDATAALIGN_HALFWORD

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);		 	 // LL_DMA_MDATAALIGN_HALFWORD

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_CC_IRQn, 1);
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM1);

  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_TI1FP1);
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_RESET);

  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);  // LL_TIM_IC_POLARITY_BOTHEDGE	 LL_TIM_IC_POLARITY_RISING
  LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_INDIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);


}

/**
  * @brief extract byte value from TIM capture buffer
  * @param *pfr - capture buffer
  * 		pos - start position in buffer
  * @retval converted value
  */
uint8_t extractValue(volatile uint8_t *pfr, uint16_t pos)
{
	uint16_t pStart , pStop ;
	uint8_t value = 0;
	if(pos >= LED_COLOR_NR) return 0;
	pStart = pos;
	pStop = pStart + 8;
	uint8_t ui = 7;
	for(uint16_t i = pStart; i < pStop; i ++) {
		if (pfr[i] > DUTY_VALUE_0) {
			value |= 1 << ui;
		}
		ui--;
	}
	return value;
}

/**
  * @brief : constructor default
  */
SigDecoder::SigDecoder() {
	isMeasured = false;
#ifdef UART_SERIAL_ENABLE
	printDecoded = false;
	printData = false;
#endif
	func_ptr = NULL;
}

SigDecoder::~SigDecoder() {
	// TODO Auto-generated destructor stub
}

/**
  * @brief : Handles low level init for decoding
  */
void SigDecoder::Setup() {
	ll_init();
	ll_gpioInit(true);	 // initial init for reading reset
#ifdef SIGDECODER_PIN_TEST_USE
	// added this for test
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	  GPIO_InitStruct.Alternate = 0;

	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	  GPIO_InitStruct.Alternate = 0;

	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
}

/**
  * @brief : set callback function
  * @param : pointer to function
  * @retval : true - success
  * 		  false - fail
  */
bool SigDecoder::setCallback(bool(*callback)(uint8_t* data, uint16_t nBytes))
{
	if(callback == NULL) return false;
	  func_ptr = callback;
	  return true;
}
#ifdef UART_SERIAL_ENABLE
void SigDecoder::printValues(uint16_t nrValue) {
	if(!nrValue && printData) {
			printData = false;
			for(uint16_t i = 0; i< 24; i++)
			{
				Serial.print("\ni = ");Serial.print((int)fallData[i]);
			}
	} else if(nrValue && printDecoded) {
		printDecoded = false;
		for(uint16_t i = 0; i< nrValue; i++)
		{
			Serial.print("\n r= ");  Serial.print((int)fallData[i * 3 + 1]);
			Serial.print("g= ");  Serial.print((int)fallData[i * 3]);
			Serial.print("b= ");  Serial.print((int)fallData[i * 3 + 2]);
		}
	}

}
#endif
/**
  * @brief : Contain decoder logic
  * @param : void
  * @retval : void
  */
void SigDecoder::Loop() {
	// for now block for 1 ms because we must find the reset line state
	static status state = FRMAE_SYNC;
	static uint32_t timeStamp = SystickTime10us;
	static uint32_t sTimeDecod = SystickTime10us;
  static uint32_t lastDMA_CNT = 0;
	switch(state) {
		case FRMAE_SYNC:		// case finding reset line
		{
			if(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ) {
				timeStamp = SystickTime10us;
				while(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)){
					if(SystickTime10us - timeStamp >= US_50) {
						ll_gpioInit(false);
#ifdef SIGDECODER_PIN_TEST_USE
						LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
#endif
						// reset state detected :
						state = DECODING;
						isMeasured = false;
						TIM1->CNT = 0;
						// HAL_TIM_IC_Start_DMA(TIM1, TIM_CHANNEL_1, (uint32_t*)&riseData[0], LED_COLOR_NR);

						HAL_TIM_IC_Start_DMA(TIM1, TIM_CHANNEL_2, (uint32_t*)&fallData[0], LED_COLOR_NR);
#ifdef SIGDECODER_PIN_TEST_USE
						LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
#endif
						sTimeDecod = SystickTime10us;
            lastDMA_CNT = LED_COLOR_NR;
						break;
					//				  printf("R");
					}
          else battery.Loop();
				}
			}
		}
		break;

		case DECODING:
			if(isMeasured) {
				isMeasured = false;
				ll_gpioInit(true);
				state = FRMAE_SYNC;
#ifdef UART_SERIAL_ENABLE
				printValues(0);
#endif
#ifdef SIGDECODER_PIN_TEST_USE
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
#endif
				for(uint16_t i = 0; i< NR_COLOR; i++)
				{
					fallData[i * 3] = extractValue(fallData, i * 24 + 1);				// Green
					fallData[i * 3 + 1] = extractValue(fallData, i * 24  + 8 + 1);		// Red
					fallData[i * 3 + 2] = extractValue(fallData, i * 24  + 16 + 1);		// Blue
				}
//#ifdef SIGDECODER_PIN_TEST_USE
//				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
//#endif

#ifdef UART_SERIAL_ENABLE
				printValues(NR_COLOR);
#endif
				// CALLBACK function
				if(func_ptr)(*func_ptr)((uint8_t*)fallData, NR_COLOR * 3);
				else Serial.print("\n func_ptr NULL");
#ifdef SIGDECODER_PIN_TEST_USE
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
#endif
			}
			else if(SystickTime10us - sTimeDecod > 10) {	//
				  uint32_t currentDMA_CNT = DMA1_Channel2->CNDTR;
				  if(currentDMA_CNT == LED_COLOR_NR || currentDMA_CNT == LED_COLOR_NR -1) {
					sTimeDecod = SystickTime10us;
					return;	 // not yet started just exit
				  }
				  if(lastDMA_CNT != currentDMA_CNT) {  // it means that dma managed to colect new data , we are still receiving
					lastDMA_CNT = currentDMA_CNT;
					sTimeDecod = SystickTime10us;
					return;
				  }

		    	__HAL_DMA_DISABLE(DMA1_Channel2); // disable dma
		    	__HAL_DMA_DISABLE_IT(DMA1_Channel2, (DMA_CCR_TCIE | DMA_CCR_TEIE));
		    	LL_DMA_ClearFlag_TC2(DMA1);

				ll_gpioInit(true);		// reset pin to input reading
				state = FRMAE_SYNC;		// something is wrong , go and find frame sync again
				// cehck available data carried by dma
		    	uint32_t nrCaptures = LED_COLOR_NR - currentDMA_CNT;
		    	uint32_t nrColors;
		    	nrCaptures -= nrCaptures % 24;
		    	nrColors = nrCaptures/24;
		    	if(nrColors) {

#ifdef UART_SERIAL_ENABLE
		    		printValues(0);
#endif

		    		// we can decode
					for(uint32_t i = 0; i< nrColors; i++)
					{
						fallData[i * 3] = extractValue(fallData, i * 24 );				// Green
						fallData[i * 3 + 1] = extractValue(fallData, i * 24  + 8);		// Red
						fallData[i * 3 + 2] = extractValue(fallData, i * 24  + 16);		// Blue
					}
#ifdef UART_SERIAL_ENABLE
					printValues(nrColors);
#endif

					if(func_ptr)(*func_ptr)((uint8_t*)fallData, nrColors * 3);
					else Serial.print("\n func_ptr NULL");
		    	}
			}
			break;

		default :
			break;

	}
}

SigDecoder decoder;
