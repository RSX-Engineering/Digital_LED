/*
 * BatMon.cpp
 *
 *  Created on: Apr 12, 2024
 *      Author: Cosmin
 */

#include "BatMon.h"
#include "main.h"
#include "string.h"
#include "UartSerial.h"

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

  /* Timeout values for ADC operations. */
  /* (calibration, enable settling time, disable settling time, ...)          */
  /* Values defined to be higher than worst cases: low clock frequency,       */
  /* maximum prescalers.                                                      */
  /* Note: ADC channel configuration ready (ADC_CHANNEL_CONF_RDY_TIMEOUT_MS)  */
  /*       is added in CubeMx code section.                                   */
  /* Unit: ms                                                                 */
  #define ADC_CALIBRATION_TIMEOUT_MS       (   1UL)
  #define ADC_ENABLE_TIMEOUT_MS            (   1UL)
  #define ADC_DISABLE_TIMEOUT_MS           (   1UL)
  #define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1UL)
  #define ADC_CONVERSION_TIMEOUT_MS        (4000UL)

  /* Delay between ADC end of calibration and ADC enable.                     */
  /* Delay estimation in CPU cycles: Case of ADC enable done                  */
  /* immediately after ADC calibration, ADC clock setting slow                */
  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
  /* (CPU clock / ADC clock) is above 32.                                     */
  #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       (3000UL)

/* Definitions of data related to this example */
  /* Init variable out of expected ADC conversion data range */

  #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)

  #define CH_BATTERY LL_ADC_CHANNEL_12
  #define CH_TEMP	 LL_ADC_CHANNEL_TEMPSENSOR
  #define TEMP_CALIBRATION_ADD 0x1FFF7568

  #define Avg_Slope  (float)2.53
  #define Avg_Slope_Code (float)(((float)Avg_Slope * (float)4096)/(float)3000)
  #define TS_CAL1_TEMP 30

BatMon::BatMon() {
	// TODO Auto-generated constructor stub
	uhADCxReading = 0; /* ADC group regular conversion data */

	/* Variables for ADC conversion data computation to physical values */
	uhADCxReading_Voltage_mVolt = 0;  /* Value of voltage calculated from ADC conversion data (unit: mV) */
#ifdef BATMON_ENABLE_TEMP
	activeCh = CH_BATTERY;
#endif
}

BatMon::~BatMon() {
	// TODO Auto-generated destructor stub
}
/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @param  None
  * @retval None
  */
void BatMon::Activate() {
	  __IO uint32_t wait_loop_index = 0U;
	  __IO uint32_t backup_setting_adc_dma_transfer = 0U;
	  #if (USE_TIMEOUT == 1)
	  uint32_t Timeout = 0U; /* Variable used for timeout management */
	  #endif /* USE_TIMEOUT */

	  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

	  /* Note: Hardware constraint (refer to description of the functions         */
	  /*       below):                                                            */
	  /*       On this STM32 series, setting of these features is conditioned to  */
	  /*       ADC state:                                                         */
	  /*       ADC must be disabled.                                              */
	  /* Note: In this example, all these checks are not necessary but are        */
	  /*       implemented anyway to show the best practice usages                */
	  /*       corresponding to reference manual procedure.                       */
	  /*       Software can be optimized by removing some of these checks, if     */
	  /*       they are not relevant considering previous settings and actions    */
	  /*       in user application.                                               */
	  if (LL_ADC_IsEnabled(ADC1) == 0)
	  {
	    /* Enable ADC internal voltage regulator */
	    LL_ADC_EnableInternalRegulator(ADC1);

	    /* Delay for ADC internal voltage regulator stabilization.                */
	    /* Compute number of CPU cycles to wait for, from delay in us.            */
	    /* Note: Variable divided by 2 to compensate partially                    */
	    /*       CPU processing cycles (depends on compilation optimization).     */
	    /* Note: If system core clock frequency is below 200kHz, wait time        */
	    /*       is only a few CPU processing cycles.                             */
	    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	    while(wait_loop_index != 0)
	    {
	      wait_loop_index--;
	    }

	    /* Disable ADC DMA transfer request during calibration */
	    /* Note: Specificity of this STM32 series: Calibration factor is          */
	    /*       available in data register and also transferred by DMA.          */
	    /*       To not insert ADC calibration factor among ADC conversion data   */
	    /*       in DMA destination address, DMA transfer must be disabled during */
	    /*       calibration.                                                     */
	    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
	    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

	    /* Run ADC self calibration */
	    LL_ADC_StartCalibration(ADC1);

	    /* Poll for ADC effectively calibrated */
	    #if (USE_TIMEOUT == 1)
	    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
	    #endif /* USE_TIMEOUT */

	    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
	    {
	    #if (USE_TIMEOUT == 1)
	      /* Check Systick counter flag to decrement the time-out value */
	      if (LL_SYSTICK_IsActiveCounterFlag())
	      {
	        if(Timeout-- == 0)
	        {
	          /* Error: Time-out */
	          Error_Handler();
	        }
	      }
	    #endif /* USE_TIMEOUT */
	    }

	    /* Restore ADC DMA transfer request after calibration */
	    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

	    /* Delay between ADC end of calibration and ADC enable.                   */
	    /* Note: Variable divided by 2 to compensate partially                    */
	    /*       CPU processing cycles (depends on compilation optimization).     */
	    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
	    while(wait_loop_index != 0)
	    {
	      wait_loop_index--;
	    }

	    /* Enable ADC */
	    LL_ADC_Enable(ADC1);

	    /* Poll for ADC ready to convert */
	    #if (USE_TIMEOUT == 1)
	    Timeout = ADC_ENABLE_TIMEOUT_MS;
	    #endif /* USE_TIMEOUT */

	    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
	    {
	    #if (USE_TIMEOUT == 1)
	      /* Check Systick counter flag to decrement the time-out value */
	      if (LL_SYSTICK_IsActiveCounterFlag())
	      {
	        if(Timeout-- == 0)
	        {
	          /* Error: Time-out */
	          Error_Handler();
	        }
	      }
	    #endif /* USE_TIMEOUT */
	    }

	    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
	    /*       status afterwards.                                               */
	    /*       This flag should be cleared at ADC Deactivation, before a new    */
	    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
	  }

	  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
	  /* Note: No operation on ADC group regular performed here.                  */
	  /*       ADC group regular conversions to be performed after this function  */
	  /*       using function:                                                    */
	  /*       "LL_ADC_REG_StartConversion();"                                    */

	  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
	  /* Note: Feature not available on this STM32 series */
}
#ifdef BATMON_ENABLE_TEMP
/*
*	@param CH_BATTERY , CH_TEMP
*
*/
bool BatMon::ChangeChannel(uint32_t Channel)
{
	 /** Configure Regular Channel
	  */
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, Channel);

	/* Poll for ADC channel configuration ready */
	while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0){}	 // TODO add timeout managment
	/* Clear flag ADC channel configuration ready */
	LL_ADC_ClearFlag_CCRDY(ADC1);
	LL_ADC_SetChannelSamplingTime(ADC1, Channel, LL_ADC_SAMPLINGTIME_COMMON_1);

	  /* Configuration of ADC interruptions */
	  /* Enable interruption ADC group regular end of unitary conversion */
	LL_ADC_EnableIT_EOC(ADC1);

	/* Configuration of ADC interruptions */
	/* Enable interruption ADC group regular overrun */
	LL_ADC_EnableIT_OVR(ADC1);
	activeCh = Channel;

	return true;
}
#endif

void BatMon::Setup() {

	  /* USER CODE BEGIN ADC1_Init 0 */
	  /* USER CODE END ADC1_Init 0 */

	  LL_ADC_InitTypeDef ADC_InitStruct = {0};
	  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);

	  /* Peripheral clock enable */
	  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

	  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	  /**ADC1 GPIO Configuration
	  PA4   ------> ADC1_IN4
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* USER CODE BEGIN ADC1_Init 1 */

	  /* USER CODE END ADC1_Init 1 */

	  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  */


	   #if (USE_TIMEOUT == 1)
	   uint32_t Timeout ; /* Variable used for Timeout management */
	   #endif /* USE_TIMEOUT */

	  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
	  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	  LL_ADC_Init(ADC1, &ADC_InitStruct);
	  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);

	   /* Poll for ADC channel configuration ready */
	   #if (USE_TIMEOUT == 1)
	   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
	   #endif /* USE_TIMEOUT */
	   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
	     {
	   #if (USE_TIMEOUT == 1)
	   /* Check Systick counter flag to decrement the time-out value */
	   if (LL_SYSTICK_IsActiveCounterFlag())
	     {
	   if(Timeout-- == 0)
	         {
	   Error_Handler();
	         }
	     }
	   #endif /* USE_TIMEOUT */
	     }
	   /* Clear flag ADC channel configuration ready */
	   LL_ADC_ClearFlag_CCRDY(ADC1);
	  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
	  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
	  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
	  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_79CYCLES_5);
	  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_79CYCLES_5);
	  LL_ADC_DisableIT_EOC(ADC1);
	  LL_ADC_DisableIT_EOS(ADC1);

	   /* Enable ADC internal voltage regulator */
	   LL_ADC_EnableInternalRegulator(ADC1);
	   /* Delay for ADC internal voltage regulator stabilization. */
	   /* Compute number of CPU cycles to wait for, from delay in us. */
	   /* Note: Variable divided by 2 to compensate partially */
	   /* CPU processing cycles (depends on compilation optimization). */
	   /* Note: If system core clock frequency is below 200kHz, wait time */
	   /* is only a few CPU processing cycles. */
	   uint32_t wait_loop_index;
	   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	   while(wait_loop_index != 0)
	     {
	   wait_loop_index--;
	     }

	  /** Configure Regular Channel
	  */
	  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, CH_BATTERY);

	   /* Poll for ADC channel configuration ready */
	   #if (USE_TIMEOUT == 1)
	   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
	   #endif /* USE_TIMEOUT */
	   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
	     {
	   #if (USE_TIMEOUT == 1)
	   /* Check Systick counter flag to decrement the time-out value */
	   if (LL_SYSTICK_IsActiveCounterFlag())
	     {
	   if(Timeout-- == 0)
	         {
	   Error_Handler();
	         }
	     }
	   #endif /* USE_TIMEOUT */
	     }
	   /* Clear flag ADC channel configuration ready */
	   LL_ADC_ClearFlag_CCRDY(ADC1);
	  LL_ADC_SetChannelSamplingTime(ADC1, CH_BATTERY, LL_ADC_SAMPLINGTIME_COMMON_1);
#ifdef BATMON_ENABLE_TEMP
	  activeCh = CH_BATTERY;
	  calibrationValue = *((uint16_t*)(TEMP_CALIBRATION_ADD));
	  LL_ADC_SetCommonPathInternalChAdd(ADC1_COMMON, LL_ADC_PATH_INTERNAL_TEMPSENSOR);
#endif

	  /* Configuration of ADC interruptions */
	  /* Enable interruption ADC group regular end of unitary conversion */
	  LL_ADC_EnableIT_EOC(ADC1);

	  /* Configuration of ADC interruptions */
	  /* Enable interruption ADC group regular overrun */
	  LL_ADC_EnableIT_OVR(ADC1);

	  /* USER CODE END ADC1_Init 2 */
}

void BatMon::ConversionStartPoll_ADC_GrpRegular(void)
{
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the function          */
  /*       below):                                                            */
  /*       On this STM32 series, setting of this feature is conditioned to    */
  /*       ADC state:                                                         */
  /*       ADC must be enabled without conversion on going on group regular,  */
  /*       without ADC disable command on going.                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
      (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    LL_ADC_REG_StartConversion(ADC1);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
    Error_Handler();
  }

  #if (USE_TIMEOUT == 1)
  Timeout = ADC_UNITARY_CONVERSION_TIMEOUT_MS;
  #endif /* USE_TIMEOUT */

  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0)
  {
  #if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        Error_Handler();
      }
    }
  #endif /* USE_TIMEOUT */
  }

  /* Clear flag ADC group regular end of unitary conversion */
  /* Note: This action is not needed here, because flag ADC group regular   */
  /*       end of unitary conversion is cleared automatically when          */
  /*       software reads conversion data from ADC data register.           */
  /*       Nevertheless, this action is done anyway to show how to clear    */
  /*       this flag, needed if conversion data is not always read          */
  /*       or if group injected end of unitary conversion is used (for      */
  /*       devices with group injected available).                          */
 LL_ADC_ClearFlag_EOC(ADC1);

}


#define ADC_OVERSAMPLE 1000	// number of samples to acquire and average as a single reading
void BatMon::Loop() {	 // the loop runs at 100ms (see in main )
	static uint32_t accumulator=0;		// add samples here before taking an average
	static uint16_t sampleCount=0;		// count the accumulated samples samples

    // ConversionStartPoll_ADC_GrpRegular();
	STATE_MACHINE_BEGIN();
	while (true) {
	while( !LL_ADC_IsEnabled(ADC1) && LL_ADC_IsDisableOngoing(ADC1) && LL_ADC_REG_IsConversionOngoing(ADC1))
	YIELD();
	
	LL_ADC_REG_StartConversion(ADC1);
	while (!LL_ADC_IsActiveFlag_EOC(ADC1)) YIELD();
	LL_ADC_ClearFlag_EOC(ADC1);

		
    /* Retrieve ADC conversion data */
	accumulator += LL_ADC_REG_ReadConversionData12(ADC1);		// accumulate 
	sampleCount++;		// count the samples
	if (sampleCount >= ADC_OVERSAMPLE)		// if we have enough samples
	{	// got enough samples, take the average and change channel if needed
		#ifdef BATMON_ENABLE_TEMP
			switch (activeCh)
			{
			case CH_BATTERY:/* measuring battery */
					uhADCxReading = accumulator/ADC_OVERSAMPLE;		// calculate the average
					ChangeChannel(CH_TEMP);
				break;

				case CH_TEMP:/* measuring temp */
					uhTMPReading = accumulator/ADC_OVERSAMPLE;		// calculate the average
					ChangeChannel(CH_BATTERY);
				break;
			default:
				break;
			}
		#else
			uhADCxReading = accumulator/ADC_OVERSAMPLE;		// calculate the average							
		#endif

		accumulator = 0;		// reset the accumulator
		sampleCount = 0;		// reset the sample count
		// Serial.print((int)(SystickTime10us/100)); Serial.print(", ");
	}




	}
	STATE_MACHINE_END();
}

#ifdef BATMON_ENABLE_TEMP
uint16_t BatMon::tempReading()
{
	return uhTMPReading;
}

float BatMon::tempC()
{
	// calibrationValue around 21
	float celcius = (((float)(uhTMPReading - calibrationValue))/Avg_Slope_Code) + TS_CAL1_TEMP;
	return celcius;
}
#endif

// Raw ADC value
uint16_t BatMon::batteryReading()
{
	return uhADCxReading;
}

#ifdef LIGHTTEST
// Voltage in mV 
float BatMon::batteryVoltage()
{
    float bat_mv = 3000.0f + (1200.0f / (ADCRAW_BAT42-ADCRAW_BAT30))  * ((float)battery.batteryReading() - ADCRAW_BAT30);    // ADC reading in [mV]
	return bat_mv;
}
#endif


BatMon battery;
