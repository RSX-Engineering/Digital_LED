/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "app_includer.h"

 __IO uint32_t SystickTime10us = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);



// Drivers, renderers and LED channels
#ifdef DIRECT_LED_DRIVERS
  AnalogLED_Driver_Direct driverR, driverG, driverB;
  AnalogLED_Driver_Direct driverW;
#else
  AnalogLED_Driver_pPWL driverR, driverG, driverB;
  AnalogLED_Driver_pPWL driverW;
#endif
ColorRenderer_Direct rendererRGB;
ColorRenderer_SatStat rendererWHT;
AnalogLED_Channel  RGBchannel, WHTchannel;  // 2 LED channels: RGB and white
PwmPin pwm_pin;
PixelProc pixels;


#ifdef LIGHTTEST
// Receives a battery voltage in mV and prints it in volts with two decimal places
void printFormattedVoltage(int batVoltage_mV) {
    int integerPart = batVoltage_mV / 1000; // Get the integer part of the voltage
    int fractionalPart = batVoltage_mV % 1000; // Get the fractional part of the voltage
    // Serial.print("milliVolts: "); Serial.print(batVoltage_mV); Serial.print(", ");
    // Serial.print("Integer part: "); Serial.print((int)integerPart); Serial.print(", Fractional part: "); Serial.print((int)fractionalPart); 

    // Round the fractional part to two decimal places
    fractionalPart = (fractionalPart + 5) / 10;
    if(fractionalPart >= 100) {
        fractionalPart = 0;
        integerPart++;
    }
    // Serial.print(", Fractional part rounded: "); Serial.print((int)fractionalPart); Serial.print("\n");

    // Print the integer part
    Serial.print((int)integerPart);
    Serial.print(".");

    // Print the fractional part with leading zero if necessary
    if (fractionalPart < 10) {
        Serial.print("0"); // Add leading zero if less than 10
    }
    Serial.print((int)fractionalPart);
}
#endif // LIGHTTEST

#ifdef UART_SERIAL_ENABLE
// parse serial commands
void ParseCommand(const char* cmd, const char* arg) {
  
  if(!strcmp(cmd, "version")) {   // need to include "Installed" in the answer, for automated color tester
    Serial.print("--- Digital LED version "); Serial.print(FWversion); Serial.print(" Installed at baud ");  
    Serial.print(SERIAL_BAUD); Serial.print(" ---\n");
    return;
  }
  
  if(!strcmp(cmd, "color")) {
    decoder.printDecoded = true;
    return;
  }

  if(!strcmp(cmd, "data")) {
     decoder.printData = true;
    return;
  }

#ifdef LIGHTTEST
  // if (!strcmp(cmd, "aled_chtest")) {     
  if (!strcmp(cmd, "light_colortest")) {     
    // 1. Extract RGB      
    char initial_arg[20];
    for(uint8_t i=0; i<=strlen(arg); i++) initial_arg[i] = arg[i];
    char* token = strtok(initial_arg, ",");
    uint8_t targetCh = atoi(token);   // Extract channel number (starts at 1 in command argument)
            
    token = strtok(NULL, ",");
    // Serial.print("Token = "); Serial.print(token); 
    uint8_t Red= atoi(token);   // Extract R
    token = strtok(NULL, ",");
    // Serial.print(", Token = "); Serial.print(token); 
    uint8_t Green= atoi(token);   // Extract G
    token = strtok(NULL, ",");
    // Serial.print(", Token = "); Serial.print(token); Serial.print("\n");
    uint8_t Blue= atoi(token);   // Extract B
    // Serial.print("Channel "); Serial.print(targetCh); Serial.print(" set to "); Serial.print(Red); Serial.print(", "); Serial.print(Green); Serial.print(", "); Serial.print(Blue); Serial.print("\n");

    // 2. Set channel and report
    uint32_t ctrlValue;
    switch(targetCh) {
        case 1: // set RGB channel with hexRGB control value
            HEXRGB_SET8b(ctrlValue, chR, Red); HEXRGB_SET8b(ctrlValue, chG, Green); HEXRGB_SET8b(ctrlValue, chB, Blue);
            RGBchannel.Set(ctrlValue);  
            RGBchannel.print_bright();
        break;
        
        case 2: {// set White channel with Saturation Statistics control value
            // Serial.print("Red= "); Serial.print(Red); Serial.print(", Green= "); Serial.print(Green); Serial.print(", Blue= "); Serial.print(Blue);
            uint8_t minC = Red; if (Green<minC) minC=Green; if (Blue<minC) minC=Blue;
            uint8_t maxC = Red; if (Green>maxC) maxC=Green; if (Blue>maxC) maxC=Blue;
            // Serial.print(", Min: "); Serial.print(minC); Serial.print(", Max: "); Serial.print(maxC);
            uint8_t sat = 255-maxC+minC;
            // Serial.print(", Sat: "); Serial.print(sat);
            // Serial.print("\n");
            uint16_t lum16b = (Red+Green+Blue) / 3; // need 16 bits for calculation, to prevent overflow
            uint8_t lum = (uint8_t)lum16b;
            SATSTAT_SET8b(ctrlValue, minSAT, sat); SATSTAT_SET8b(ctrlValue, avgSAT, sat); 
            SATSTAT_SET8b(ctrlValue, maxSAT, sat); SATSTAT_SET8b(ctrlValue, avgLUM, lum); 
            WHTchannel.Set(ctrlValue);
            WHTchannel.print_bright();
        } break;
        
        default: 
          Serial.print("Unknown channel index "); Serial.print(targetCh); Serial.print("\n"); 
        return;  // Wrong channel index
    }


    return;
  }

//  if (!strcmp(cmd, "aled_pintest")) {      
  if (!strcmp(cmd, "light_chtest")) {     
    char initial_arg[20];
    for(uint8_t i=0; i<strlen(arg); i++) initial_arg[i] = arg[i];
    char* token = strtok(initial_arg, ",");
    
    // 1. Identify driver
    uint8_t ledIndex = atoi(token);   
    AnalogLED_DriverInterface* ledDriver;
    switch(ledIndex) {
        case 1: ledDriver = &driverR; break;
        case 2: ledDriver = &driverG; break;
        case 3: ledDriver = &driverB; break;
        case 4: ledDriver = &driverW; break;
        default: // Serial.print("Unknown driver index "); Serial.print(ledIndex); Serial.print("\n"); 
        return;  // Wrong driver index
    }

    // 2. Extract brightness
    token = strtok(NULL, ",");
    uint16_t bri= atoi(token);   // Extract brightness   
    ledDriver->UpdateReferences(battery.batteryReading());    // Update references to current battery voltage
    // Serial.print("Emitter "); Serial.print(ledIndex); Serial.print(" set to brightness "); Serial.print(bri); 

    // 3. Set and report   
    uint16_t regVal = ledDriver->Get_regVal(bri);          // Get PWM register value (0-32767)
    pwm_pin.setChValue(ledIndex, regVal);                 // Apply 
    int batVoltage = (int)battery.batteryVoltage();       // Get battery voltage
    // Serial.print(batVoltage); Serial.print(" mV = ");
    // Serial.print(batVoltage/1000); Serial.print(".");  Serial.print((batVoltage%1000)); 
    printFormattedVoltage(batVoltage); 
    #ifdef LIGHTTEST
      Serial.print(", ");    // automatic tester gets confused by 'V'
    #else
      Serial.print("V, "); 
    #endif
    Serial.print((int)regVal); Serial.print("\n");
    return;
}  
#endif // LIGHTTEST

  if(!strcmp(cmd, "battery")) {
    Serial.print("Raw = "); Serial.print(battery.batteryReading()); Serial.print(", "); Serial.print("Converted = "); 
    #ifdef LIGHTTEST
      Serial.print((int)battery.batteryVoltage());  // Voltage in [mV]
    #else // no LIGHTTEST means voltage is not available in mV, need to calculate it here
      float bat_mv = 3000.0f + (12000.0f / (ADCRAW_BAT42-ADCRAW_BAT30))  * ((float)battery.batteryReading() - ADCRAW_BAT30);    // ADC reading in [mV]
      Serial.print((int)bat_mv);
    #endif
    Serial.print(" mV\n");
    return;
	}
#ifdef BATMON_ENABLE_TEMP
  if(!strcmp(cmd, "temp")) {
    Serial.print("TRaw = "); Serial.print(battery.tempReading()); Serial.print(", "); Serial.print("Temperature = "); Serial.print((int)battery.tempC()); Serial.print(" C\n");
    return;
  }
#endif

  Serial.print("Unknown command "); Serial.print(cmd); Serial.print(".\n");
}
#endif // UART_SERIAL_ENABLE


bool callbackFunct_test(uint8_t* data, uint16_t nBytes)
{
	// printf("Working callback , nrBytes %d" , nBytes);
//	for(uint8_t i = 0; i< nBytes; i++)
//	{
//		Serial.print("\n r= ");  Serial.print((int)data[i * 3 + 1]);
//		Serial.print("g= ");  Serial.print((int)data[i * 3]);
//		Serial.print("b= ");  Serial.print((int)data[i * 3 + 2]);
//	}
	return pixels.Process(data, nBytes);
}

int main(void)
{
  
  uint32_t sTime = 0;
  // Configure the system clock
  SystemClock_Config();


#ifdef UART_SERIAL_ENABLE
  Serial.begin(SERIAL_BAUD);
  Serial.print("\n");
  ParseCommand("version", 0);
#endif


  sTime = SystickTime10us;
#ifndef SIGDECODER_PIN_TEST_USE
  pwm_pin.Setup();
#endif
  battery.Setup();
  battery.Activate();
  decoder.Setup();
  decoder.setCallback(&callbackFunct_test);	// it wil call pixels.Process ( it will need to be a static function and also its member to use it as callback
  	  	  	  	  	  	  	  	  	  	  	// leave it as is for now
 
  // Initialize drivers, renderers and LED channels
  #ifdef DIRECT_LED_DRIVERS
    driverR.Init(0, 0); 
    driverG.Init(0, 0); 
    driverB.Init(0, 0); 
    driverW.Init(0, 0); 
  #else
    driverR.Init((void*)DRIVER_RED[0], NELEM(DRIVER_RED)); 
    driverG.Init((void*)DRIVER_GREEN[0], NELEM(DRIVER_GREEN)); 
    driverB.Init((void*)DRIVER_BLUE[0], NELEM(DRIVER_BLUE)); 
    driverW.Init((void*)DRIVER_WHITE[0], NELEM(DRIVER_WHITE)); 
  #endif

  RGBchannel.AssignDriver(1, &driverR);   // Red driver on LED1
  RGBchannel.AssignDriver(2, &driverG);   // Green driver on LED2
  RGBchannel.AssignDriver(3, &driverB);   // Blue driver on LED3
  rendererRGB.Init(0); 
  RGBchannel.AssignRenderer(&rendererRGB);

  WHTchannel.AssignDriver(4, &driverW);   // White driver on LED4
  rendererWHT.Init(0); 
  WHTchannel.AssignRenderer(&rendererWHT);


  // uint8_t rawpixeldata[1*3];
  // uint32_t Time = SystickTime10us;
  // pixels.Process(&rawpixeldata[0], 1*3);
  // Time = SystickTime10us - Time;  // tick at 0.01 ms
  // Serial.print("rawdatapixel starts at "); Serial.print((int)&rawpixeldata[0]); Serial.print(" and ends at "); Serial.print((int)&rawpixeldata[1*3]); Serial.print("\n");
  // Serial.print("Processing time: "); Serial.print((int)(Time/100)); Serial.print(".");  Serial.print((int)(Time%100));  Serial.print(" ms\n");
  // Serial.print("Processing time: "); Serial.print((int)Time); Serial.print(" ticks\n");


  // uint32_t ctrlValue;
  // HEXRGB_SET8b(ctrlValue, chR, 100); HEXRGB_SET8b(ctrlValue, chG, 127); HEXRGB_SET8b(ctrlValue, chB, 200);
  // RGBchannel.Set(ctrlValue);
  // WHTchannel.Set(ctrlValue);

  // SATSTAT_SET8b(ctrlValue, minSAT, pixels.minSat); 
  // SATSTAT_SET8b(ctrlValue, avgSAT, pixels.avgSat); 
  // SATSTAT_SET8b(ctrlValue, maxSAT, pixels.maxSat); 
  // SATSTAT_SET8b(ctrlValue, avgLUM, pixels.avgLum); 
  // WHTchannel.Set(ctrlValue);



  /* Infinite loop */
  while (1)
  {
	  decoder.Loop();
	  battery.Loop();

	  // call every 10 ms
    if(SystickTime10us - sTime > 1000)
    // if(SystickTime10us - sTime > 10000) // @100 ms to print ADC readings
	  {
#ifdef UART_SERIAL_ENABLE
		  parser.Loop();
#endif
    
		// #ifdef BATMON_ENABLE_TEMP    
    //   Serial.print((int)battery.batteryVoltage()); Serial.print("mV, "); Serial.print((int)battery.tempC()); Serial.print("C\n");
    // #else
    //   Serial.print((int)battery.batteryVoltage()); Serial.print("\n");
    // #endif

#ifndef LIGHTTEST   // don't update in test mode, LEDs will be controlled remotely
    uint32_t ctrlValue;
    HEXRGB_SET8b(ctrlValue, chR, pixels.avgR); HEXRGB_SET8b(ctrlValue, chG, pixels.avgG); HEXRGB_SET8b(ctrlValue, chB, pixels.avgB);
    RGBchannel.Set(ctrlValue);    // refresh RGB emitters
  

    SATSTAT_SET8b(ctrlValue, minSAT, pixels.minSat); 
    SATSTAT_SET8b(ctrlValue, avgSAT, pixels.avgSat); 
    SATSTAT_SET8b(ctrlValue, maxSAT, pixels.maxSat); 
    SATSTAT_SET8b(ctrlValue, avgLUM, pixels.avgLum); 
    // WHTchannel.Set(ctrlValue);  
#endif 
  


		  sTime = SystickTime10us;      
	  }

  }

}

/**	TODO : maybe make a separate file for this function , not to sit in main.c
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  //Reset of all peripherals, Initializes the Flash interface and the Systick
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  // SysTick_IRQn interrupt configuration
  NVIC_SetPriority(SysTick_IRQn, 3);
  NVIC_EnableIRQ(SysTick_IRQn);
  // FLASH LATENCY 1 , because will work on 48 MHZ
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  // HSI configuration and activation
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1){}
  // default calibration
  LL_RCC_HSI_SetCalibTrimming(64);
  // HSI DIV 1 , for 48 MHZ
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
  // Set AHB prescaler
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_DIV_1);

  // Sysclk activation on the HSI
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI){}
  // Set APB1 prescaler
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  // Init 1ms TICK
  // LL_Init1msTick(48000000);
  LL_Init10usTick(48000000);
  // Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function)
  LL_SetSystemCoreClock(48000000);

}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  // default error handler
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
