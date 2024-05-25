/*
 * test2class.cpp
 *
 *  Created on: Apr 5, 2024
 *      Author: Cosmin
 */

#include "UartSerial.h"
#include <cmath>

#ifdef UART_SERIAL_ENABLE

//__IO uint8_t ubSend = 0;
//uint8_t aStringToSend[] = "STM32C0xx USART LL API Example : TX in IT mode\r\nConfiguration UART 115200 bps, 8 data bit/1 stop bit/No parity/No HW flow control\r\n";
//uint8_t ubSizeToSend = sizeof(aStringToSend);

/**
  * @brief Text strings printed on PC Com port for user information
  */
//uint8_t aTextInfoStart[] = "\r\nDigital: ready to receive cmds.\r\n";

/**
  * @brief RX buffers for storing received data
  */

#define RX_BUFFER_SIZE   64

uint8_t rx_buff[RX_BUFFER_SIZE];
//uint8_t tx_buff[RX_BUFFER_SIZE];
volatile uint16_t rx_tail;
volatile uint16_t rx_head;

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void UartSerial::uart1_ll_Init(uint32_t baudrate)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PA1   ------> USART1_RX
  PB6   ------> USART1_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */
  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 2);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = baudrate;	 // 115200
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief  This function prints user info on PC com port and initiates RX transfer
  * @param  None
  * @retval None
  */
void UartSerial::startReception(void)
{
	rx_tail = 0;
	rx_head = 0;

	/* Clear Overrun flag, in case characters have already been sent to USART */
	LL_USART_ClearFlag_ORE(USART1);

	/* Enable RXNE and Error interrupts */
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_ERROR(USART1);
}





/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
  uint16_t i = (uint16_t)(rx_head + 1) % RX_BUFFER_SIZE;
  uint8_t data8 = LL_USART_ReceiveData8(USART1);

  if (i != rx_tail) {
    rx_buff[rx_head] = data8;
    rx_head = i;
  }

}



//
///**
//  * @brief  Function to manage User push-button
//  * @param  None
//  * @retval None
//  */
//void Start_Transfer(void)
//{
//  /* Start transfer only if not already ongoing */
//  if (ubSend == 0)
//  {
//    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
//    LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]);
//
//    /* Enable TXE interrupt */
//    LL_USART_EnableIT_TXE(USART1);
//  }
//}
//
///**
//  * @brief  Function called for achieving next TX Byte sending
//  * @param  None
//  * @retval None
//  */
//void USART_TXEmpty_Callback(void)
//{
//  if (ubSend == (ubSizeToSend - 1))
//  {
//    /* Disable TXE interrupt */
//    LL_USART_DisableIT_TXE(USART1);
//
//    /* Enable TC interrupt */
//    LL_USART_EnableIT_TC(USART1);
//  }
//
//  /* Fill TDR with a new char */
//  LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]);
//}
//
///**
//  * @brief  Function called at completion of last byte transmission
//  * @param  None
//  * @retval None
//  */
//void USART_CharTransmitComplete_Callback(void)
//{
//  if (ubSend == sizeof(aStringToSend))
//  {
//    ubSend = 0;
//
//    /* Disable TC interrupt */
//    LL_USART_DisableIT_TC(USART1);
//
//  }
//}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void USART_Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);

  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
  }
}

/* USER CODE END 4 */

UartSerial::UartSerial() {
	// TODO Auto-generated constructor stub

}

UartSerial::~UartSerial() {
	// TODO Auto-generated destructor stub
}

void UartSerial::begin(uint32_t baud = 115200) {
	uart1_ll_Init(baud);
	startReception();
}

//void UartSerial::HandleContinuousReception() {
//	  /* Checks if Buffer full indication has been set */
//	  if (uwBufferReadyIndication != 0)
//	  {
//	    /* Call user Callback in charge of consuming data from filled buffer */
//	    UserDataTreatment(pBufferReadyForUser, uwBufferReadyIndication);
//	    uwBufferReadyIndication = 0;
//	  }
//}


int UartSerial::available(void)
{
  return ((uint16_t)(RX_BUFFER_SIZE + rx_head - rx_tail)) % RX_BUFFER_SIZE;
}

int UartSerial::peek(void)
{
  if (rx_head == rx_tail) {
    return -1;
  } else {
    return rx_buff[rx_tail];
  }
}

int UartSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_head == rx_tail) {
    return -1;
  } else {
	uint8_t c = rx_buff[rx_tail];
    rx_tail = (uint16_t)(rx_tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}

// for now just send data by waiting , no need for INT managment , we try to save some flash
// TODO maybe implment rest of writes ,
void UartSerial::write(const uint8_t *buffer, uint16_t size)
{
  for (uint16_t DataIdx = 0; DataIdx < size; DataIdx++)
  {
	  while (!LL_USART_IsActiveFlag_TXE(USART1)){}
	  LL_USART_TransmitData8(USART1, *buffer++);
  }
}

// Write a string to the serial port
void UartSerial::print(const char* string) {
  const char* ptr = string;
  while(*ptr != '\0') { // Loop until the end of string
      while (!LL_USART_IsActiveFlag_TXE(USART1)) {
          // Wait for TXE flag to be raised
      }
      LL_USART_TransmitData8(USART1, (uint8_t)*ptr); // Transmit the character
      ptr++; // Move to the next character
  }
}

// Overloaded print method for integers
void UartSerial::print(int number) {
    char buffer[12]; // Enough for 32-bit integer, including negative sign and null terminator
    itoa(number, buffer, 10); // Convert to decimal string
    print(buffer);
}

// Overloaded print method for single character
void UartSerial::print(char character) {
    while (!LL_USART_IsActiveFlag_TXE(USART1)) {
        // Wait for TXE flag to be raised indicating data register is empty
    }
    LL_USART_TransmitData8(USART1, (uint8_t)character); // Transmit the character
}


#else // UART_SERIAL_ENABLE

void USART_Error_Callback(void)
{
}

void USART_CharReception_Callback(void)
{

}

#endif // UART_SERIAL_ENABLE

UartSerial Serial;    // empty if UART_SERIAL_ENABLE not defined
