/*
 * test2class.h
 *
 *  Created on: Apr 5, 2024
 *      Author: Cosmin
 */

#ifndef SRC_UARTSERIAL_H_
#define SRC_UARTSERIAL_H_

#include "main.h"

#ifdef UART_SERIAL_ENABLE
class UartSerial {
public:
	UartSerial();
	virtual ~UartSerial();
	void begin(uint32_t baud );
//	void HandleContinuousReception();
	int available(void);
	int peek(void);
	int read(void);
	void write(const uint8_t *buffer, uint16_t size);
	void print(const char* string);
	void print(int number);
	void print(char character);
private:
	void uart1_ll_Init(uint32_t baud);
	void startReception(void);

};


#else // empty serial class if serial not enabled
class UartSerial {
public:
	UartSerial() {}
	virtual ~UartSerial() {}
	void begin(uint32_t baud ) {}
//	void HandleContinuousReception();
	int available(void) { return 0; }
	int peek(void) { return 0; }
	int read(void) { return 0; }
	void write(const uint8_t *buffer, uint16_t size) {}
	void print(const char* string) {}
	void print(int number) {}
	void print(char character) {}

};
#endif

extern UartSerial Serial;


#endif /* SRC_TEST2CLASS_H_ */
