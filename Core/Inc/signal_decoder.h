/*
 * signal_decoder.h
 *
 *  Created on: Apr 4, 2024
 *      Author: Cosmin
 */

#ifndef INC_SIGNAL_DECODER_H_
#define INC_SIGNAL_DECODER_H_


#include "main.h"
#include "state_machine.h"

// #define SIGDECODER_PIN_TEST_USE

#ifdef UART_SERIAL_ENABLE
class SigDecoder :  StateMachine {
#else
class SigDecoder : StateMachine {
#endif
public:
	SigDecoder();
	virtual ~SigDecoder();
	void Setup();
	void Loop();
	bool setCallback(bool(*callback)(uint8_t* data, uint16_t nBytes));
#ifdef UART_SERIAL_ENABLE
	// bool Parse(const char* cmd, const char* e) override;
	// void Help() override;
	void printValues(uint16_t nrValue);
	bool printDecoded;
	bool printData;
#endif
private:
	typedef enum {
		FRMAE_SYNC,
		DECODING,
		ERROR
	}status;
	bool (*func_ptr)(uint8_t*, uint16_t);

};

extern SigDecoder decoder;


#endif /* INC_SIGNAL_DECODER_H_ */
