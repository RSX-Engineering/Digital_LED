/*
 * BatMon.h
 *
 *  Created on: Apr 12, 2024
 *      Author: Cosmin
 */

#ifndef SRC_BATMON_H_
#define SRC_BATMON_H_

#include "main.h"
#include "state_machine.h"

#define ADCRAW_BAT30 725	// ADC raw reading at battery voltage 3.0V
#define ADCRAW_BAT42 1010	// ADC raw reading at battery voltage 4.2V

class BatMon : public StateMachine {
public:

	BatMon();
	virtual ~BatMon();
	void Setup();
	void Loop();
	void Activate();
	uint16_t batteryReading();
#ifdef BATMON_ENABLE_TEMP
	uint16_t tempReading();
	float tempC();
#endif


#ifdef LIGHTTEST
	float batteryVoltage(); // Voltage in mV 
#endif	

private:
	uint16_t uhADCxReading;
	uint16_t uhADCxReading_Voltage_mVolt;
#ifdef BATMON_ENABLE_TEMP
	uint32_t activeCh;
	uint16_t calibrationValue;
	uint16_t uhTMPReading;
	bool ChangeChannel(uint32_t Channel);
#endif
	void ConversionStartPoll_ADC_GrpRegular();

};

extern BatMon battery;

#endif /* SRC_BATMON_H_ */
