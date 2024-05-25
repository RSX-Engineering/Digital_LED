/*
 * app_includer.h
 *
 *  Created on: Apr 12, 2024
 *      Author: Cosmin
 */

#ifndef APP_INCLUDER_H_
#define APP_INCLUDER_H_

const char* FWversion = "1.2";

#define NELEM(x) (sizeof(x) / sizeof(x[0]))

#include "UartSerial.h"
#include "serial.h"
#include "pwm_pin.h"
#include "BatMon.h"
#include "analogLED.h"
#include "PixelProc.h"
#include "signal_decoder.h"

// LED data
#include "X4em_XP2E-RD.h"
#include "X4em_XP2E-GR.h"
#include "X4em_XP2E-BL.h"
#include "X4em_XP2E-WT.h"

#define DRIVER_RED pwl_XPEBRD
#define DRIVER_GREEN pwl_XPEBGR
#define DRIVER_BLUE pwl_XPEBBL
#define DRIVER_WHITE pwl_XPEBWT


#endif /* APP_INCLUDER_H_ */
