/*
 * AnalogLED.cpp
 *
 *  Created on: Apr 20, 2024
 *      Author: mariusrangu
 */

#include "analogLED.h"
#include "BatMon.h"
#include <cmath>

// Implementation of AnalogLED_Driver
// ----------------------------------

// Init driver. Data = scale (uint16*) = 0.004 (1/256) ... 10 (2560). Size is for polymorphism only, does not matter
bool AnalogLED_Driver_Direct::Init(void* data, uint8_t size) { 
    if (data)
        scale = *((uint16_t*)data);
    else
        scale = 256;                // default scale is 1, if no data provided
    if (scale>2560) scale=2560;  // maximum scale is 10 (2560/256) =10
    if (!scale) return false;
    brightness = 0;      
    // Serial.print("[AnalogLED_Driver_Direct::Init] Initialized with scale "); Serial.print(scale); Serial.print("\n");
    return true;
}

// Calculate register value
uint16_t AnalogLED_Driver_Direct::Get_regVal(uint16_t bright)  {
    // float retval = scale * (float)bright;
    uint32_t retval = bright;
    if (scale==256) retval = bright >> 1;  // no scaling, so register value [0-32767] is brightness [0-65535] / 2
    else {
        retval = (retval * scale) >> 9;   // register value [0-32767] is brightness [0-65535] * scale[0-2560] / 512
        if (retval>32767) retval=32767;
    }
    brightness = bright;    // store latest brightness
    #ifdef LIGHTTEST 
        regVal = retval;    // store register value to report back 
    #endif
    return (uint16_t)retval;
} 



 // Initialize from internal memory. Data = pointer to array. size number of rows = number of reference points - 1 (origin added later)
bool AnalogLED_Driver_pPWL::Init(void* pwldata, uint8_t size) { 
    refData = (uint16_t*)pwldata;      // point to data
    // NR = sizeof(refData) / sizeof(refData[0])+1;        // Number of references
    // NR = size+1;        // Number of reference points / voltage
    NR = size;        // Number of reference points / voltage
    
    if (!NR || NR>pPWL_NR_MAX) return false;  // invalid number of reference points
    // Initialize lookup tables
    lutRef.resize(NR);
    ctrlData.resize(NR+1);  // Control LUT has one extra point not carried by the COD file: the origin. The (0,0) point is added manually by UpdateReferences()
    // uint16_t vBatMin = pPWL_VBAT_MIN / battery_monitor.voltageLSB();  // For LED driving, we're monitoring battery voltage as a low-resolution integer, to prevent excessive recalculations
    // uint16_t vBatMax = pPWL_VBAT_MAX / battery_monitor.voltageLSB() ; //        
    bool success = true;
    for (uint8_t ref=0; ref<NR; ref++) {
        if (!lutRef[ref].Init(refData + ref*pPWL_NVP, pPWL_NVP)) success=false;   // Initialize each reference LUT with one row of data
        lutRef[ref].Start(pPWL_VBAT_MIN, pPWL_VBAT_MAX);                                 // Start each reference LUT and initialize voltage range
    }
    // Serial.print("[AnalogLED_Driver_pPWL::Init] Initialized with "); Serial.print(NR); Serial.print(" reference points/voltage\n");
    // if (!UpdateReferences(ADCRAW_BAT42)) success=false;                 // Calculate references and initialize control LUT at 4.2V (mainly to check data is valid)    
    if (!UpdateReferences(890)) success=false;                 // Calculate references and initialize control LUT at 4.2V (mainly to check data is valid)    
    if (!success) {
        // failed somewhere, clear all data
        refData = nullptr;
        lutRef.clear();
        ctrlData.clear();
        return false;
    }
    brightness = 0;      
    return true;
}

// Calculate control references = f(battery voltage)
bool AnalogLED_Driver_pPWL::UpdateReferences(uint16_t iVoltage) {
    ctrlData[0] = 0;    // add origin  manually
    for (uint8_t i=0; i<NR; i++) {
        ctrlData[i+1] = lutRef[i].Get(iVoltage);              // Calculate references for current voltage
        // Serial.print(ctrlData[i+1]); Serial.print(" ");
    }
    // Serial.print("references. ");
    if (!lutCtrl.Init(ctrlData.data(), NR+1)) return false;   // Reinitialize control LUT with new references
    lutCtrl.Start(0, 65535);                                // Start control LUT, with brightness range [0-65535]
    refVoltage = iVoltage;                                  // store last voltage used to calculate references
    // Serial.print("Voltage set to (raw) "); Serial.print(iVoltage); Serial.print("\n");
    return true;
}

 // Calculate register value = f(brightness)
uint16_t AnalogLED_Driver_pPWL::Get_regVal(uint16_t bright) {
    uint16_t iVoltage = battery.batteryReading();       // current battery voltage as raw integer
    int16_t diff = iVoltage - refVoltage;
    if (abs(diff) >= PWL_VBAT_MINDIFF) UpdateReferences(iVoltage);         // Recalculate references only if voltage changed with at least PWL_VBAT_MINDIFF
    uint16_t retval = lutCtrl.Get(bright);
    if (retval>32767) retval=32767;                     // clamp to range
    brightness = bright;    // store latest brightness
    // Serial.print("[AnalogLED_Driver_pPWL::Get_regVal]: "); Serial.print(bright); Serial.print(" -> "); Serial.print(retval); Serial.print("\n");
    #ifdef LIGHTTEST
        regVal = retval;    // store register value to report back 
    #endif
    return retval;
} 


// Gets the integer needed to rescale [0,255] -> [0,65535] when *data = gain as uint16:
//  1->gain=0.004, 257->gain=1, 2570->gain=10
uint16_t ColorRenderer_Interface::IntegerScale(void* data) {  
    uint16_t iScale=0;
    if (!data) iScale=257;                   // no data means default scale = 1
    else iScale = *((uint16_t*)data);        // scale provided in the range 0 .. 2570
    if (!iScale) iScale = 1;                 // 0 scale not allowed, minimum is 1/257
    if (iScale>2570) iScale = 2570;          // maximum scale is 10
    return iScale;
}


// Initialize from internal memory. Data = gain as uint16*: 1->gain=0.004, 257->gain=1, 2570->gain=10
bool ColorRenderer_Direct::Init(void* data) { 
    scale = IntegerScale(data);
    ctrlVal = 0;
    // Serial.print("[ColorRenderer_Direct.Init] Initialized with scale "); Serial.print(scale); Serial.print("\n");
    return true; 
}  

// Calculates brightness (0-65535) based on control value (hexRGB) 
bool ColorRenderer_Direct::UpdateBrightness(uint32_t ctrlval)  {
    uint32_t iBrightness;   
    if (!scale) return false;   // renderer not initialized
    ctrlVal = ctrlval;
    for (uint8_t i=0; i<3; i++) {   // Input comes as tristimulus so a direct renderer must have 3 outputs
        uint8_t channelColor = HEXRGB_GET8b(ctrlVal, i);    // 0-255 R, G or B
        iBrightness = scale * channelColor;                 // this might exeed 65535 because scale can be more than 1
        if (iBrightness > 65535) iBrightness = 65535;
        brightness[i] = iBrightness;
        // Serial.print(channelColor); Serial.print("->"); Serial.print((int)iBrightness); Serial.print("\n");
    }
    return true;
}

// Initialize from internal memory (proportional only). 
// Data = 0 or gain as uint16 = [1, 2570]; 
bool ColorRenderer_Sat::Init(void* data) { 
    scale = IntegerScale(data);
    // Serial.print("[ColorRenderer_Sat.Init] Initialized with scale "); Serial.print(scale); Serial.print("\n");
    for (uint8_t i=0; i<ALED_MAXEM; i++)
        brightness[i] = 0;
    ctrlVal = 0;
    return true; 
}

 // Calculates brightness (0-65535) based on control value (hexRGB) 
bool ColorRenderer_Sat::UpdateBrightness(uint32_t ctrlVal_)  { 
    if (!scale) return false;   // renderer not initialized
    ctrlVal = ctrlVal_;
    if (ctrlVal != last_ctrlVal) {  // recalculate Cmin and Cmax only if control value changed
        Cmax = std::max<uint8_t>(HEXRGB_GET8b(ctrlVal_, chR), std::max<uint8_t>(HEXRGB_GET8b(ctrlVal_, chG), HEXRGB_GET8b(ctrlVal_, chB)));   // max {R, G, B}
        Cmin = std::min<uint8_t>(HEXRGB_GET8b(ctrlVal_, chR), std::min<uint8_t>(HEXRGB_GET8b(ctrlVal_, chG), HEXRGB_GET8b(ctrlVal_, chB)));   // min {R, G, B}
        last_ctrlVal =  ctrlVal;
    }
    if (!Cmax) {    // Render 0 if all inputs are 0, regardless of type. Also prevent division by 0
        for (uint8_t i=0; i<ALED_MAXEM; i++) 
                brightness[i] = 0;
        return true;
    }
    uint32_t sat = Cmin * scale;       // saturation  (non-standard)
    if (sat>65535) sat=65535;   // color saturation [0,65535] 
    for (uint8_t i=0; i<ALED_MAXEM; i++) 
        brightness[i] = sat;
    return true;      
}

// Transfer function saturation -> white brightness
const uint8_t satTFpoints[] = {
    0, 255,       // saturation control value (uint8)
    0, 255     // white brightness (uint8 scaled to uint16)
};

// Transfer function luminance -> white brightness
const uint8_t lumTFpoints[] = {
    0, 255,           // saturation control value (uint8)
    0, 255     // white brightness (uint8 scaled to uint16)
};

#define NELEM(x) (sizeof(x) / sizeof(x[0]))
bool ColorRenderer_SatStat::Init(void* data) {
    // Serial.print("[ColorRenderer_SatStat::Init] Initialized\n");
    satTF.Init((uint8_t*)&satTFpoints[0], NELEM(satTFpoints)); // Initialize saturation transfer function
    satTF.Start();
    lumTF.Init((uint8_t*)&lumTFpoints[0], NELEM(lumTFpoints));  // Initialize luminance transfer function
    lumTF.Start();
    return true;
}


#define SATSTAT_MINDIFF 10      // minimum difference allowed between pixel saturation values to consider the whole blade having the same saturation
// Calculates brightness [0-65535] based on control value (Sat Statistics word) 
bool ColorRenderer_SatStat::UpdateBrightness(uint32_t ctrlval) {
    uint8_t minSat = SATSTAT_GET8b(ctrlval, minSAT);
    uint8_t avgSat = SATSTAT_GET8b(ctrlval, avgSAT);
    uint8_t maxSat = SATSTAT_GET8b(ctrlval, maxSAT);
    uint8_t avgLum = SATSTAT_GET8b(ctrlval, avgLUM);
    // Serial.print("[ColorRenderer_SatStat::UpdateBrightness] minSat="); Serial.print(minSat); Serial.print(", avgSat="); Serial.print(avgSat); 
    // Serial.print(", maxSat="); Serial.print(maxSat); Serial.print(", avgLum="); Serial.print(avgLum); Serial.print("\n");
    
    // 1. Get saturation control value
    uint8_t satCtrl;    // saturation control value
    if (maxSat - minSat < SATSTAT_MINDIFF) 
        satCtrl = avgSat;           // almost same saturation across the entire blade, use average to control saturation
    else     
        satCtrl = maxSat-avgSat;    // saturation varies across the blade, probably localized white; use maximum-average to control saturation    
    // Serial.print("satCtrl="); Serial.print(satCtrl); 
    
    // 2. Get brightness from satuation and luminance, through transfer functions
    uint16_t bri = satTF.Get(satCtrl);    // get white brightness based on saturation
    // Serial.print(", satTF(");   Serial.print((int)satCtrl); Serial.print(") =" ); Serial.print((int)bri);
    uint16_t lum = lumTF.Get(avgLum);    
    // Serial.print(", lumTF(");   Serial.print((int)avgLum); Serial.print(") =" ); Serial.print((int)lum);
    uint32_t brightness_ = bri*lum;             // scale white brightness with luminance
    brightness_ >>= 16;     
    // Serial.print(", brightness = "); Serial.print((int)brightness_); Serial.print("\n");

    // 3. Apply brightness to all emitters                     
    for (uint8_t i=0; i<ALED_MAXEM; i++) 
        brightness[i] = brightness_;
    return true;
}


 // Calculates brightness [0-65535] based on control value (hexRGB) 
/*      |  brightness[0]  |     |  crm[0]  crm[1]  crm[2]  |     |  R  |
        |  brightness[1]  | =   |  crm[3]  crm[4]  crm[5]  |  *  |  G  |
        |  brightness[2]  |     |  crm[6]  crm[7]  crm[8]  |     |  B  |
*/
bool ColorRenderer_CRM::UpdateBrightness(uint32_t ctrlval)  { 
    if (!nOuts) return false;   // renderer not initialized
    float R = (float)HEXRGB_GET8b(ctrlVal, 0);
    float G = (float)HEXRGB_GET8b(ctrlVal, 1);
    float B = (float)HEXRGB_GET8b(ctrlVal, 2); 
    for (uint8_t ch=0; ch<nOuts; ch++) {
        brightness[ch] = (uint16_t) (crm[ch]*R + crm[ch+1]*G + crm[ch+2]*B);
        if (gammaTF[ch])    // Apply gamma correction on each emitter, if transfer function assigned
            brightness[ch] = gammaTF[ch]->Get(brightness[ch]);
    }
    return true;
}


int8_t AnalogLED_Channel::nextEmitter() {
        for (uint8_t i=0; i<=nEm; i++)
            if (!drivers[i]) return i;  // found fist driver pointer not initialized
        return -1;      // all emitters have drivers
    }


void AnalogLED_Channel::AssignRenderer(ColorRenderer_Interface* rend)  { 
    renderer = rend;
}


// Creates and initializes a driver from internal memory
// Order of emitters = order of driver assignment !!!
bool AnalogLED_Channel::AssignDriver(uint8_t pin, AnalogLED_DriverInterface* driver) { 
    uint8_t currentEmitter = nextEmitter(); // find index of the first unassigned emitter (-1 if fails)
    if (currentEmitter == -1) return false; // could not find unassigned emitter    
    drivers[currentEmitter] = driver;
    pins[currentEmitter] = pin;
    nEm ++;
    return true;
}


extern PwmPin pwm_pin;

// Set color based on control value (hexRGB)
void AnalogLED_Channel::Set(uint32_t ctrlVal) { 
    if (!renderer) return;          // LED channel not initialized
    // Serial.print("RGB=("); Serial.print(HEXRGB_GET8b(ctrlVal, chR)); Serial.print(", "); 
    // Serial.print(HEXRGB_GET8b(ctrlVal, chG)); Serial.print(", "); Serial.print(HEXRGB_GET8b(ctrlVal, chB)); Serial.print("): ");
    if (!renderer->UpdateBrightness(ctrlVal)) return;      // Renderer failure
    for (uint8_t i=0; i<nEm; i++) {   
        uint16_t regVal = drivers[i]->Get_regVal(renderer->brightness[i]);   // Get PWM register value (0-32767)                                   // Apply register value        
        if (pins[i]!=NO_PIN) pwm_pin.setChValue(pins[i], regVal);            // Apply register value        
        // Serial.print("LED"); Serial.print(pins[i]); Serial.print(": br="); Serial.print(renderer->brightness[i]); 
        // Serial.print(" -> regval="); Serial.print(regVal); Serial.print(". ");
    }
    // Serial.print("\n");
}


#ifdef LIGHTTEST
void AnalogLED_Channel::print_bright() {
    if (nEm>0) { Serial.print(renderer->brightness[0]);  } // report renderer's calculated brightness on 1st channel
    if (nEm>1) { Serial.print(", ");  Serial.print(renderer->brightness[1]);  } // report renderer's calculated brightness on 2nd channel
    if (nEm>2) { Serial.print(", ");  Serial.print(renderer->brightness[2]);  } // report renderer's calculated brightness on 3rd channel          
    Serial.print("\n");
}
#endif

