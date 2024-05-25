/*
 * AnalogLED.h
 *
 *  Created on: Apr 20, 2024
 *      Author: mariusrangu
 */

#ifndef ANALOGLED_H_
#define ANALOGLED_H_


#include "main.h"
#include "interpolators.h"
#include "PWM_Pin.h"

#include <vector>  
using namespace std;


#define ALED_MAXCH      4   // Maximum number of channels an analog LED suppports
#define ALED_MAXEM      3   // Maximum number of emitters an analog LED channel supports
#define pPWL_NR_MAX     8   // Maximum number of control references (segments)
#define pPWL_NVP        7   // Number of voltage points: 3.0, 3.2, 3.4, 3.6, 3.8, 4.0 and 4.2
#define pPWL_VBAT_MIN   ADCRAW_BAT30   // Minimum battery voltage used for optical calibration, as ADC reading
#define pPWL_VBAT_MAX   ADCRAW_BAT42   // Maximum battery voltage used for optical calibration. as ADC reading
#define PWL_VBAT_MINDIFF 2    // Minimum voltage change (raw integer) to recalculate references
#define NO_PIN 255      // Use 255 for no pin



class AnalogLED_DriverInterface  {
protected:
    uint16_t brightness;                // Latest brightness for which the register value was calculated
#ifdef LIGHTTEST
    uint16_t regVal;                    // Latest register value - only needed for remote light testing
#endif
public:
    virtual bool Init(void* data, uint8_t size=0) { return false; } // initialize from internal memory
    virtual uint16_t Get_regVal(uint16_t bright) = 0; // Calculates the register value (0-32767) based on desired brightness (0-65535) and battery voltage
    virtual bool UpdateReferences(uint16_t iVoltage) { return true; }        // Calculate control references = f(battery voltage)    

};

// Direct drive: use only if the LED can withstand the full battery voltage or an external PWM-driven current regulator is used
class AnalogLED_Driver_Direct : public AnalogLED_DriverInterface {
private:
    uint16_t scale;                        // register value [0-32767] = scale [1-2560] * brightness [0-65535] / 512
public:    
    bool Init(void* data, uint8_t size=0) override;     // Init driver. Data = scale (uint16*)    
    uint16_t Get_regVal(uint16_t bright) override; // Calculate register value
};


// Photometric piecewise linear driver
class AnalogLED_Driver_pPWL : public AnalogLED_DriverInterface {
private:
    uint16_t refVoltage;                // Voltage at which the references were calculated last time
    uint8_t NR;                         // Number of references = number of control segments
    uint16_t* refData;           // References data. This is a resizeable vector, containing all the LUT values row-by-row
    vector<LUT<uint16_t, uint16_t>> lutRef; // Lookup tables to calculate references = f ( voltage )
    vector<uint16_t> ctrlData;          // Control data. This vector stores the references and provides data for the control LUT
    LUT<uint16_t, uint16_t> lutCtrl;    // Control lookup table - used to calculate register value = f ( brightness )
public:
    // bool Init(const uint16_t* pwldata);    // Initialize from internal memory
    bool Init(void* pwldata, uint8_t size=0) override;    // Initialize from internal memory    
    bool UpdateReferences(uint16_t iVoltage) override;        // Calculate control references = f(battery voltage)    
    uint16_t Get_regVal(uint16_t bright) override;                          // Calculate register value = f(brightness)


};


// -----------------------------------------------------------
// COLOR RENDERER
#define chR 0       // channel Red
#define chG 1       // channel Green
#define chB 2       // channel Blue
// Get 8-bit value from hexRGB (0x00RRGGBB): CH: 0=Red, 1=Green, 2=Blue, anything else = rubbish
#define HEXRGB_GET8b(uint32_RGB, CH) *((((uint8_t*)&uint32_RGB)+2-CH))
// Set 8-bit value to hexRGB (0x00RRGGBB): CH: 0=Red, 1=Green, 2=Blue, val=0-255
#define HEXRGB_SET8b(uint32_RGB, CH, val) *((((uint8_t*)&uint32_RGB)+2-CH)) = val


#define minSAT  0   // minimum saturation
#define avgSAT  1 // average saturation
#define maxSAT  2 // maximum saturation
#define avgLUM  3 // average luminance
// Get 8-bit value from Saturation Statistics word: (0xmS.MmS.aS.aS.MS.MS.aL.aL): CH: 0=minSAT, 1=maxSAT, 2=avgSAT, 3=maxSAT, 4=avgLUM, anything else = rubbish
#define SATSTAT_GET8b(uint32_STAT, CH) *((((uint8_t*)&uint32_STAT)+3-CH))
// Set 8-bit value to Saturation Statistics word: (0xmS.MmS.aS.aS.MS.MS.aL.aL): CH: 0=minSAT, 1=maxSAT, 2=avgSAT, 3=maxSAT, 4=avgLUM, anything else = rubbish
#define SATSTAT_SET8b(uint32_STAT, CH, val) *((((uint8_t*)&uint32_STAT)+3-CH)) = val



// Renderer interface
class ColorRenderer_Interface {
friend class AnalogLED_Channel;
protected:
    uint32_t ctrlVal;                   // Current control value [ 0x00RRGGBB aka hexRGB ]
    uint16_t brightness[ALED_MAXEM];    // Current brightness for each emitter
public:
    virtual bool Init(void* data) { return false; };        // Initialize from internal memory


    virtual bool UpdateBrightness(uint32_t ctrlval) = 0;    // Calculates brightness [0-65535] based on control value (hexRGB), for all channels 
    virtual bool AppplyGamma(const char* filename, uint16_t* IDs)  { return true; } // Assign a transfer function for gamma correction to each emitter
protected:
    uint16_t IntegerScale(void* data);
};


// Scales and drives directly: R -> out1, G -> out2, B -> out3
class ColorRenderer_Direct : public ColorRenderer_Interface {
private:
    uint16_t scale;     // scale = 257  => 255 scales to 65535

public:
    ColorRenderer_Direct() { scale = 0; }
    bool Init(void* data) override;
    // bool Init(void* data, float channelBrightness);  // Initialize from internal memory.
    bool UpdateBrightness(uint32_t ctrlval) override;  // Calculates brightness (0-65535) based on control value (hexRGB)
    
};

// // Single-output renderer controlled by hue, saturation or lightness and an enhancement curve
class ColorRenderer_Sat : public ColorRenderer_Interface {
private:
    uint16_t scale;         // scale = 257  -> 255 scales to 65535
    char type;              // use 'h', 's' or 'l' to specify which parameter to respond to
    uint16_t Cmin, Cmax;    // min and max of last {R, G, B}, for S and L calculations. Those are static so can reuse Cmin and Cmax across successive renderers
    uint32_t last_ctrlVal;  // ... to know when to recalculate Cmin and Cmax
public:
    // ColorRenderer_Sat() { enhancer = 0; scale = 0; }
    ColorRenderer_Sat() { scale = 0; }

   
    bool Init(void* data) override;
    bool UpdateBrightness(uint32_t ctrlVal_) override;       // Calculates brightness (0-65535) based on control value (hexRGB) 

};


class ColorRenderer_CRM : public ColorRenderer_Interface {
private:
    uint8_t nOuts;                                  // number of outpus (comes from CRM size)
    vector<float> crm;                              // Color Rendering Matrix (vector of 3-column rows)
    TF<uint16_t, uint16_t>* gammaTF[ALED_MAXEM];      // pointers to tranfer functions for gamma correction for each output 
    vector<uint8_t> gammaRef;                       // reference points for gamma transfer function
public:
    ColorRenderer_CRM() { 
        nOuts=0;
        for (uint8_t i=0; i<ALED_MAXEM; i++) 
            gammaTF[i] = 0;
    }
    bool UpdateBrightness(uint32_t ctrlval) override; // Calculates brightness [0-65535] based on control value (hexRGB) 
    
};


// Single-output renderer controlled by saturation statistics
// Normally used to control WHITE based on multi-pixel saturation
class ColorRenderer_SatStat : public ColorRenderer_Interface {
private:
    TF<uint8_t, uint16_t> satTF, lumTF;  // Transfer functions for saturation and luminance
public:
    bool Init(void* data) override;
    bool UpdateBrightness(uint32_t ctrlval) override; // Calculates brightness [0-65535] based on control value (Sat Statistics word) 
};

class AnalogLED_Channel   {
public: AnalogLED_Channel() {  // 
        nEm = 0;
        renderer = 0;
        active = false;
        for (uint8_t i=0; i<ALED_MAXEM; i++) {
            pins[i] = NO_PIN;
            drivers[i] = 0;
        }
    }     




private:
    uint8_t nEm;                                // Number of emitters
    uint8_t pins[ALED_MAXEM];                            // PWM pins
    AnalogLED_DriverInterface* drivers[ALED_MAXEM];     // Drivers
    ColorRenderer_Interface* renderer;         // Color renderer
    bool active;                                //

    // Get the index of the first emitter with unassigned driver. Returns [0,3] or -1 if all emitters have drivers
    int8_t nextEmitter();

public:
    bool AssignDriver(uint8_t pin, AnalogLED_DriverInterface* driver);   // Assign an already initialized driver to an emitter
    void AssignRenderer(ColorRenderer_Interface* rend);
    void Set(uint32_t ctrlVal);  // Set color
#ifdef LIGHTTEST
    void print_bright();
#endif

};



#endif /* ANALOGLED_H_ */
