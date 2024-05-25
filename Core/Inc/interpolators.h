/********************************************************************
 *  INTERPOLATORS                                                   *
 *  (C) Marius RANGU & Cosmin PACA @ RSX Engineering                *
 *  fileversion: v1.2 @ 2022/07;    license: GPL3                   *
 ********************************************************************
 *  Base class Interpolator provides a common interface for        *
 *  multiple interpolators:                                         *
 *  - LUT (look-up table): Linear, uni-dimensional, on constant grid
 ********************************************************************/

#ifndef XINTERPOLATOR_H
#define XINTERPOLATOR_H

#include <type_traits>
#include <limits>
#include "UartSerial.h" 

enum InterpolatorState {
    interpolator_not_initialized=0,   
    interpolator_ready,
    interpolator_running,
    interpolator_error
};


// -----------------------------------------------------------
// Template of parent class for multiple interpolators
// REFT = reference type; usually we want to store reference points on lower resolution, to save memory
// WORKT = working type, normally higher resolution than REFT. WORKT must be able to represent -REFT^2 !!! 
template<class REFT, class WORKT>
class Interpolator {   
protected:
    uint16_t refSize;            // Number of reference values. This is the number of <REFT> values in refData and not necessary the number of reference points.
    REFT* refData;              // Pointer to reference data points
    WORKT* precalcData;         // Pointer to extra RAM to keep pre-calculated data - for speedup!
public:
    InterpolatorState state;    // Current state

public: 
     uint16_t GetSize() {
        return refSize;
    }
protected:   
    // Check validity of reference data; size = number of values in data* (not number of bytes!)
    virtual bool CheckRef(REFT* data, uint16_t size)  {
        return true;
    }
public:
    // Initialize from internal memory - includes reset:
    bool Init(REFT* data, uint16_t size)  {
        state = interpolator_not_initialized;        // assume failure
        if (!CheckRef(data, size)) return false;    // check validity
        // all good, initialize and reset:
        refSize = size; 
        refData = data;  
        precalcData = 0;     
        Reset();                                    // reset internal data and set state to ready
        return true;                                
    }

    // De-initialize. 
    void Free() { 
        refSize = 0;              
        refData = 0;
        precalcData = 0;
        state = interpolator_not_initialized;
    }


    // Start interpolator:
    virtual void  Start() { 
        if (state != interpolator_ready) return;
        state = interpolator_running;
    }

    // Pre-calculate and store whatever possible
    // Working RAM must be reserved externally. Please don't malloc!
    // ramSize is not really needed, but we'll check it to guard the working RAM
    virtual bool SpeedUp(WORKT* ram, uint32_t ramSize) { 
        precalcData = 0;    // unless overritten there's no precalculated data
        return false;
    }


    // Stop and reset internal state:
    virtual void  Reset() { 
        state = interpolator_ready;
    }

    virtual WORKT Get(WORKT x = 0) { 
        if (state != interpolator_running) return 0;   // interpolator should be initialized and started before Get()
        // ...
        return 0; 
    }

};



//  -----------------------------------------------------------
// LOOK UP TABLE
// Linear uni-dimensional interpolator on X grid 
template<class REFT, class WORKT>
class LUT  : public Interpolator<REFT, WORKT> {
    // Bring dependent names in scope (lost when binding templates against inheritance):
    using Interpolator<REFT, WORKT>::refSize;          
    using Interpolator<REFT, WORKT>::refData;          
    using Interpolator<REFT, WORKT>::precalcData;      // | slope[0]| slope[1] | ... | slope[N-2] |
    using Interpolator<REFT, WORKT>::state;
public:
// private:
    WORKT xMin, xMax;        // range on X axis
    WORKT xStep;             // scale factor on X axis. x_scaled = xMin + index * xStep
 public:   
    // CONSTRUCTORS:
    // Constructor with default range
    LUT() {
        xMin = 0;
        xMax = 1;
        if (std::is_floating_point<WORKT>::value) xMax = 1;    // default range when working with floats is 0:1
        else xMax = std::numeric_limits<WORKT>::max();    // default range when working with integers is 0:typemax
        xStep = 0;          // calculated by start(), as it depends on refSize, which is not specified yet
    }

    // Constructor with specified range
    LUT(WORKT xMin_, WORKT xMax_) {
        xMin = xMin_;
        xMax = xMax_;
        xStep = 0;          // calculated by start(), as it depends on refSize, which is not specified yet
    }


    // Check validity of reference data; size = number of values in data* (not number of bytes!)
    bool CheckRef(REFT* data, uint16_t size) override {
        if (size<2) return false;                     // need at least two reference points
        if (!data) return false;                      // need actual data
        return true;
    }

    // Pre-calculate the slope of all segments and store them in RAM, to speed up further calculations
    bool SpeedUp(WORKT* ram, uint32_t ramSize) override {
        uint16_t i;        
        // 1. Check state:
        precalcData = 0;              // assume failure
        if (state!=interpolator_running) return false;          // Need everything initialized for speedup
        // 2. Assign RAM:
        if (ramSize < (refSize-1) * sizeof(WORKT)) return false;        // not enough RAM
        precalcData = ram;      // assign RAM for pre-calculated data
        // 3. Calculate and store slopes:
        for (i=0; i<refSize-1; i++) {
            ram[i] = ((WORKT)refData[i+1] - (WORKT)refData[i]);       
            ram[i] /= xStep;
        }
        return true;
    }

    // Start interpolator:
    void  Start() override { 
        if (state != interpolator_ready) return;
        xStep = ( xMax - xMin ) / (refSize-1);
        state = interpolator_running;     
    }

    // Update range, then start interpolator with updated range:
    void Start(WORKT xMin_, WORKT xMax_) {
        if (xMax_>xMin_) {
            xMin = xMin_;       
            xMax = xMax_;
        }     
        Start();
    }



    // Run the interpolator on input x and return output
    WORKT Get(WORKT x = 0) override {   
        if (state != interpolator_running) return 0;   // interpolator should be initialized and started before Get()
        uint16_t k = (x-xMin) / xStep;              // index of segment in refData
        WORKT xk = xMin + k*xStep;
        WORKT slope;
        if (precalcData) 
            slope = precalcData[k];        // slope already pre-calculated
        else 
            slope = ((WORKT)refData[k+1] - (WORKT)refData[k]) / xStep; // calculate slope now
        return (WORKT)refData[k] + slope * (x - xk);      // calculate and return y(x)
    }
    
};

// LUT specialization for <uint16_t, uint16_t>. 
// This is an illegal <REFT, WORKT> pair as WORKT must be able to hold -REFT^2, normally we should use <uint16_t, int32_t> 
// This specialization worksaround the integer overflow problem but speedup will no longer be available, as we can't store signed slopes as uint.
template<>
inline uint16_t LUT<uint16_t, uint16_t>::Get(uint16_t x) {   
    if (state != interpolator_running) return 0;   // interpolator should be initialized and started before Get()
    if (x<xMin)  x=xMin; 
    if (x>xMax)  x=xMax;     
    uint16_t k = (x-xMin) / xStep;              // index of segment in refData
    if (k>refSize-2) k = refSize-2;             // clamp to last segment
    uint16_t xk = xMin + k*xStep;
    int32_t retval = x - xk;           // we'll calculate in int32 to avoid overflowing 
    int16_t deltaY = refData[k+1] - refData[k];
    retval *= deltaY;
    retval /= xStep;
    retval += refData[k];
    if (retval>65535) {   
        retval = 65535;     // clamp to uint16
    }
    return (uint16_t)retval;
}


//  -----------------------------------------------------------
// TRANSFER FUNCTION
// Linear uni-dimensional interpolator 
template<class REFT, class WORKT>
class TF  : public Interpolator<REFT, WORKT> {
    // Bring dependent names in scope (lost when binding templates against inheritance):
    using Interpolator<REFT, WORKT>::refSize;          
    using Interpolator<REFT, WORKT>::refData;          
    using Interpolator<REFT, WORKT>::precalcData;      // | slope[0]| slope[1] | ... | slope[N-2] |
    using Interpolator<REFT, WORKT>::state;
// public:
protected:
    uint16_t N;      // number of reference points = refSize / 2

    // Find index of the leftmost reference X (=Xk), for a specified x. No protection at ends.
    uint16_t FindK(WORKT x) {
        uint16_t k = 1;
        while(k <= N-1) {
            if (x <= (WORKT)refData[k]) return k-1;
            k++;
        }
        return k-1;
    }

 public:   
    // Check validity of reference data; size = number of values in data* (not number of bytes!)
    bool CheckRef(REFT* data, uint16_t size) override {
        if (size<4) return false;                     // need at least two reference points
        if (size%2) return false;                     // need an even number of values (X and Y for each reference point)
        if (!data) return false;                      // need actual data
        for (uint16_t i=0; i<size/2-1; i++)
            if (data[i] >= data[i+1]) return false;    // X values must be increasing monotonic
        return true;
    }

    // Pre-calculate the slope of all segments and store them in RAM, to speed up further calculations
    // ram should be able to store N-1 WORKT numbers
    bool SpeedUp(WORKT* ram, uint32_t ramSize) override {
        uint16_t i;        
        // 1. Check state:
        precalcData = 0;              // assume failure
        if (state!=interpolator_running) return false;          // Need everything initialized for speedup
        // 2. Assign RAM:
        if (ramSize < (N-1)*sizeof(WORKT)) return false;        // not enough RAM
        precalcData = ram;      // assign RAM for pre-calculated data
        // 3. Calculate and store slopes:
        for (i=0; i<N-1; i++) {
            ram[i] = (WORKT)refData[N+i+1] - (WORKT)refData[N+i]; // y[k+1] - y[k]
            ram[i] /= (WORKT)refData[i+1] - (WORKT)refData[i];    // / ( x[k+1]-x[k] )
        }
        return true;
    }

    // Start interpolator:
    void  Start() override { 
        if (state != interpolator_ready) return;
        N = refSize / 2;
        state = interpolator_running;       
    }


    // Run the interpolator on input x and return output
    WORKT Get(WORKT x = 0) override {   
        if (state != interpolator_running) return 0;   // interpolator should be initialized and started before Get()
        // if (x<=xMin) return refData[0];                // outside range extend first / last value 
        // if (x>=xMax) return refData[refSize-1];        
        uint16_t k = FindK(x);              // index of segment in refData
        WORKT slope;
        if (precalcData) {
            slope = precalcData[k];        // slope already pre-calculated
            return (WORKT)refData[k+N] + slope * (x - (WORKT)refData[k]);      // calculate and return y(x)
        }
        else {  // calculate now
            slope = ((WORKT)refData[k+N+1] - (WORKT)refData[k+N]) * (x - (WORKT)refData[k]);
            slope /= ((WORKT)refData[k+1] - (WORKT)refData[k]);
            return (WORKT)refData[k+N] + slope;
        }
    }



    
};

// Rogue TF specialization for <uint8, int16>. Y gets scaled [0, 255] --> [0, 65535]
template<>
inline bool TF<uint8_t, int16_t>::SpeedUp(int16_t* ram, uint32_t ramSize)  {
    uint16_t i;        
    // 1. Check state:
    precalcData = 0;              // assume failure
    if (state!=interpolator_running) return false;          // Need everything initialized for speedup
    // 2. Assign RAM:
    if (ramSize < (N-1)*sizeof(int16_t)) return false;        // not enough RAM
    precalcData = ram;      // assign RAM for pre-calculated data
    // 3. Calculate and store slopes:
    for (i=0; i<N-1; i++) {
        ram[i] = refData[N+i+1] - refData[N+i];         // y[k+1] - y[k]
        ram[i] <<= 8;       // 256 * ( y[k+1] - y[k] )
        ram[i] /= refData[i+1] - refData[i];    // 256 * ( y[k+1] - y[k] ) / ( x[k+1]-x[k] )
    }
    return true;
}

template<>
inline int16_t TF<uint8_t, int16_t>::Get(int16_t x) {
    if (state != interpolator_running) return 0;   // interpolator should be initialized and started before Get() 
    uint16_t k = FindK(x);              // index of segment in refData
    int16_t slope;
    if (precalcData) 
        slope = precalcData[k];        // slope already pre-calculated
    else {
        slope = refData[N+k+1] - refData[N+k];         // y[k+1] - y[k]
        slope <<= 8;       // 256 * ( y[k+1] - y[k] )
        slope /= refData[k+1] - refData[k];    // 256 * ( y[k+1] - y[k] ) / ( x[k+1]-x[k] )
    }
    slope *= (x -refData[k]);                   // 256 * deltaY
    slope >>= 8;                                // delta Y
    return refData[k+N] + slope;                // Y

 } 


// TF specialization for <uint8, uint16>. This is illegal as WORKT must be able to represent -REFT^2. Slope and output are scaled with 256
// Input range is [0,255]. Output range is [0,65535]. No speedup for this specialization!
template<>
inline uint16_t TF<uint8_t, uint16_t>::Get(uint16_t x) {
    if (state != interpolator_running) return 0;   // interpolator should be initialized and started before Get() 
    uint16_t k = FindK(x);              // index of segment in refData
    int32_t slope;
    slope = refData[N+k+1] - refData[N+k];  // y[k+1] - y[k]
    slope <<= 8;                            // 256 * ( y[k+1] - y[k] )
    slope /= refData[k+1] - refData[k];     // 256 * ( y[k+1] - y[k] ) / ( x[k+1]-x[k] )
    slope *= (x -refData[k]);               // 256 * deltaY
    return (refData[k+N]<<8) + slope;       // 256 * Y
 } 


//  -----------------------------------------------------------
// SCALABLE TRANSFER FUNCTION 
// Same as TF but allows rescaling of both axes. Much slower than TF
template<class REFT, class WORKT>
class scalableTF  : public TF<REFT, WORKT> {

    using TF<REFT, WORKT>::refSize;          
    using TF<REFT, WORKT>::refData;          
    using TF<REFT, WORKT>::state;          
private:    
    float xScale, yScale;
    REFT xOffset, yOffset;
    REFT xMin, yMin;


// Re-initialize a transfer function with scaled references
public: 
    // Stop and reset internal state. Scale reverts to 1:1
    void  Reset() override { 
        state = interpolator_ready;
        xMin = 0;   xOffset = 0;    xScale = 1;  // no scaling on X
        yMin = 0;   yOffset = 0;    yScale = 1;  // no scaling on Y        
    }

    // Set new input range: X = [xMin, xMax]
    bool SetXrange(REFT xMin_, REFT xMax_) {
        if (state!=interpolator_ready && state!=interpolator_running) return false;     // need it initialized to apply scale 
        if (xMin_ >= xMax_) return false;     // invalid scale
        xMin = xMin_;
        xOffset = refData[0];
        xScale = (refData[refSize/2-1] - refData[0]) / (xMax_ - xMin_);
        return true;
    }

    // Set new output range: Y = [yMin, yMax]
    bool SetYrange(REFT yMin_, REFT yMax_) {
        if (state!=interpolator_ready && state!=interpolator_running) return false;     // need it initialized to apply scale 
        if (!(refData[refSize-1] - refData[refSize/2])) return false;                   // invalid reference points, would result in division by 0
        yMin = yMin_;
        yOffset = refData[refSize/2];
        yScale = (yMax_ - yMin_) / (refData[refSize-1] - refData[refSize/2]);
        return true;
    }

    WORKT Get(WORKT x = 0) override {   
        // 1. Scale X
        float temp = (x-xMin);       // calculate scaled values in float, to avoid integer losses
        temp *= xScale;
        temp += xOffset;            // X scaled
        
        // 2. Get unscaled Y from regular TF
        temp =  TF<REFT, WORKT>::Get((WORKT)temp);  // Y unscaled
        
        // 3. Scale Y
        temp -= yOffset;  
        temp *= yScale;
        temp += yMin;            // Y scaled
        return temp;        
    }

};





#endif  // INTERPOLATOR_H
