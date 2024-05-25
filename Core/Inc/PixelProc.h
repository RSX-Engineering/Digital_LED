/*
 * PixelProc.h
 *
 *  Created on: Apr 22, 2024
 *      Author: mariusrangu
 */

#ifndef PIXELPROC_H_
#define PIXELPROC_H_

#include "main.h"

// Multi-pixel processor to generate single-pixel color
class PixelProc {
public:
    uint16_t avgR, avgG, avgB;       // average R, G, B
    uint16_t minSat, avgSat, maxSat; // min, avg, max saturation
    uint16_t avgLum;                 // average luminanceish

    PixelProc() { Reset(); }
    void Reset() { avgR=avgG=avgB=0; minSat=avgSat=maxSat=0; avgLum=0; }
    bool Process(uint8_t* data, uint16_t nBytes);   // calculate statistics on a buffer

};

#endif /* PIXELPROC_H_ */
