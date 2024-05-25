/*
 * PixelProc.cpp
 *
 *  Created on: Apr 22, 2024
 *      Author: mariusrangu
 */

#include "PixelProc.h"
#include "UartSerial.h"
// #include "app_includer.h"


bool PixelProc::Process(uint8_t* data, uint16_t nBytes) {
    

    if (nBytes<3) return false;      // not enough data
    if (nBytes%3) return false;      // expecting r, g and b for each pixel
    uint16_t nPixels = nBytes/3;
    // Serial.print("Processing "); Serial.print(nPixels); Serial.print(" pixels.\n");
    
    // 1. Process first pixel
    // uint8_t r = data[0];        // byte order: RGB
    // uint8_t g = data[1];
    // uint8_t b = data[2];
    uint8_t g = data[0];        // byte order: GRB
    uint8_t r = data[1];
    uint8_t b = data[2];    
    avgR = r;  avgG = g;  avgB = b;   // init color components
    uint8_t minC = r; if (g<minC) minC=g; if (b<minC) minC=b;
    uint8_t maxC = r; if (g>maxC) maxC=g; if (b>maxC) maxC=b;
    uint8_t sat = maxC-minC;
    minSat = sat;       // update min and max  saturation
    maxSat = sat;
    avgSat = sat;                      // accumulate saturation
    uint32_t lum = r+g+b;        
    // Serial.print("Number of pixels: "); Serial.print(nPixels);
    // Serial.print(", First pixel: "); Serial.print(r); Serial.print(", "); Serial.print(g); Serial.print(", "); Serial.print(b);
    // 2. Process subsequent pixels
    if (nPixels > 1) {
        for (uint16_t i=1; i<nPixels; i++) {
            // Serial.print(" --"); Serial.print(i);
            // uint8_t r = data[i*3];      // byte order: RGB
            // uint8_t g = data[i*3+1];
            // uint8_t b = data[i*3+2];
            uint8_t g = data[i*3];      // byte order: RGB
            uint8_t r = data[i*3+1];
            uint8_t b = data[i*3+2];            
            // Serial.print(", ("); Serial.print(r); Serial.print(", "); Serial.print(g); Serial.print(", "); Serial.print(b); Serial.print(") ");
            avgR += r;  avgG += g;  avgB += b;   // accumulate color components
            uint8_t minC = r; if (g<minC) minC=g; if (b<minC) minC=b;
            uint8_t maxC = r; if (g>maxC) maxC=g; if (b>maxC) maxC=b;
            // Serial.print(", Min: "); Serial.print(minC); Serial.print(", Max: "); Serial.print(maxC);
            uint8_t sat = 255-maxC+minC;
            if (sat<minSat) minSat = sat;       // update min and max  saturation
            if (sat>maxSat) maxSat = sat;
            avgSat += sat;                      // accumulate saturation
            // Serial.print(", Sat: "); Serial.print(sat); Serial.print(", Min saturation: "); Serial.print(minSat); Serial.print(", Max saturation: "); Serial.print(maxSat);
            lum += r+g+b;                      // accumulate luminance
            // Serial.print(", Lum: "); Serial.print(lum); Serial.print(", Avg luminance: "); Serial.print(avgLum);
        }
    // Serial.print(", Last pixel: "); Serial.print(r); Serial.print(", "); Serial.print(g); Serial.print(", "); Serial.print(b);
    // Serial.print(", Min saturation: "); Serial.print(minSat); Serial.print(", Max saturation: "); Serial.print(maxSat); Serial.print(", Avg saturation: "); Serial.print(avgSat);
    // Serial.print(", Avg luminance: "); Serial.print(avgLum);
    // Serial.print("\n");
    } 

    // 3. Calculate averages   
    avgR /= nPixels; avgG /= nPixels; avgB /= nPixels; // average color components
    avgSat /= nPixels;                                  // average saturation
    avgLum = lum / (3*nPixels);                         // average luminance

    // Serial.print("Average color: "); Serial.print(avgR); Serial.print(", "); Serial.print(avgG); Serial.print(", "); Serial.print(avgB);
    // Serial.print(", Min saturation: "); Serial.print(minSat); Serial.print(", Max saturation: "); Serial.print(maxSat); Serial.print(", Avg saturation: "); Serial.print(avgSat);
    // Serial.print(", Avg luminance: "); Serial.print(avgLum); Serial.print("\n");

    return true;
}


