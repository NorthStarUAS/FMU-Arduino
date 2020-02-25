/*
AMS5915.h
Brian R Taylor
brian.taylor@bolderflight.com
2016-11-03

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "Arduino.h"
#include "Wire.h"

enum ams5915_transducer
{
    AMS5915_0005_D,
    AMS5915_0010_D,
    AMS5915_0005_D_B,
    AMS5915_0010_D_B,
    AMS5915_0020_D,
    AMS5915_0050_D,
    AMS5915_0100_D,
    AMS5915_0020_D_B,
    AMS5915_0050_D_B,
    AMS5915_0100_D_B,
    AMS5915_0200_D,
    AMS5915_0350_D,
    AMS5915_1000_D,
    AMS5915_2000_D,
    AMS5915_4000_D,
    AMS5915_7000_D,
    AMS5915_10000_D,
    AMS5915_0200_D_B,
    AMS5915_0350_D_B,
    AMS5915_1000_D_B,
    AMS5915_1000_A,
    AMS5915_1200_B
};

class AMS5915 {
 public:
    AMS5915();
    AMS5915(uint8_t address, TwoWire *bus, ams5915_transducer type);
    void configure(uint8_t address, TwoWire *bus, ams5915_transducer type);
    void begin();
    bool getData(float* pressure, float* temperature);
 private:
    uint8_t _address;
    TwoWire *_bus;
    ams5915_transducer _type;
    float _pMin;
    float _pMax;
    float _tMin;
    float _tMax;

    // i2c bus frequency
    const uint32_t _i2cRate = 400000;

    // conversion millibar to PA
    const float _mBar2Pa = 100.0f; 

    // min and max pressures, millibar
    const float AMS5915_0005_D_P_MIN = 0.0f;
    const float AMS5915_0005_D_P_MAX = 5.0f;
    const float AMS5915_0010_D_P_MIN = 0.0f;
    const float AMS5915_0010_D_P_MAX = 10.0f;
    const float AMS5915_0005_D_B_P_MIN = -5.0f;
    const float AMS5915_0005_D_B_P_MAX = 5.0f;
    const float AMS5915_0010_D_B_P_MIN = -10.0f;
    const float AMS5915_0010_D_B_P_MAX = 10.0f;
    const float AMS5915_0020_D_P_MIN = 0.0f;
    const float AMS5915_0020_D_P_MAX = 20.0f;
    const float AMS5915_0050_D_P_MIN = 0.0f;
    const float AMS5915_0050_D_P_MAX = 50.0f;
    const float AMS5915_0100_D_P_MIN = 0.0f;
    const float AMS5915_0100_D_P_MAX = 100.0f;
    const float AMS5915_0020_D_B_P_MIN = -20.0f;
    const float AMS5915_0020_D_B_P_MAX = 20.0f;
    const float AMS5915_0050_D_B_P_MIN = -50.0f;
    const float AMS5915_0050_D_B_P_MAX = 50.0f;
    const float AMS5915_0100_D_B_P_MIN = -100.0f;
    const float AMS5915_0100_D_B_P_MAX = 100.0f;
    const float AMS5915_0200_D_P_MIN = 0.0f;
    const float AMS5915_0200_D_P_MAX = 200.0f;
    const float AMS5915_0350_D_P_MIN = 0.0f;
    const float AMS5915_0350_D_P_MAX = 350.0f;
    const float AMS5915_1000_D_P_MIN = 0.0f;
    const float AMS5915_1000_D_P_MAX = 1000.0f;
    const float AMS5915_2000_D_P_MIN = 0.0f;
    const float AMS5915_2000_D_P_MAX = 2000.0f;
    const float AMS5915_4000_D_P_MIN = 0.0f;
    const float AMS5915_4000_D_P_MAX = 4000.0f;
    const float AMS5915_7000_D_P_MIN = 0.0f;
    const float AMS5915_7000_D_P_MAX = 7000.0f;
    const float AMS5915_10000_D_P_MIN = 0.0f;
    const float AMS5915_10000_D_P_MAX = 10000.0f;
    const float AMS5915_0200_D_B_P_MIN = -200.0f;
    const float AMS5915_0200_D_B_P_MAX = 200.0f;
    const float AMS5915_0350_D_B_P_MIN = -350.0f;
    const float AMS5915_0350_D_B_P_MAX = 350.0f;
    const float AMS5915_1000_D_B_P_MIN = -1000.0f;
    const float AMS5915_1000_D_B_P_MAX = 1000.0f;
    const float AMS5915_1000_A_P_MIN = 0.0f;
    const float AMS5915_1000_A_P_MAX = 1000.0f;
    const float AMS5915_1200_B_P_MIN = 700.0f;
    const float AMS5915_1200_B_P_MAX = 1200.0f;

    void getTransducer();
    bool readBytes(uint16_t* pressureCounts, uint16_t* temperatureCounts);
};

