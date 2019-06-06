/*
MS5525DO.h
Curtis L. Olson
olson126@umn.edu

Copyright (c) 2018 Curtis L. Olson

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <Arduino.h>
#include <Wire.h>

class MS5525DO {
  public:
    MS5525DO();
    MS5525DO(uint8_t address, TwoWire *bus);
    void configure(uint8_t address, TwoWire *bus);
    bool begin();
    bool send_command(uint8_t command);
    bool getData(float *pressure, float *temperature);
    
  private:
    bool _ready;
    uint8_t _address;
    TwoWire *_bus;

    // Qx Coefficients Matrix by Pressure Range
    //  5525DSO-pp001DS (Pmin = -1, Pmax = 1)
    const uint8_t Q1 = 15;
    const uint8_t Q2 = 17;
    const uint8_t Q3 = 7;
    const uint8_t Q4 = 5;
    const uint8_t Q5 = 7;
    const uint8_t Q6 = 21;

    // calibration prom
    uint16_t prom[8];
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
    uint64_t Tref{0};
    
    // i2c bus frequency
    const uint32_t _i2cRate = 100000;

    // conversion millibar to PA
    const float _mBar2Pa = 100.0f;

    uint8_t state{0};              // collection state
    uint32_t D1{0};
    uint32_t D2{0};
};
