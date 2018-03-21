/*
MS4525DO.h
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

#ifndef MS4525DO_H
#define MS4525DO_H

#include <Arduino.h>
#include <Wire.h>

class MS4525DO {
  public:
    MS4525DO();
    MS4525DO(uint8_t address, TwoWire *bus);
    bool begin();
    bool getData(float *pressure, float *temperature);
    
  private:
    bool _ready;
    uint8_t _address;
    TwoWire *_bus;

    // i2c bus frequency
    const uint32_t _i2cRate = 100000;

    // conversion millibar to PA
    const float _mBar2Pa = 100.0f; 
};

#endif // MS4525DO_H
