/*
AMS5915.cpp
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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "AMS5915.h"

/* Default constructor */
AMS5915::AMS5915(){
  _address = 0x28; // I2C address
  _bus = NULL; // I2C bus
  _type = AMS5915_1200_B; // transducer type
}

/* AMS5915 object, input the I2C address and enumerated chip name (i.e. AMS5915_1200_B) */
AMS5915::AMS5915(uint8_t address, TwoWire *bus, ams5915_transducer type){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _type = type; // transducer type
}

void AMS5915::configure(uint8_t address, TwoWire *bus, ams5915_transducer type){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _type = type; // transducer type
}

/* starts the I2C communication and sets the pressure and temperature ranges using getTransducer */
void AMS5915::begin(){
    // starting the I2C bus
    _bus->begin();
    _bus->setClock(_i2cRate);
    
    _bus->beginTransmission(_address);
    _bus->endTransmission();
    delay(100);

    // setting the min and max pressure and temperature based on the chip
    getTransducer();
}

/* reads pressure and temperature and returns values in counts */
bool AMS5915::readBytes(uint16_t* pressureCounts, uint16_t* temperatureCounts){
    uint8_t b[4]; // buffer
    const uint8_t numBytes = 4;
    bool result = true;

    // 4 bytes from address
    _bus->requestFrom(_address, numBytes); 
  
    // put the data in buffer
    int counter = 0;
    while ( _bus->available() && counter < numBytes ) {
        b[counter] = _bus->read();
        counter++;
    }

    if (counter == numBytes) {
        result = true;
        // assemble into a uint16_t
        *pressureCounts = (((uint16_t) (b[0]&0x3F)) <<8) + (((uint16_t) b[1]));
        *temperatureCounts = (((uint16_t) (b[2])) <<3) + (((uint16_t) b[3]&0xE0)>>5);
    } else {
        // problem with bytes available
        result = false;
    }

    return result;
}

/* sets the pressure range based on the chip */
void AMS5915::getTransducer(){

  // setting the min and max pressures based on which transducer it is
  switch(_type) {
    case AMS5915_0005_D:
        _pMin = AMS5915_0005_D_P_MIN;
        _pMax = AMS5915_0005_D_P_MAX;
        break;
    case AMS5915_0010_D:
        _pMin = AMS5915_0010_D_P_MIN;
        _pMax = AMS5915_0010_D_P_MAX;
        break;
    case AMS5915_0005_D_B:
        _pMin = AMS5915_0005_D_B_P_MIN;
        _pMax = AMS5915_0005_D_B_P_MAX;
        break;
    case AMS5915_0010_D_B:
        _pMin = AMS5915_0010_D_B_P_MIN;
        _pMax = AMS5915_0010_D_B_P_MAX;
        break;
    case AMS5915_0020_D:
        _pMin = AMS5915_0020_D_P_MIN;
        _pMax = AMS5915_0020_D_P_MAX;
        break;
    case AMS5915_0050_D:
        _pMin = AMS5915_0050_D_P_MIN;
        _pMax = AMS5915_0050_D_P_MAX;
        break;
    case AMS5915_0100_D:
        _pMin = AMS5915_0100_D_P_MIN;
        _pMax = AMS5915_0100_D_P_MAX;
        break;
    case AMS5915_0020_D_B:
        _pMin = AMS5915_0020_D_B_P_MIN;
        _pMax = AMS5915_0020_D_B_P_MAX;
        break;
    case AMS5915_0050_D_B:
        _pMin = AMS5915_0050_D_B_P_MIN;
        _pMax = AMS5915_0050_D_B_P_MAX;
        break;
    case AMS5915_0100_D_B:
        _pMin = AMS5915_0100_D_B_P_MIN;
        _pMax = AMS5915_0100_D_B_P_MAX;
        break;
    case AMS5915_0200_D:
        _pMin = AMS5915_0200_D_P_MIN;
        _pMax = AMS5915_0200_D_P_MAX;
        break;
    case AMS5915_0350_D:
        _pMin = AMS5915_0350_D_P_MIN;
        _pMax = AMS5915_0350_D_P_MAX;
        break;
    case AMS5915_1000_D:
        _pMin = AMS5915_1000_D_P_MIN;
        _pMax = AMS5915_1000_D_P_MAX;
        break;
    case AMS5915_2000_D:
        _pMin = AMS5915_2000_D_P_MIN;
        _pMax = AMS5915_2000_D_P_MAX;
        break;
    case AMS5915_4000_D:
        _pMin = AMS5915_4000_D_P_MIN;
        _pMax = AMS5915_4000_D_P_MAX;
        break;
    case AMS5915_7000_D:
        _pMin = AMS5915_7000_D_P_MIN;
        _pMax = AMS5915_7000_D_P_MAX;
        break;
    case AMS5915_10000_D:
        _pMin = AMS5915_10000_D_P_MIN;
        _pMax = AMS5915_10000_D_P_MAX;
        break;
    case AMS5915_0200_D_B:
        _pMin = AMS5915_0200_D_B_P_MIN;
        _pMax = AMS5915_0200_D_B_P_MAX;
        break;
    case AMS5915_0350_D_B:
        _pMin = AMS5915_0350_D_B_P_MIN;
        _pMax = AMS5915_0350_D_B_P_MAX;
        break;
    case AMS5915_1000_D_B:
        _pMin = AMS5915_1000_D_B_P_MIN;
        _pMax = AMS5915_1000_D_B_P_MAX;
        break;
    case AMS5915_1000_A:
        _pMin = AMS5915_1000_A_P_MIN;
        _pMax = AMS5915_1000_A_P_MAX;
        break;
    case AMS5915_1200_B:
        _pMin = AMS5915_1200_B_P_MIN;
        _pMax = AMS5915_1200_B_P_MAX;
        break;
  }
}

/* gets both the pressure (PA) and temperature (C) values */
bool AMS5915::getData(float* pressure, float* temperature){
    uint16_t digOutPmin = 1638;   // digital output at minimum pressure
    uint16_t digOutPmax = 14745;  // digital output at maximum pressure
    uint16_t pressureCounts; // pressure digital output, counts
    uint16_t temperatureCounts; // temperature digital output, counts

    // get pressure and temperature off transducer
    bool result = readBytes(&pressureCounts, &temperatureCounts);
    if ( result ) {
        // convert counts to pressure, PA
        *pressure = ((pressureCounts - digOutPmin)/((digOutPmax - digOutPmin)/(_pMax - _pMin)) + _pMin) * _mBar2Pa;
        // convert counts to temperature, C
        *temperature = (temperatureCounts * 200.0f)/2048.0f - 50.0f;
    }
    return result;
}

#endif
