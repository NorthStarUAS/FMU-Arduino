// MS4525DO.cpp

#include "Arduino.h"
#include "MS4525DO.h"

/* Default constructor */
MS4525DO::MS4525DO(){
  _address = 0x28; // I2C address
  _bus = NULL; // I2C bus
  _ready = false;
}

/* MS4525DO object, input the I2C address and enumerated chip name (i.e. MS4525DO_1200_B) */
MS4525DO::MS4525DO(uint8_t address, TwoWire *bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
}

/* configure bus and i2c address */
void MS4525DO::configure(uint8_t address, TwoWire *bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
}

/* starts the I2C communication and sets the pressure and temperature ranges using getTransducer */
bool MS4525DO::begin(){
    // starting the I2C bus
    _bus->begin();
    _bus->setClock(_i2cRate);

    _bus->beginTransmission(_address);
    int result = _bus->endTransmission();
    delay(100);
    if ( result > 0 ) {
        Serial.print("MS4525DO init error: "); Serial.println(result);
        return false;
    } else {
        _ready = true;
        return true;
    }
}

/* reads pressure and temperature and returns values in counts */
bool MS4525DO::getData(float* pressure, float* temperature) {
    if ( ! _ready ) {
        return false;
    }

    uint8_t b[4]; // buffer
    const uint8_t numBytes = 4;

    // 4 bytes from address
    _bus->requestFrom(_address, numBytes);

    // put the data in buffer
    int counter = 0;
    while ( _bus->available() && counter < numBytes ) {
        b[counter] = _bus->read();
        counter++;
    }
    // _bus->endTransmission();

    if ( counter < numBytes ) {
        // Serial.println("Error, fewer than expected bytes available on i2c read");
        return false;
    } else {
        uint8_t status = (b[0] & 0xC0) >> 6;
        b[0] = b[0] & 0x3f;
        uint16_t dp_raw = (b[0] << 8) + b[1];

        uint16_t T_dat = (b[2] << 8) + b[3];
	    T_dat = (0xFFE0 & T_dat) >> 5;
        //b[3] = (b[3] >> 5);
        //uint16_t T_dat = ((b[2]) << 8) | b[3];

        // PR = (double)((P_dat-819.15)/(14744.7)) ;
        // PR = (PR - 0.49060678) ;
        // PR = abs(PR);
        // V = ((PR*13789.5144)/1.225);
        // VV = (sqrt((V)));

        // Calculate differential pressure. As its centered around 8000
        // and can go positive or negative
        const float P_min = -1.0f;
        const float P_max = 1.0f;
        const float PSI_to_Pa = 6894.757f;
        /*
        this equation is an inversion of the equation in the
        pressure transfer function figure on page 4 of the datasheet
        We negate the result so that positive differential pressures
        are generated when the bottom port is used as the static
        port on the pitot and top port is used as the dynamic port
        */
        float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
        *pressure = diff_press_PSI * PSI_to_Pa; // pa

        const float T_factor = 200.0 / 2047.0;
        *temperature = (float)T_dat * T_factor - 50.0; // C

        // #define DEBUG_ME
        #if defined DEBUG_ME
            Serial.print(status); Serial.print("\t");
            Serial.print(dp_raw); Serial.print("\t");
            Serial.print(*pressure,2); Serial.print("\t");
            Serial.print(T_dat); Serial.print("\t");
            Serial.print(*temperature,1); Serial.print("\t");
            Serial.println();
        #endif
    }
    return true;
}
