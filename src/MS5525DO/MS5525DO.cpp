// MS5525DO.cpp

#include "Arduino.h"
#include "MS5525DO.h"

const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_ADC_READ = 0x00;
const uint8_t CMD_CONVERT_PRESS = 0x44;
const uint8_t CMD_CONVERT_TEMP = 0x54;
const uint8_t CMD_PROM_START = 0xA0;

/* Default constructor */
MS5525DO::MS5525DO(){
    _address = 0x76; // I2C address
    _bus = NULL; // I2C bus
    _ready = false;
    state = 0;
}

/* MS5525DO object, input the I2C address and enumerated chip name (i.e. MS5525DO_1200_B) */
MS5525DO::MS5525DO(uint8_t address, TwoWire *bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    state = 0;
}

/* configure bus and i2c address */
void MS5525DO::configure(uint8_t address, TwoWire *bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    state = 0;
}

bool MS5525DO::send_command(uint8_t command) {
    _bus->beginTransmission(_address);
    _bus->write(command);  // update register pointer
    _bus->endTransmission();
    return true;
}

/* starts the I2C communication and sets the pressure and temperature ranges using getTransducer */
bool MS5525DO::begin() {
    Serial.print("Bringing up MS5525DO: ");
    
    // starting the I2C bus
    _bus->begin();
    _bus->setClock(_i2cRate);

    _bus->beginTransmission(_address);
    int result = _bus->endTransmission();
    delay(100);
    if ( result > 0 ) {
        Serial.print("MS5525DO init error: "); Serial.println(result);
        return false;
    }

    Serial.print("reseting. ");
    send_command(CMD_RESET);
    delay(500);
    
    Serial.print("reading prom: ");
    uint8_t numBytes = 2;
    uint8_t b[numBytes];
    for ( uint8_t i = 0; i < 8; i++ ) {
        _bus->beginTransmission(_address);
        _bus->write(CMD_PROM_START + i*2);  // update register pointer
        _bus->requestFrom(_address, numBytes);
        uint8_t counter = 0;
        while ( _bus->available() && counter < numBytes ) {
            b[counter] = _bus->read();
            counter++;
        }
        _bus->endTransmission();
        if ( counter < numBytes ) {
            Serial.print(" error, read bytes: "); Serial.println(counter);
            return false;
        }
        prom[i] = (b[0] << 8) | b[1];
        Serial.print("*");
        
    }
    Serial.println();

    C1 = prom[1];
    C2 = prom[2];
    C3 = prom[3];
    C4 = prom[4];
    C5 = prom[5];
    C6 = prom[6];
    Tref = int64_t(C5) * (1UL << Q5);
    _ready = true;
    return true;
}

/* reads pressure and temperature and returns values in counts */
bool MS5525DO::getData(float* pressure, float* temperature) {
    if ( ! _ready ) {
        return false;
    }

    if ( state == 0 ) {
        // request pressure on next cycle
        send_command(CMD_CONVERT_PRESS);
    } else if ( state == 1 ) {
    } else if ( state == 2 ) {
        // request temperature on next cycle
        send_command(CMD_CONVERT_TEMP);
    } else if ( state == 3 ) {
    }

    if ( state == 1 || state == 3 ) {
        const uint8_t numBytes = 3;
        uint8_t b[numBytes];
    
        _bus->beginTransmission(_address);
        _bus->write(CMD_ADC_READ);  // update register pointer
        _bus->requestFrom(_address, numBytes);
        int counter = 0;
        while ( _bus->available() && counter < numBytes ) {
            b[counter] = _bus->read();
            counter++;
        }
        _bus->endTransmission();

        if ( counter < numBytes ) {
            Serial.print("Error, fewer than expected bytes available on i2c read:");
            Serial.println(counter);
            return false;
        }

        uint32_t adc = (b[0] << 16) | (b[1] << 8) | b[2];
        if ( adc == 0 ) {
            // read not ready
            Serial.print(millis()); Serial.print(" ");
            Serial.println("MS5525 adc conversion not finished before read");
        } else {
            Serial.print("read ok: "); Serial.println(adc);
        }

        if ( state == 1 ) {
            D2 = adc;
        } else if ( state == 3 ) {
            D1 = adc;
        }
    }
    
    // Difference between actual and reference temperature
    //  dT = D2 - Tref
    int64_t dT = D2 - Tref;

    // Measured temperature
    //  TEMP = 20°C + dT * TEMPSENS
    int64_t TEMP = 2000 + (dT * int64_t(C6)) / (1UL << Q6);

    // Offset at actual temperature
    //  OFF = OFF_T1 + TCO * dT
    int64_t OFF = int64_t(C2) * (1UL << Q2) + (int64_t(C4) * dT) / (1UL << Q4);

    // Sensitivity at actual temperature
    //  SENS = SENS_T1 + TCS * dT
    int64_t SENS = int64_t(C1) * (1UL << Q1) + (int64_t(C3) * dT) / (1UL << Q3);
        
    *temperature = TEMP * 0.01f;

    // Temperature Compensated Pressure (example 24996 = 2.4996 psi)
    //  P = D1 * SENS - OFF
    int64_t P = (D1 * SENS / (1UL << 21) - OFF) / (1UL << 15);

    float diff_press_PSI = P * 0.0001f;

    // 1 PSI = 6894.76 Pascals
    const float PSI_to_Pa = 6894.757f;
    *pressure /* diff_press_pa_raw */ = diff_press_PSI * PSI_to_Pa;

    state = (state + 1) % 4;

    delay(100);
    return true;
}
