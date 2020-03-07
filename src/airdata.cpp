// Module to query air data sensors

#include "airdata.h"

#include "sensors/BMP180/SFE_BMP180.h"
static SFE_BMP180 bmp180;
static bool bmp180_status = false;

#include "sensors/BME280/BME280.h"
static BME280 bme280;
static bool bme280_status = false;

#include "sensors/AMS5915/AMS5915.h"
static AMS5915 ams_barometer;
static AMS5915 ams_pitot;

#include "sensors/MS4525DO/MS4525DO.h"
static MS4525DO ms45_pitot;

#include "sensors/MS5525DO/MS5525DO.h"
static MS5525DO ms55_pitot;

void airdata_t::defaults_none() {
    config.barometer = 0;
    config.pitot = 0;
    config.swift_baro_addr = 0;
    config.swift_pitot_addr = 0;
}

void airdata_t::defaults_goldy3() {
    config.barometer = 0; // 0 = onboard bmp280
    config.pitot = 2;     // 2 = swift
    config.swift_baro_addr = 0x24;
    config.swift_pitot_addr = 0x25;
}

void airdata_t::defaults_aura3() {
    config.barometer = 1; // 1 = bmp280/i2c
    config.pitot = 0;     // 0 = ms4525
    config.swift_baro_addr = 0;
    config.swift_pitot_addr = 0;
}

void airdata_t::setup() {
    if ( config.barometer == 0 || config.barometer == 1 ) {
        if ( config.barometer == 0 ) {
            // BME280/SPI
            Serial.println("BME280 on SPI:26");
            bme280.configure(26);
        } else if ( config.barometer == 1 ) {
            // BMP280/I2C
            Serial.println("BMP280 on I2C:0x76");
            bme280.configure(0x76, &Wire);
        }
        bme280_status = bme280.begin();
        if ( bme280_status < 0 ) {
            Serial.println("BME280 barometer initialization unsuccessful");
            Serial.println("Check wiring or try cycling power");
            delay(1000);
        } else {
            Serial.println("BME280 barometer driver ready.");
        }
    } else if (config.barometer == 1 ) {
        // BMP280/I2C
        Serial.println("BMP280 on I2C:0x76");
        bme280.configure(0x76, &Wire);
        bme280_status = bme280.begin();
    } else if (config.barometer == 2 ) {
        // BFS Swift
        Serial.print("Swift barometer on I2C: 0x");
        Serial.println(config.swift_baro_addr, HEX);
        ams_barometer.configure(config.swift_baro_addr, &Wire1, AMS5915_1200_B);
        ams_barometer.begin();
    } else if (config.barometer == 3 ) {
        // BMP180
        Serial.println("BMP180 on I2C");
        bmp180_status = bmp180.begin();
        if ( !bmp180_status ) {
            Serial.println("Onboard barometer initialization unsuccessful");
            Serial.println("Check wiring or try cycling power");
            delay(1000);
        } else {
            Serial.println("BMP180 barometer ready.");
        }
    }
    if ( bme280_status < 0 ) {
        Serial.println("Onboard barometer initialization unsuccessful");
        Serial.println("Check wiring or try cycling power");
        delay(1000);
    } else {
        Serial.println("Onboard barometer driver ready.");
    }

    if ( config.pitot == 0 ) {
        ms45_pitot.configure(0x28, &Wire);
        ms45_pitot.begin();
    } else if ( config.pitot == 1 ) {
        ms55_pitot.configure(0x76, &Wire);
        ms55_pitot.begin();
    } else if ( config.pitot == 2 ) {
        Serial.print("Swift pitot on I2C: 0x");
        Serial.println(config.swift_pitot_addr, HEX);
        ams_pitot.configure(config.swift_pitot_addr, &Wire1, AMS5915_0020_D);
        ams_pitot.begin();
    }
}

void airdata_t::update() {
    bool result;

    // read barometer (static pressure sensor)
    if ( config.barometer == 0 || config.barometer == 1 ) {
        if ( bme280_status ) {
            bme280.getData(&baro_press, &baro_temp, &baro_hum);
        }
    } else if ( config.barometer == 2 ) {
        if ( ams_barometer.getData(&baro_press, &baro_temp) ) {
            ams_baro_found = true;
        } else {
            if ( ams_baro_found ) {
                // Serial.println("Error while reading sPress sensor.");
                error_count++;
            }
        }
    } else if ( config.barometer == 3 ) {
        // BMP180 (requires a delicate dance of requesting a read,
        // then coming back some amount of millis later to do the
        // actual read.)
        static int bmp180_state = 0;
        static unsigned long wait_until;
        double tmp_temp;
        if ( bmp180_status ) {
            if ( bmp180_state == 0 ) {
                wait_until = bmp180.startTemperature() + millis();
                bmp180_state += 1;
            } else if ( bmp180_state == 1 ) {
                if ( millis() > wait_until ) {
                    if ( bmp180.getTemperature(tmp_temp) ) {
                        baro_temp = tmp_temp;
                    }
                    bmp180_state += 1;
                }
            } else if ( bmp180_state == 2 ) {
                wait_until = bmp180.startPressure(2) + millis();
                bmp180_state += 1;
            } else if ( bmp180_state == 3 ) {
                if ( millis() > wait_until ) {
                    double tmp;
                    if ( bmp180.getPressure(tmp, tmp_temp) ) {
                        baro_press = tmp;
                    }
                    bmp180_state = 0;
                }
            }
        }
    }

    if ( config.pitot == 0 ) {
        result = ms45_pitot.getData(&diffPress_pa, &temp_C);
    } else if ( config.pitot == 1 ) {
        result = ms55_pitot.getData(&diffPress_pa, &temp_C);
    } else if ( config.pitot == 2 ) {
        result = ams_pitot.getData(&diffPress_pa, &temp_C);
    } else {
        result = false;
    }
    if ( !result ) {
        if ( pitot_found ) {
            // Serial.println("Error while reading pitot sensor.");
            error_count++;
        }
    } else {
        pitot_found = true;
    }
}
