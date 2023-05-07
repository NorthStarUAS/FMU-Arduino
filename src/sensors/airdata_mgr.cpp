// Module to query air data sensors

#include <Arduino.h>

#include "airdata_mgr.h"

#include "BMP180/SFE_BMP180.h"
static SFE_BMP180 bmp180;
static bool bmp180_status = false;

#include "BME280/BME280.h"
static BME280 bme280;
static bool bme280_status = false;

#include "AMS5915/AMS5915.h"
static AMS5915 ams_barometer;
static AMS5915 ams_pitot;

#include "MS4525DO/MS4525DO.h"
static MS4525DO ms45_pitot;

#include "MS5525DO/MS5525DO.h"
static MS5525DO ms55_pitot;

// void airdata_mgr_t::defaults_none() {
//     config.airdata.barometer = 0;
//     config.airdata.pitot = 0;
//     config.airdata.swift_baro_addr = 0;
//     config.airdata.swift_pitot_addr = 0;
// }

// void airdata_mgr_t::defaults_goldy3() {
//     config.airdata.barometer = 0; // 0 = onboard bmp280
//     config.airdata.pitot = 2;     // 2 = swift
//     config.airdata.swift_baro_addr = 0x24;
//     config.airdata.swift_pitot_addr = 0x25;
// }

// void airdata_mgr_t::defaults_aura3() {
//     config.airdata.barometer = 1; // 1 = bmp280/i2c
//     config.airdata.pitot = 0;     // 0 = ms4525
//     config.airdata.swift_baro_addr = 0;
//     config.airdata.swift_pitot_addr = 0;
// }

void airdata_mgr_t::setup() {
    airdata_node = PropertyNode("/sensors/airdata");
    config_node = PropertyNode("/config/airdata");

#if defined(MARMOT_V2)
    barometer = 1;
#elif defined(AURA_V2)
    barometer = 2;
#endif

    if ( config_node.getString("barometer") == "swift" ) {
        barometer = 3;
    } else if ( config_node.getString("barometer") == "bmp180" ) {
        barometer = 4;
    }

    if ( barometer == 1 || barometer == 2 ) {
        if ( barometer == 1 ) {
            // BME280/SPI
            printf("BME280 on SPI:26\n");
            bme280.configure(26);
        } else if ( barometer == 2 ) {
            // BMP280/I2C
            Serial.println("BMP280 on I2C:0x76");
            bme280.configure(0x76, &Wire);
        }
        bme280_status = bme280.begin();
        if ( !bme280_status ) {
            printf("BME280 barometer initialization unsuccessful\n");
            printf("Check wiring or try cycling power\n");
            delay(1000);
        } else {
            printf("BME280 barometer driver ready.\n");
        }
    } else if (barometer == 3 ) {
        int baro_addr = config_node.getInt("swift_baro_addr_dec");
        if ( baro_addr > 0 ) {
            // BFS Swift
            printf("Swift barometer on I2C: 0x%02x\n", baro_addr);
            ams_barometer.configure(baro_addr, &Wire1, AMS5915_1200_B);
            ams_barometer.begin();
        } else {
            barometer = 0;
            printf("/config/airdata/swift_baro_addr_dec not specified correctly.\n");
        }
    } else if ( barometer == 4 ) {
        // BMP180
        printf("BMP180 on I2C\n");
        bmp180_status = bmp180.begin();
        if ( !bmp180_status ) {
            printf("Onboard barometer initialization unsuccessful.\n");
            printf("Check wiring or try cycling power.\n");
            delay(1000);
        } else {
            printf("BMP180 barometer ready.\n");
        }
    } else {
        printf("No barometer detected or configured.\n");
    }

    if ( config_node.getString("pitot") == "ms45" ) {
        pitot = 1;
        ms45_pitot.configure(0x28, &Wire);
        ms45_pitot.begin();
    } else if ( config_node.getString("pitot") == "ms55" ) {
        pitot = 2;
        ms55_pitot.configure(0x76, &Wire);
        ms55_pitot.begin();
    } else if ( config_node.getString("pitot") == "swift" ) {
        pitot = 3;
        int pitot_addr = config_node.getInt("swift_pitot_addr_dec");
        if ( pitot_addr > 0 ) {
            printf("Swift pitot on I2C: 0x%02x\n", pitot_addr);
            ams_pitot.configure(pitot_addr, &Wire1, AMS5915_0020_D);
            ams_pitot.begin();
        } else {
            pitot = 0;
            printf("/config/airdata/swift_pitot_addr_dec not specified correctly.\n");
        }
    }
}

void airdata_mgr_t::update() {
    bool result;

    // read barometer (static pressure sensor)
    if ( barometer == 1 || barometer == 2 ) {
        if ( bme280_status ) {
            bme280.getData(&baro_press, &baro_temp, &baro_hum);
        }
    } else if ( barometer == 3 ) {
        if ( ams_barometer.getData(&baro_press, &baro_temp) ) {
            ams_baro_found = true;
        } else {
            if ( ams_baro_found ) {
                // Serial.println("Error while reading sPress sensor.");
                error_count++;
            }
        }
    } else if ( barometer == 4 ) {
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

    if ( pitot == 1 ) {
        result = ms45_pitot.getData(&diffPress_pa, &temp_C);
    } else if ( pitot == 2 ) {
        result = ms55_pitot.getData(&diffPress_pa, &temp_C);
    } else if ( pitot == 3 ) {
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

// shared global instance
airdata_mgr_t airdata_mgr;
