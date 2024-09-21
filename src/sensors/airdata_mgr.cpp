// Module to query air data sensors

#include <Arduino.h>

#include "../../setup_board.h"
#include "../nodes.h"
#include "../util/constants.h"

#include "airdata_mgr.h"

#include "BMP180/SFE_BMP180.h"
static SFE_BMP180 bmp180;
static bool bmp180_status = false;

#include "BME280/BME280.h"
static BME280 bme280;
static int bme280_status = -1;

#include "AMS5915/AMS5915.h"
static AMS5915 ams_barometer;
static AMS5915 ams_pitot;

#include "MS4525DO/MS4525DO.h"
static MS4525DO ms45_pitot;

#include "MS5525DO/MS5525DO.h"
static MS5525DO ms55_pitot;

void airdata_mgr_t::init() {
    config_airdata_node = PropertyNode("/config/airdata");

#if defined(MARMOT_V1)
    barometer = 1;
#elif defined(AURA_V2) || defined(NORTHSTAR_V3)
    barometer = 2;
#endif

    if ( config_airdata_node.getString("barometer") == "swift" ) {
        barometer = 3;
    } else if ( config_airdata_node.getString("barometer") == "bmp180" ) {
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
        if ( bme280_status < 0 ) {
            printf("BME280 barometer initialization unsuccessful\n");
            printf("Check wiring or try cycling power\n");
        } else {
            printf("BME280 barometer driver ready.\n");
        }
    } else if (barometer == 3 ) {
        int baro_addr = config_airdata_node.getInt("swift_baro_addr_dec");
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
        } else {
            printf("BMP180 barometer ready.\n");
        }
    } else {
        printf("No barometer detected or configured.\n");
    }

    if ( config_airdata_node.getString("pitot") == "ms45" ) {
        pitot = 1;
        ms45_pitot.configure(0x28, &Wire1);
        ms45_pitot.begin();
    } else if ( config_airdata_node.getString("pitot") == "ms55" ) {
        pitot = 2;
        ms55_pitot.configure(0x76, &Wire1);
        ms55_pitot.begin();
    } else if ( config_airdata_node.getString("pitot") == "swift" ) {
        pitot = 3;
        int pitot_addr = config_airdata_node.getInt("swift_pitot_addr_dec");
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

void airdata_mgr_t::compute_altitude() {
    // Compute altitude from airdata barometer

    // Forumula taken from:
    //   http://keisan.casio.com/exec/system/1224585971
    // or possibly:
    //   http://keisan.casio.com/has10/SpecExec.cgi?path=06000000%2eScience%2f02100100%2eEarth%20science%2f12000300%2eAltitude%20from%20atmospheric%20pressure%2fdefault%2exml&charset=utf-8
    //
    // h = (((P0/P)^(1/5.257) - 1) * (T+273.15)) / 0.0065
    // T = h*0.0065 / ((P0/P)^(1/5.257) - 1) - 273.15
    //
    // This formula tracks pretty well < 10k but starts to diverge at the higher altitudes.

    const float P0 = 1013.25;	// standard sea level pressure (mbar)
    float P = airdata_node.getDouble("baro_press_pa") * 0.01; // baro (converted from Pa to mbar)
    if ( P < 0.1 ) {
        P = P0;
    }

    // Typically the onboard pressure sensor reports board/electronics
    // temperature or at best, cabin temp.  This formula wants outside air temp.
    // We probably do not have that so just pick the standard temp and live with
    // a consistent error that doesn't drift with board or aircraft cabin temp
    // changes. Later we have a later system that estimates the error between
    // gps altitude and pressure altitude so we can auto correct these errors
    // anyway.
    const float T = 15.0;	// standard temp

    // Compute altitude on a standard day
    float tmp1 = pow((P0/P), 1.0/5.257) - 1.0;
    float alt_m = (tmp1 * (T + 273.15)) / 0.0065;
    airdata_node.setDouble( "altitude_m", alt_m );
}

void airdata_mgr_t::compute_airspeed() {
    // basic pressure to airspeed formula: v = sqrt((2/p) * q)
    // where v = velocity, q = dynamic pressure (pitot tube sensor
    // value), and p = air density.

    // if p is specified in kg/m^3 (value = 1.225) and if q is
    // specified in Pa (N/m^2) where 1 psi == 6900 Pa, then the
    // velocity will be in meters per second.

    float diff_press_pa = airdata_node.getDouble("diff_press_pa");

    // zero calibrate the diff pressure sensor
    if ( ! airspeed_inited ) {
        if ( airspeed_init_start_millis > 0 ) {
            pitot_sum += diff_press_pa;
            pitot_count++;
            pitot_offset = pitot_sum / (double)pitot_count;
            /* printf("a1 raw=%.1f filt=%.1f a1 off=%.1f a1 sum=%.1f a1 count=%d\n",
               analog[0], pitot_filt.get_value(), pitot_offset, pitot_sum,
               pitot_count); */
        } else {
            airspeed_init_start_millis = millis();
            pitot_sum = 0.0;
            pitot_count = 0;
        }
        if ( millis() > airspeed_init_start_millis + 10.0 * 1000 ) {
            Serial.print("pitot_offset inited = "); Serial.println(pitot_offset);
            airspeed_inited = true;
        }
    }

    float pitot_butter = pitot_filter.update(diff_press_pa);
    float pitot_cal = config_airdata_node.getDouble("pitot_calibrate");
    if ( pitot_cal < 0.5 or pitot_cal > 1.5 ) {
        pitot_cal = 1.0;
    }
    float Pa = pitot_butter - pitot_offset;
    if ( Pa < 0 ) { Pa = 0.0; }
    float airspeed_mps = sqrt( 2*Pa / 1.225 ) * pitot_cal;
    airdata_node.setDouble( "airspeed_mps", airspeed_mps );
    airdata_node.setDouble( "airspeed_kt", airspeed_mps * mps2kt );
}

void airdata_mgr_t::update() {

    // read barometer (static pressure sensor)
    if ( barometer == 1 || barometer == 2 ) {
        if ( bme280_status == 0 ) {
            bme280.getData(&baro_press, &baro_temp, &baro_hum);
            airdata_node.setDouble("baro_press_pa", baro_press);
            airdata_node.setDouble("baro_temp_C", baro_temp);
        }
    } else if ( barometer == 3 ) {
        if ( ams_barometer.getData(&baro_press, &baro_temp) ) {
            ams_baro_found = true;
            airdata_node.setDouble("baro_press_pa", baro_press);
            airdata_node.setDouble("baro_temp_C", baro_temp);
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
                        airdata_node.setDouble("baro_temp_C", baro_temp);
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
                        airdata_node.setDouble("baro_press_pa", baro_press);
                    }
                    bmp180_state = 0;
                }
            }
        }
    }

    compute_altitude();

    bool result = false;
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
        airdata_node.setDouble("diff_press_pa", diffPress_pa);
        airdata_node.setDouble("air_temp_C", temp_C);

        // airdata_node.setDouble("diff_press_pa", 819.2); // should be a little over 70 kts
        compute_airspeed();
    }
}
