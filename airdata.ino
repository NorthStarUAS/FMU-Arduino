#include "config.h"

#if defined HAVE_AURA_BMP280
 #include "src/BME280/BME280.h"
 BME280 barometer(0x76, &Wire);
#elif defined HAVE_MARMOT_BME280
 #include "src/BME280/BME280.h"
 BME280 barometer(26);
#endif

#if defined HAVE_MS4525DO
# include "src/MS4525DO/MS4525DO.h"
#elif defined HAVE_BFS_SWIFT
# include "src/AMS5915/AMS5915.h"
#endif

#if defined HAVE_MS4525DO
 MS4525DO dPress(0x28, &Wire);
#elif defined HAVE_BFS_SWIFT
 AMS5915 dPress(0x27, &Wire1, AMS5915_0020_D);
 AMS5915 sPress(0x26, &Wire1, AMS5915_1200_B);
#endif

int baro_status = -1;
float baro_press, baro_temp, baro_hum;

bool dpress_found = false;
bool spress_found = false;

void airdata_setup() {
   #if defined HAVE_AURA_BMP280 || defined HAVE_MARMOT_BME280
    baro_status = barometer.begin();
    if ( baro_status < 0 ) {
        Serial.println("Onboard barometer initialization unsuccessful");
        Serial.println("Check wiring or try cycling power");
        delay(1000);
    } else {
        Serial.println("Onboard barometer driver ready.");
    }
   #endif

   #if defined HAVE_MS4525DO
    dPress.begin();
   #elif defined HAVE_BFS_SWIFT
    dPress.begin();
    sPress.begin();
   #endif
}

void airdata_update() {
    // onboard static pressure sensor
   #if defined HAVE_AURA_BMP280 || defined HAVE_MARMOT_BME280
    if ( baro_status >= 0 ) {
        // get the pressure (Pa), temperature (C),
        // and humidity data (%RH) all at once
        barometer.getData(&baro_press, &baro_temp, &baro_hum);
    }
   #endif

    bool result;
    
   #if defined HAVE_MS4525DO || defined HAVE_BFS_SWIFT
    // external differential pressure sensor
    result = dPress.getData(&airdata_diffPress_pa, &airdata_temp_C);
    if ( !result ) {
        if ( dpress_found ) {
            Serial.println("Error while reading dPress sensor.");
            airdata_error_count++;
        }
    } else {
        dpress_found = true;
    }
   #endif
   #if defined HAVE_BFS_SWIFT
    // external differential pressure sensor
    float tmp;
    result = sPress.getData(&airdata_staticPress_pa, &tmp);
    if ( !result ) {
        if ( spress_found ) {
            Serial.println("Error while reading sPress sensor.");
            airdata_error_count++;
        }
    } else {
        spress_found = true;
    }
   #endif
}
