#pragma once

//////////////////////////////////////////////////////////////////////////
// Hardware configuration section
//////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

// automatic configuration
#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
 #define NORTHSTAR_V3
#elif defined(ARDUINO_TEENSY32)
 #define AURA_V2
#elif defined(ARDUINO_TEENSY36)
 #define MARMOT_V1
#endif

// teensy32 can barely run the ekf at 50hz so only enable this feature
// for the newer processors.
#if defined(ARDUINO_TEENSY32)
 #undef AURA_ONBOARD_EKF
#else
 #define AURA_ONBOARD_EKF
#endif

// Firmware rev (needs to be updated here manually to match release number)
const int FIRMWARE_REV = 410;

// this is the master loop update rate.
const int MASTER_HZ = 100;
const int DT_MILLIS = (1000 / MASTER_HZ);

// Please read the important notes in the source tree about Teensy
// baud rates vs. host baud rates.
// const int HOST_BAUD = 500000;
// const int TELEMETRY_BAUD = 115200;

#if defined(ARDUINO_TEENSY36)
const uint32_t lfs_progm_bytes = 1024*128;   // allocate 128Kb flash disk,
#elif defined(ARDUINO_TEENSY40)
const uint32_t lfs_progm_bytes = 1024*1024;   // allocate 1.25Mb flash disk,
#elif defined(ARDUINO_TEENSY41)
const uint32_t lfs_progm_bytes = 1024*1024 * 7; // should be able to go up to 7Mb
#else
const uint32_t lfs_progm_bytes = 0;
#endif
