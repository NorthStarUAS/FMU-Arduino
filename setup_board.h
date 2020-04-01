#pragma once

//////////////////////////////////////////////////////////////////////////
// Hardware configuration section
//////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

// automatic configuration
#if defined(ARDUINO_TEENSY32) || defined(ARDUINO_TEENSY40)
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
const int FIRMWARE_REV = 400;

// this is the master loop update rate.
const int MASTER_HZ = 100;
const int DT_MILLIS = (1000 / MASTER_HZ);

// Please read the important notes in the source tree about Teensy
// baud rates vs. host baud rates.
const int DEFAULT_BAUD = 500000;

extern uint16_t serial_number;

