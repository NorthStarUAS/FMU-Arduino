#ifndef AURA_SETUP_H_INCLUDED
#define AURA_SETUP_H_INCLUDED

#include <Arduino.h>


//////////////////////////////////////////////////////////////////////////
// Hardware configuration section
//////////////////////////////////////////////////////////////////////////

// Specify one of the following board/build variants
// #define AURA_V2
#define MARMOT_V1

// automatic configuration
#if defined AURA_V2
 #if defined PIN_A22
 # error "Make sure you have selected the Teensy-3.2 board"
 #endif
 #define HAVE_AURA_BMP280
 #define HAVE_MS4525DO
 #define HAVE_ATTOPILOT
#elif defined MARMOT_V1
 #if ! defined PIN_A22
 # error "Make sure you have selected the Teensy-3.6 board"
 #endif
 #define HAVE_MARMOT_BME280
 #define HAVE_BFS_SWIFT
#endif

// Configuration flags available:

// Specify which onboard pressure sensor is installed
// #define HAVE_AURA_BMP280
// #define HAVE_MARMOT_BME280

// Specify which external pressure sensor is installed
// #define HAVE_MS4525DO
// #define HAVE_BFS_SWIFT

// Specify Attopilot if supported
// #define HAVE_ATTOPILOT

// Firmware rev (needs to be updated here manually to match release number)
const int FIRMWARE_REV = 334;

// this is the master loop update rate.
const int MASTER_HZ = 100;
const int DT_MILLIS = (1000 / MASTER_HZ);

// Please read the important notes in the source tree about Teensy
// baud rates vs. host baud rates.
const int DEFAULT_BAUD = 500000;


//////////////////////////////////////////////////////////////////////////
// AP definitions
//////////////////////////////////////////////////////////////////////////

// Maximum number of input or output channels supported
const int AP_CHANNELS = 6;


extern uint16_t serial_number;

#endif // AURA_SETUP_H_INCLUDED
