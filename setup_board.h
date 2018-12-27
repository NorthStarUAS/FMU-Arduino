#ifndef AURA_SETUP_H_INCLUDED
#define AURA_SETUP_H_INCLUDED

#include <Arduino.h>


//////////////////////////////////////////////////////////////////////////
// Hardware configuration section
//////////////////////////////////////////////////////////////////////////

// automatic configuration
#if defined(__MK20DX256__)
 #define HAVE_TEENSY32
 #define AURA_V2
#elif defined(__MK66FX1M0__)
 #define HAVE_TEENSY36
 #define MARMOT_V1
#endif

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