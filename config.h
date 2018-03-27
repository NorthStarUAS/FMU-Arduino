#ifndef _AURA_CONFIG_H_INCLUDED
#define _AURA_CONFIG_H_INCLUDED

#include <Arduino.h>

#include "structs.h"


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
 #define HAVE_IMU_I2C
 #define HAVE_AURA_BMP280
 #define HAVE_MS4525DO
 #define HAVE_ATTOPILOT
 #define HAVE_PWM_AURA
 const uint8_t avionics_pin = A1;
 const uint8_t pwr_pin = A0;
 #define HAVE_AURA_LED
 const int led_pin = 13;
#elif defined MARMOT_V1
 #if ! defined PIN_A22
 # error "Make sure you have selected the Teensy-3.6 board"
 #endif
 #define HAVE_IMU_SPI
 #define HAVE_MARMOT_BME280
 #define HAVE_BFS_SWIFT
 #define HAVE_PWM_MARMOT
 const uint8_t avionics_pin = A22;
 const uint8_t pwr_pin = 15;
#endif

// Configuration flags available:

// Specify IMU interface
// #define HAVE_IMU_I2C
// #define HAVE_IMU_SPI

// Specify which onboard pressure sensor is installed
// #define HAVE_AURA_BMP280
// #define HAVE_MARMOT_BME280

// Specify which external pressure sensor is installed
// #define HAVE_MS4525DO
// #define HAVE_BFS_SWIFT

// Specify Attopilot if supported
// #define HAVE_ATTOPILOT

// Specify PWM pin layout
// #define HAVE_PWM_AURA
// #define HAVE_PWM_MARMOT

// Do we have an LED we wish to blink
// #define HAVE_AURA_LED

// Firmware rev (needs to be updated here manually to match release number)
const int FIRMWARE_REV = 333;

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


extern config_t config;

extern uint16_t serial_number;

#endif /* _AURA_CONFIG_H_INCLUDED */
