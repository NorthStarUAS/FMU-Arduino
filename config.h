#ifndef _AURA_CONFIG_H_INCLUDED
#define _AURA_CONFIG_H_INCLUDED

#include <Arduino.h>

//////////////////////////////////////////////////////////////////////////
// Hardware configuration section
//////////////////////////////////////////////////////////////////////////

// Specify one of the following board/build variants
#define AURA_V2
// #define MARMOT_V1

// automatic configuration
#if defined AURA_V2
 #if defined PIN_A22
 # error "Make sure you have selected the Teensy-3.2 board"
 #endif
 #define HAVE_IMU_I2C
 // #define HAVE_AURA_BMP280
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
const int FIRMWARE_REV = 332;

// this is the master loop update rate.
const int MASTER_HZ = 100;
const int DT_MILLIS = (1000 / MASTER_HZ);

// Please read the important notes in the source tree about Teensy
// baud rates vs. host baud rates.
const int DEFAULT_BAUD = 500000;


//////////////////////////////////////////////////////////////////////////
// This is a section for RC / PWM constants to be shared around the sketch
//////////////////////////////////////////////////////////////////////////

// Maximum number of input or output channels supported
const int SBUS_CHANNELS = 16;
const int PWM_CHANNELS = 8;
const int AP_CHANNELS = 6;

const uint8_t SBUS_FRAMELOST = (1 << 2);
const uint8_t SBUS_FAILSAFE = (1 << 3);

// This is the hardware PWM generation rate note the default is 50hz
// and this is the max we can drive analog servos.  Digital servos
// should be able to run at 200hz.  250hz is getting up close to the
// theoretical maximum of a 100% duty cycle.  Advantage for running
// this at 200+hz with digital servos is we should catch commanded
// position changes slightly faster for a slightly more responsive
// system (emphasis on slightly).  In practice, changing this to
// something higher than 50 hz has little practical effect and can
// often cause problems with ESC's that expect 50hz pwm signals.
const int servoFreq_hz = 50; // servo pwm update rate

// For a Futaba T6EX 2.4Ghz FASST system:
//   Assuming all controls are at default center trim, no range scaling or endpoint adjustments:
//   Minimum position = 1107
//   Center position = 1520
//   Max position = 1933
#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_QUARTER_RANGE 206
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)


//////////////////////////////////////////////////////////////////////////
// config structure saved to eeprom
//////////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)           // set alignment to 1 byte boundary
typedef struct {
    int version;

    /* IMU orientation matrix */
    float imu_orient[9];
    
    /* hz for pwm output signal, 50hz default for analog servos, maximum rate is servo dependent:
       digital servos can usually do 200-250hz
       analog servos and ESC's typically require 50hz */
    uint16_t pwm_hz[PWM_CHANNELS];
    
    /* actuator gain (reversing/scaling) */
    float act_gain[PWM_CHANNELS];
    
    /* mixing modes */
    bool mix_autocoord;
    bool mix_throttle_trim;
    bool mix_flap_trim;
    bool mix_elevon;
    bool mix_flaperon;
    bool mix_vtail;
    bool mix_diff_thrust;

    /* mixing gains */
    float mix_Gac; // aileron gain for autocoordination
    float mix_Get; // elevator trim w/ throttle gain
    float mix_Gef; // elevator trim w/ flap gain
    float mix_Gea; // aileron gain for elevons
    float mix_Gee; // elevator gain for elevons
    float mix_Gfa; // aileron gain for flaperons
    float mix_Gff; // flaps gain for flaperons
    float mix_Gve; // elevator gain for vtail
    float mix_Gvr; // rudder gain for vtail
    float mix_Gtt; // throttle gain for diff thrust
    float mix_Gtr; // rudder gain for diff thrust
    
    /* sas modes */
    bool sas_rollaxis;
    bool sas_pitchaxis;
    bool sas_yawaxis;
    bool sas_tune;

    /* sas gains */
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_max_gain;
} config_t;
#pragma pack(pop)

extern config_t config;

extern uint16_t serial_number;

#endif /* _AURA_CONFIG_H_INCLUDED */
