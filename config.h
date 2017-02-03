#ifndef _AURA_CONFIG_H_INCLUDED
#define _AURA_CONFIG_H_INCLUDED


// Firmware rev (needs to be updated here manually to match release number)
#define FIRMWARE_REV 300

// this is the master loop update rate.  For 115,200 baud communication, 100hz is as fast as
// we can go without saturating our uart link to the host.
#define MASTER_HZ 100

// This comment relates to the APM2 (atmega256) and not necessarily (?!?) the Teensy:
// Starting communication baud (16000000/16)/x where x is an integer produces the possible baud
// rates.  Note this is not the standard 300, 1200, 4800, 9600, 115,200, etc. series.  We can get
// close enough at the lower baud rates so both ends tolerate any slight discrepancy, but above
// 115,200 we diverge enough that it may not always work well end to end.  500,000 baud is a
// 1-to-1 match.
#define DEFAULT_BAUD 115200


//////////////////////////////////////////////////////////////////////////
// This is a section for RC / PWM constants to be shared around the sketch
//////////////////////////////////////////////////////////////////////////

// Maximum number of input or output channels supported
#define MAX_CHANNELS 16
#define NUM_PWM_CHANNELS 5

// this is the hardware PWM generation rate
// note the default is 50hz and this is the max we can drive analog servos
// digital servos should be able to run at 200hz -- 250hz is getting up close to the theoretical maximum
// of a 100% duty cycle.  Advantage for running this at 200+hz with digital servos is we should catch commanded
// position changes slightly faster for a slightly more responsive system (emphasis on slightly)
// TODO: make this configurable via an external command.
#define DEFAULT_PWM_HZ 50

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

typedef struct {
    int version;
    
    /* hz for pwm output signal, 50hz default for analog servos, maximum rate is servo dependent:
       digital servos can usually do 200-250hz
       analog servos and ESC's typically require 50hz */
    uint16_t pwm_hz[MAX_CHANNELS];
    
    /* actuator gain (reversing) */
    float act_gain[MAX_CHANNELS];
    
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
    bool sas_ch7tune;

    /* sas gains */
    float sas_rollgain;
    float sas_pitchgain;
    float sas_yawgain;
    float sas_ch7gain;
} config_t;

extern config_t config;

extern uint16_t apm2_serial_number;

#endif /* _AURA_CONFIG_H_INCLUDED */

