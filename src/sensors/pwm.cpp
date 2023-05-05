#include "../config.h" // fixme remove?
#include "pwm.h"

// For a Futaba T6EX 2.4Ghz FASST system:
//   These number match a futaba T6EX FASST system output
//   Minimum position = 1107
//   Center position = 1520
//   Max position = 1933
static const int PWM_CENTER = 1520;
static const int PWM_HALF_RANGE = 413;
static const int PWM_QUARTER_RANGE = 206;
static const int PWM_RANGE = PWM_HALF_RANGE * 2;
static const int PWM_MIN = PWM_CENTER - PWM_HALF_RANGE;
static const int PWM_MAX = PWM_CENTER + PWM_HALF_RANGE;

static const uint8_t marmot1_pins[PWM_CHANNELS] = {21, 22, 23, 2, 3, 4, 5, 6};
static const uint8_t aura2_pins[PWM_CHANNELS] = {6, 5, 4, 3, 23, 22, 21, 20};
static uint8_t servoPins[PWM_CHANNELS];

// define if a channel is symmetrical or not (i.e. mapped to [0,1] for
// throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
// static bool pwm_symmetrical[PWM_CHANNELS] = {0, 1, 1, 1, 1, 0, 0, 0};
static const uint16_t pwm_symmetrical = ~(1 << 0 | 1 << 4 | 1 << 5);

// This is the hardware PWM generation rate note the default is 50hz
// and this is the max we can drive analog servos.  Digital servos
// should be able to run at 200hz.  250hz is getting up close to the
// theoretical maximum of a 100% duty cycle.  Advantage for running
// this at 200+hz with digital servos is we should catch commanded
// position changes slightly faster for a slightly more responsive
// system (emphasis on slightly).  In practice, changing this to
// something higher than 50 hz has little practical effect and can
// often cause problems with ESC's that expect 50hz pwm signals.
static const int servoFreq_hz = 50; // servo pwm update rate

// reset actuator gains (reversing) to startup defaults
void pwm_t::act_gain_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
        config.pwm.act_gain[i] = 1.0;
    }
}

void pwm_t::setup(int board) {
    Serial.print("PWM: ");
    if ( board == 0 ) {
        Serial.println("Marmot v1 pin mapping.");
        for ( int i = 0; i < PWM_CHANNELS; i++ ) {
            servoPins[i] = marmot1_pins[i];
        }
    } else if ( board == 1 ) {
        Serial.println("Aura v2 pin mapping.");
        for ( int i = 0; i < PWM_CHANNELS; i++ ) {
            servoPins[i] = aura2_pins[i];
        }
    } else {
        Serial.println("No valid PWM pin mapping defined");
    }

    analogWriteResolution(16);
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        analogWrite(servoPins[i], 0); // zero signal to avoid surprises
        analogWriteFrequency(servoPins[i], servoFreq_hz);
    }

    // set default safe values for actuator outputs (should already at
    // a higher level, but this is important enough to do it again
    // just in case someone changed the higher level and messed up the
    // init order without realizing)
    // actuators.setup();
    // update();
}

// float pilot_t::rcin2norm(uint16_t pwm_val, uint8_t channel) {
//     float norm = 0.0;
//     if ( rcin_symmetrical & (1<<channel) ) {
//         // i.e. aileron, rudder, elevator
//         norm = (float)((int)pwm_val - PWM_CENTER) / PWM_HALF_RANGE;
//     } else {
//         // i.e. throttle, flaps, etc.
//         norm = (float)((int)pwm_val - PWM_MIN) / PWM_RANGE;
//     }
//     return norm;
// }

uint16_t pwm_t::norm2pwm(float norm_val, uint8_t channel) {
    uint16_t output = PWM_CENTER;
    if ( pwm_symmetrical & (1<<channel) ) {
        output = PWM_CENTER + (int)(PWM_HALF_RANGE * norm_val); // * config.pwm_cfg.act_gain[i]);
    } else {
        output = PWM_MIN + (int)(PWM_RANGE * norm_val); // * config.pwm_cfg.act_gain[i]);
    }
    if ( output < PWM_MIN ) {
        output = PWM_MIN;
    }
    if ( output > PWM_MAX ) {
        output = PWM_MAX;
    }
    return output;
}

// compute raw pwm values from normalized command values.  (handle
// actuator reversing here.)
void pwm_t::norm2pwm_batch( float *norm ) {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        // convert to pulse length (special case ch6 when in flaperon mode)
        if ( pwm_symmetrical & (1<<i) ) /* FIXME: flaperon? */ {
            // i.e. aileron, rudder, elevator
            // Serial1.println(i);
            // Serial1.println(config_actuators.act_rev[i]);
            output_pwm[i] = PWM_CENTER + (int)(PWM_HALF_RANGE * norm[i] * config.pwm.act_gain[i]);
        } else {
            // i.e. throttle, flaps
            if ( config.pwm.act_gain[i] > 0.0 ) {
                output_pwm[i] = PWM_MIN + (int)(PWM_RANGE * norm[i] * config.pwm.act_gain[i]);
            } else {
                output_pwm[i] = PWM_MAX + (int)(PWM_RANGE * norm[i] * config.pwm.act_gain[i]);
            }
        }
        if ( output_pwm[i] < PWM_MIN ) {
            output_pwm[i] = PWM_MIN;
        }
        if ( output_pwm[i] > PWM_MAX ) {
            output_pwm[i] = PWM_MAX;
        }
    }
}


// write the raw actuator values to the RC system
void pwm_t::write(uint8_t test_pwm_channel) {
    // hook for testing servos
    if ( test_pwm_channel >= 0 && test_pwm_channel < PWM_CHANNELS ) {
        output_pwm[test_pwm_channel] = gen_pwm_test_value();
    }

    // sending servo pwm commands
    for ( uint8_t i = 0; i < PWM_CHANNELS; i++ ) {
        analogWrite(servoPins[i], output_pwm[i] / ((1/((float) servoFreq_hz)) * 1000000.0f )*65535.0f);
    }
}

// test drive a servo channel (sine wave)
uint16_t pwm_t::gen_pwm_test_value() {
    return sin((float)millis() / 500.0) * PWM_HALF_RANGE + PWM_CENTER;
}

// make a global instance
pwm_t pwm;
