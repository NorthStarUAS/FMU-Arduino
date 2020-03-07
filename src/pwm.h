#pragma once

#include <Arduino.h>

// max number of pwm output channels
const int PWM_CHANNELS = 8;

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
//   These number match a futaba T6EX FASST system output
//   Minimum position = 1107
//   Center position = 1520
//   Max position = 1933
#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_QUARTER_RANGE 206
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)

class pwm_t {
private:
    uint16_t gen_pwm_test_value();
    
public:
    uint16_t actuator_pwm[PWM_CHANNELS];
    void setup(int board);
    void update(uint8_t test_pwm_channel = -1);
    void norm2pwm( float *norm, uint16_t *pwm);
};

// a global instance is available for use
extern pwm_t pwm;
