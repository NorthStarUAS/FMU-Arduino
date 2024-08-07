#include "../../setup_board.h"

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

#if defined(MARMOT_V1)
static uint8_t servoPins[PWM_CHANNELS] = {21, 22, 23, 2, 3, 4, 5, 6};
#elif defined(AURA_V2) || defined(NORTHSTAR_V3)
static uint8_t servoPins[PWM_CHANNELS] = {6, 5, 4, 3, 23, 22, 21, 20};
#else
static uint8_t servoPins[PWM_CHANNELS] = {0};
#endif

// define if a channel is symmetrical or not (i.e. mapped to [0,1] for
// throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
// static bool pwm_symmetrical[PWM_CHANNELS] = {0, 1, 1, 1, 1, 0, 0, 0};
static const uint16_t pwm_symmetrical = ~(1 << 0 | 1 << 4 | 1 << 5);

// This is the hardware PWM generation rate note the default is 50hz and this is
// the max we can drive analog servos.  Digital servos should be able to run at
// 200hz.  250hz is getting up close to the theoretical maximum of a 100% duty
// cycle.  Advantage for running this at 200+hz with digital servos is we should
// catch commanded position changes slightly faster for a slightly more
// responsive system (emphasis on slightly).  In practice, changing this to
// something higher than 50 hz has little practical effect and can often cause
// problems with ESC's that expect 50hz pwm signals.
static const int servoFreq_hz = 50; // servo pwm update rate

void pwm_t::init(int board) {
    printf("PWM: ");
#if defined(MARMOT_V1)
    printf("Marmot v1 pin mapping.\n");
#elif defined(AURA_V2) || defined(NORTHSTAR_V3)
    printf("Aura v2 / NorthStar v3 pin mapping.\n");
#else
    printf("No valid PWM pin mapping defined.\n");
#endif

    analogWriteResolution(16);
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        analogWrite(servoPins[i], 0); // zero signal to avoid surprises
        analogWriteFrequency(servoPins[i], servoFreq_hz);
    }
}

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
