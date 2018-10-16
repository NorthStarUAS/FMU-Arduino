#if defined HAVE_PWM_AURA
const uint8_t servoPins[PWM_CHANNELS] = {6, 5, 4, 3, 23, 22, 21, 20};
#elif defined HAVE_PWM_MARMOT
 const uint8_t servoPins[PWM_CHANNELS] = {21, 22, 23, 2, 3, 4, 5, 6};
#else
 #error "No PWM servo pin layout defined!"
#endif

// define if a channel is symmetrical or not (i.e. mapped to [0,1] for
// throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
bool pwm_symmetrical[PWM_CHANNELS] = {0, 1, 1, 1, 1, 0, 0, 0};

void pwm_setup() {
    // fixme: honor config rates
    // setting up pwm outputs and rates
    analogWriteResolution(16);
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        analogWrite(servoPins[i], 0); // zero signal to avoid surprises
        analogWriteFrequency(servoPins[i], servoFreq_hz);
    }
    
    // set default safe values for actuator outputs
    actuator_set_defaults();
    pwm_update();
}

// compute raw pwm values from normalized command values.  (handle
// actuator reversing here.)
void pwm_norm2pwm( float *norm, uint16_t *pwm ) {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        // convert to pulse length (special case ch6 when in flaperon mode)
        if ( pwm_symmetrical[i] || (i == 4 && config.actuators.mix_flaperon) ) {
            // i.e. aileron, rudder, elevator
            // Serial1.println(i);
            // Serial1.println(config.actuators.act_rev[i]);
            pwm[i] = PWM_CENTER + (int)(PWM_HALF_RANGE * norm[i] * config.actuators.act_gain[i]);
        } else {
            // i.e. throttle, flaps
            if ( config.actuators.act_gain[i] > 0.0 ) {
                pwm[i] = PWM_MIN + (int)(PWM_RANGE * norm[i] * config.actuators.act_gain[i]);
            } else {
                pwm[i] = PWM_MAX + (int)(PWM_RANGE * norm[i] * config.actuators.act_gain[i]);
            }
        }
        if ( pwm[i] < PWM_MIN ) {
            pwm[i] = PWM_MIN;
        }
        if ( pwm[i] > PWM_MAX ) {
            pwm[i] = PWM_MAX;
        }
    }
}


// write the raw actuator values to the RC system
void pwm_update() {
    // hook for testing servos
    if ( test_pwm_channel >= 0 && test_pwm_channel < PWM_CHANNELS ) {
        actuator_pwm[test_pwm_channel] = gen_pwm_test_value();
    }

    // sending servo pwm commands
    for ( uint8_t i = 0; i < PWM_CHANNELS; i++ ) {
        analogWrite(servoPins[i], actuator_pwm[i] / ((1/((float) servoFreq_hz)) * 1000000.0f )*65535.0f);
    }
}

// test drive a servo channel (sine wave)
uint16_t gen_pwm_test_value() {
    return sin((float)millis() / 500.0) * PWM_HALF_RANGE + PWM_CENTER;
}
