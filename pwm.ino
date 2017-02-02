// compute normalized command values from the raw pwm values
void pwm_pwm2norm( uint16_t *pwm, float *norm ) {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // convert to normalized form
        if ( symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
	    norm[i] = (float)((int)pwm[i] - PWM_CENTER) / PWM_HALF_RANGE;
        } else {
	    // i.e. throttle, flaps
	    norm[i] = (float)((int)pwm[i] - PWM_MIN) / PWM_RANGE;
        }
    }
}


// compute raw pwm values from normalized command values.
// (handle actuator reversing here.)
void pwm_norm2pwm( float *norm, uint16_t *pwm ) {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // convert to pulse length (special case ch6 when in flaperon mode)
        if ( symmetrical[i] || (i == 5 && config.mix_flaperon) ) {
            // i.e. aileron, rudder, elevator
            //Serial.println(i);
            //Serial.println(config.act_rev[i]);
	    pwm[i] = PWM_CENTER + (int)(PWM_HALF_RANGE * norm[i] * config.act_gain[i]);
        } else {
	    // i.e. throttle, flaps
            if ( config.act_gain[i] > 0.0 ) {
	        pwm[i] = PWM_MIN + (int)(PWM_RANGE * norm[i] * config.act_gain[i]);
            } else {
	        pwm[i] = PWM_MAX + (int)(PWM_RANGE * norm[i] * config.act_gain[i]);
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


// reset output pwm rates to safe startup defaults
void pwm_set_rates() {
    Serial.print("PWM rates: ");
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        uint16_t rate = config.pwm_hz[i];
        Serial.print(rate);
        Serial.print(" ");
	uint32_t ch_mask = _BV(i);
        // fixme: APM_RC.SetFastOutputChannels( ch_mask, rate );
    }
    Serial.println();
}


int pwm_init() {
    // fixme: APM_RC.Init(&isr_registry);	 // APM Radio initialization
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // fixme: APM_RC.enable_out(i);
    }
    // set default safe values for actuator outputs
    actuator_set_defaults();
    pwm_update();
    
    // set the PWM output rateas as defined above
    //uint32_t ch_mask = _BV(CH_1) | _BV(CH_2) | _BV(CH_3) | _BV(CH_4) | _BV(CH_5) | _BV(CH_6) | _BV(CH_7) | _BV(CH_8);
    //Serial.print("PWM rate: ");
    //Serial.println(config.pwm_hz);
    //APM_RC.SetFastOutputChannels( ch_mask, config.pwm_hz );
    pwm_set_rates();
    

}


// read the receiver if new data is available.  If manual mode requested, immediate do the mixing and send the result to the servos
int pwm_process() {
  #if 0 // fixme:
    // New radio frame?
    if ( APM_RC.GetState() == 1 ) {
        // read channel data
	      for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            // save raw pulse value
            receiver_pwm[i] = APM_RC.InputCh(i);
	      }
 
        pwm_pwm2norm( receiver_pwm, receiver_norm );
        
        if ( receiver_norm[CH_8] < 0.0 ) {
            // manual pass through requested, let's get it done right now
            sas_update( receiver_norm );
            mixing_update( receiver_norm, true /* ch1-6 */, true /* ch7 */, false /* no ch8 */ );
            pwm_update();
        } else {
            // autopilot mode, but let's update the sas gain tuning channel if requested
            mixing_update( receiver_norm, false /* ch1-6 */, config.sas_ch7gain /* ch7 */, false /* no ch8 */ );
        }
        return 1;
    }
    #endif // fixme
    return 0;
}


// write the raw actuator values to the RC system
int pwm_update() {
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        // fixme: APM_RC.OutputCh(i, actuator_pwm[i] );
    }

    return 0;
}

