// Module to handle actuator input/output and mixing.


// Actuator gain (reversing) commands, format is cmd(byte) ch(byte)
// gain(float)
#define ACT_GAIN_DEFAULTS 0
#define ACT_GAIN_SET 1

// SAS mode command, format is cmd(byte), gain(float)
#define SAS_DEFAULTS 0
#define SAS_ROLLAXIS 1
#define SAS_PITCHAXIS 2
#define SAS_YAWAXIS 3
#define SAS_TUNE 10

// Mix mode commands, format is cmd(byte), gain 1(float), gain 2(float)
#define MIX_DEFAULTS 0
#define MIX_AUTOCOORDINATE 1
#define MIX_THROTTLE_TRIM 2
#define MIX_FLAP_TRIM 3
#define MIX_ELEVONS 4
#define MIX_FLAPERONS 5
#define MIX_VTAIL 6
#define MIX_DIFF_THRUST 7

// official flight command values.  These could source from the RC
// receiver or the autopilot depending on the auto/manual selection
// switch state.  These are pre-mix commands and will be mixed and
// written to the actuators for both manual and autonomous flight
// modes.
float aileron_cmd = 0.0;
float elevator_cmd = 0.0;
float throttle_cmd = 0.0;
float rudder_cmd = 0.0;
float gear_cmd = 0.0;
float flap_cmd = 0.0;
float ch7_cmd = 0.0;
float ch8_cmd = 0.0;

// reset pwm output rates to safe startup defaults
void pwm_defaults() {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
         config.actuators.pwm_hz[i] = 50;    
    }
}

// reset actuator gains (reversing) to startup defaults
void act_gain_defaults() {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        config.actuators.act_gain[i] = 1.0;
    }
}

// reset sas parameters to startup defaults
void sas_defaults() {
    config.actuators.sas_rollaxis = false;
    config.actuators.sas_pitchaxis = false;
    config.actuators.sas_yawaxis = false;
    config.actuators.sas_tune = false;

    config.actuators.sas_rollgain = 0.0;
    config.actuators.sas_pitchgain = 0.0;
    config.actuators.sas_yawgain = 0.0;
    config.actuators.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
void mixing_defaults() {
    config.actuators.mix_autocoord = false;
    config.actuators.mix_throttle_trim = false;
    config.actuators.mix_flap_trim = false;
    config.actuators.mix_elevon = false;
    config.actuators.mix_flaperon = false;
    config.actuators.mix_vtail = false;
    config.actuators.mix_diff_thrust = false;

    config.actuators.mix_Gac = 0.5;       // aileron gain for autocoordination
    config.actuators.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config.actuators.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config.actuators.mix_Gea = 1.0;       // aileron gain for elevons
    config.actuators.mix_Gee = 1.0;       // elevator gain for elevons
    config.actuators.mix_Gfa = 1.0;       // aileron gain for flaperons
    config.actuators.mix_Gff = 1.0;       // flaps gain for flaperons
    config.actuators.mix_Gve = 1.0;       // elevator gain for vtail
    config.actuators.mix_Gvr = 1.0;       // rudder gain for vtail
    config.actuators.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config.actuators.mix_Gtr = 0.1;       // rudder gain for diff thrust
};

bool act_gain_command_parse(byte *buf) {
    uint8_t ch = buf[0];
    if ( ch >= PWM_CHANNELS ) {
        return false;
    }

    byte lo = buf[1];
    byte hi = buf[2];
    uint16_t val = hi*256 + lo; 
    float gain = ((float)val - 32767.0) / 10000.0;
    if ( gain < -2.0 || gain > 2.0 ) {
        return false;
    }

    config.actuators.act_gain[ch] = gain;
    
    return true;
}


bool sas_command_parse(byte *buf) {
    bool enable = buf[1];
    uint8_t lo, hi;
    uint16_t val;

    lo = buf[2];
    hi = buf[3];
    val = hi*256 + lo; 
    float gain = ((float)val - 32767.0) / 10000.0;

    if ( buf[0] == SAS_DEFAULTS ) {
        sas_defaults();
    } else if ( buf[0] == SAS_ROLLAXIS ) {
        config.actuators.sas_rollaxis = enable;
        config.actuators.sas_rollgain = gain;
    } else if ( buf[0] == SAS_PITCHAXIS ) {
        config.actuators.sas_pitchaxis = enable;
        config.actuators.sas_pitchgain = gain;
    } else if ( buf[0] == SAS_YAWAXIS ) {
        config.actuators.sas_yawaxis = enable;
        config.actuators.sas_yawgain = gain;
    } else if ( buf[0] == SAS_TUNE ) {
        config.actuators.sas_tune = enable;
    } else {
        return false;
    }
    
    return true;
}


bool mixing_command_parse(byte *buf) {
    bool enable = buf[1];
    uint8_t lo, hi;
    uint16_t val;

    lo = buf[2];
    hi = buf[3];
    val = hi*256 + lo; 
    float g1 = ((float)val - 32767.0) / 10000.0;

    lo = buf[4];
    hi = buf[5];
    val = hi*256 + lo;    
    float g2 = ((float)val - 32767.0) / 10000.0;
    
    if ( buf[0] == MIX_DEFAULTS ) {
        mixing_defaults();
    } else if ( buf[0] == MIX_AUTOCOORDINATE ) {
        config.actuators.mix_autocoord = enable;
        config.actuators.mix_Gac = g1;
    } else if ( buf[0] == MIX_THROTTLE_TRIM ) {
        config.actuators.mix_throttle_trim = enable;
        config.actuators.mix_Get = g1;
    } else if ( buf[0] == MIX_FLAP_TRIM ) {
        config.actuators.mix_flap_trim = enable;
        config.actuators.mix_Gef = g1;
    } else if ( buf[0] == MIX_ELEVONS ) {
        config.actuators.mix_elevon = enable;
        config.actuators.mix_Gea = g1;
        config.actuators.mix_Gee = g2;
    } else if ( buf[0] == MIX_FLAPERONS ) {
        config.actuators.mix_flaperon = enable;
        config.actuators.mix_Gfa = g1;
        config.actuators.mix_Gff = g2;
    } else if ( buf[0] == MIX_VTAIL ) {
        config.actuators.mix_vtail = enable;
        config.actuators.mix_Gve = g1;
        config.actuators.mix_Gvr = g2;
    } else if ( buf[0] == MIX_DIFF_THRUST ) {
        config.actuators.mix_diff_thrust = enable;
        config.actuators.mix_Gtt = g1;
        config.actuators.mix_Gtr = g2;
    } else {
        return false;
    }
    
    return true;
}


// compute the sas compensation in normalized 'command' space so that
// we can do proper output channel mixing later
void sas_update( float control_norm[SBUS_CHANNELS] ) {
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)

    float tune = 1.0;
    if ( config.actuators.sas_tune ) {
        tune = config.actuators.sas_max_gain * receiver_norm[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config.actuators.sas_rollaxis ) {
        control_norm[3] -= tune * config.actuators.sas_rollgain * imu_calib[3];  // p
    }
    if ( config.actuators.sas_pitchaxis ) {
        control_norm[4] += tune * config.actuators.sas_pitchgain * imu_calib[4]; // q
    }
    if ( config.actuators.sas_yawaxis ) {
        control_norm[5] += tune * config.actuators.sas_yawgain * imu_calib[5];   // r
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void mixing_update( float control_norm[SBUS_CHANNELS] ) {
    bool throttle_enabled = control_norm[1] > 0.0;
    aileron_cmd = control_norm[3];
    elevator_cmd = control_norm[4];
    throttle_cmd = control_norm[2];
    rudder_cmd = control_norm[5];
    flap_cmd = control_norm[6];
    gear_cmd = control_norm[7];
        
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)
    if ( config.actuators.mix_autocoord ) {
        rudder_cmd += config.actuators.mix_Gac * aileron_cmd;
    }
    if ( config.actuators.mix_throttle_trim ) {
        elevator_cmd += config.actuators.mix_Get * throttle_cmd;
    }
    if ( config.actuators.mix_flap_trim ) {
        elevator_cmd += config.actuators.mix_Gef * flap_cmd;
    }

    if ( throttle_enabled ) {
        actuator_norm[0] = throttle_cmd;
    } else {
        actuator_norm[0] = 0.0;
    }
    actuator_norm[1] = aileron_cmd;
    actuator_norm[2] = elevator_cmd;
    actuator_norm[3] = rudder_cmd;
    actuator_norm[4] = flap_cmd;
    actuator_norm[5] = gear_cmd;
 
    // elevon and flaperon mixing are mutually exclusive
    if ( config.actuators.mix_elevon ) {
        actuator_norm[1] = config.actuators.mix_Gea * aileron_cmd + config.actuators.mix_Gee * elevator_cmd;
        actuator_norm[2] = config.actuators.mix_Gea * aileron_cmd - config.actuators.mix_Gee * elevator_cmd;
    } else if ( config.actuators.mix_flaperon ) {
        actuator_norm[1] = config.actuators.mix_Gfa * aileron_cmd + config.actuators.mix_Gff * flap_cmd;
        actuator_norm[4] = -config.actuators.mix_Gfa * aileron_cmd + config.actuators.mix_Gff * flap_cmd;
    }
    // vtail mixing can't work with elevon mixing
    if ( config.actuators.mix_vtail && !config.actuators.mix_elevon) {
        actuator_norm[2] = config.actuators.mix_Gve * elevator_cmd + config.actuators.mix_Gvr * rudder_cmd;
        actuator_norm[3] = config.actuators.mix_Gve * elevator_cmd - config.actuators.mix_Gvr * rudder_cmd;
    }
    if ( config.actuators.mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // actuator_norm[0] = config.actuators.mix_Gtt * throttle_cmd + config.actuators.mix_Gtr * rudder_cmd;
        // actuator_norm[5] = config.actuators.mix_Gtt * throttle_cmd - config.actuators.mix_Gtr * rudder_cmd;
    }

    // compute pwm actuator output values from the normalized values
    pwm_norm2pwm( actuator_norm, actuator_pwm );
}

// set default raw actuator values
void actuator_set_defaults() {
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        actuator_norm[i] = 0.0;
    }
    pwm_norm2pwm(actuator_norm, actuator_pwm);
}
