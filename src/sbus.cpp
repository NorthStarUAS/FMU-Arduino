#include "actuators.h"
#include "pwm.h"
#include "sbus.h"

static const uint8_t SBUS_SIGNAL_OK = 0x00;
static const uint8_t SBUS_SIGNAL_LOST = 0x01;
static const uint8_t SBUS_SIGNAL_FAILSAFE = 0x03;

static const uint8_t SBUS_HEADER_VALUE = 0x0F;
static const uint8_t SBUS_FOOTER_VALUE = 0x00;

static const int SBUS_MIN_VALUE = 172;
static const int SBUS_CENTER_VALUE = 992;
static const int SBUS_MAX_VALUE = 1811;
static const int SBUS_RANGE = 1640;
static const int SBUS_HALF_RANGE = 820;
static const int SBUS_QUARTER_RANGE = 410;

void sbus_t::parse() {
    uint16_t sbus_raw[SBUS_CHANNELS];
    
    // we don't need to return from these, these are just notifying us
    // of receiver state
    if ( sbus_data.failsafe_act ) {
        // Serial1.println("SBUS: failsafe activated!");
    }
    if ( sbus_data.frame_lost ) {
        // Serial1.println("SBUS: frame lost");
    }
    sbus_raw[  0 ] = sbus_data.ch1;
    sbus_raw[  1 ] = sbus_data.ch2;
    sbus_raw[  2 ] = ( sbus_data.ch3_hi << 10 ) | sbus_data.ch3_lo;
    sbus_raw[  3 ] = sbus_data.ch4;
    sbus_raw[  4 ] = sbus_data.ch5;
    sbus_raw[  5 ] = ( sbus_data.ch6_hi <<  9 ) | sbus_data.ch6_lo;
    sbus_raw[  6 ] = sbus_data.ch7;
    sbus_raw[  7 ] = sbus_data.ch8;
    sbus_raw[  8 ] = ( sbus_data.ch9_hi <<  8 ) | sbus_data.ch9_lo;
    sbus_raw[  9 ] = sbus_data.ch10;
    sbus_raw[ 10 ] = sbus_data.ch11;
    sbus_raw[ 11 ] = ( sbus_data.ch12_hi << 7 ) | sbus_data.ch12_lo;
    sbus_raw[ 12 ] = sbus_data.ch13;
    sbus_raw[ 13 ] = sbus_data.ch14;
    sbus_raw[ 14 ] = ( sbus_data.ch15_hi << 6 ) | sbus_data.ch15_lo;
    sbus_raw[ 15 ] = sbus_data.ch16;

    uint8_t sbus_flags = 0x00;
    sbus_flags |= sbus_data.ch17;
    sbus_flags |= sbus_data.ch18 << 1;
    sbus_flags |= sbus_data.frame_lost << 2;
    sbus_flags |= sbus_data.failsafe_act << 3;
    
#if 0    
    Serial1.print(" ");
    Serial1.print(sbus_raw[0]);
    Serial1.print(" ");
    Serial1.print(sbus_raw[1]);
    Serial1.print(" ");
    Serial1.print(sbus_raw[2]);
    Serial1.print(" ");
    Serial1.print(sbus_raw[3]);
    for ( int i = 0; i < SBUS_PAYLOAD_LEN; i++ ) {
        Serial1.print(" ");
        Serial1.print(sbus_data.buf[i], DEC);
    }
    Serial1.println();
#endif

#if 0    
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        if ( ch_data[i] < SBUS_MIN_VALUE || ch_data[i] > SBUS_MAX_VALUE ) {
            Serial1.print("Warning detected a problem with sbus packet data, skipping frame, ch = ");
            Serial1.println(i);
            return;
        }
    }
#endif
    
    raw2norm(sbus_raw, receiver_norm);
    receiver_flags = sbus_flags;

    if ( receiver_norm[0] < 0.0 ) {
        // manual flight mode requested, let's get it done right now
        actuators.sas_update( receiver_norm );
        actuators.mixing_update( receiver_norm );
        pwm.update(); // for the outputs
    }
}

// setup the sbus (currently hard coded on Serial2)
void sbus_t::setup() {
    Serial2.begin(100000,SERIAL_8E1_RXINV_TXINV); // newer teensies should use SERIAL_8E2_RXINV_TXINV
    Serial.println("SBUS on Serial2 (SERIAL_8E2)");

    // seed receiver_norm to safe values (pending receipt of first valid sbus packet)
    receiver_norm[0] = -1.0;        // manual mode
    receiver_norm[1] = -1.0;        // throttle safety enabled
}

// read available bytes on the sbus uart and return true if any new
// data is read when a full packet is read, send that to the parser
// which will push the new data into the various data structures and
// trigger and output of new actuator (currently PWM only) values.
bool sbus_t::process() {
    static byte state = 0;
    byte input;
    bool new_data = false;
    
    //Serial1.print("state = ");
    //Serial1.println(state);
    if ( state == 0 ) {
        // scan for start of frame
        while ( Serial2.available() > 0 ) {
            new_data = true;
            input = Serial2.read();
            if ( input == SBUS_HEADER_VALUE ) {
                state = 1;
                break;
            }
        }
    }
    if ( state == 1 ) {
        // fill in sbus frame (when enough bytes are available)
        if ( Serial2.available() >= SBUS_PAYLOAD_LEN ) {
            new_data = true;
            for ( int i = 0; i < SBUS_PAYLOAD_LEN; i++ ) {
                input = Serial2.read();
                //Serial1.print(" ");
                //Serial1.print(input, DEC);
                sbus_data.buf[i] = input;
            }
            //Serial1.println();
            state = 2;
        }   
    }
    if  ( state == 2 ) {
        //Serial1.println("here in state = 2");
        // end of frame
        if ( Serial2.available() > 0 ) {
            new_data = true;
            //Serial1.println("bytes are available");
            input = Serial2.read();
            if ( input == SBUS_FOOTER_VALUE ) {
                parse();
                state = 0; 
            } else {
                //Serial1.println("wrong sbus footer value, skipping ahead to next footer byte");
                input = Serial2.read();
                while ( Serial2.available() > 0 && input != SBUS_FOOTER_VALUE ) {
                    input = Serial2.read();
                }
                if ( input == SBUS_FOOTER_VALUE ) {
                    // set state to zero so we begin parsing the next new frame correctly
                    // (but something went wrong with this frame so don't parse the data/discard.)
                    state = 0;
                }
            }
        }
    }
    
    return new_data;
}

// compute normalized command values from the raw sbus values
void sbus_t::raw2norm( uint16_t *raw, float *norm ) {
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        // convert to normalized form
        if ( sbus_symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
            norm[i] = (float)((int)raw[i] - SBUS_CENTER_VALUE) / SBUS_HALF_RANGE;
        } else {
            // i.e. throttle, flaps
            norm[i] = (float)((int)raw[i] - SBUS_MIN_VALUE) / SBUS_RANGE;
        }
    }
}

// global shared instance
sbus_t sbus;
