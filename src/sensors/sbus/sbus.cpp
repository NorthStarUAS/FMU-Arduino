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
    // we don't need to return from these, these are just notifying us
    // of receiver state
    if ( sbus_data.failsafe_act ) {
        // printf("SBUS: failsafe activated!\n");
    }
    if ( sbus_data.frame_lost ) {
        // printf("SBUS: frame lost.\n");
    }
    raw_val[  0 ] = sbus_data.ch1;
    raw_val[  1 ] = sbus_data.ch2;
    raw_val[  2 ] = ( sbus_data.ch3_hi << 10 ) | sbus_data.ch3_lo;
    raw_val[  3 ] = sbus_data.ch4;
    raw_val[  4 ] = sbus_data.ch5;
    raw_val[  5 ] = ( sbus_data.ch6_hi <<  9 ) | sbus_data.ch6_lo;
    raw_val[  6 ] = sbus_data.ch7;
    raw_val[  7 ] = sbus_data.ch8;
    raw_val[  8 ] = ( sbus_data.ch9_hi <<  8 ) | sbus_data.ch9_lo;
    raw_val[  9 ] = sbus_data.ch10;
    raw_val[ 10 ] = sbus_data.ch11;
    raw_val[ 11 ] = ( sbus_data.ch12_hi << 7 ) | sbus_data.ch12_lo;
    raw_val[ 12 ] = sbus_data.ch13;
    raw_val[ 13 ] = sbus_data.ch14;
    raw_val[ 14 ] = ( sbus_data.ch15_hi << 6 ) | sbus_data.ch15_lo;
    raw_val[ 15 ] = sbus_data.ch16;

    uint8_t sbus_flags = 0x00;
    sbus_flags |= sbus_data.ch17;
    sbus_flags |= sbus_data.ch18 << 1;
    sbus_flags |= sbus_data.frame_lost << 2;
    sbus_flags |= sbus_data.failsafe_act << 3;

    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        // convert to normalized form
        if ( sbus_symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
            norm_val[i] = (float)((int)raw_val[i] - SBUS_CENTER_VALUE) / SBUS_HALF_RANGE;
            pwm_val[i] = (norm_val[i] + 1) * 500 + 1000;
        } else {
            // i.e. throttle, flaps
            norm_val[i] = (float)((int)raw_val[i] - SBUS_MIN_VALUE) / SBUS_RANGE;
            pwm_val[i] = norm_val[i] * 1000 + 1000;
        }
    }

#if 0
    Serial1.print(" ");
    Serial1.print(raw_val[0]);
    Serial1.print(" ");
    Serial1.print(raw_val[1]);
    Serial1.print(" ");
    Serial1.print(raw_val[2]);
    Serial1.print(" ");
    Serial1.print(raw_val[3]);
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

    receiver_flags = sbus_flags;
}

// setup the sbus (currently hard coded on Serial2)
void sbus_t::init() {
    Serial2.begin(100000,SERIAL_8E1_RXINV_TXINV); // newer teensies should use SERIAL_8E2_RXINV_TXINV
    Serial.println("SBUS on Serial2 (SERIAL_8E2)");
    // initialize to zero position
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        if ( sbus_symmetrical[i] ) {
            // i.e. aileron, rudder, elevator
            norm_val[i] = 0.0;
            pwm_val[i] = (norm_val[i] + 1) * 500 + 1000;
        } else {
            // i.e. throttle, flaps
            norm_val[i] = 0.0;
            pwm_val[i] = norm_val[i] * 1000 + 1000;
        }
    }
    // special case channels 0 (master autopilot enable) and 1 (throttle safety) to full off
    norm_val[0] = -1; pwm_val[0] = 1000;
    norm_val[1] = -1; pwm_val[1] = 1000;
}

// read available bytes on the sbus uart and return true if any new
// data is read when a full packet is read, send that to the parser
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
