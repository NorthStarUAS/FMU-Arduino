#include "../nodes.h"

#include "sbus/sbus.h"
#include "inceptors.h"

bfs::SbusRx sbus_rx(&Serial2);
bfs::SbusData sbus_data;

// define if an sbus input channel is symmetrical or not (i.e. mapped to
// [0,1] for throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
static const bool sbus_symmetrical[sbus_data.NUM_CH] = {1, 1, 0, 1, 1, 1, 1, 0, 0};

void inceptors_t::init() {
    // setup the hardware inputs and outputs
    sbus_rx.Begin();
    switches.init();
}

bool inceptors_t::read() {
    bool new_input = false;

    if ( sbus_rx.Read() ) {
        // sbus_rx is expected to continue outputing data after transmitter signal
        // is lost if failsafe setup correctly on RC transmitter/receiver.

        new_input = true;
        sbus_data = sbus_rx.data();

        if ( sbus_data.failsafe ) {
            // super bad situation if this happens when AP not enabled!

            // fixme: consider forcing at least some simple reversionary mode
            // such as power off, wings level (or a few degrees for gentle
            // turning to avoid flyaway too far) and pitch to zero degrees or
            // something safe/slow or tecs to minimum speed with throttle off?
            inceptors_node.setBool("failsafe", true);
        } else {
            // rcin_node is the raw sbus data for switches and setup/debugging RC transmitter
            for ( uint8_t i = 0; i < sbus_data.NUM_CH; i++ ) {
                rcin_node.setUInt("channel", sbus_data.ch[i], i);
            }

            float norm_val[sbus_data.NUM_CH];    // normalized value (range of -1 to 1, or 0 to 1)
            for ( int i = 0; i < sbus_data.NUM_CH; i++ ) {
                // convert to normalized form
                if ( sbus_symmetrical[i] ) {
                    // i.e. aileron, rudder, elevator
                    norm_val[i] = (float)((int)sbus_data.ch[i] - SBUS_CENTER_VALUE) / SBUS_HALF_RANGE;
                } else {
                    // i.e. throttle, flaps
                    norm_val[i] = (float)((int)sbus_data.ch[i] - SBUS_MIN_VALUE) / SBUS_RANGE;
                }
            }
            // Serial.print(millis()); Serial.print(" ail: "); Serial.print(sbus_data.ch[3]); Serial.print(" "); Serial.println(norm_val[3], 3);

            last_input = millis();
            inceptors_node.setUInt("millis", last_input);
            inceptors_node.setBool("failsafe", false); // (good thing)
            inceptors_node.setDouble("power", norm_val[2]);
            inceptors_node.setDouble("roll", norm_val[3]);
            inceptors_node.setDouble("pitch", norm_val[4]);
            inceptors_node.setDouble("yaw", norm_val[5]);
            inceptors_node.setDouble("flaps", norm_val[6]);
            inceptors_node.setDouble("gear", norm_val[7]);
            inceptors_node.setDouble("aux1", norm_val[8]);
            inceptors_node.setDouble("aux2", norm_val[9]);

            switches.update();
        }
    }

    return new_input;
}