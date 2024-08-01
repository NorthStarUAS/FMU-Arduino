#include "../props2.h"
#include "../nav/nav_mgr.h"
#include "../sensors/sensor_mgr.h"
#include "remote_command.h"

void execute_command(string command, SerialLink *serial) {
    uint8_t result = 0;
    if ( command == "hb" ) {
        result = 1;
    } else if ( command == "zero_gyros" ) {
        sensor_mgr->imu_mgr.gyros_calibrated = 0;   // start state
        result = 1;
    } else if ( command == "reset_ekf" ) {
        nav_mgr->reinit();
        result = 1;
    } else if ( command.substr(0, 4) == "get " ) {
        string path = command.substr(4);
        // printf("command: get  node: %s\n", path.c_str());
        PropertyNode node(path);
        ns_message::command_v1_t reply;
        reply.sequence_num = 0;
        reply.message = "set " + path + " " + node.get_json_string();
        reply.pack();
        serial->write_packet( reply.id, reply.payload, reply.len);
        result = 1;
    } else {
        // printf("unknown message: %s, relaying to host\n", command.c_str());
        // if ( relay_id == "gcs" ) {
        //     relay.forward_packet(relay_t::dest_enum::host_dest,
        //                             id, buf, message_size);
        // }
        result = 1;
    }
}