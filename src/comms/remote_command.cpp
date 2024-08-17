#include "../props2.h"
#include "../mission/mission_mgr.h"
#include "../nav/nav_mgr.h"
#include "../sensors/sensor_mgr.h"
#include "../util/strutils.h"
#include "events.h"
#include "remote_command.h"

void execute_command(string command, SerialLink *serial) {
    uint8_t result = 0;
    vector<string> tokens = split(command, " ", 3);
    // fixme: use tokens[0] rather than raw command string
    if ( command == "hb" ) {
        result = 2;
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
    } else if ( tokens[0] == "set" and tokens.size() == 3 ) {
        if ( tokens[1][0] == '/' ) {
            // requires absolute path
            printf("  %s\n", tokens[1].c_str());
            size_t pos = tokens[1].rfind("/");
            if ( pos != string::npos ) {
                string node_path = tokens[1].substr(0, pos);
                string name = tokens[1].substr(pos+1);
                PropertyNode node( node_path );

                // estimate value type (quick, not perfect!)
                bool is_int = true;
                bool is_float = false;
                bool is_string = false;
                for ( unsigned int i = 0; i < tokens[2].length(); i++ ){
                    char c = tokens[2][i];
                    if ( (c == '+' or c == '-') and i == 0 ) {
                        // ok for the first character of int/float to be +/-
                    } else if ( c >= '0' and c <='9' ) {
                        // stick with the default (or current type estimate)
                    } else if (c == '.') {
                        if ( is_int ) {
                            // we see a decimal point, promote to float if we currently think we have an int
                            is_int = false;
                            is_float = true;
                        } else if ( is_float ) {
                            // we already saw a '.' so another '.' is an invalid number, this must be a string!
                            is_int = false;
                            is_float = false;
                            is_string = true;
                        }
                    } else {
                        // non numeric characters, must be a string
                        is_int = false;
                        is_float = false;
                        is_string = true;
                    }
                }
                if ( is_int ) {
                    node.setInt(name.c_str(), atoi(tokens[2].c_str()));
                } else if ( is_float ) {
                    node.setDouble(name.c_str(), atof(tokens[2].c_str()));
                } else if ( is_string ) {
                    if ( tokens[2] == "True" or tokens[2] == "true" ) {
                        node.setBool(name.c_str(), true);
                    } else if ( tokens[2] == "False" or tokens[2] == "false" ) {
                        node.setBool(name.c_str(), false);
                    } else {
                        node.setString(name.c_str(), tokens[2]);
                    }
                }
                result = 1;
            } else {
                // not a valid property name
            }
        }
    } else if ( tokens[0] == "home" and tokens.size() >= 4 ) {
        home_node.setDouble("longitude_deg", atof(tokens[1].c_str()));
        home_node.setDouble("latitude_deg", atof(tokens[2].c_str()));
        home_node.setDouble("azimuth_deg",atof(tokens[3].c_str()));
        result = 1;
    } else if ( tokens[0] == "task" and tokens.size() >= 2 ) {
        if ( tokens[1] == "circle" and tokens.size() == 4 ) {
            double lon_deg = atof(tokens[2].c_str());
            double lat_deg = atof(tokens[3].c_str());
            if ( mission_mgr != nullptr ) {
                mission_mgr->start_circle_task(lon_deg, lat_deg);
            }
            result = 1;
        }
    } else {
        // printf("unknown message: %s, relaying to host\n", command.c_str());
        // if ( relay_id == "gcs" ) {
        //     relay.forward_packet(relay_t::dest_enum::host_dest,
        //                             id, buf, message_size);
        // }
    }

    if ( result == 0 ) {
        event_mgr->add_event("fail", command);
    } else if ( result == 1 ) {
        event_mgr->add_event("uas", command);
    } else {
        // success but don't report (like for hb command)
    }
}