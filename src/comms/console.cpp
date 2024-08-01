#include <Arduino.h>

#include "../nodes.h"

#include "info.h"
#include "serial_link.h"

#include "console.h"

void console_t::display_menu() {
    printf("%s\n",
            "  1) Pilot input\n"
            "  2) GPS\n"
            "  3) Airdata\n"
            "  4) IMU\n"
            "  5) Nav/EKF\n"
            "  6) EKF biases/covariances\n"
            "  7) Actuator output\n"
            "  8) Calibrate IMU strapdown\n"
            "  9) Pretty print property tree\n"
            "  Reboot: type \"reboot\"\n");
}

void console_t::init() {
    info_timer = RateLimiter(10);
    interactive = true;

    // flush input buffer
    while ( Serial.available() ) {
        Serial.read();
    }
}

void console_t::update() {
    if ( interactive ) {
        if ( info_timer.update() ) {
            if ( display_pilot ) { write_pilot_in_ascii(); }
            if ( display_gps ) { write_gps_ascii(); }
            if ( display_airdata ) { write_airdata_ascii(); }
            if ( display_imu ) { write_imu_ascii(); }
            if ( display_nav ) { write_nav_ascii(); }
            if ( display_nav_stats ) { write_nav_stats_ascii(); }
            if ( display_act ) { write_actuator_out_ascii(); }

            // discard non-user input (but watch for the reboot command)
            int16_t user_input = 0;
            if ( Serial.available() > 1 ) {
                while ( Serial.available() ) {
                    user_input = Serial.read();
                    if ( user_input == reboot_cmd[reboot_count] ) {
                        reboot_count++;
                        if ( reboot_count == strlen(reboot_cmd) ) {
                            printf("rebooting by external request ...\n");
                            delay(250);
                            // _reboot_Teensyduino_();  // reboot and reload firmware
                            SCB_AIRCR = 0x05FA0004;     // simple reboot
                        }
                    } else if ( user_input == reboot_cmd[0] ) {
                        // allow immediate restart of a failed or partial command
                        reboot_count = 1;
                    } else if ( user_input == SerialLink::START_OF_MSG0 ) {
                        // looks like we are receiving a binary packet, switch console modes to binary
                        printf("Switching console to binary mode: %lu\n", millis());
                        interactive = false;
                        console_link.init(0, 500000/*, "console"*/);
                        break;
                    } else {
                        reboot_count = 0;
                    }
                }
            }
            if ( Serial.available() ) {
                user_input = Serial.read();
                printf("read character: %c\n", (char)user_input);
                if ( user_input == '1' ) {
                    display_pilot = !display_pilot;
                } else if ( user_input == '2' ) {
                    display_gps = !display_gps;
                } else if ( user_input == '3' ) {
                    display_airdata = !display_airdata;
                } else if ( user_input == '4' ) {
                    display_imu = !display_imu;
                } else if ( user_input == '5' ) {
                    display_nav = !display_nav;
                } else if ( user_input == '6' ) {
                    display_nav_stats = !display_nav_stats;
                } else if ( user_input == '7' ) {
                    display_act = !display_act;
                } else if ( user_input == '8' ) {
                    imu_node.setString("request", "calibrate-accels");
                    printf("request: %s\n", imu_node.getString("request").c_str());
                } else if ( user_input == '9' ) {
                    PropertyNode("/").pretty_print();
                } else if ( user_input == reboot_cmd[reboot_count] ) {
                    reboot_count++;
                    if ( reboot_count == strlen(reboot_cmd) ) {
                        printf("rebooting by user request ...\n");
                        delay(250);
                        // _reboot_Teensyduino_();  // reboot and reload firmware
                        SCB_AIRCR = 0x05FA0004;     // simple reboot
                    }
                } else if ( user_input == reboot_cmd[0] ) {
                    // allow immediate restart of a failed or partial command
                    reboot_count = 1;
                } else if ( user_input == SerialLink::START_OF_MSG0 ) {
                    // looks like we are receiving a binary packet, switch console modes to binary
                    printf("Switching console to binary mode: %lu\n", millis());
                    interactive = false;
                    console_link.init(0, 500000 /*, "console"*/);
                } else {
                    reboot_count = 0;
                    display_menu();
                }
            }
        }
    } else {
        // binary mode
        console_link.read_commands();
        console_link.update();
    }
}