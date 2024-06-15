#include <Arduino.h>

#include "../nodes.h"

#include "menu.h"

void menu_t::display() {
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

void menu_t::init() {
    // flush input buffer
    while ( Serial.available() ) {
        Serial.read();
    }
}

void menu_t::update() {
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
        } else {
            reboot_count = 0;
            display();
        }
    }
}
