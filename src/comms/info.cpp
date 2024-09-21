// fixme: bad comments

/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "../../setup_board.h"
#include "../nodes.h"

#include "../util/constants.h"
#include "../sensors/pwm.h"

#include "info.h"

void write_inceptors_ascii() {
    // inceptor (sbus receiver) input data
    if ( inceptors_node.getBool("failsafe") ) {
        Serial.print("FAILSAFE! ");
    }
    if ( inceptors_node.getBool("master_switch") ) {
        Serial.print("(Auto) ");
    } else {
        Serial.print("(Manual) ");
    }
    if ( inceptors_node.getBool("throttle_safety") ) {
        Serial.print("(Throttle enable) ");
    } else {
        Serial.print("(Throttle safe) ");
    }
    Serial.print(inceptors_node.getDouble("power"), 2); Serial.print(" ");
    Serial.print(inceptors_node.getDouble("roll"), 2); Serial.print(" ");
    Serial.print(inceptors_node.getDouble("pitch"), 2); Serial.print(" ");
    Serial.print(inceptors_node.getDouble("yaw"), 2); Serial.println();
}

void write_effectors_ascii() {
    // final effector (servo) positions
    Serial.print("RCOUT: ");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        Serial.print(effectors_node.getDouble("channel", i), 2); Serial.print(" ");
    }
    Serial.println();
}

static void write_padded_double(double val, int prec) {
    if ( val >= 0  ) {
        Serial.print(" ");
    }
    Serial.print(val, prec);
}

static void write_zero_padded_int(int val, int width) {
    int base = 1;
    for ( int i = 1; i < width; i++ ) {
        base *= 10;
    }
    while ( base >= 10 ) {
        if ( val < base ) {
            Serial.print("0");
        }
        base /= 10;
    }
    Serial.print(val);
}

void write_imu_ascii() {
    // output imu data
    Serial.print("IMU: ");
    Serial.print(imu_node.getDouble("timestamp"), 2); Serial.print(" ");
    write_padded_double(imu_node.getDouble("p_rps"), 2); Serial.print(" ");
    write_padded_double(imu_node.getDouble("q_rps"), 2); Serial.print(" ");
    write_padded_double(imu_node.getDouble("r_rps"), 2); Serial.print(" ");
    write_padded_double(imu_node.getDouble("ax_mps2"), 2); Serial.print(" ");
    write_padded_double(imu_node.getDouble("ay_mps2"), 2); Serial.print(" ");
    write_padded_double(imu_node.getDouble("az_mps2"), 2); Serial.print(" ");
    Serial.print(imu_node.getDouble("temp_C"), 1); Serial.println();
}

void write_gps_ascii() {
    Serial.print("GPS:");
    Serial.print(" Lat: "); Serial.print(gps_node.getDouble("latitude_deg"), 7);
    Serial.print(" Lon: "); Serial.print(gps_node.getDouble("longitude_deg"), 7);
    Serial.print(" Alt: "); Serial.print(gps_node.getDouble("altitude_m"), 1);
    Serial.print(" Vel: ");
    write_padded_double(gps_node.getDouble("vn_mps"), 1); Serial.print(" ");
    write_padded_double(gps_node.getDouble("ve_mps"), 1); Serial.print(" ");
    write_padded_double(gps_node.getDouble("vd_mps"), 1);
    Serial.print(" Sat: "); Serial.print(gps_node.getInt("num_sats"));
    Serial.print(" Fix: "); Serial.print(gps_node.getInt("status"));
    Serial.print(" Time: ");
    write_zero_padded_int(gps_node.getInt("hour"), 2); Serial.print(":");
    write_zero_padded_int(gps_node.getInt("min"), 2); Serial.print(":");
    write_zero_padded_int(gps_node.getInt("sec"), 2);
    Serial.print(" Date: ");
    write_zero_padded_int(gps_node.getInt("month"), 2); Serial.print("/");
    write_zero_padded_int(gps_node.getInt("day"), 2); Serial.print("/");
    write_zero_padded_int(gps_node.getInt("year"), 4);
    Serial.println();
}

void write_nav_ascii() {
    // values
    Serial.print("Pos: ");
    Serial.print(nav_node.getDouble("latitude_deg"), 7); Serial.print(" ");
    Serial.print(nav_node.getDouble("longitude_deg"), 7); Serial.print(" ");
    Serial.print(nav_node.getDouble("altitude_m"), 1);
    Serial.print(" Vel: ");
    write_padded_double(nav_node.getDouble("vn_mps"), 2); Serial.print(" ");
    write_padded_double(nav_node.getDouble("ve_mps"), 2); Serial.print(" ");
    write_padded_double(nav_node.getDouble("vd_mps"), 2);
    Serial.print(" Att: ");
    write_padded_double(nav_node.getDouble("phi_rad")*r2d, 2); Serial.print(" ");
    write_padded_double(nav_node.getDouble("the_rad")*r2d, 2); Serial.print(" ");
    write_padded_double(nav_node.getDouble("psi_rad")*r2d, 2); Serial.println();
}

void write_nav_stats_ascii() {
    // covariances
    Serial.print("gbx: ");
    Serial.print(nav_node.getDouble("p_bias"), 2); Serial.print(" ");
    Serial.print(nav_node.getDouble("q_bias"), 2); Serial.print(" ");
    Serial.print(nav_node.getDouble("r_bias"), 2);
    Serial.print(" abx: ");
    Serial.print(nav_node.getDouble("ax_bias"), 2); Serial.print(" ");
    Serial.print(nav_node.getDouble("ay_bias"), 2); Serial.print(" ");
    Serial.print(nav_node.getDouble("az_bias"), 2);
    float num = 3.0;            // how many standard deviations
    Serial.print(" cov pos: ");
    Serial.print(num * nav_node.getDouble("Pp0"), 2); Serial.print(" ");
    Serial.print(num * nav_node.getDouble("Pp1"), 2); Serial.print(" ");
    Serial.print(num * nav_node.getDouble("Pp2"), 2);
    Serial.print(" vel: ");
    Serial.print(num * nav_node.getDouble("Pv0"), 2); Serial.print(" ");
    Serial.print(num * nav_node.getDouble("Pv1"), 2); Serial.print(" ");
    Serial.print(num * nav_node.getDouble("Pv2"), 2);
    Serial.print(" att: ");
    Serial.print(num * nav_node.getDouble("Pa0")*r2d, 2); Serial.print(" ");
    Serial.print(num * nav_node.getDouble("Pa1")*r2d, 2); Serial.print(" ");
    Serial.print(num * nav_node.getDouble("Pa2")*r2d, 2); Serial.println();
    if ( false ) {
        nav_node.pretty_print();
    }
}

void write_airdata_ascii() {
    Serial.print("Baro: "); Serial.print(airdata_node.getDouble("baro_press_pa"), 0); Serial.print(" pa ");
    Serial.print(airdata_node.getDouble("baro_temp_C"), 1); Serial.print(" C ");
    Serial.print(airdata_node.getDouble("altitude_m"), 1); Serial.print(" m ");
    Serial.print("Pitot: ");
    Serial.print(airdata_node.getDouble("airspeed_mps"), 1); Serial.print(" mps (");
	Serial.print(airdata_node.getDouble("diff_press_pa"), 1); Serial.print(" pa) ");
    Serial.print(airdata_node.getDouble("air_temp_C"), 1); Serial.print(" C ");
    Serial.print(airdata_node.getUInt("error_count")); Serial.println(" errors");
}

void write_power_ascii() {
    Serial.print("Avionics v: ");
    Serial.print(power_node.getDouble("avionics_vcc"));
    Serial.print(" Batt v: ");
    Serial.print(power_node.getDouble("main_vcc"));
    Serial.print(" Batt amp: ");
    Serial.println(power_node.getDouble("main_amps"));
}

void write_status_info_ascii() {
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    printf("Uptime: %d(sec)", (unsigned int)(millis() / 1000));
    printf(" SN: %d", config_node.getInt("serial_number"));
    printf(" Firmware: %d", FIRMWARE_REV);
    printf(" Main loop hz: %d\n", MASTER_HZ);
    // printf(" Baud: %d\n", HOST_BAUD);
}
