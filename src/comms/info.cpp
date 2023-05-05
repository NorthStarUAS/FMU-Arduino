/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "../nav/nav_constants.h"

#include "info.h"

void info_t::init() {
    config_node = PropertyNode("/config");
    effector_node = PropertyNode("/effectors");
    nav_node = PropertyNode("/filters/nav");
    airdata_node = PropertyNode("/sensors/airdata");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    pilot_node = PropertyNode("/pilot");
    power_node = PropertyNode("/sensors/power");
    switches_node = PropertyNode("/switches");
}

void info_t::write_pilot_in_ascii()
{
    // pilot (receiver) input data
    if ( pilot_node.getBool("failsafe") ) {
        console->printf("FAILSAFE! ");
    }
    if ( switches_node.getBool("master_switch") ) {
        console->printf("(Auto) ");
    } else {
        console->printf("(Manual) ");
    }
    if ( switches_node.getBool("throttle_safety") ) {
        console->printf("(Throttle enable) ");
    } else {
        console->printf("(Throttle safe) ");
    }
    for ( int i = 0; i < 8; i++ ) {
        console->printf("%.3f ", pilot_node.getDouble("channel", i));
    }
    console->printf("\n");
}

void info_t::write_actuator_out_ascii()
{
    // actuator output
    console->printf("RCOUT:");
    for ( int i = 0; i < MAX_RCOUT_CHANNELS; i++ ) {
        console->printf("%.2f ", effector_node.getDouble("channel", i));
    }
    console->printf("\n");
}

void info_t::write_imu_ascii()
{
    // output imu data
    console->printf("IMU: ");
    console->printf("%.3f ", imu_node.getDouble("timestamp"));
    console->printf("%.2f ", imu_node.getDouble("p_rps"));
    console->printf("%.2f ", imu_node.getDouble("q_rps"));
    console->printf("%.2f ", imu_node.getDouble("r_rps"));
    console->printf("%.2f ", imu_node.getDouble("ax_mps2"));
    console->printf("%.2f ", imu_node.getDouble("ay_mps2"));
    console->printf("%.2f ", imu_node.getDouble("az_mps2"));
    console->printf("%.2f ", imu_node.getDouble("temp_C"));
    console->printf("\n");
}

void info_t::write_gps_ascii() {
    console->printf("GPS:");
    console->printf(" Lat: %.7f", gps_node.getDouble("latitude_deg"));
    console->printf(" Lon: %.7f", gps_node.getDouble("longitude_deg"));
    console->printf(" Alt: %.1f", gps_node.getDouble("altitude_m"));
    console->printf(" Vel: %.1f %.1f %.1f",
                    gps_node.getDouble("vn_mps"),
                    gps_node.getDouble("ve_mps"),
                    gps_node.getDouble("vd_mps"));
    console->printf(" Sat: %d", gps_node.getInt("num_sats"));
    console->printf(" Fix: %d", gps_node.getInt("status"));
    console->printf(" Time: %02d:%02d:%02d ",
                    gps_node.getInt("hour"),
                    gps_node.getInt("min"),
                    gps_node.getInt("sec"));
    console->printf(" Date: %02d/%02d/%04d",
                    gps_node.getInt("month"),
                    gps_node.getInt("day"),
                    gps_node.getInt("year"));
    console->printf("\n");
}

void info_t::write_nav_ascii() {
    // values
    console->printf("Pos: %.7f, %.7f, %.2f",
                    nav_node.getDouble("latitude_deg"),
                    nav_node.getDouble("longitude_deg"),
                    nav_node.getDouble("altitude_m"));
    console->printf(" Vel: %.2f, %.2f, %.2f",
                    nav_node.getDouble("vn_mps"),
                    nav_node.getDouble("ve_mps"),
                    nav_node.getDouble("vd_mps"));
    console->printf(" Att: %.2f, %.2f, %.2f\n",
                    nav_node.getDouble("phi_rad")*R2D,
                    nav_node.getDouble("the_rad")*R2D,
                    nav_node.getDouble("psi_rad")*R2D);
}

void info_t::write_nav_stats_ascii() {
    // covariances
    console->printf("gxb: %.2f %.2f %.2f",
                    nav_node.getDouble("p_bias"),
                    nav_node.getDouble("q_bias"),
                    nav_node.getDouble("r_bias"));
    console->printf(" axb: %.2f %.2f %.2f",
                    nav_node.getDouble("ax_bias"),
                    nav_node.getDouble("ay_bias"),
                    nav_node.getDouble("az_bias"));
    float num = 3.0;            // how many standard deviations
    console->printf(" cov pos: %.2f %.2f %.2f",
                    num * nav_node.getDouble("Pp0"),
                    num * nav_node.getDouble("Pp1"),
                    num * nav_node.getDouble("Pp2"));
    console->printf(" vel: %.2f %.2f %.2f",
                    num * nav_node.getDouble("Pv0"),
                    num * nav_node.getDouble("Pv1"),
                    num * nav_node.getDouble("Pv2"));
    console->printf(" att: %.2f %.2f %.2f\n",
                    num * nav_node.getDouble("Pa0")*R2D,
                    num * nav_node.getDouble("Pa1")*R2D,
                    num * nav_node.getDouble("Pa2")*R2D);
    if ( false ) {
        nav_node.pretty_print();
    }
}

void info_t::write_airdata_ascii()
{
    console->printf("Baro: %.2fpa %.1fC ",
                    airdata_node.getDouble("baro_press_pa"),
                    airdata_node.getDouble("baro_temp_C"));
    console->printf("Pitot: %.4f mps (%.1f pa) %.1f C %d errors\n",
                    airdata_node.getDouble("airspeed_mps"),
		    airdata_node.getDouble("diffPress_pa"),
                    airdata_node.getDouble("air_temp_C"),
                    airdata_node.getUInt("error_count"));
}

void info_t::write_power_ascii()
{
    printf("Avionics v: %.2f  Batt v: %.2f  Batt amp: %.2f\n",
           power_node.getDouble("avionics_vcc"),
           power_node.getDouble("main_vcc"),
           power_node.getDouble("main_amps"));
}

void info_t::write_status_info_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    printf("Uptime: %d(sec)", (unsigned int)(AP_HAL::millis() / 1000));
    printf(" SN: %d", config_node.getInt("serial_number"));
    printf(" Firmware: %d", FIRMWARE_REV);
    printf(" Main loop hz: %d", MASTER_HZ);
    printf(" Baud: %d\n", HOST_BAUD);
}
