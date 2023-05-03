/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include <Arduino.h>

#include "airdata.h"
#include "config.h"
#include "ekf.h"
#include "gps.h"
#include "imu_mgr.h"
#include "led.h"
#include "mixer.h"
#include "pilot.h"
#include "power.h"
#include "pwm.h"
#include "nav_common/constants.h"
#include "sensors/sbus/sbus.h"
#include "util/serial_link.h"
#include "aura4_messages.h"
#include "../setup_board.h"

#include "comms.h"

void comms_t::setup() {
    serial.open(HOST_BAUD, &Serial1);
}

bool comms_t::parse_message_bin( byte id, byte *buf, byte message_size )
{
    bool result = false;

    // Serial.print("message id = "); Serial.print(id); Serial.print(" len = "); Serial.println(message_size);

    if ( id == message::command_inceptors_id ) {
        static message::command_inceptors_t inceptors;
        inceptors.unpack(buf, message_size);
        if ( message_size == inceptors.len ) {
            pilot.update_ap(&inceptors);
            result = true;
        }
    } else if ( id == message::config_airdata_id ) {
        config.airdata.unpack(buf, message_size);
        if ( message_size == config.airdata.len ) {
            Serial.println("received new airdata config");
            Serial.print("Swift barometer on I2C: 0x");
            Serial.println(config.airdata.swift_baro_addr, HEX);
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_board_id ) {
        config.board.unpack(buf, message_size);
        if ( message_size == config.board.len ) {
            Serial.println("received board config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_ekf_id ) {
        config.ekf.unpack(buf, message_size);
        if ( message_size == config.ekf.len ) {
            Serial.println("received ekf config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_imu_id ) {
        config.imu.unpack(buf, message_size);
        if ( message_size == config.imu.len ) {
            Serial.println("received imu config");
            imu.set_strapdown_calibration(); // update accel_affine matrix
            imu.set_mag_calibration(); // update mag_affine matrix
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_mixer_id ) {
        message::config_mixer_t config_mixer;
        config_mixer.unpack(buf, message_size);
        if ( message_size == config_mixer.len ) {
            Serial.println("received new logic level mixer config");
            mixer.update_matrix(&config_mixer);
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_mixer_matrix_id ) {
        config.mixer_matrix.unpack(buf, message_size);
        if ( message_size == config.mixer_matrix.len ) {
            Serial.println("received new mixer matrix config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_power_id ) {
        config.power.unpack(buf, message_size);
        if ( message_size == config.power.len ) {
            Serial.println("received new power config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_pwm_id ) {
        config.pwm.unpack(buf, message_size);
        if ( message_size == config.pwm.len ) {
            Serial.println("received new pwm config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_stability_damping_id ) {
        config.stab.unpack(buf, message_size);
        if ( message_size == config.stab.len ) {
            Serial.println("received new stability damping config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::command_zero_gyros_id && message_size == 1 ) {
        Serial.println("received zero gyros command");
        imu.gyros_calibrated = 0;   // start state
        write_ack_bin( id, 0 );
        result = true;
    } else if ( id == message::command_reset_ekf_id && message_size == 1 ) {
        Serial.println("received reset ekf command");
        ekf.reinit();
        write_ack_bin( id, 0 );
        result = true;
    } else {
        Serial.print("unknown message id = "); Serial.print(id); Serial.print(" len = "); Serial.println(message_size);
    }
    return result;
}


// output an acknowledgement of a message received
int comms_t::write_ack_bin( uint8_t command_id, uint8_t subcommand_id )
{
    static message::command_ack_t ack;
    ack.command_id = command_id;
    ack.subcommand_id = subcommand_id;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}


// output a binary representation of the pilot manual (rc receiver) data
int comms_t::write_pilot_in_bin()
{
    static message::pilot_t pilot1;

    if (message::sbus_channels > SBUS_CHANNELS) {
        return 0;
    }

    // receiver data
    for ( int i = 0; i < message::sbus_channels; i++ ) {
        pilot1.channel[i] = pilot.manual_inputs[i];
    }

    // flags
    pilot1.flags = sbus.receiver_flags;

    pilot1.pack();
    return serial.write_packet( pilot1.id, pilot1.payload, pilot1.len);
}

void write_pilot_in_ascii()
{
    // pilot (receiver) input data
    if ( sbus.receiver_flags & SBUS_FAILSAFE ) {
        Serial.print("FAILSAFE! ");
    }
    if ( pilot.ap_enabled() ) {
        Serial.print("(Auto) ");
    } else {
        Serial.print("(Manual) ");
    }
    if ( pilot.throttle_safety() ) {
        Serial.print("(Throttle safety) ");
    } else {
        Serial.print("(Throttle enable) ");
    }
    for ( int i = 0; i < 8; i++ ) {
        Serial.print(pilot.manual_inputs[i], 3);
        Serial.print(" ");
    }
    Serial.println();
}

void write_actuator_out_ascii()
{
    // actuator output
    Serial.print("RCOUT:");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        Serial.print(pwm.output_pwm[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// output a binary representation of the IMU data (note: scaled to 16bit values)
int comms_t::write_imu_bin()
{
    const float _pi = 3.14159265358979323846;
    const float _g = 9.807;
    const float _d2r = _pi / 180.0;

    const float _gyro_lsb_per_dps = 32767.5 / 500;  // -500 to +500 spread across 65535
    const float gyroScale = _d2r / _gyro_lsb_per_dps;

    const float _accel_lsb_per_dps = 32767.5 / 8;   // -4g to +4g spread across 65535
    const float accelScale = _g / _accel_lsb_per_dps;

    const float magScale = 0.01;
    const float tempScale = 0.01;

    static message::imu_t imu1;
    imu1.millis = imu.imu_millis;
    imu1.raw[0] = imu.get_ax_raw() / accelScale;
    imu1.raw[1] = imu.get_ay_raw() / accelScale;
    imu1.raw[2] = imu.get_az_raw() / accelScale;
    imu1.raw[3] = imu.get_hx_raw() / magScale;
    imu1.raw[4] = imu.get_hy_raw() / magScale;
    imu1.raw[5] = imu.get_hz_raw() / magScale;
    imu1.cal[0] = imu.get_ax_cal() / accelScale;
    imu1.cal[1] = imu.get_ay_cal() / accelScale;
    imu1.cal[2] = imu.get_az_cal() / accelScale;
    imu1.cal[3] = imu.get_p_cal() / gyroScale;
    imu1.cal[4] = imu.get_q_cal() / gyroScale;
    imu1.cal[5] = imu.get_r_cal() / gyroScale;
    imu1.cal[6] = imu.get_hx_cal() / magScale;
    imu1.cal[7] = imu.get_hy_cal() / magScale;
    imu1.cal[8] = imu.get_hz_cal() / magScale;
    imu1.cal[9] = imu.get_tempC() / tempScale;
    imu1.pack();
    return serial.write_packet( imu1.id, imu1.payload, imu1.len );
}

void write_imu_ascii()
{
    // output imu data
    Serial.print("IMU: ");
    Serial.print(imu.imu_millis); Serial.print(" ");
    Serial.print(imu.get_p_cal(), 2); Serial.print(" ");
    Serial.print(imu.get_q_cal(), 2); Serial.print(" ");
    Serial.print(imu.get_r_cal(), 2); Serial.print(" ");
    Serial.print(imu.get_ax_cal(), 3); Serial.print(" ");
    Serial.print(imu.get_ay_cal(), 3); Serial.print(" ");
    Serial.print(imu.get_az_cal(), 3); Serial.print(" ");
    Serial.print(imu.get_tempC(), 2);
    Serial.println();
}

// output a binary representation of the GPS data
int comms_t::write_gps_bin()
{
    if ( gps.gps_millis > gps_last_millis ) {
        gps_last_millis = gps.gps_millis;
        return serial.write_packet( message::aura_nav_pvt_id,
                                    (uint8_t *)(&(gps.gps_data)),
                                    sizeof(gps.gps_data) );
    } else {
        return 0;
    }
}

void comms_t::write_gps_ascii() {
    Serial.print("GPS:");
    Serial.print(" Lat:");
    Serial.print((double)gps.gps_data.lat / 10000000.0, 7);
    //Serial.print(gps.gps_data.lat);
    Serial.print(" Lon:");
    Serial.print((double)gps.gps_data.lon / 10000000.0, 7);
    //Serial.print(gps.gps_data.lon);
    Serial.print(" Alt:");
    Serial.print((float)gps.gps_data.hMSL / 1000.0);
    Serial.print(" Vel:");
    Serial.print(gps.gps_data.velN / 1000.0);
    Serial.print(", ");
    Serial.print(gps.gps_data.velE / 1000.0);
    Serial.print(", ");
    Serial.print(gps.gps_data.velD / 1000.0);
    Serial.print(" GSP:");
    Serial.print(gps.gps_data.gSpeed, DEC);
    Serial.print(" COG:");
    Serial.print(gps.gps_data.heading, DEC);
    Serial.print(" SAT:");
    Serial.print(gps.gps_data.numSV, DEC);
    Serial.print(" FIX:");
    Serial.print(gps.gps_data.fixType, DEC);
    Serial.print(" TIM:");
    Serial.print(gps.gps_data.hour); Serial.print(':');
    Serial.print(gps.gps_data.min); Serial.print(':');
    Serial.print(gps.gps_data.sec);
    Serial.print(" DATE:");
    Serial.print(gps.gps_data.month); Serial.print('/');
    Serial.print(gps.gps_data.day); Serial.print('/');
    Serial.print(gps.gps_data.year);
    Serial.println();
}

// output a binary representation of the Nav data
int comms_t::write_nav_bin()
{
    static message::ekf_t nav;
    nav.millis = imu.imu_millis;
    nav.lat_rad = ekf.nav.lat;
    nav.lon_rad = ekf.nav.lon;
    nav.altitude_m = ekf.nav.alt;
    nav.vn_ms = ekf.nav.vn;
    nav.ve_ms = ekf.nav.ve;
    nav.vd_ms = ekf.nav.vd;
    nav.phi_rad = ekf.nav.phi;
    nav.the_rad = ekf.nav.the;
    nav.psi_rad = ekf.nav.psi;
    nav.p_bias = ekf.nav.gbx;
    nav.q_bias = ekf.nav.gby;
    nav.r_bias = ekf.nav.gbz;
    nav.ax_bias = ekf.nav.abx;
    nav.ay_bias = ekf.nav.aby;
    nav.az_bias = ekf.nav.abz;
    float max_pos_cov = ekf.nav.Pp0;
    if ( ekf.nav.Pp1 > max_pos_cov ) { max_pos_cov = ekf.nav.Pp1; }
    if ( ekf.nav.Pp2 > max_pos_cov ) { max_pos_cov = ekf.nav.Pp2; }
    if ( max_pos_cov > 655.0 ) { max_pos_cov = 655.0; }
    nav.max_pos_cov = max_pos_cov;
    float max_vel_cov = ekf.nav.Pv0;
    if ( ekf.nav.Pv1 > max_vel_cov ) { max_vel_cov = ekf.nav.Pv1; }
    if ( ekf.nav.Pv2 > max_vel_cov ) { max_vel_cov = ekf.nav.Pv2; }
    if ( max_vel_cov > 65.5 ) { max_vel_cov = 65.5; }
    nav.max_vel_cov = max_vel_cov;
    float max_att_cov = ekf.nav.Pa0;
    if ( ekf.nav.Pa1 > max_att_cov ) { max_att_cov = ekf.nav.Pa1; }
    if ( ekf.nav.Pa2 > max_att_cov ) { max_att_cov = ekf.nav.Pa2; }
    if ( max_att_cov > 6.55 ) { max_vel_cov = 6.55; }
    nav.max_att_cov = max_att_cov;
    nav.status = ekf.status;
    nav.pack();
    return serial.write_packet( nav.id, nav.payload, nav.len );
}

void comms_t::write_nav_ascii() {
    if ( false ) {
        // values
        Serial.print("Pos: ");
        Serial.print(ekf.nav.lat*R2D, 7);
        Serial.print(", ");
        Serial.print(ekf.nav.lon*R2D, 7);
        Serial.print(", ");
        Serial.print(ekf.nav.alt, 2);
        Serial.print(" Vel: ");
        Serial.print(ekf.nav.vn, 2);
        Serial.print(", ");
        Serial.print(ekf.nav.ve, 2);
        Serial.print(", ");
        Serial.print(ekf.nav.vd, 2);
        Serial.print(" Att: ");
        Serial.print(ekf.nav.phi*R2D, 2);
        Serial.print(", ");
        Serial.print(ekf.nav.the*R2D, 2);
        Serial.print(", ");
        Serial.print(ekf.nav.psi*R2D, 2);
        Serial.println();
    } else {
        // covariances
        float num = 3.0;        // how many standard deviations
        Serial.print("cov pos: ");
        Serial.print(num * ekf.nav.Pp0, 2);
        Serial.print(", ");
        Serial.print(num * ekf.nav.Pp1, 2);
        Serial.print(", ");
        Serial.print(num * ekf.nav.Pp2, 2);
        Serial.print(" vel: ");
        Serial.print(num * ekf.nav.Pv0, 2);
        Serial.print(", ");
        Serial.print(num * ekf.nav.Pv1, 2);
        Serial.print(", ");
        Serial.print(num * ekf.nav.Pv2, 2);
        Serial.print(" att: ");
        Serial.print(num * ekf.nav.Pa0*R2D, 2);
        Serial.print(", ");
        Serial.print(num * ekf.nav.Pa1*R2D, 2);
        Serial.print(", ");
        Serial.print(num * ekf.nav.Pa2*R2D, 2);
        Serial.println();
    }
}

// output a binary representation of the barometer data
int comms_t::write_airdata_bin()
{
    static message::airdata_t airdata1;
    airdata1.baro_press_pa = airdata.baro_press;
    airdata1.baro_temp_C = airdata.baro_temp;
    airdata1.baro_hum = airdata.baro_hum;
    airdata1.ext_diff_press_pa = airdata.diffPress_pa;
    airdata1.ext_static_press_pa = 0.0; // fixme!
    airdata1.ext_temp_C = airdata.temp_C;
    airdata1.error_count = airdata.error_count;
    airdata1.pack();
    return serial.write_packet( airdata1.id, airdata1.payload, airdata1.len );
}

void comms_t::write_airdata_ascii()
{
    Serial.print("Barometer: ");
    Serial.print(airdata.baro_press, 2); Serial.print(" (st pa) ");
    Serial.print(airdata.baro_temp, 2); Serial.print(" (C) ");
    Serial.print(airdata.baro_hum, 1); Serial.print(" (%RH) ");
    Serial.print("Pitot: ");
    Serial.print(airdata.diffPress_pa, 4); Serial.print(" (diff pa) ");
    Serial.print(airdata.temp_C, 2); Serial.print(" (C) ");
    Serial.print(airdata.error_count); Serial.print(" (errors) ");
    Serial.println();
}

// output a binary representation of various volt/amp sensors
int comms_t::write_power_bin()
{
    static message::power_t power1;
    power1.int_main_v = power.pwr1_v;
    power1.avionics_v = power.avionics_v;
    power1.ext_main_v = power.pwr2_v;
    power1.ext_main_amp = power.pwr_a;
    power1.pack();
    return serial.write_packet( power1.id, power1.payload, power1.len );
}

void comms_t::write_power_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    Serial.print("Volts Main: "); Serial.print(power.pwr1_v, 2);
    Serial.print(" avionics: "); Serial.println(power.avionics_v, 2);
}

// output a binary representation of various status and config information
int comms_t::write_status_info_bin()
{
    static uint32_t write_millis = millis();
    static message::status_t status;

    // This info is static or slow changing so we don't need to send
    // it at a high rate.
    static int counter = 0;
    if ( counter > 0 ) {
        counter--;
        return 0;
    } else {
        counter = MASTER_HZ * 1 - 1; // a message every 1 seconds (-1 so we aren't off by one frame)
    }

    status.serial_number = serial_number;
    status.firmware_rev = FIRMWARE_REV;
    status.master_hz = MASTER_HZ;
    status.baud = HOST_BAUD;

    // estimate sensor output byte rate
    unsigned long current_time = millis();
    unsigned long elapsed_millis = current_time - write_millis;
    unsigned long byte_rate = output_counter * 1000 / elapsed_millis;
    write_millis = current_time;
    output_counter = 0;
    status.byte_rate = byte_rate;
    status.timer_misses = main_loop_timer_misses;

    status.pack();
    return serial.write_packet( status.id, status.payload, status.len );
}

void comms_t::write_status_info_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    Serial.print("SN: ");
    Serial.print(config.read_serial_number());
    Serial.print(" Firmware: ");
    Serial.print(FIRMWARE_REV);
    Serial.print(" Main loop hz: ");
    Serial.print( MASTER_HZ);
    Serial.print(" Baud: ");
    Serial.println(HOST_BAUD);
}

void comms_t::read_commands() {
    while ( serial.update() ) {
        parse_message_bin( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}

// global shared instance
comms_t comms;
