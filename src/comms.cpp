/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include <Arduino.h>

#include "actuators.h"
#include "airdata.h"
#include "config.h"
#include "ekf.h"
#include "gps.h"
#include "imu.h"
#include "led.h"
#include "mixer.h"
#include "pilot.h"
#include "power.h"
#include "pwm.h"
#include "sensors/sbus/sbus.h"
#include "util/serial_link.h"
#include "aura4_messages.h"
#include "../setup_board.h"

#include "comms.h"

void comms_t::setup() {
    serial.open(DEFAULT_BAUD, &Serial1);
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
    } else if ( id == message::config_master_id ) {
        config.master.unpack(buf, message_size);
        if ( message_size == config.master.len ) {
            Serial.println("received master config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_imu_id ) {
        imu.config.unpack(buf, message_size);
        if ( message_size == imu.config.len ) {
            Serial.println("received imu config");
            imu.set_orientation(); // update R matrix
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_actuators_id ) {
        config.actuators.unpack(buf, message_size);
        if ( message_size == config.actuators.len ) {
            Serial.println("received new actuator config");
            // update pwm config in case it has been changed.
            config.write_eeprom();
            mixer.setup();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_airdata_id ) {
        airdata.config.unpack(buf, message_size);
        if ( message_size == airdata.config.len ) {
            Serial.println("received new airdata config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_power_id ) {
        power.config.unpack(buf, message_size);
        if ( message_size == power.config.len ) {
            Serial.println("received new power config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::config_led_id ) {
        led.config.unpack(buf, message_size);
        if ( message_size == led.config.len ) {
            Serial.println("received new led config");
            config.write_eeprom();
            write_ack_bin( id, 0 );
            result = true;
        }
    } else if ( id == message::command_zero_gyros_id && message_size == 0 ) {
        Serial.println("received zero gyros command");
        imu.gyros_calibrated = 0;   // start state
        write_ack_bin( id, 0 );
        result = true;
    } else {
        // Serial.print("unknown message id = "); Serial.print(id); Serial.print(" len = "); Serial.println(message_size);
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
    
    static message::imu_raw_t imu1;
    imu1.micros = imu.imu_micros;
    imu1.channel[0] = imu.get_ax() / accelScale;
    imu1.channel[1] = imu.get_ay() / accelScale;
    imu1.channel[2] = imu.get_az() / accelScale;
    imu1.channel[3] = imu.get_p() / gyroScale;
    imu1.channel[4] = imu.get_q() / gyroScale;
    imu1.channel[5] = imu.get_r() / gyroScale;
    imu1.channel[6] = imu.get_hx() / magScale;
    imu1.channel[7] = imu.get_hy() / magScale;
    imu1.channel[8] = imu.get_hz() / magScale;
    imu1.channel[0] = imu.get_temp() / tempScale;
    imu1.pack();
    return serial.write_packet( imu1.id, imu1.payload, imu1.len );
}

void write_imu_ascii()
{
    // output imu data
    Serial.print("IMU: ");
    Serial.print(imu.imu_micros); Serial.print(" ");
    Serial.print(imu.get_p(), 2); Serial.print(" ");
    Serial.print(imu.get_q(), 2); Serial.print(" ");
    Serial.print(imu.get_r(), 2); Serial.print(" ");
    Serial.print(imu.get_ax(), 3); Serial.print(" ");
    Serial.print(imu.get_ay(), 3); Serial.print(" ");
    Serial.print(imu.get_az(), 3); Serial.print(" ");
    Serial.print(imu.get_temp(), 2);
    Serial.println();
}

// output a binary representation of the GPS data
int comms_t::write_gps_bin()
{
    byte size = sizeof(gps.gps_data);

    if ( !gps.new_gps_data ) {
        return 0;
    } else {
        gps.new_gps_data = false;
    }

    return serial.write_packet( message::aura_nav_pvt_id, (uint8_t *)(&(gps.gps_data)), size );
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
    nav.micros = imu.imu_micros;
    nav.lat_rad = ekf.nav.lat;
    nav.lon_rad = ekf.nav.lon;
    nav.altitude_m = ekf.nav.alt;
    nav.vn_ms = ekf.nav.vn;
    nav.ve_ms = ekf.nav.ve;
    nav.vd_ms = ekf.nav.vd;
    nav.phi_rad = ekf.nav.phi;
    nav.the_rad = ekf.nav.the;
    nav.psi_rad = ekf.nav.psi;
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

void write_airdata_ascii()
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

void write_power_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    Serial.print("SN: ");
    Serial.println(config.read_serial_number());
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Main loop hz: ");
    Serial.println( MASTER_HZ);
    Serial.print("Baud: ");Serial.println(DEFAULT_BAUD);
    Serial.print("Main v: "); Serial.print(power.pwr1_v, 2);
    Serial.print(" av: "); Serial.println(power.avionics_v, 2);
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
    status.baud = DEFAULT_BAUD;

    // estimate sensor output byte rate
    unsigned long current_time = millis();
    unsigned long elapsed_millis = current_time - write_millis;
    unsigned long byte_rate = output_counter * 1000 / elapsed_millis;
    write_millis = current_time;
    output_counter = 0;
    status.byte_rate = byte_rate;

    status.pack();
    return serial.write_packet( status.id, status.payload, status.len );
}

void write_status_info_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    Serial.print("SN: ");
    Serial.println(config.read_serial_number());
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Main loop hz: ");
    Serial.println( MASTER_HZ);
    Serial.print("Baud: ");Serial.println(DEFAULT_BAUD);
}

void comms_t::read_commands() {
    while ( serial.update() ) {
        parse_message_bin( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}

// global shared instance
comms_t comms;
