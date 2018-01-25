/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "config.h"

#define START_OF_MSG0 147
#define START_OF_MSG1 224

#define ACK_PACKET_ID 20

#define CONFIG_PACKET_ID 21
#define FLIGHT_COMMAND_PACKET_ID 22
#define WRITE_EEPROM_PACKET_ID 23

#define PILOT_PACKET_ID 50
#define IMU_PACKET_ID 51
#define GPS_PACKET_ID 52
#define AIRDATA_PACKET_ID 53
#define ANALOG_PACKET_ID 54
#define STATUS_INFO_PACKET_ID 55

void ugear_cksum( byte hdr1, byte hdr2, byte *buf, byte size,
                  byte *cksum0, byte *cksum1 )
{
    byte c0 = 0;
    byte c1 = 0;

    c0 += hdr1;
    c1 += c0;

    c0 += hdr2;
    c1 += c0;

    for ( byte i = 0; i < size; i++ ) {
        c0 += (byte)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}


bool parse_message_bin( byte id, byte *buf, byte message_size )
{
    bool result = false;

    if ( id == FLIGHT_COMMAND_PACKET_ID && message_size == AP_CHANNELS * 2 ) {
        /* flight commands are 2 byte ints, normalized, then scaled to +/- 16384 */
        float ap_tmp[AP_CHANNELS];
        for ( int i = 0; i < AP_CHANNELS; i++ ) {
            int16_t val = *(int16_t *)buf; buf += 2;
            ap_tmp[i] = (float)val / 16384.0;
            //Serial1.println(ap_tmp[i]);
        }
        // autopilot_norm uses the same channel mapping as sbus_norm, so map ap_tmp values to their
        // correct places in autopilot_norm
        autopilot_norm[2] = ap_tmp[0];
        autopilot_norm[3] = ap_tmp[1];
        autopilot_norm[4] = ap_tmp[2];
        autopilot_norm[5] = ap_tmp[3];
        autopilot_norm[6] = ap_tmp[4];
        autopilot_norm[7] = ap_tmp[5];

        if ( receiver_norm[0] > 0.0 ) {
            // autopilot mode active (determined elsewhere when each
            // new receiver frame is ready) mix the inputs and write
            // the actuator outputs now
            sas_update( autopilot_norm );
            mixing_update( autopilot_norm );
            pwm_update();
        } else {
            // manual mode, do nothing with actuator commands from the autopilot
        }
        result = true;
    } else if ( id == CONFIG_PACKET_ID && message_size == sizeof(config) ) {
        Serial.println("received new config");
        config = *(config_t *)buf;
        pwm_setup();  // reset pwm rates in case they've been changed
        write_ack_bin( id, 0 );
        result = true;
    } else if ( id == WRITE_EEPROM_PACKET_ID && message_size == 0 ) {
        config_write_eeprom();
        write_ack_bin( id, 0 );
        result = true;
    } else {
        // Serial.print(message_size);
        // Serial.print(" ");
        // Serial.println(sizeof(config));
    }
    return result;
}


bool read_commands() {
    static byte state = 0; // 0 = looking for SOM0, 1 = looking for SOM1, 2 = looking for packet id & size, 3 = looking for packet data and checksum
    byte input;
    static byte buf[256];
    static int buf_counter = 0;
    static byte message_id = 0;
    static byte message_size = 0;
    byte cksum0 = 0, cksum1 = 0;
    bool new_data = false;
    // Serial.print("top: "); Serial.println(state);

    if ( state == 0 ) {
        while ( Serial1.available() >= 1 ) {
            // scan for start of message
            input = Serial1.read();
            if ( input == START_OF_MSG0 ) {
                // Serial.println("start of msg0");
                state = 1;
                break;
            }
        }
    }
    if ( state == 1 ) {
        if ( Serial1.available() >= 1 ) {
            input = Serial1.read();
            if ( input == START_OF_MSG1 ) {
                // Serial.println("start of msg1");
                state = 2;
            } 
            else if ( input == START_OF_MSG0 ) {
                // no change
            } else {
                // oops
                state = 0;
            }
        }
    }
    if ( state == 2 ) {
        if ( Serial1.available() >= 2 ) {
            message_id = Serial1.read();
            // Serial.print("id="); Serial.println(message_id);
            message_size = Serial1.read();
            // Serial.print("size="); Serial.println(message_size);
            if ( message_size > 200 ) {
                // ignore nonsensical sizes
                state = 0;
            }  else {
                state = 3;
                buf_counter = 0;
            }
        }
    }
    if ( state == 3 ) {
        while ( Serial1.available() >= 1 && buf_counter < message_size ) {
            buf[buf_counter] = Serial1.read();
            buf_counter++;
            // Serial.println(buf[i], DEC);
        }
        if ( buf_counter >= message_size ) {
            state = 4;
        }
    }
    if ( state == 4 ) {
        if ( Serial1.available() >= 2 ) {
            cksum0 = Serial1.read();
            cksum1 = Serial1.read();
            byte new_cksum0, new_cksum1;
            ugear_cksum( message_id, message_size, buf, message_size, &new_cksum0, &new_cksum1 );
            if ( cksum0 == new_cksum0 && cksum1 == new_cksum1 ) {
                // Serial.println("passed check sum!");
                // Serial.print("size="); Serial.println(message_size);
                parse_message_bin( message_id, buf, message_size );
                new_data = true;
                state = 0;
            } else {
                // Serial.println("failed check sum");
                // check sum failure
                state = 0;
            }
        }
    }

    return new_data;
}


/* output an acknowledgement of a message received */
void write_ack_bin( uint8_t command_id, uint8_t subcommand_id )
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 0;
    byte packet[256]; // hopefully never larger than this!

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = ACK_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (2 bytes)
    buf[0] = 2;
    Serial1.write( buf, 1 );

    // ack id
    packet[size++] = command_id;
    packet[size++] = subcommand_id;
    
    // write packet
    Serial1.write( packet, size );

    // check sum (2 bytes)
    ugear_cksum( ACK_PACKET_ID, size, packet, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
}


/* output a binary representation of the pilot (rc receiver) data */
uint8_t write_pilot_in_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 2 * SBUS_CHANNELS + 1;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = PILOT_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial1.write( buf, 1 );

    // receiver data
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        int16_t val = receiver_norm[i] * 16384.0;
        *(int16_t *)packet = val; packet += 2;
    }

    // flags
    *(uint8_t *)packet = receiver_flags; packet += 1;
    
    // write packet
    Serial1.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( PILOT_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
    
    return size + 6;
}

void write_pilot_in_ascii()
{
    // pilot (receiver) input data
    if ( receiver_flags & SBUS_FAILSAFE ) {
        Serial.print("FAILSAFE! ");
    }
    if ( receiver_norm[0] < 0 ) {
        Serial.print("(Manual) ");
    } else {
        Serial.print("(Auto) ");
    }
    if ( receiver_norm[1] < 0 ) {
        Serial.print("(Throttle safety) ");
    } else {
        Serial.print("(Throttle enable) ");
    }
    for ( int i = 0; i < 7; i++ ) {
        Serial.print(receiver_norm[i], 3);
        Serial.print(" ");
    }
    Serial.println();
}

void write_actuator_out_ascii()
{
    // actuator output
    Serial.print("RCOUT:");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        Serial.print(actuator_pwm[i]);
        Serial.print(" ");
    }
    Serial.println();
}

/* output a binary representation of the IMU data (note: scaled to 16bit values) */
uint8_t write_imu_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 4 /* for timestamp */ + 2 * 10 /* 10 imu values */;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = IMU_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial1.write( buf, 1 );

    *(uint32_t *)packet = imu_micros; packet += 4;

    for ( int i = 0; i < 10; i++ ) {
        *(int16_t *)packet = imu_packed[i]; packet += 2;
    }
    
    // write packet
    Serial1.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( IMU_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
    
    return size + 6;
}

void write_imu_ascii()
{
    // output imu data
    Serial.print("IMU: ");
    Serial.print(imu_micros); Serial.print(" ");
    for ( int i = 0; i < 10; i++ ) {
        Serial.print(imu_calib[i], 3); Serial.print(" ");
    }
    Serial.println();
}

/* output a binary representation of the GPS data */
uint8_t write_gps_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = sizeof(gps_data);

    if ( !new_gps_data ) {
        return 0;
    }
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = GPS_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial1.write( buf, 1 );

    // write packet
    Serial1.write( (byte *)&gps_data, size );

    // check sum (2 bytes)
    ugear_cksum( GPS_PACKET_ID, size, (byte *)(&gps_data), size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
  
    new_gps_data = false;
    
    return size + 6;
}

#define T6 1000000
#define T7 10000000
void write_gps_ascii() {
    Serial.print("GPS:");
    Serial.print(" Lat:");
    Serial.print((double)gps_data.lat / T7, 7);
    //Serial.print(gps_data.lat);
    Serial.print(" Lon:");
    Serial.print((double)gps_data.lon / T7, 7);
    //Serial.print(gps_data.lon);
    Serial.print(" Alt:");
    Serial.print((float)gps_data.hMSL / 1000.0);
    Serial.print(" Vel:");
    Serial.print(gps_data.velN / 1000.0);
    Serial.print(", ");
    Serial.print(gps_data.velE / 1000.0);
    Serial.print(", ");
    Serial.print(gps_data.velD / 1000.0);
    Serial.print(" GSP:");
    Serial.print(gps_data.gSpeed, DEC);
    Serial.print(" COG:");
    Serial.print(gps_data.heading, DEC);
    Serial.print(" SAT:");
    Serial.print(gps_data.numSV, DEC);
    Serial.print(" FIX:");
    Serial.print(gps_data.fixType, DEC);
    Serial.print(" TIM:");
    Serial.print(gps_data.hour); Serial.print(':');
    Serial.print(gps_data.min); Serial.print(':');
    Serial.print(gps_data.sec);
    Serial.print(" DATE:");
    Serial.print(gps_data.month); Serial.print('/');
    Serial.print(gps_data.day); Serial.print('/');
    Serial.print(gps_data.year);
    Serial.println();
}

/* output a binary representation of the barometer data */
uint8_t write_airdata_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 20;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = AIRDATA_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial1.write( buf, 1 );
    
    *(float *)packet = bme_press; packet += 4;
    *(float *)packet = bme_temp; packet += 4;
    *(float *)packet = bme_hum; packet += 4;
    *(float *)packet = airdata_diffPress_pa; packet += 4;
    *(float *)packet = airdata_temp_C; packet += 4;
      
    // write packet
    Serial1.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( AIRDATA_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
    
    return size + 6;
}

void write_airdata_ascii()
{
    Serial.print("Air: ");
    Serial.print(bme_press, 4); Serial.print(" (pa) ");
    Serial.print(bme_temp, 2); Serial.print(" (C) ");
    Serial.print(bme_hum, 1); Serial.print(" (%RH) ");
    Serial.print("Ext Diff (pa): "); Serial.print(airdata_diffPress_pa, 4);
    Serial.print(" Ext Temp (C): "); Serial.println(airdata_temp_C, 2);
}

/* output a binary representation of the analog input data */
uint8_t write_analog_bin()
{
#if 0 // fixme
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 0;
    byte packet[256]; // hopefully never larger than this!

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = ANALOG_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (1 byte)
    // fixme: buf[0] = 2 * MAX_ANALOG_INPUTS;
    Serial1.write( buf, 1 );

    // channel data
    for ( int i = 0; i < MAX_ANALOG_INPUTS; i++ ) {
        uint16_t val = analog[i];
        int hi = val / 256;
        int lo = val - (hi * 256);
        packet[size++] = byte(lo);
        packet[size++] = byte(hi);
    }
    
    // write packet
    Serial1.write( packet, size );

    // check sum (2 bytes)
    ugear_cksum( ANALOG_PACKET_ID, size, packet, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
    
    return size + 6;
#endif // fixme
}

void write_analog_ascii()
{
#if 0 // fixme
    /*
      static float amp_filt = 0.0;
      amp_filt = 0.999 * amp_filt + 0.001 * battery_amps;
    */

    // output servo data
    Serial1.print("Analog:");
    for ( int i = 0; i < MAX_ANALOG_INPUTS - 1; i++ ) {
        Serial1.print((float)analog[i] / 64.0, 2);
        Serial1.print(" ");
    }
    Serial1.println((float)analog[MAX_ANALOG_INPUTS-1] / 1000.0, 2);

    /*
      Serial1.printf("%.2f ", vcc_average);
      Serial1.printf("%.2f ", (float)battery_voltage);
      Serial1.printf("%.4f ", (float)amp_filt);
      Serial1.printf("%.4f\n", (float)amps_sum);
    */
#endif // fixme
}

/* output a binary representation of various status and config information */
uint8_t write_status_info_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 16;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;

    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    static int counter = 0;
    if ( counter > 0 ) {
        counter--;
        return 0;
    } else {
        counter = MASTER_HZ * 1 - 1; // a message every 1 seconds (-1 so we aren't off by one frame) 
    }

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = STATUS_INFO_PACKET_ID; 
    buf[1] = 0;
    Serial1.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial1.write( buf, 1 );

    *(uint16_t *)packet = (uint16_t)serial_number; packet += 2;
    *(uint16_t *)packet = (uint16_t)FIRMWARE_REV; packet += 2;
    *(uint16_t *)packet = (uint16_t)MASTER_HZ; packet += 2;
    *(uint32_t *)packet = (uint32_t)DEFAULT_BAUD; packet += 4;

    // estimate sensor output byte rate
    unsigned long current_time = millis();
    unsigned long elapsed_millis = current_time - write_millis;
    unsigned long byte_rate = output_counter * 1000 / elapsed_millis;
    write_millis = current_time;
    output_counter = 0;
    *(uint16_t *)packet = (uint16_t)byte_rate; packet += 2;
    *(uint16_t *)packet = (uint16_t)(pwr_v*100); packet += 2;
    *(uint16_t *)packet = (uint16_t)(avionics_v*100); packet += 2;
 
    // write packet
    Serial1.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( STATUS_INFO_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial1.write( buf, 2 );
    
    return size + 6;
}

void write_status_info_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    Serial.print("SN: ");
    Serial.println(read_serial_number());
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Main loop hz: ");
    Serial.println( MASTER_HZ);
    Serial.print("Baud: ");Serial.println(DEFAULT_BAUD);
    Serial.print("Volt ext: "); Serial.print(pwr_v, 2);
    Serial.print(" av: "); Serial.println(avionics_v, 2);
}

