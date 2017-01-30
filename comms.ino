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

#define PWM_RATE_PACKET_ID 21
#define BAUD_PACKET_ID 22
#define FLIGHT_COMMAND_PACKET_ID 23
#define ACT_GAIN_PACKET_ID 24
#define MIX_MODE_PACKET_ID 25
#define SAS_MODE_PACKET_ID 26
#define SERIAL_NUMBER_PACKET_ID 27
#define WRITE_EEPROM_PACKET_ID 28

#define PILOT_PACKET_ID 50
#define IMU_PACKET_ID 51
#define GPS_PACKET_ID 52
#define BARO_PACKET_ID 53
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
    int counter = 0;
    bool result = false;

    if ( id == FLIGHT_COMMAND_PACKET_ID && message_size == MAX_CHANNELS * 2 ) {
	/* flight commands are 2 bytes, lo byte first, then hi byte.
	 * Integer values correspond to servo pulse lenght in us 1100
	 * is a normal minimum value, 1500 is center, 1900 is a normal
	 * maximum value although the min and max can be extended a
	 * bit. */
	for ( int i = 0; i < MAX_CHANNELS; i++ ) {
	    byte lo = buf[counter++];
	    byte hi = buf[counter++];
	    // fixme: autopilot_pwm[i] = hi*256 + lo;
	}
	// fixme: pwm_pwm2norm( autopilot_pwm, autopilot_norm );

#if 0 // fixme
	if ( receiver_norm[CH_8] > 0.0 ) {
	    // autopilot mode active (determined elsewhere when each
	    // new receiver frame is ready) mix the inputs and write
	    // the actuator outputs now
            sas_update( autopilot_norm );
            // don't overwrite manual ch7 value if sas_ch7tune enabled
	    mixing_update( autopilot_norm, true /* ch1-6 */, !config.sas_ch7tune /* ch7 */, true /* no ch8 */ );
	    pwm_update();
	} else {
	    // we are in manual mode
	    // update ch8 only from the autopilot.  We don't pass the
	    // auto/manual switch state through to a servo.  Instead
	    // we simply relay ch8 from the autopilot to the actuator
	    // which gives the autopilot an extra useful output
	    // channel.  This channel is always driven by the
	    // autopilot, no matter what the state, but we'll let the
	    // output to the APM_RC wait until it happens
	    // automatically with the next receiver frame.
            // don't overwrite manual ch7 value if sas_ch7tune enabled
            mixing_update( autopilot_norm, false /* ch1-6 */, !config.sas_ch7tune /* ch7 */, true /* no ch8 */ );
	}
 #endif // fixme
	result = true;

#if 0
	// disable baud changing until I have more time to work out
	// the nuances seems like when the remote end closes and
	// reopens at the new baud, this side may get reset and put
	// back to 115,200 and the whole app starts over -- when connected
        // via the usb port.
    } else if ( id == BAUD_PACKET_ID && message_size == 4 ) {
	//Serial.println("read Baud command");
	/* of course changing baud could can break communication until
	   the requesting side changes it's own baud rate to match*/
	uint32_t baud = *(uint32_t *)buf;
	// Serial.printf("Changing baud to %ld.  See you on the other side!\n", baud);
	/* sends "ack" at both old baud and new baud rates */
	write_ack_bin( id );
	Serial.flush();
	delay(100);
	Serial.end();
    
	Serial.begin(baud);
	delay(500);
    
	write_ack_bin( id );
	write_ack_bin( id );
	write_ack_bin( id );
    
	result = true;
#endif

    } else if ( id == PWM_RATE_PACKET_ID && message_size == MAX_CHANNELS * 2 ) {
	//Serial.println("read PWM command");
	/* note that CH1/CH2, CH3/CH4/CH5, and CH6/CH7/CH8 run off
	 * grouped timers so setting any of the group will also force
	 * the same rate for the other channels in the group, channels
	 * with rate of 0 are left untouched (but will be affected if
	 * another group member rate is set.) */
	for ( int i = 0; i < MAX_CHANNELS; i++ ) {
	    uint8_t lo = buf[counter++];
	    uint8_t hi = buf[counter++];
	    uint16_t rate = hi*256 + lo;
	    //Serial.printf("ch %d rate %d\n", i, rate);
	    if ( rate > 0 ) {
                config.pwm_hz[i] = rate;
                // sanity checks
		if ( rate < 50 ) { rate = 50; }
		if ( rate > 250 ) { rate = 250; }
                uint32_t ch_mask = _BV(i);
		// fixme: APM_RC.SetFastOutputChannels( ch_mask, rate );
	    }
	}
	write_ack_bin( id, 0 );
	result = true;
  #if 0 // fixme
    } else if ( id == ACT_GAIN_PACKET_ID && message_size == 3 ) {
	if ( act_gain_command_parse( buf ) ) {
	    write_ack_bin( id, buf[0] /* sub command */ );
            result = true;
	}
    } else if ( id == SAS_MODE_PACKET_ID && message_size == 4 ) {
	if ( sas_command_parse( buf ) ) {
	    write_ack_bin( id, buf[0] /* sub command */ );
            result = true;
	}
    } else if ( id == MIX_MODE_PACKET_ID && message_size == 6 ) {
	if ( mixing_command_parse( buf ) ) {
	    write_ack_bin( id, buf[0] /* sub command */ );
            result = true;
	}
    } else if ( id == SERIAL_NUMBER_PACKET_ID && message_size == 2 ) {
	uint8_t lo = buf[0];
	uint8_t hi = buf[1];
	set_serial_number(hi*256 + lo);
	write_ack_bin( id, 0 );
	result = true;
    } else if ( id == WRITE_EEPROM_PACKET_ID && message_size == 0 ) {
	config_write_eeprom();
	write_ack_bin( id, 0 );
	result = true;
  #endif // fixme
    }
    return result;
}


bool read_commands()
{
    static byte state = 0; // 0 = looking for SOM0, 1 = looking for SOM1, 2 = looking for packet id & size, 3 = looking for packet data and checksum
    byte input;
    static byte buf[256];
    static byte message_id = 0;
    static byte message_size = 0;
    byte cksum0 = 0, cksum1 = 0;
    bool new_data = false;
    // Serial.print("top: "); Serial.println(state);

    if ( state == 0 ) {
	while ( Serial.available() >= 1 ) {
	    // scan for start of message
	    input = Serial.read();
	    if ( input == START_OF_MSG0 ) {
		// Serial.println("start of msg0");
		state = 1;
		break;
	    }
	}
    }
    if ( state == 1 ) {
	if ( Serial.available() >= 1 ) {
	    input = Serial.read();
	    if ( input == START_OF_MSG1 ) {
		// Serial.println("start of msg1");
		state = 2;
	    } 
	    else if ( input == START_OF_MSG0 ) {
		// no change
	    }
	    else {
		// oops
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	if ( Serial.available() >= 2 ) {
	    message_id = Serial.read();
	    //Serial.print("id="); Serial.println(message_id);
	    message_size = Serial.read();
	    //Serial.print("size="); Serial.println(message_size);
	    //if ( message_id != COMMAND_PACKET_ID ) {
	    // ignore bogus message id's
	    //  state = 0;
	    //} else
	    if ( message_size > 20 ) {
		// ignore nonsensical sizes
		state = 0;
	    } 
	    else {
		state = 3;
	    }
	}
    }
    if ( state == 3 ) {
	if ( Serial.available() >= message_size ) {
	    for ( int i = 0; i < message_size; i++ ) {
		buf[i] = Serial.read();
		// Serial.println(buf[i], DEC);
	    }
	    state = 4;
	}
    }
    if ( state == 4 ) {
	if ( Serial.available() >= 2 ) {
	    cksum0 = Serial.read();
	    cksum1 = Serial.read();
	    byte new_cksum0, new_cksum1;
	    ugear_cksum( message_id, message_size, buf, message_size, &new_cksum0, &new_cksum1 );
	    if ( cksum0 == new_cksum0 && cksum1 == new_cksum1 ) {
		//Serial.println("passed check sum!");
		parse_message_bin( message_id, buf, message_size );
		new_data = true;
		// fixme: binary_output = true;
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
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = ACK_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (2 bytes)
    buf[0] = 2;
    Serial.write( buf, 1 );

    // ack id
    packet[size++] = command_id;
    packet[size++] = subcommand_id;
    
    // write packet
    Serial.write( packet, size );

    // check sum (2 bytes)
    ugear_cksum( ACK_PACKET_ID, size, packet, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
}


/* output a binary representation of the pilot (rc receiver) data */
uint8_t write_pilot_in_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 2 * MAX_CHANNELS;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = PILOT_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = 2 * MAX_CHANNELS;
    Serial.write( buf, 1 );

    // receiver data
    for ( int i = 0; i < MAX_CHANNELS; i++ ) {
	// fixme: int16_t val = receiver_norm[i] * 16384.0;
  // fixme:	*(int16_t *)packet = val; packet += 2;
    }
    
    // write packet
    Serial.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( PILOT_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
    
    return size + 6;
}

void write_pilot_in_ascii()
{
    // receiver input data
    Serial.print("RCIN:");
    for ( int i = 0; i < MAX_CHANNELS - 1; i++ ) {
        // fixme: Serial.print(receiver_norm[i], 3);
        Serial.print(" ");
    }
    // fixme: Serial.println(receiver_norm[MAX_CHANNELS-1], 3);
}

void write_actuator_out_ascii()
{
    // actuator output
    Serial.print("RCOUT:");
    for ( int i = 0; i < MAX_CHANNELS - 1; i++ ) {

        // fixme: Serial.print(actuator_pwm[i]);
        Serial.print(" ");
    }
    // fixme: Serial.println(actuator_pwm[MAX_CHANNELS-1]);
}

/* output a binary representation of the IMU data (note: scaled to 16bit values) */
uint8_t write_imu_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    // fixme: byte size = 4 /* for timestamp */ + 2 * MAX_IMU_SENSORS + 6 /* for mags */;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = IMU_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (1 byte)
    // fixme: buf[0] = size;
    Serial.write( buf, 1 );

    // fixme: *(uint32_t *)packet = imu_micros; packet += 4;

    int16_t val = 0;
  
    // gyro data
    for ( int i = 0; i < 3; i++ ) {
	// fixme: val = imu_sensors[i] / MPU6000_gyro_scale;
        *(int16_t *)packet = val; packet += 2;
    }

    // accel data
    for ( int i = 3; i < 6; i++ ) {
	// fixme: val = imu_sensors[i] / MPU6000_accel_scale;
        *(int16_t *)packet = val; packet += 2;
    }
  
    // mag is a signed int16_t but transport it as unsigned
    // fixme: *(int16_t *)packet = compass.mag_x; packet += 2;
    // fixme: *(int16_t *)packet = compass.mag_y; packet += 2;
    // fixme: *(int16_t *)packet = compass.mag_z; packet += 2;

    // fixme: val = imu_sensors[6] / MPU6000_temp_scale;
    // fixme: *(int16_t *)packet = val; packet += 2;

    // write packet
    // fixme: Serial.write( packet_buf, size );

    // check sum (2 bytes)
    // fixme: ugear_cksum( IMU_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
    
    // fixme: return size + 6;
}

void write_imu_ascii()
{
    // output imu data
    Serial.print("IMU:");
    for ( int i = 0; i < 6; i++ ) {
        // fixme: Serial.print(imu_sensors[i]);
        Serial.print(",");
    }
    // fixme: Serial.print(compass.mag_x); Serial.print(",");
    // fixme: Serial.print(compass.mag_y); Serial.print(",");
    // fixme: Serial.print(compass.mag_z); Serial.print(",");
    // fixme: Serial.println(imu_sensors[6]); // temp C last
}

/* output a binary representation of the GPS data */
uint8_t write_gps_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 30;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;

#if 0 // fixme
    if ( !g_gps->new_data ) {
	return 0;
    }
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = GPS_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial.write( buf, 1 );

    *(uint32_t *)packet = g_gps->time; packet += 4;
    *(uint32_t *)packet = g_gps->date; packet += 4;
    *(int32_t *)packet = g_gps->latitude; packet += 4;
    *(int32_t *)packet = g_gps->longitude; packet += 4;
    *(int32_t *)packet = g_gps->altitude; packet += 4;
    //*(uint16_t *)packet = (uint16_t)g_gps->ground_speed; packet += 2;
    //if ( g_gps->ground_course < 0 ) { g_gps->ground_course += 36000; }
    //*(uint16_t *)packet = (uint16_t)g_gps->ground_course; packet += 2;
    *(int16_t *)packet = (int16_t)g_gps->vn_cms; packet += 2;
    *(int16_t *)packet = (int16_t)g_gps->ve_cms; packet += 2;
    *(int16_t *)packet = (int16_t)g_gps->vd_cms; packet += 2;
    *(int16_t *)packet = g_gps->pdop; packet += 2;
    *(uint8_t *)packet = g_gps->num_sats; packet += 1;
    *(uint8_t *)packet = g_gps->status(); packet += 1;
  
    // write packet
    Serial.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( GPS_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
  
    g_gps->new_data = false;
    
    return size + 6;
    #endif // fixme
}

#define T6 1000000
#define T7 10000000
void write_gps_ascii()
{
  #if 0 // fixme
    if ( !g_gps->new_data ) {
	return;
    }
    // output gps data
    Serial.print("GPS:");
    Serial.print(" Lat:");
    //Serial.print((double)g_gps->latitude / T7, DEC);
    Serial.print(g_gps->latitude);
    Serial.print(" Lon:");
    //Serial.print((double)g_gps->longitude / T7, DEC);
    Serial.print(g_gps->longitude);
    Serial.print(" Alt:");
    Serial.print((float)g_gps->altitude / 100.0, DEC);
    Serial.print(" Vel:");
    Serial.print(g_gps->vn_cms / 100.0);
    Serial.print(", ");
    Serial.print(g_gps->ve_cms / 100.0);
    Serial.print(", ");
    Serial.print(g_gps->vd_cms / 100.0);
    Serial.print(" GSP:");
    Serial.print(g_gps->ground_speed / 100.0, DEC);
    Serial.print(" COG:");
    Serial.print(g_gps->ground_course / 100.0, DEC);
    Serial.print(" SAT:");
    Serial.print(g_gps->num_sats, DEC);
    Serial.print(" FIX:");
    Serial.print(g_gps->fix, DEC);
    Serial.print(" TIM:");
    Serial.print(g_gps->time, DEC);
    Serial.print(" DATE:");
    Serial.print(g_gps->date, DEC);
    Serial.println();
    /*Serial.print("long:");
    Serial.print(sizeof(long));
    Serial.print(" uint32_t:");
    Serial.print(sizeof(uint32_t));
    Serial.print(" int:");
    Serial.print(sizeof(int));
    Serial.print(" uint8_t:");
    Serial.println(sizeof(uint8_t));*/
    g_gps->new_data = 0; // mark the data as read
    #endif // fixme
}

/* output a binary representation of the barometer data */
uint8_t write_baro_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 12;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;
#if 0 // fixme
    if ( !baro.healthy ) {
	return 0;
    }
    
    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = BARO_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial.write( buf, 1 );
    
    // update the filters so climb rate works
    baro.get_altitude();
    
    *(float *)packet = baro.get_pressure(); packet += 4;
    *(float *)packet = baro.get_temperature(); packet += 4;
    *(float *)packet = baro.get_climb_rate(); packet += 4;
  
    // write packet
    Serial.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( BARO_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
    
    return size + 6;
    #endif // fixme
}

void write_baro_ascii()
{
  #if 0 // fixme
    if ( !baro.healthy ) {
	return;
    }
    // output barometer data
    Serial.print("Pressure:");
    Serial.print(baro.get_pressure());
    Serial.print(" Temperature:");
    Serial.print(baro.get_temperature());
    Serial.print(" Altitude:");
    Serial.print(baro.get_altitude());
    Serial.print(" climb=");
    Serial.print(baro.get_climb_rate());
    Serial.print(" samples="),
    Serial.println(baro.get_pressure_samples());
    #endif // fixme
}

/* output a binary representation of the analog input data */
uint8_t write_analog_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 0;
    byte packet[256]; // hopefully never larger than this!

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = ANALOG_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (1 byte)
    // fixme: buf[0] = 2 * MAX_ANALOG_INPUTS;
    Serial.write( buf, 1 );

#if 0 // fixme
    // channel data
    for ( int i = 0; i < MAX_ANALOG_INPUTS; i++ ) {
	uint16_t val = analog[i];
	int hi = val / 256;
	int lo = val - (hi * 256);
	packet[size++] = byte(lo);
	packet[size++] = byte(hi);
    }
    #endif // fixme
    
    // write packet
    Serial.write( packet, size );

    // check sum (2 bytes)
    ugear_cksum( ANALOG_PACKET_ID, size, packet, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
    
    return size + 6;
}

void write_analog_ascii()
{
    /*
    static float amp_filt = 0.0;
    amp_filt = 0.999 * amp_filt + 0.001 * battery_amps;
    */
    #if 0 // fixme
    // output servo data
    Serial.print("Analog:");
    for ( int i = 0; i < MAX_ANALOG_INPUTS - 1; i++ ) {
        Serial.print((float)analog[i] / 64.0, 2);
        Serial.print(" ");
    }
    Serial.println((float)analog[MAX_ANALOG_INPUTS-1] / 1000.0, 2);
    #endif // fixme
    /*
    Serial.printf("%.2f ", vcc_average);
    Serial.printf("%.2f ", (float)battery_voltage);
    Serial.printf("%.4f ", (float)amp_filt);
    Serial.printf("%.4f\n", (float)amps_sum);
    */
}

/* output a binary representation of various status and config information */
uint8_t write_status_info_bin()
{
    byte buf[3];
    byte cksum0, cksum1;
    byte size = 12;
    byte packet_buf[256]; // hopefully never larger than this!
    byte *packet = packet_buf;

    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    static int counter = 0;
    if ( counter > 0 ) {
        counter--;
        return 0;
    } else {
        counter = MASTER_HZ * 5 - 1; // a message every 10 seconds (-1 so we aren't off by one frame) 
    }

    // start of message sync bytes
    buf[0] = START_OF_MSG0; 
    buf[1] = START_OF_MSG1; 
    buf[2] = 0;
    Serial.write( buf, 2 );

    // packet id (1 byte)
    buf[0] = STATUS_INFO_PACKET_ID; 
    buf[1] = 0;
    Serial.write( buf, 1 );

    // packet length (1 byte)
    buf[0] = size;
    Serial.write( buf, 1 );

    *(uint16_t *)packet = (uint16_t)apm2_serial_number; packet += 2;
    *(uint16_t *)packet = (uint16_t)FIRMWARE_REV; packet += 2;
    *(uint16_t *)packet = (uint16_t)MASTER_HZ; packet += 2;
    *(uint32_t *)packet = (uint32_t)DEFAULT_BAUD; packet += 4;

    // estimate sensor output byte rate
    unsigned long current_time = millis();
    // fixme: unsigned long elapsed_millis = current_time - write_millis;
    // fixme: unsigned long byte_rate = output_counter * 1000 / elapsed_millis;
    // fixme: write_millis = current_time;
    // fixme: output_counter = 0;
    // fixme: *(uint16_t *)packet = (uint16_t)byte_rate; packet += 2;
    
    // write packet
    Serial.write( packet_buf, size );

    // check sum (2 bytes)
    ugear_cksum( STATUS_INFO_PACKET_ID, size, packet_buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; 
    buf[1] = cksum1; 
    buf[2] = 0;
    Serial.write( buf, 2 );
    
    return size + 6;
}

void write_status_info_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    static int counter = 0;
    if ( counter > 0 ) {
        counter--;
        return;
    } else {
        counter = MASTER_HZ * 10 - 1; // a message every 10 seconds (-1 so we aren't off by one frame) 
    }
    Serial.print("SN: ");
    Serial.println(read_serial_number());
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Main loop: ");
    Serial.println( MASTER_HZ);
    Serial.print("Baud: ");
}

