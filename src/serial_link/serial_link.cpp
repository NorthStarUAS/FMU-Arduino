#include "serial_link.h"

SerialLink::SerialLink() {
}

SerialLink::~SerialLink() {
}

void SerialLink::checksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += hdr1;
    c1 += c0;

    c0 += hdr2;
    c1 += c0;

    for ( uint8_t i = 0; i < size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}

bool SerialLink::open( int baud, HardwareSerial *port ) {
    _port = port;
    _port->begin(baud);
    return true;
}

bool SerialLink::update() {
    // 0 = looking for SOM0
    // 1 = looking for SOM1
    // 2 = looking for packet id
    // 3 = looking for packet len
    // 4 = looking for packet data
    // 5 = looking for checksum_lo
    // 6 = looking for checksum_l=hi
    uint8_t input;
    static int buf_counter = 0;
    bool new_data = false;
    // Serial.print("start read_commands(): "); Serial.println(state);

    if ( state == 0 ) {
        counter = 0;
        while ( _port->available() >= 1 ) {
            // scan for start of message
            input = _port->read();
            // Serial.println(input);
            if ( input == START_OF_MSG0 ) {
                // Serial.println("start of msg0");
                state = 1;
                break;
            }
        }
    }
    if ( state == 1 ) {
        if ( _port->available() >= 1 ) {
            input = _port->read();
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
        if ( _port->available() >= 1 ) {
            pkt_id = _port->read();
            // Serial.print("id="); Serial.println(message_id);
            state = 3;
        }
    }
    if ( state == 3 ) {
        if ( _port->available() >= 1 ) {
            pkt_len = _port->read();
            // Serial.print("size="); Serial.println(pkt_len);
            if ( pkt_len > 200 ) {
                // ignore nonsensical sizes
                state = 0;
            }  else {
                state = 4;
            }
        }
    }
    if ( state == 4 ) {
        while ( _port->available() >= 1 && counter < pkt_len ) {
            if ( counter < MAX_MESSAGE_LEN ) {
                payload[counter++] = _port->read();
                // Serial.println(buf[i], DEC);
            } else {
                state = 0;
            }
        }
        if ( counter >= pkt_len ) {
            state = 5;
        }
    }
    if ( state == 5 ) {
        if ( _port->available() >= 1 ) {
            cksum_lo = _port->read();
            state = 6;
        }
    }
    if ( state == 6 ) {
        if ( _port->available() >= 1 ) {
            cksum_hi = _port->read();
            byte cksum0, cksum1;
            checksum( pkt_id, pkt_len, payload, pkt_len, &cksum0, &cksum1 );
            if ( cksum_lo == cksum0 && cksum_hi == cksum1 ) {
                // Serial.println("passed check sum!");
                // Serial.print("pkt_id = "); Serial.println(pkt_id);
                // Serial.print("size="); Serial.println(pkt_len);
                // parse_message_bin( pkt_id, payload, pkt_len );
                new_data = true;
                state = 0;
            } else {
                Serial.print("failed check sum, id = "); Serial.print(pkt_id); Serial.print(" len = "); Serial.println(pkt_len);
                // check sum failure
                state = 0;
            }
        }
    }

    return new_data;
}

int SerialLink::bytes_available() {
    return _port->available();
}

int SerialLink::write_packet(uint8_t packet_id, uint8_t *payload, uint8_t len) {
    // start of message sync (2) bytes
    _port->write(START_OF_MSG0);
    _port->write(START_OF_MSG1);

    // packet id (1 byte)
    _port->write(packet_id);
    
    // packet length (1 byte)
    _port->write(len);

    // write payload
    _port->write( payload, len );
    
    // check sum (2 bytes)
    uint8_t cksum0, cksum1;
    checksum( packet_id, len, payload, len, &cksum0, &cksum1 );
    _port->write(cksum0);
    _port->write(cksum1);

    return len + 6;
}

bool SerialLink::close() {
    _port->end();
    return true;
}
