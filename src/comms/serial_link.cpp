#include "serial_link.h"

void SerialLink::checksum( uint8_t id, uint8_t len_lo, uint8_t len_hi, uint8_t *buf, uint16_t buf_size, uint8_t *cksum0, uint8_t *cksum1 ) {
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += id;
    c1 += c0;

    c0 += len_lo;
    c1 += c0;

    c0 += len_hi;
    c1 += c0;

    for ( uint16_t i = 0; i < buf_size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}

bool SerialLink::open( int baud, int port ) {
    if ( port == 0 ) {
        Serial.begin(baud);
        _port = &Serial;
    } else if ( port == 1 ) {
        Serial1.begin(baud);
        _port = &Serial1;
    } else if ( port == 2 ) {
        Serial2.begin(baud);
        _port = &Serial2;
    } else if ( port == 3 ) {
        Serial3.begin(baud);
        _port = &Serial3;
    } else if ( port == 4 ) {
        Serial4.begin(baud);
        _port = &Serial4;
    } else {
        printf("unsupported port number: %d, defaulting to Serial, will probably clash with the console.", port);
        Serial.begin(baud);
        _port = &Serial;
    }
    if ( _port->availableForWrite() < 128 ) {
        printf("ERROR: Serial %d TX/RX buffer size not configured correctly, see note in docs.\n", port);
        printf("buffer size available for write: %d\n", _port->availableForWrite());
        delay(5000);
    }

    return true;
}

bool SerialLink::update() {
    // 0 = looking for SOM0
    // 1 = looking for SOM1
    // 2 = looking for packet id
    // 3 = looking for packet len (note 2 bytes)
    // 4 = looking for packet data
    // 5 = looking for checksum_lo
    // 6 = looking for checksum_l=hi
    uint8_t input;
    bool new_data = false;
    // console->print("start read_commands(): "); console->println(state);

    if ( state == 0 ) {
        counter = 0;
        while ( _port->available() >= 1 ) {
            // scan for start of message
            input = _port->read();
            // printf("%02x ", input);
            if ( input == START_OF_MSG0 ) {
                // console->println("start of msg0");
                state = 1;
                break;
            }
        }
    }
    if ( state == 1 ) {
        if ( _port->available() >= 1 ) {
            input = _port->read();
            if ( input == START_OF_MSG1 ) {
                // console->println("start of msg1");
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
            // console->print("id="); console->println(message_id);
            state = 3;
        }
    }
    if ( state == 3 ) {
        if ( _port->available() >= 2 ) {
            pkt_len_lo = _port->read();
            pkt_len_hi = _port->read();
            pkt_len = pkt_len_hi << 8 | pkt_len_lo;
            // console->printf("size=%d\n", pkt_len);
            if ( pkt_len > MAX_MESSAGE_LEN ) {
                printf("nonsense packet size, skipping.\n");
                // ignore nonsensical sizes
                state = 0;
            }  else {
                state = 4;
            }
        }
    }
    if ( state == 4 ) {
        while ( _port->available() >= 1 && counter < pkt_len ) {
            payload[counter++] = _port->read();
            // printf(buf[i], DEC);
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
            uint8_t cksum0, cksum1;
            checksum( pkt_id, pkt_len_lo, pkt_len_hi, payload, pkt_len, &cksum0, &cksum1 );
            if ( cksum_lo == cksum0 && cksum_hi == cksum1 ) {
                // console->println("passed check sum!");
                // console->print("pkt_id = "); console->println(pkt_id);
                // console->print("size="); console->println(pkt_len);
                // parse_message_bin( pkt_id, payload, pkt_len );
                new_data = true;
                state = 0;
            } else {
                printf("failed check sum id: %d len: %d\n", pkt_id, pkt_len);
                // check sum failure
                state = 0;
            }
        }
    }

    return new_data;
}

// bool SerialLink::update() {
//     // 0 = looking for SOM0
//     // 1 = looking for SOM1
//     // 2 = looking for packet id
//     // 3 = looking for packet len
//     // 4 = looking for packet data
//     // 5 = looking for checksum_lo
//     // 6 = looking for checksum_l=hi
//     uint8_t input;
//     bool new_data = false;
//     // Serial.print("start read_commands(): "); Serial.println(state);

//     if ( state == 0 ) {
//         counter = 0;
//         while ( _port->available() >= 1 ) {
//             // scan for start of message
//             input = _port->read();
//             // Serial.println(input);
//             if ( input == START_OF_MSG0 ) {
//                 // Serial.println("start of msg0");
//                 state = 1;
//                 break;
//             }
//         }
//     }
//     if ( state == 1 ) {
//         if ( _port->available() >= 1 ) {
//             input = _port->read();
//             if ( input == START_OF_MSG1 ) {
//                 // Serial.println("start of msg1");
//                 state = 2;
//             }
//             else if ( input == START_OF_MSG0 ) {
//                 // no change
//             } else {
//                 // oops
//                 state = 0;
//             }
//         }
//     }
//     if ( state == 2 ) {
//         if ( _port->available() >= 1 ) {
//             pkt_id = _port->read();
//             // Serial.print("id="); Serial.println(message_id);
//             state = 3;
//         }
//     }
//     if ( state == 3 ) {
//         if ( _port->available() >= 1 ) {
//             pkt_len = _port->read();
//             // Serial.print("size="); Serial.println(pkt_len);
//             if ( pkt_len > 200 ) {
//                 // ignore nonsensical sizes
//                 state = 0;
//             }  else {
//                 state = 4;
//             }
//         }
//     }
//     if ( state == 4 ) {
//         while ( _port->available() >= 1 && counter < pkt_len ) {
//             if ( counter < MAX_MESSAGE_LEN ) {
//                 payload[counter++] = _port->read();
//                 // Serial.println(buf[i], DEC);
//             } else {
//                 state = 0;
//             }
//         }
//         if ( counter >= pkt_len ) {
//             state = 5;
//         }
//     }
//     if ( state == 5 ) {
//         if ( _port->available() >= 1 ) {
//             cksum_lo = _port->read();
//             state = 6;
//         }
//     }
//     if ( state == 6 ) {
//         if ( _port->available() >= 1 ) {
//             cksum_hi = _port->read();
//             byte cksum0, cksum1;
//             checksum( pkt_id, pkt_len, payload, pkt_len, &cksum0, &cksum1 );
//             if ( cksum_lo == cksum0 && cksum_hi == cksum1 ) {
//                 // Serial.println("passed check sum!");
//                 // Serial.print("pkt_id = "); Serial.println(pkt_id);
//                 // Serial.print("size="); Serial.println(pkt_len);
//                 // parse_message_bin( pkt_id, payload, pkt_len );
//                 new_data = true;
//                 state = 0;
//             } else {
//                 Serial.print("failed check sum, id = "); Serial.print(pkt_id); Serial.print(" len = "); Serial.println(pkt_len);
//                 // check sum failure
//                 state = 0;
//             }
//         }
//     }

//     return new_data;
// }

int SerialLink::bytes_available() {
    return _port->available();
}

int SerialLink::write_packet(uint8_t packet_id, uint8_t *payload, uint16_t len) {
    // start of message sync (2) bytes
    _port->write(START_OF_MSG0);
    _port->write(START_OF_MSG1);

    // packet id (1 byte)
    _port->write(packet_id);

    // packet length (2 bytes)
    uint8_t len_lo = len & 0xFF;
    uint8_t len_hi = len >> 8;
    _port->write(len_lo);
    _port->write(len_hi);

    // write payload
    _port->write( payload, len );

    // check sum (2 bytes)
    uint8_t cksum0, cksum1;
    checksum( packet_id, len_lo, len_hi, payload, len, &cksum0, &cksum1 );
    _port->write(cksum0);
    _port->write(cksum1);

    return len + 7;
}

bool SerialLink::close() {
    // fixme: do we ever need to close the serial port or is this just for show?
    // _port->end();
    return true;
}
