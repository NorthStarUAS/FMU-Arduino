#pragma once

#include "Arduino.h"

class SerialLink {

private:

    // port
    Stream *_port;

    // parser
    int state = 0;
    int counter = 0;
    uint8_t cksum_lo = 0, cksum_hi = 0;

    static const uint16_t MAX_MESSAGE_LEN = 200;
    static const uint8_t START_OF_MSG0 = 147;
    static const uint8_t START_OF_MSG1 = 224;

public:

    int pkt_id = 0;
    int pkt_len = 0;
    uint8_t payload[MAX_MESSAGE_LEN];

    uint32_t parse_errors = 0;

    SerialLink();
    ~SerialLink();

    bool open( int baud, int port );
    bool update();
    int bytes_available();
    int write_packet(uint8_t packet_id, uint8_t *payload, uint8_t len);
    bool close();
    void checksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 );
};
