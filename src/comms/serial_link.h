#pragma once

#include "Arduino.h"

static const uint8_t START_OF_MSG0 = 147;
static const uint8_t START_OF_MSG1 = 224;
void checksum( uint8_t id, uint8_t len_lo, uint8_t len_hi, uint8_t *buf, uint16_t buf_size, uint8_t *cksum0, uint8_t *cksum1 );

class SerialLink {

private:

    // port
    Stream *_port;

    // parser
    int state = 0;
    int counter = 0;
    uint8_t cksum_lo = 0, cksum_hi = 0;

    static const uint16_t MAX_MESSAGE_LEN = 1024;

public:

    uint8_t pkt_id = 0;
    uint8_t pkt_len_lo = 0;
    uint8_t pkt_len_hi = 0;
    uint16_t pkt_len = 0;
    uint8_t payload[MAX_MESSAGE_LEN+1];

    uint32_t parse_errors = 0;

    bool open( int baud, int port );
    bool update();
    int bytes_available();
    int write_packet(uint8_t packet_id, uint8_t *payload, uint16_t len);
    bool close();
};
