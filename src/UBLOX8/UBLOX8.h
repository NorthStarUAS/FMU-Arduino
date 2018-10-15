/*
  UBLOX8.h
  Copyright (C) 2017 Curtis L. Olson curtolson@flightgear.org
*/

#ifndef UBLOX8_H
#define UBLOX8_H

#include "Arduino.h"				

#pragma pack(push, 1)           // set alignment to 1 byte boundary
struct ublox8_nav_pvt_t {
    uint32_t iTOW;
    int16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
    uint8_t reserved[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};
# pragma pack(pop)              // restore original alignment

class UBLOX8 {
    
private:

    HardwareSerial* _port;
    ublox8_nav_pvt_t data;
    
    bool parse_msg( uint8_t msg_class, uint8_t msg_id,
                    uint16_t payload_length, uint8_t *payload );

public:
    
    UBLOX8(HardwareSerial* port);
    void begin(int baud);
    bool read_ublox8();
    ublox8_nav_pvt_t get_data() const { return data; }
    void update_data(void *dest, int n);
};

#endif // UBLOX8_H
