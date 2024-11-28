#pragma once

// Ardupilot realloc() support
#if defined(ARDUPILOT_BUILD)
  #include <AP_HAL/AP_HAL.h>
  extern const AP_HAL::HAL& hal;
  #define REALLOC(X, Y) hal.util->std_realloc( (X), (Y) )
#else
  #define REALLOC(X, Y) std::realloc( (X), (Y) )
#endif

#include <stdint.h>  // uint8_t, et. al.
#include <stdlib.h>  // malloc() / free()
#include <string.h>  // memcpy()

#include <string>
using std::string;

#include "props2.h"  // github.com/NorthStarUAS/props2

namespace nst_message {

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t airdata_v8_id = 54;
const uint8_t airdata_v9_id = 66;
const uint8_t environment_v1_id = 70;
const uint8_t gps_v5_id = 49;
const uint8_t imu_v6_id = 50;
const uint8_t power_v2_id = 68;
const uint8_t nav_v6_id = 52;
const uint8_t nav_metrics_v6_id = 53;
const uint8_t inceptors_v2_id = 63;
const uint8_t fcs_outputs_v1_id = 71;
const uint8_t effectors_v1_id = 61;
const uint8_t fcs_refs_v1_id = 65;
const uint8_t mission_v1_id = 60;
const uint8_t status_v8_id = 69;
const uint8_t event_v3_id = 64;
const uint8_t command_v1_id = 28;
const uint8_t ack_v1_id = 57;

// Constants
static const uint8_t sbus_channels = 16;  // number of sbus channels
static const uint8_t ap_channels = 6;  // number of sbus channels

// Message: airdata_v8 (id: 54)
class airdata_v8_t {
public:

    uint32_t millis;
    float baro_press_pa;
    float diff_press_pa;
    float air_temp_C;
    float airspeed_mps;
    float altitude_agl_m;
    float altitude_true_m;
    float altitude_ground_m;
    uint8_t is_airborne;
    uint32_t flight_timer_millis;
    float wind_deg;
    float wind_mps;
    float pitot_scale_factor;
    uint16_t error_count;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint16_t baro_press_pa;
        int16_t diff_press_pa;
        int16_t air_temp_C;
        int16_t airspeed_mps;
        float altitude_agl_m;
        float altitude_true_m;
        float altitude_ground_m;
        uint8_t is_airborne;
        uint32_t flight_timer_millis;
        uint16_t wind_deg;
        uint8_t wind_mps;
        uint8_t pitot_scale_factor;
        uint16_t error_count;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 54;
    uint8_t *payload = nullptr;
    int len = 0;

    ~airdata_v8_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->baro_press_pa = uintround(baro_press_pa * 0.5);
        _buf->diff_press_pa = intround(diff_press_pa * 2.0);
        _buf->air_temp_C = intround(air_temp_C * 250.0);
        _buf->airspeed_mps = intround(airspeed_mps * 100.0);
        _buf->altitude_agl_m = altitude_agl_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->is_airborne = is_airborne;
        _buf->flight_timer_millis = flight_timer_millis;
        _buf->wind_deg = uintround(wind_deg * 100.0);
        _buf->wind_mps = uintround(wind_mps * 10.0);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100.0);
        _buf->error_count = error_count;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        baro_press_pa = _buf->baro_press_pa / (float)0.5;
        diff_press_pa = _buf->diff_press_pa / (float)2.0;
        air_temp_C = _buf->air_temp_C / (float)250.0;
        airspeed_mps = _buf->airspeed_mps / (float)100.0;
        altitude_agl_m = _buf->altitude_agl_m;
        altitude_true_m = _buf->altitude_true_m;
        altitude_ground_m = _buf->altitude_ground_m;
        is_airborne = _buf->is_airborne;
        flight_timer_millis = _buf->flight_timer_millis;
        wind_deg = _buf->wind_deg / (float)100.0;
        wind_mps = _buf->wind_mps / (float)10.0;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100.0;
        error_count = _buf->error_count;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("baro_press_pa", baro_press_pa);
        node.setDouble("diff_press_pa", diff_press_pa);
        node.setDouble("air_temp_C", air_temp_C);
        node.setDouble("airspeed_mps", airspeed_mps);
        node.setDouble("altitude_agl_m", altitude_agl_m);
        node.setDouble("altitude_true_m", altitude_true_m);
        node.setDouble("altitude_ground_m", altitude_ground_m);
        node.setUInt("is_airborne", is_airborne);
        node.setUInt("flight_timer_millis", flight_timer_millis);
        node.setDouble("wind_deg", wind_deg);
        node.setDouble("wind_mps", wind_mps);
        node.setDouble("pitot_scale_factor", pitot_scale_factor);
        node.setUInt("error_count", error_count);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        baro_press_pa = node.getDouble("baro_press_pa");
        diff_press_pa = node.getDouble("diff_press_pa");
        air_temp_C = node.getDouble("air_temp_C");
        airspeed_mps = node.getDouble("airspeed_mps");
        altitude_agl_m = node.getDouble("altitude_agl_m");
        altitude_true_m = node.getDouble("altitude_true_m");
        altitude_ground_m = node.getDouble("altitude_ground_m");
        is_airborne = node.getUInt("is_airborne");
        flight_timer_millis = node.getUInt("flight_timer_millis");
        wind_deg = node.getDouble("wind_deg");
        wind_mps = node.getDouble("wind_mps");
        pitot_scale_factor = node.getDouble("pitot_scale_factor");
        error_count = node.getUInt("error_count");
    }
};

// Message: airdata_v9 (id: 66)
class airdata_v9_t {
public:

    uint32_t millis;
    float baro_press_pa;
    float diff_press_pa;
    float air_temp_C;
    float airspeed_mps;
    float altitude_m;
    uint16_t error_count;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint16_t baro_press_pa;
        int16_t diff_press_pa;
        int16_t air_temp_C;
        int16_t airspeed_mps;
        float altitude_m;
        uint16_t error_count;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 66;
    uint8_t *payload = nullptr;
    int len = 0;

    ~airdata_v9_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->baro_press_pa = uintround(baro_press_pa * 0.5);
        _buf->diff_press_pa = intround(diff_press_pa * 2.0);
        _buf->air_temp_C = intround(air_temp_C * 250.0);
        _buf->airspeed_mps = intround(airspeed_mps * 100.0);
        _buf->altitude_m = altitude_m;
        _buf->error_count = error_count;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        baro_press_pa = _buf->baro_press_pa / (float)0.5;
        diff_press_pa = _buf->diff_press_pa / (float)2.0;
        air_temp_C = _buf->air_temp_C / (float)250.0;
        airspeed_mps = _buf->airspeed_mps / (float)100.0;
        altitude_m = _buf->altitude_m;
        error_count = _buf->error_count;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("baro_press_pa", baro_press_pa);
        node.setDouble("diff_press_pa", diff_press_pa);
        node.setDouble("air_temp_C", air_temp_C);
        node.setDouble("airspeed_mps", airspeed_mps);
        node.setDouble("altitude_m", altitude_m);
        node.setUInt("error_count", error_count);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        baro_press_pa = node.getDouble("baro_press_pa");
        diff_press_pa = node.getDouble("diff_press_pa");
        air_temp_C = node.getDouble("air_temp_C");
        airspeed_mps = node.getDouble("airspeed_mps");
        altitude_m = node.getDouble("altitude_m");
        error_count = node.getUInt("error_count");
    }
};

// Message: environment_v1 (id: 70)
class environment_v1_t {
public:

    uint32_t millis;
    float altitude_agl_m;
    float altitude_true_m;
    float altitude_ground_m;
    uint8_t is_airborne;
    uint32_t flight_timer_millis;
    float wind_deg;
    float wind_mps;
    float pitot_scale_factor;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        float altitude_agl_m;
        float altitude_true_m;
        float altitude_ground_m;
        uint8_t is_airborne;
        uint32_t flight_timer_millis;
        uint16_t wind_deg;
        uint8_t wind_mps;
        uint8_t pitot_scale_factor;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 70;
    uint8_t *payload = nullptr;
    int len = 0;

    ~environment_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->altitude_agl_m = altitude_agl_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->is_airborne = is_airborne;
        _buf->flight_timer_millis = flight_timer_millis;
        _buf->wind_deg = uintround(wind_deg * 100.0);
        _buf->wind_mps = uintround(wind_mps * 10.0);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        altitude_agl_m = _buf->altitude_agl_m;
        altitude_true_m = _buf->altitude_true_m;
        altitude_ground_m = _buf->altitude_ground_m;
        is_airborne = _buf->is_airborne;
        flight_timer_millis = _buf->flight_timer_millis;
        wind_deg = _buf->wind_deg / (float)100.0;
        wind_mps = _buf->wind_mps / (float)10.0;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("altitude_agl_m", altitude_agl_m);
        node.setDouble("altitude_true_m", altitude_true_m);
        node.setDouble("altitude_ground_m", altitude_ground_m);
        node.setUInt("is_airborne", is_airborne);
        node.setUInt("flight_timer_millis", flight_timer_millis);
        node.setDouble("wind_deg", wind_deg);
        node.setDouble("wind_mps", wind_mps);
        node.setDouble("pitot_scale_factor", pitot_scale_factor);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        altitude_agl_m = node.getDouble("altitude_agl_m");
        altitude_true_m = node.getDouble("altitude_true_m");
        altitude_ground_m = node.getDouble("altitude_ground_m");
        is_airborne = node.getUInt("is_airborne");
        flight_timer_millis = node.getUInt("flight_timer_millis");
        wind_deg = node.getDouble("wind_deg");
        wind_mps = node.getDouble("wind_mps");
        pitot_scale_factor = node.getDouble("pitot_scale_factor");
    }
};

// Message: gps_v5 (id: 49)
class gps_v5_t {
public:

    uint32_t millis;
    uint64_t unix_usec;
    uint8_t num_sats;
    uint8_t status;
    int32_t longitude_raw;
    int32_t latitude_raw;
    float altitude_m;
    float vn_mps;
    float ve_mps;
    float vd_mps;
    float hAcc_m;
    float vAcc_m;
    float hdop;
    float vdop;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint64_t unix_usec;
        uint8_t num_sats;
        uint8_t status;
        int32_t longitude_raw;
        int32_t latitude_raw;
        float altitude_m;
        int16_t vn_mps;
        int16_t ve_mps;
        int16_t vd_mps;
        int16_t hAcc_m;
        int16_t vAcc_m;
        int16_t hdop;
        int16_t vdop;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 49;
    uint8_t *payload = nullptr;
    int len = 0;

    ~gps_v5_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->unix_usec = unix_usec;
        _buf->num_sats = num_sats;
        _buf->status = status;
        _buf->longitude_raw = longitude_raw;
        _buf->latitude_raw = latitude_raw;
        _buf->altitude_m = altitude_m;
        _buf->vn_mps = intround(vn_mps * 100.0);
        _buf->ve_mps = intround(ve_mps * 100.0);
        _buf->vd_mps = intround(vd_mps * 100.0);
        _buf->hAcc_m = intround(hAcc_m * 100.0);
        _buf->vAcc_m = intround(vAcc_m * 100.0);
        _buf->hdop = intround(hdop * 100.0);
        _buf->vdop = intround(vdop * 100.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        unix_usec = _buf->unix_usec;
        num_sats = _buf->num_sats;
        status = _buf->status;
        longitude_raw = _buf->longitude_raw;
        latitude_raw = _buf->latitude_raw;
        altitude_m = _buf->altitude_m;
        vn_mps = _buf->vn_mps / (float)100.0;
        ve_mps = _buf->ve_mps / (float)100.0;
        vd_mps = _buf->vd_mps / (float)100.0;
        hAcc_m = _buf->hAcc_m / (float)100.0;
        vAcc_m = _buf->vAcc_m / (float)100.0;
        hdop = _buf->hdop / (float)100.0;
        vdop = _buf->vdop / (float)100.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setUInt64("unix_usec", unix_usec);
        node.setUInt("num_sats", num_sats);
        node.setUInt("status", status);
        node.setInt("longitude_raw", longitude_raw);
        node.setInt("latitude_raw", latitude_raw);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_mps", vn_mps);
        node.setDouble("ve_mps", ve_mps);
        node.setDouble("vd_mps", vd_mps);
        node.setDouble("hAcc_m", hAcc_m);
        node.setDouble("vAcc_m", vAcc_m);
        node.setDouble("hdop", hdop);
        node.setDouble("vdop", vdop);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        unix_usec = node.getUInt64("unix_usec");
        num_sats = node.getUInt("num_sats");
        status = node.getUInt("status");
        longitude_raw = node.getInt("longitude_raw");
        latitude_raw = node.getInt("latitude_raw");
        altitude_m = node.getDouble("altitude_m");
        vn_mps = node.getDouble("vn_mps");
        ve_mps = node.getDouble("ve_mps");
        vd_mps = node.getDouble("vd_mps");
        hAcc_m = node.getDouble("hAcc_m");
        vAcc_m = node.getDouble("vAcc_m");
        hdop = node.getDouble("hdop");
        vdop = node.getDouble("vdop");
    }
};

// Message: imu_v6 (id: 50)
class imu_v6_t {
public:

    uint32_t millis;
    float ax_raw;
    float ay_raw;
    float az_raw;
    float hx_raw;
    float hy_raw;
    float hz_raw;
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float p_rps;
    float q_rps;
    float r_rps;
    float hx;
    float hy;
    float hz;
    float temp_C;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t ax_raw;
        int16_t ay_raw;
        int16_t az_raw;
        int16_t hx_raw;
        int16_t hy_raw;
        int16_t hz_raw;
        int16_t ax_mps2;
        int16_t ay_mps2;
        int16_t az_mps2;
        int16_t p_rps;
        int16_t q_rps;
        int16_t r_rps;
        int16_t hx;
        int16_t hy;
        int16_t hz;
        int16_t temp_C;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 50;
    uint8_t *payload = nullptr;
    int len = 0;

    ~imu_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->ax_raw = intround(ax_raw * 835.296217);
        _buf->ay_raw = intround(ay_raw * 835.296217);
        _buf->az_raw = intround(az_raw * 835.296217);
        _buf->hx_raw = intround(hx_raw * 30000.0);
        _buf->hy_raw = intround(hy_raw * 30000.0);
        _buf->hz_raw = intround(hz_raw * 30000.0);
        _buf->ax_mps2 = intround(ax_mps2 * 835.296217);
        _buf->ay_mps2 = intround(ay_mps2 * 835.296217);
        _buf->az_mps2 = intround(az_mps2 * 835.296217);
        _buf->p_rps = intround(p_rps * 3754.82165);
        _buf->q_rps = intround(q_rps * 3754.82165);
        _buf->r_rps = intround(r_rps * 3754.82165);
        _buf->hx = intround(hx * 30000.0);
        _buf->hy = intround(hy * 30000.0);
        _buf->hz = intround(hz * 30000.0);
        _buf->temp_C = intround(temp_C * 250.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        ax_raw = _buf->ax_raw / (float)835.296217;
        ay_raw = _buf->ay_raw / (float)835.296217;
        az_raw = _buf->az_raw / (float)835.296217;
        hx_raw = _buf->hx_raw / (float)30000.0;
        hy_raw = _buf->hy_raw / (float)30000.0;
        hz_raw = _buf->hz_raw / (float)30000.0;
        ax_mps2 = _buf->ax_mps2 / (float)835.296217;
        ay_mps2 = _buf->ay_mps2 / (float)835.296217;
        az_mps2 = _buf->az_mps2 / (float)835.296217;
        p_rps = _buf->p_rps / (float)3754.82165;
        q_rps = _buf->q_rps / (float)3754.82165;
        r_rps = _buf->r_rps / (float)3754.82165;
        hx = _buf->hx / (float)30000.0;
        hy = _buf->hy / (float)30000.0;
        hz = _buf->hz / (float)30000.0;
        temp_C = _buf->temp_C / (float)250.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("ax_raw", ax_raw);
        node.setDouble("ay_raw", ay_raw);
        node.setDouble("az_raw", az_raw);
        node.setDouble("hx_raw", hx_raw);
        node.setDouble("hy_raw", hy_raw);
        node.setDouble("hz_raw", hz_raw);
        node.setDouble("ax_mps2", ax_mps2);
        node.setDouble("ay_mps2", ay_mps2);
        node.setDouble("az_mps2", az_mps2);
        node.setDouble("p_rps", p_rps);
        node.setDouble("q_rps", q_rps);
        node.setDouble("r_rps", r_rps);
        node.setDouble("hx", hx);
        node.setDouble("hy", hy);
        node.setDouble("hz", hz);
        node.setDouble("temp_C", temp_C);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        ax_raw = node.getDouble("ax_raw");
        ay_raw = node.getDouble("ay_raw");
        az_raw = node.getDouble("az_raw");
        hx_raw = node.getDouble("hx_raw");
        hy_raw = node.getDouble("hy_raw");
        hz_raw = node.getDouble("hz_raw");
        ax_mps2 = node.getDouble("ax_mps2");
        ay_mps2 = node.getDouble("ay_mps2");
        az_mps2 = node.getDouble("az_mps2");
        p_rps = node.getDouble("p_rps");
        q_rps = node.getDouble("q_rps");
        r_rps = node.getDouble("r_rps");
        hx = node.getDouble("hx");
        hy = node.getDouble("hy");
        hz = node.getDouble("hz");
        temp_C = node.getDouble("temp_C");
    }
};

// Message: power_v2 (id: 68)
class power_v2_t {
public:

    uint32_t millis;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float pwm_vcc;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t pwm_vcc;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 68;
    uint8_t *payload = nullptr;
    int len = 0;

    ~power_v2_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->avionics_vcc = uintround(avionics_vcc * 1000.0);
        _buf->main_vcc = uintround(main_vcc * 1000.0);
        _buf->cell_vcc = uintround(cell_vcc * 1000.0);
        _buf->pwm_vcc = uintround(pwm_vcc * 1000.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        avionics_vcc = _buf->avionics_vcc / (float)1000.0;
        main_vcc = _buf->main_vcc / (float)1000.0;
        cell_vcc = _buf->cell_vcc / (float)1000.0;
        pwm_vcc = _buf->pwm_vcc / (float)1000.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("avionics_vcc", avionics_vcc);
        node.setDouble("main_vcc", main_vcc);
        node.setDouble("cell_vcc", cell_vcc);
        node.setDouble("pwm_vcc", pwm_vcc);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        avionics_vcc = node.getDouble("avionics_vcc");
        main_vcc = node.getDouble("main_vcc");
        cell_vcc = node.getDouble("cell_vcc");
        pwm_vcc = node.getDouble("pwm_vcc");
    }
};

// Message: nav_v6 (id: 52)
class nav_v6_t {
public:

    uint32_t millis;
    int32_t latitude_raw;
    int32_t longitude_raw;
    float altitude_m;
    float vn_mps;
    float ve_mps;
    float vd_mps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int32_t latitude_raw;
        int32_t longitude_raw;
        float altitude_m;
        int16_t vn_mps;
        int16_t ve_mps;
        int16_t vd_mps;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        uint8_t sequence_num;
        uint8_t status;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 52;
    uint8_t *payload = nullptr;
    int len = 0;

    ~nav_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->latitude_raw = latitude_raw;
        _buf->longitude_raw = longitude_raw;
        _buf->altitude_m = altitude_m;
        _buf->vn_mps = intround(vn_mps * 100.0);
        _buf->ve_mps = intround(ve_mps * 100.0);
        _buf->vd_mps = intround(vd_mps * 100.0);
        _buf->roll_deg = intround(roll_deg * 50.0);
        _buf->pitch_deg = intround(pitch_deg * 50.0);
        _buf->yaw_deg = intround(yaw_deg * 50.0);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        latitude_raw = _buf->latitude_raw;
        longitude_raw = _buf->longitude_raw;
        altitude_m = _buf->altitude_m;
        vn_mps = _buf->vn_mps / (float)100.0;
        ve_mps = _buf->ve_mps / (float)100.0;
        vd_mps = _buf->vd_mps / (float)100.0;
        roll_deg = _buf->roll_deg / (float)50.0;
        pitch_deg = _buf->pitch_deg / (float)50.0;
        yaw_deg = _buf->yaw_deg / (float)50.0;
        sequence_num = _buf->sequence_num;
        status = _buf->status;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setInt("latitude_raw", latitude_raw);
        node.setInt("longitude_raw", longitude_raw);
        node.setDouble("altitude_m", altitude_m);
        node.setDouble("vn_mps", vn_mps);
        node.setDouble("ve_mps", ve_mps);
        node.setDouble("vd_mps", vd_mps);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("pitch_deg", pitch_deg);
        node.setDouble("yaw_deg", yaw_deg);
        node.setUInt("sequence_num", sequence_num);
        node.setUInt("status", status);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        latitude_raw = node.getInt("latitude_raw");
        longitude_raw = node.getInt("longitude_raw");
        altitude_m = node.getDouble("altitude_m");
        vn_mps = node.getDouble("vn_mps");
        ve_mps = node.getDouble("ve_mps");
        vd_mps = node.getDouble("vd_mps");
        roll_deg = node.getDouble("roll_deg");
        pitch_deg = node.getDouble("pitch_deg");
        yaw_deg = node.getDouble("yaw_deg");
        sequence_num = node.getUInt("sequence_num");
        status = node.getUInt("status");
    }
};

// Message: nav_metrics_v6 (id: 53)
class nav_metrics_v6_t {
public:

    uint32_t metrics_millis;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    float Pp0;
    float Pp1;
    float Pp2;
    float Pv0;
    float Pv1;
    float Pv2;
    float Pa0;
    float Pa1;
    float Pa2;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t metrics_millis;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint16_t Pp0;
        uint16_t Pp1;
        uint16_t Pp2;
        uint16_t Pv0;
        uint16_t Pv1;
        uint16_t Pv2;
        uint16_t Pa0;
        uint16_t Pa1;
        uint16_t Pa2;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 53;
    uint8_t *payload = nullptr;
    int len = 0;

    ~nav_metrics_v6_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->metrics_millis = metrics_millis;
        _buf->p_bias = intround(p_bias * 10000.0);
        _buf->q_bias = intround(q_bias * 10000.0);
        _buf->r_bias = intround(r_bias * 10000.0);
        _buf->ax_bias = intround(ax_bias * 1000.0);
        _buf->ay_bias = intround(ay_bias * 1000.0);
        _buf->az_bias = intround(az_bias * 1000.0);
        _buf->Pp0 = uintround(Pp0 * 100.0);
        _buf->Pp1 = uintround(Pp1 * 100.0);
        _buf->Pp2 = uintround(Pp2 * 100.0);
        _buf->Pv0 = uintround(Pv0 * 1000.0);
        _buf->Pv1 = uintround(Pv1 * 1000.0);
        _buf->Pv2 = uintround(Pv2 * 1000.0);
        _buf->Pa0 = uintround(Pa0 * 10000.0);
        _buf->Pa1 = uintround(Pa1 * 10000.0);
        _buf->Pa2 = uintround(Pa2 * 10000.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        metrics_millis = _buf->metrics_millis;
        p_bias = _buf->p_bias / (float)10000.0;
        q_bias = _buf->q_bias / (float)10000.0;
        r_bias = _buf->r_bias / (float)10000.0;
        ax_bias = _buf->ax_bias / (float)1000.0;
        ay_bias = _buf->ay_bias / (float)1000.0;
        az_bias = _buf->az_bias / (float)1000.0;
        Pp0 = _buf->Pp0 / (float)100.0;
        Pp1 = _buf->Pp1 / (float)100.0;
        Pp2 = _buf->Pp2 / (float)100.0;
        Pv0 = _buf->Pv0 / (float)1000.0;
        Pv1 = _buf->Pv1 / (float)1000.0;
        Pv2 = _buf->Pv2 / (float)1000.0;
        Pa0 = _buf->Pa0 / (float)10000.0;
        Pa1 = _buf->Pa1 / (float)10000.0;
        Pa2 = _buf->Pa2 / (float)10000.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("metrics_millis", metrics_millis);
        node.setDouble("p_bias", p_bias);
        node.setDouble("q_bias", q_bias);
        node.setDouble("r_bias", r_bias);
        node.setDouble("ax_bias", ax_bias);
        node.setDouble("ay_bias", ay_bias);
        node.setDouble("az_bias", az_bias);
        node.setDouble("Pp0", Pp0);
        node.setDouble("Pp1", Pp1);
        node.setDouble("Pp2", Pp2);
        node.setDouble("Pv0", Pv0);
        node.setDouble("Pv1", Pv1);
        node.setDouble("Pv2", Pv2);
        node.setDouble("Pa0", Pa0);
        node.setDouble("Pa1", Pa1);
        node.setDouble("Pa2", Pa2);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        metrics_millis = node.getUInt("metrics_millis");
        p_bias = node.getDouble("p_bias");
        q_bias = node.getDouble("q_bias");
        r_bias = node.getDouble("r_bias");
        ax_bias = node.getDouble("ax_bias");
        ay_bias = node.getDouble("ay_bias");
        az_bias = node.getDouble("az_bias");
        Pp0 = node.getDouble("Pp0");
        Pp1 = node.getDouble("Pp1");
        Pp2 = node.getDouble("Pp2");
        Pv0 = node.getDouble("Pv0");
        Pv1 = node.getDouble("Pv1");
        Pv2 = node.getDouble("Pv2");
        Pa0 = node.getDouble("Pa0");
        Pa1 = node.getDouble("Pa1");
        Pa2 = node.getDouble("Pa2");
    }
};

// Message: inceptors_v2 (id: 63)
class inceptors_v2_t {
public:

    uint32_t millis;
    float roll;
    float pitch;
    float yaw;
    float power;
    float flaps;
    uint8_t gear;
    float aux1;
    float aux2;
    uint8_t master_switch;
    uint8_t motor_enable;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        uint16_t power;
        uint16_t flaps;
        uint8_t gear;
        int16_t aux1;
        int16_t aux2;
        uint8_t master_switch;
        uint8_t motor_enable;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 63;
    uint8_t *payload = nullptr;
    int len = 0;

    ~inceptors_v2_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->roll = intround(roll * 30000.0);
        _buf->pitch = intround(pitch * 30000.0);
        _buf->yaw = intround(yaw * 30000.0);
        _buf->power = uintround(power * 60000.0);
        _buf->flaps = uintround(flaps * 60000.0);
        _buf->gear = gear;
        _buf->aux1 = intround(aux1 * 30000.0);
        _buf->aux2 = intround(aux2 * 30000.0);
        _buf->master_switch = master_switch;
        _buf->motor_enable = motor_enable;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        roll = _buf->roll / (float)30000.0;
        pitch = _buf->pitch / (float)30000.0;
        yaw = _buf->yaw / (float)30000.0;
        power = _buf->power / (float)60000.0;
        flaps = _buf->flaps / (float)60000.0;
        gear = _buf->gear;
        aux1 = _buf->aux1 / (float)30000.0;
        aux2 = _buf->aux2 / (float)30000.0;
        master_switch = _buf->master_switch;
        motor_enable = _buf->motor_enable;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("roll", roll);
        node.setDouble("pitch", pitch);
        node.setDouble("yaw", yaw);
        node.setDouble("power", power);
        node.setDouble("flaps", flaps);
        node.setUInt("gear", gear);
        node.setDouble("aux1", aux1);
        node.setDouble("aux2", aux2);
        node.setUInt("master_switch", master_switch);
        node.setUInt("motor_enable", motor_enable);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        roll = node.getDouble("roll");
        pitch = node.getDouble("pitch");
        yaw = node.getDouble("yaw");
        power = node.getDouble("power");
        flaps = node.getDouble("flaps");
        gear = node.getUInt("gear");
        aux1 = node.getDouble("aux1");
        aux2 = node.getDouble("aux2");
        master_switch = node.getUInt("master_switch");
        motor_enable = node.getUInt("motor_enable");
    }
};

// Message: fcs_outputs_v1 (id: 71)
class fcs_outputs_v1_t {
public:

    uint32_t millis;
    float roll;
    float pitch;
    float yaw;
    float power;
    float flaps;
    uint8_t gear;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        uint16_t power;
        uint16_t flaps;
        uint8_t gear;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 71;
    uint8_t *payload = nullptr;
    int len = 0;

    ~fcs_outputs_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->roll = intround(roll * 30000.0);
        _buf->pitch = intround(pitch * 30000.0);
        _buf->yaw = intround(yaw * 30000.0);
        _buf->power = uintround(power * 60000.0);
        _buf->flaps = uintround(flaps * 60000.0);
        _buf->gear = gear;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        roll = _buf->roll / (float)30000.0;
        pitch = _buf->pitch / (float)30000.0;
        yaw = _buf->yaw / (float)30000.0;
        power = _buf->power / (float)60000.0;
        flaps = _buf->flaps / (float)60000.0;
        gear = _buf->gear;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("roll", roll);
        node.setDouble("pitch", pitch);
        node.setDouble("yaw", yaw);
        node.setDouble("power", power);
        node.setDouble("flaps", flaps);
        node.setUInt("gear", gear);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        roll = node.getDouble("roll");
        pitch = node.getDouble("pitch");
        yaw = node.getDouble("yaw");
        power = node.getDouble("power");
        flaps = node.getDouble("flaps");
        gear = node.getUInt("gear");
    }
};

// Message: effectors_v1 (id: 61)
class effectors_v1_t {
public:

    uint32_t millis;
    float channel[8];

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t channel[8];
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 61;
    uint8_t *payload = nullptr;
    int len = 0;

    ~effectors_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        for (int _i=0; _i<8; _i++) _buf->channel[_i] = intround(channel[_i] * 20000.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf->channel[_i] / (float)20000.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        for (int _i=0; _i<8; _i++) node.setDouble("channel", channel[_i], _i);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        for (int _i=0; _i<8; _i++) channel[_i] = node.getDouble("channel", _i);
    }
};

// Message: fcs_refs_v1 (id: 65)
class fcs_refs_v1_t {
public:

    uint32_t millis;
    float groundtrack_deg;
    float altitude_agl_ft;
    float airspeed_kt;
    float roll_deg;
    float pitch_deg;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        int16_t groundtrack_deg;
        uint16_t altitude_agl_ft;
        int16_t airspeed_kt;
        int16_t roll_deg;
        int16_t pitch_deg;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 65;
    uint8_t *payload = nullptr;
    int len = 0;

    ~fcs_refs_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10.0);
        _buf->altitude_agl_ft = uintround(altitude_agl_ft * 10.0);
        _buf->airspeed_kt = intround(airspeed_kt * 10.0);
        _buf->roll_deg = intround(roll_deg * 10.0);
        _buf->pitch_deg = intround(pitch_deg * 10.0);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        groundtrack_deg = _buf->groundtrack_deg / (float)10.0;
        altitude_agl_ft = _buf->altitude_agl_ft / (float)10.0;
        airspeed_kt = _buf->airspeed_kt / (float)10.0;
        roll_deg = _buf->roll_deg / (float)10.0;
        pitch_deg = _buf->pitch_deg / (float)10.0;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setDouble("groundtrack_deg", groundtrack_deg);
        node.setDouble("altitude_agl_ft", altitude_agl_ft);
        node.setDouble("airspeed_kt", airspeed_kt);
        node.setDouble("roll_deg", roll_deg);
        node.setDouble("pitch_deg", pitch_deg);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        groundtrack_deg = node.getDouble("groundtrack_deg");
        altitude_agl_ft = node.getDouble("altitude_agl_ft");
        airspeed_kt = node.getDouble("airspeed_kt");
        roll_deg = node.getDouble("roll_deg");
        pitch_deg = node.getDouble("pitch_deg");
    }
};

// Message: mission_v1 (id: 60)
class mission_v1_t {
public:

    uint32_t millis;
    string task_name;
    uint16_t task_attribute;
    uint16_t route_size;
    uint16_t target_wpt_idx;
    uint16_t wpt_index;
    int32_t wpt_longitude_raw;
    int32_t wpt_latitude_raw;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint16_t task_name_len;
        uint16_t task_attribute;
        uint16_t route_size;
        uint16_t target_wpt_idx;
        uint16_t wpt_index;
        int32_t wpt_longitude_raw;
        int32_t wpt_latitude_raw;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 60;
    uint8_t *payload = nullptr;
    int len = 0;

    ~mission_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += task_name.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->task_name_len = task_name.length();
        _buf->task_attribute = task_attribute;
        _buf->route_size = route_size;
        _buf->target_wpt_idx = target_wpt_idx;
        _buf->wpt_index = wpt_index;
        _buf->wpt_longitude_raw = wpt_longitude_raw;
        _buf->wpt_latitude_raw = wpt_latitude_raw;
        memcpy(&(payload[len]), task_name.c_str(), task_name.length());
        len += task_name.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        task_attribute = _buf->task_attribute;
        route_size = _buf->route_size;
        target_wpt_idx = _buf->target_wpt_idx;
        wpt_index = _buf->wpt_index;
        wpt_longitude_raw = _buf->wpt_longitude_raw;
        wpt_latitude_raw = _buf->wpt_latitude_raw;
        task_name = string((char *)&(external_message[len]), _buf->task_name_len);
        len += _buf->task_name_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setString("task_name", task_name);
        node.setUInt("task_attribute", task_attribute);
        node.setUInt("route_size", route_size);
        node.setUInt("target_wpt_idx", target_wpt_idx);
        node.setUInt("wpt_index", wpt_index);
        node.setInt("wpt_longitude_raw", wpt_longitude_raw);
        node.setInt("wpt_latitude_raw", wpt_latitude_raw);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        task_name = node.getString("task_name");
        task_attribute = node.getUInt("task_attribute");
        route_size = node.getUInt("route_size");
        target_wpt_idx = node.getUInt("target_wpt_idx");
        wpt_index = node.getUInt("wpt_index");
        wpt_longitude_raw = node.getInt("wpt_longitude_raw");
        wpt_latitude_raw = node.getInt("wpt_latitude_raw");
    }
};

// Message: status_v8 (id: 69)
class status_v8_t {
public:

    uint32_t millis;
    uint16_t serial_number;
    uint16_t firmware_rev;
    uint8_t master_hz;
    uint32_t baud;
    uint32_t available_memory;
    uint16_t link_state;
    uint16_t byte_rate;
    uint16_t main_loop_timer_misses;
    uint16_t max_log_buf;
    uint16_t log_buf_overruns;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint16_t serial_number;
        uint16_t firmware_rev;
        uint8_t master_hz;
        uint32_t baud;
        uint32_t available_memory;
        uint16_t link_state;
        uint16_t byte_rate;
        uint16_t main_loop_timer_misses;
        uint16_t max_log_buf;
        uint16_t log_buf_overruns;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 69;
    uint8_t *payload = nullptr;
    int len = 0;

    ~status_v8_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->serial_number = serial_number;
        _buf->firmware_rev = firmware_rev;
        _buf->master_hz = master_hz;
        _buf->baud = baud;
        _buf->available_memory = available_memory;
        _buf->link_state = link_state;
        _buf->byte_rate = byte_rate;
        _buf->main_loop_timer_misses = main_loop_timer_misses;
        _buf->max_log_buf = max_log_buf;
        _buf->log_buf_overruns = log_buf_overruns;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        serial_number = _buf->serial_number;
        firmware_rev = _buf->firmware_rev;
        master_hz = _buf->master_hz;
        baud = _buf->baud;
        available_memory = _buf->available_memory;
        link_state = _buf->link_state;
        byte_rate = _buf->byte_rate;
        main_loop_timer_misses = _buf->main_loop_timer_misses;
        max_log_buf = _buf->max_log_buf;
        log_buf_overruns = _buf->log_buf_overruns;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setUInt("serial_number", serial_number);
        node.setUInt("firmware_rev", firmware_rev);
        node.setUInt("master_hz", master_hz);
        node.setUInt("baud", baud);
        node.setUInt("available_memory", available_memory);
        node.setUInt("link_state", link_state);
        node.setUInt("byte_rate", byte_rate);
        node.setUInt("main_loop_timer_misses", main_loop_timer_misses);
        node.setUInt("max_log_buf", max_log_buf);
        node.setUInt("log_buf_overruns", log_buf_overruns);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        serial_number = node.getUInt("serial_number");
        firmware_rev = node.getUInt("firmware_rev");
        master_hz = node.getUInt("master_hz");
        baud = node.getUInt("baud");
        available_memory = node.getUInt("available_memory");
        link_state = node.getUInt("link_state");
        byte_rate = node.getUInt("byte_rate");
        main_loop_timer_misses = node.getUInt("main_loop_timer_misses");
        max_log_buf = node.getUInt("max_log_buf");
        log_buf_overruns = node.getUInt("log_buf_overruns");
    }
};

// Message: event_v3 (id: 64)
class event_v3_t {
public:

    uint32_t millis;
    string message;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint32_t millis;
        uint16_t message_len;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 64;
    uint8_t *payload = nullptr;
    int len = 0;

    ~event_v3_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += message.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->millis = millis;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        millis = _buf->millis;
        message = string((char *)&(external_message[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("millis", millis);
        node.setString("message", message);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        millis = node.getUInt("millis");
        message = node.getString("message");
    }
};

// Message: command_v1 (id: 28)
class command_v1_t {
public:

    uint16_t sequence_num;
    string message;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t sequence_num;
        uint16_t message_len;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 28;
    uint8_t *payload = nullptr;
    int len = 0;

    ~command_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        size += message.length();
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->sequence_num = sequence_num;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        sequence_num = _buf->sequence_num;
        message = string((char *)&(external_message[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("sequence_num", sequence_num);
        node.setString("message", message);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        sequence_num = node.getUInt("sequence_num");
        message = node.getString("message");
    }
};

// Message: ack_v1 (id: 57)
class ack_v1_t {
public:

    uint16_t sequence_num;
    uint8_t result;

    // internal structure for packing
    #pragma pack(push, 1)
    struct _compact_t {
        uint16_t sequence_num;
        uint8_t result;
    };
    #pragma pack(pop)

    // id, ptr to payload and len
    static const uint8_t id = 57;
    uint8_t *payload = nullptr;
    int len = 0;

    ~ack_v1_t() {
        free(payload);
    }

    bool pack() {
        len = sizeof(_compact_t);
        // compute dynamic packet size (if neede)
        int size = len;
        payload = (uint8_t *)REALLOC(payload, size);
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->sequence_num = sequence_num;
        _buf->result = result;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        _compact_t *_buf = (_compact_t *)external_message;
        len = sizeof(_compact_t);
        sequence_num = _buf->sequence_num;
        result = _buf->result;
        return true;
    }

    void msg2props(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        msg2props(node);
    }

    void msg2props(PropertyNode &node) {
        node.setUInt("sequence_num", sequence_num);
        node.setUInt("result", result);
    }

    void props2msg(string _path, int _index = -1) {
        if ( _index >= 0 ) {
            _path += "/" + std::to_string(_index);
        }
        PropertyNode node(_path.c_str());
        props2msg(node);
    }

    void props2msg(PropertyNode &node) {
        sequence_num = node.getUInt("sequence_num");
        result = node.getUInt("result");
    }
};

} // namespace nst_message
