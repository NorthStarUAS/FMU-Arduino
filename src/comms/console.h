#pragma once

#include "../util/ratelimiter.h"

#include "message_link.h"

class console_t {

private:

    uint8_t reboot_count = 0;
    const char *reboot_cmd = "reboot";
    message_link_t console_link;
    bool interactive = true;
    RateLimiter info_timer;

    void display_menu();

public:

    bool display_inceptors = false;
    bool display_gps = false;
    bool display_airdata = false;
    bool display_imu = false;
    bool display_nav = false;
    bool display_nav_stats = false;
    bool display_act = false;

    void init();
    void update();
};
