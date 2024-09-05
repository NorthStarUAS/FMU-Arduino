// manage message-based comms and relays

#pragma once

#include "../util/ratelimiter.h"

#include "console.h"
#include "events.h"
#include "data_logger.h"
#include "message_link.h"
#include "packer.h"

class comms_mgr_t {

private:

    message_link_t gcs_link;
    message_link_t host_link;
    console_t console;

    RateLimiter status;
    uint32_t tempTimer;  // fixme use elapsedmillis?
    uint32_t counter;
    float lost_link_timeout_sec = 30.0;

public:

    packer_t packer;
    data_logger_t data_logger;

    void init();
    void update();

};

extern comms_mgr_t *comms_mgr;
