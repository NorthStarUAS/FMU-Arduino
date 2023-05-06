#include <Arduino.h>

#include "ratelimiter.h"

RateLimiter::RateLimiter() {
    dt_millis = 1000;
}

RateLimiter::RateLimiter( float hz ) {
    if ( hz > 0.00001 ) {
        // report true at requested rate
        dt_millis = 1000.0 / hz;
    } else if ( hz < -0.00001 ) {
        // always report false if requested rate < 0
        dt_millis = 0;
    } else {
        // always report true (assuming the clock has advanced a tick)
        dt_millis = 1;
    }
}

bool RateLimiter::update( bool verbose ) {
    // fixme: use elapsedmillis()?
    if ( timer == 0 ) {
        if ( dt_millis > 1 ) {
            timer = millis() + (random(dt_millis));
        } else {
            timer = millis();
        }
    }
    if ( verbose ) {
        printf("millis: %ld  timer: %ld  dt: %ld\n", millis(), timer, dt_millis);
    }
    if ( (dt_millis > 0) and (millis() >= timer + dt_millis) ) {
        if ( millis() > timer + dt_millis ) {
            if ( dt_millis > 1 ) {
                misses++;             // oops
            }
            timer = millis();           // catchup
        } else {
            timer += dt_millis;       // advance timer
        }
        return true;
    } else {
        return false;
    }
}
