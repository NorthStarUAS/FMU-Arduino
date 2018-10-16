void led_setup() {
    if ( config.led.pin > 0 ) {
        pinMode(config.led.pin, OUTPUT);
        digitalWrite(config.led.pin, HIGH);
        Serial.print("LED on pin: "); Serial.println(config.led.pin);
    } else {
        Serial.println("No LED defined.");
    }
}

void led_update() {
    if ( config.led.pin > 0 ) {
        static elapsedMillis blinkTimer = 0;
        static unsigned int blink_rate = 100;
        static bool blink_state = true;

        if ( gyros_calibrated < 2 ) {
            blink_rate = 50;
        } else if ( gps_data.fixType < 3 ) {
            blink_rate = 200;
        } else {
            blink_rate = 800;
        }
        if ( blinkTimer >= blink_rate ) {
            blinkTimer = 0;
            blink_state = !blink_state;
            digitalWrite(config.led.pin, blink_state);
        }
    }
}
        
