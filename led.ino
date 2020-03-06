void led_setup() {
    if ( config_led.pin > 0 ) {
        pinMode(config_led.pin, OUTPUT);
        digitalWrite(config_led.pin, HIGH);
        Serial.print("LED on pin: "); Serial.println(config_led.pin);
    } else {
        Serial.println("No LED defined.");
    }
}

void led_update() {
    if ( config_led.pin > 0 ) {
        static elapsedMillis blinkTimer = 0;
        static unsigned int blink_rate = 100;
        static bool blink_state = true;

        if ( imu.gyros_calibrated < 2 ) {
            blink_rate = 50;
        } else if ( gps_data.fixType < 3 ) {
            blink_rate = 200;
        } else {
            blink_rate = 800;
        }
        if ( blinkTimer >= blink_rate ) {
            blinkTimer = 0;
            blink_state = !blink_state;
            digitalWrite(config_led.pin, blink_state);
        }
    }
}
        
