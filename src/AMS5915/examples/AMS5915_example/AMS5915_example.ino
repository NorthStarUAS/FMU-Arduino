/*
AMS5915_example.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "AMS5915.h"

IntervalTimer myTimer;

// an AMS5915 object, which is a
// static pressure sensure at I2C
// address of 0x12, on I2C bus 0,
// and is a AMS5915-1200-B
AMS5915 sPress(0x27,&Wire1,AMS5915_0020_D);

int status;
volatile uint8_t data = 0;

void setup() {
  // serial to display data
  Serial.begin(9600);
  //Serial1.begin(921600);
  Serial1.begin(500000);
  while(!Serial){}

  // starting communication with the 
  // static pressure transducer
  sPress.begin();

  myTimer.begin(getData, 20000);
}

void getData() {
  data = 1;
}

void loop() {
    static uint32_t bytes_read = 0;
    while (Serial1.available()) {
        Serial1.read();
        bytes_read++;
    }

  static bool error_detected = false;
  
  if (data) {
      // read the sensor
      float pressure, temperature;
      status = sPress.getData( &pressure, &temperature);
      if ( !status ) {
          error_detected = true;
      }

      const float analogResolution = 65535.0f;
      const float pwr_scale = 11.0f;
      const float avionics_scale = 2.0f;
      const uint8_t pwr_pin = 15;
      const uint8_t avionics_pin = A22;
      uint16_t ain;
      ain = analogRead(pwr_pin);
      float pwr_v = ((float)ain) * 3.3 / analogResolution * pwr_scale;

      // marmot v1
      ain = analogRead(avionics_pin);
      float avionics_v = ((float)ain) * 3.3 / analogResolution * avionics_scale;

      // displaying the data
      Serial.print(pressure,2);
      Serial.print("\t");
      Serial.print(temperature,2);
      Serial.print("\t");
      Serial.print(error_detected);
      Serial.print("\t");
      Serial.println(bytes_read);
      data = 0;
      Serial1.write(68);
  }
}
