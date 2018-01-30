# AMS5915
Library for communicating with [Analog Microelectronics AMS 5915](http://www.analog-micro.com/en/products/sensors/pressuresensors/ams5915/) pressure transducers using Teensy 3.x and Teensy LC devices.

# Description
The Analog Microelectronics AMS 5915 pressure transducers are fully signal conditioned, amplified, and temperature compensated over a temperature range of -25 to +85 C. These sensors generate data with high precision, high stability and low drift. Digital measurements are sampled with a 14 bit resolution. The AMS 5915 sensors are available in a wide variety of pressure ranges and in configurations suited for barometric, differential, and bidirectional differential measurement.

This library communicates with the AMS 5915 sensors using an I2C interface. The default I2C address for the AMS 5915 is 0x28; however, a USB starter kit may be purchased to enable programming additional slave addresses. Pressure and temperature data can be provided up to a rate of 2 kHz.

# Usage
This library uses the [i2c_t3 enhanced I2C library](https://github.com/nox771/i2c_t3) for Teensy 3.x/LC devices.

Simply clone or download and extract the zipped library into your Arduino/libraries folder. The [i2c_t3 enhanced I2C library](https://github.com/nox771/i2c_t3) is bundled with the [Teensyduino software](http://pjrc.com/teensy/td_download.html) and is not required to download separately.

**AMS5915(uint8_t address, uint8_t bus, ams5915_transducer type)**
An AMS5915 object should be declared, specifying the AMS 5915 I2C address, the I2C bus, and the AMS 5915 sensor type. The enumerated transducer types are:

| Sensor Name       | Enumerated Type  | Pressure Type              | Pressure Range       |
| -----------       | ---------------  | ---------------            | ---------------      |
| AMS 5915-0005-D   | AMS5915_0005_D   | differential / relative    | 0...500 Pa           |
| AMS 5915-0010-D   | AMS5915_0010_D   | differential / relative    | 0...1000 Pa          |
| AMS 5915-0005-D-B | AMS5915_0005_D_B | bidirectional differential | -500...+500 Pa       |
| AMS 5915-0010-D-B | AMS5915_0010_D_B | bidirectional differential | -1000...+1000 Pa     |
| AMS 5915-0020-D   | AMS5915_0020_D   | differential / relative    | 0...2000 Pa          |
| AMS 5915-0050-D   | AMS5915_0050_D   | differential / relative    | 0...5000 Pa          |
| AMS 5915-0100-D   | AMS5915_0100_D   | differential / relative    | 0...10000 Pa         |
| AMS 5915-0020-D-B | AMS5915_0020_D_B | bidirectional differential | -2000...+2000 Pa     |
| AMS 5915-0050-D-B | AMS5915_0050_D_B | bidirectional differential | -5000...+5000 Pa     |
| AMS 5915-0100-D-B | AMS5915_0100_D_B | bidirectional differential | -10000...+10000 Pa   |
| AMS 5915-0200-D   | AMS5915_0200_D   | differential / relative    | 0...20000 Pa         |
| AMS 5915-0350-D   | AMS5915_0350_D   | differential / relative    | 0...35000 Pa         |
| AMS 5915-1000-D   | AMS5915_1000_D   | differential / relative    | 0...100000 Pa        |
| AMS 5915-2000-D   | AMS5915_2000_D   | differential / relative    | 0...200000 Pa        |
| AMS 5915-4000-D   | AMS5915_4000_D   | differential / relative    | 0...400000 Pa        |
| AMS 5915-7000-D   | AMS5915_7000_D   | differential / relative    | 0...700000 Pa        |
| AMS 5915-10000-D  | AMS5915_10000_D  | differential / relative    | 0...1000000 Pa       |
| AMS 5915-0200-D-B | AMS5915_0200_D_B | bidirectional differential | -20000...+20000 Pa   |
| AMS 5915-0350-D-B | AMS5915_0350_D_B | bidirectional differential | -35000...+35000 Pa   |
| AMS 5915-1000-D-B | AMS5915_1000_D_B | bidirectional differential | -100000...+100000 Pa |
| AMS 5915-1000-A   | AMS5915_1000_A   | absolute                   | 0...100000 Pa        |
| AMS 5915-1200-B   | AMS5915_1200_B   | barometric                 | 70000...120000 Pa    |

For example, the following code declares an AMS5915 object called *sPress* with an AMS5915-1200-B sensor located on I2C bus 0 with an I2C address of 0x10:

```C++
AMS5915 sPress(0x10,0,AMS5915_1200_B);
```

**void begin()**
This should be called in your setup function. It initializes the I2C communication and sets the minimum and maximum pressure and temperature values based on the AMS 5915 sensor.

```C++
sPress.begin();
```

**float getPressure()**
*getPressure()* samples the AMS 5915 sensor and returns the pressure measurement as a float with units of Pascal (Pa).

```C++
float pressure;
pressure = sPress.getPressure();
```

**float getTemperature()**
*getTemperature()* samples the AMS 5915 sensor and returns the temperature measurement as a float with units of Celsius (C).

```C++
float temperature;
temperature = sPress.getTemperature();
```

**void getData(float&ast; pressure, float&ast; temperature)**
*getData(float&ast; pressure, float&ast; temperature)* samples the AMS 5915 sensor and returns the pressure measurement as a float with units of Pascal (Pa) and temperature measurement as a float with units of Celsius (C).

```C++
float pressure, temperature;
sPress.getData(&pressure,&temperature);
```

# Wiring and Pullups
Please refer to the [Analog Microelectronics AMS 5915 datasheet](https://github.com/bolderflight/AMS5915/blob/master/docs/ams5915.pdf) and the [Teensy pinout diagrams](https://www.pjrc.com/teensy/pinout.html).

The silver dot on the AMS 5915 marks the location of Pin 1. The AMS 5915 pinout is:

   * Pin 1: Ground
   * Pin 2: VCC, this should be a 3.0V to 3.6V power source. This can be supplied by the Teensy 3.3V output.
   * Pin 3: SDA
   * Pin 4: SCL

The Teensy pinout is:

   * Teensy 3.0:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
   * Teensy 3.1/3.2:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 29: SCL, Pin 30: SDA
   * Teensy 3.5:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
   * Teensy 3.6:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
      * Bus 3 - Pin 56: SDA, Pin 57: SCL
   * Teensy LC:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 22: SCL, Pin 23: SDA  

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.
