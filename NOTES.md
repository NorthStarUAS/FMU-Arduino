# Teensy vs. Beaglebone baud rate notes:

* supported beaglebone bauds can be found in
  beaglebone:/usr/include/asm-generic/termbits.h

* teensy uses math (a macro) to compute a clock divisor for the given
  baud 'request.'

* on the teensy you can 'request' any integer baud rate and the
  closest lower rate will be used.

* Teensy 3.2 divisor math (from
  /usr/local/arduino-1.8.1/hardware/teensy/avr/cores/teensy3/)

* Example: 500,000 baud:
  - Teensy divisor = (96000000*2 + baud/2)/baud (ex: 500,000 baud = 384)
  - Actual baud = 96000000*2 / divisor (ex: div=384, baud=500,000)
  - Beaglebone termbits.h: B500000
  
* Example: 115,200 baud:
  - Teensy divisor = 1167
  - Actual teensy baud rate = 115176.965  (0.02% error)
  - Beaglebone termbits.h: B115200

* If teensy and beaglebone don't have exact same baud, it may work if
  it's close enough, but there is an increasing risk of dropping
  bytes the further these are apart.

* Beaglebone supports bauds like 100,000, 200,000, 250,000, ...,
  1,000,000 which the teensy can match exactly.  It is recommended to
  pick an exact match if possible.


# SBUS Channel conventions

I need to make some choices here.  Either I can spend a huge amount of
time making the input channel mapping fully configurable (which
complications configuration, saving parameters, setup chats between us
and the upstream host, etc.  Or, I can pick some defaults and declare
them to be the official mapping for AuraUAS.  For now I choose the
latter approach.  So here are the official mappings:

* Channel 0: Master auto/manual switch.
  - val < 0 (norm) = Manual
  - val > 0 (norm) = Autopilot
* Channel 1: Throttle safety
  - val < 0 (norm) = Throttle disabled
  - val > 0 (norm) = Throttle enabled
* Channel 2: Throttle
* Channel 3: Aileron
* Channel 4: Elevator
* Channel 5: Rudder
* Channel 6: Flaps
* Channel 7: Gear
* Channel 8-16: Unassigned
* Channel 17-18 (Digital on/off): Unassigned

Note: Mixing modes are handled on board the aura-sensors module (aka
"smart" receiver.)  This simplifies transmitter programming and many
other aircraft setup tasks tremendously.


# Available mixing modes

Briefly, the supported mixing modes at the time of writing are:

* Rudder autocoordination (simple)
* Elevator trim proportional to throttle
* Elevator trim proportional to flaps
* Flying wing (elevon) mixing
* Flaperon mixing (if independent aileron servos are available.)
* Vtail mixing
* Differential thrust mixing.


# PWM Output Channel Assignments

Again, for the purpose of keeping code and configurations simpler, here
are the official PWM output channel mappings:

* Channel 0: Throttle
* Channel 1: Aileron
* Channel 2: Elevator
* Channel 3: Rudder
* Channel 4: Flaps
* Channel 5: Gear
* Channel 6-n: Unassigned

Elevon configuration:

* Channels 1 and 2 are the elevon servos (todo: sort left vs. right)

Flaperon configuration:

* Channels 1 and 4 are the flaperon servos (todo: sort left vs. right)

Vtail configuration

* Channels 2 and 3 (todo: sort left, right)

Differential thrust

* Channels 0 and 6 (todo: test, sort left, right)