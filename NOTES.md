# Teensy vs. Beaglebone baud rate notes:

* supported beaglebone bauds can be found in
  beaglebone:/usr/include/asm-generic/termbits.h

* teensy uses math (a macro) to compute a clock diviser for the given
  baud 'request.'

* on the teensy you can 'request' any integer baud rate and the
  closest lower rate will be used.

* Teensy 3.2 diviser math (from
  /usr/local/arduino-1.8.1/hardware/teensy/avr/cores/teensy3/)

* Example: 500,000 baud
  - Teensy diviser = (96000000*2 + baud/2)/baud (ex: 500,000 baud = 384)
  - Actual baud = 96000000*2 / diviser (ex: div=384, baud=500,000)
  - Beaglebone termbits.h: B500000
  
* Example: 115,200 baud
  - Teensy diviser = 1167
  - Actual teensy baud rate = 115176.965  (0.02% error)
  - Beaglebone termbits.h: B115200

* If teensy and beaglebone don't have exact same baud, it may work if
  it's close engough, but there is an increasing risk of dropping
  bytes the further these are apart.

* Beaglebone supports bauds like 100,000, 200,000, 250,000, ...,
  1,000,000 which the teensy can match exaclty.  It is recommended to
  pick an exact match if possible.

