# Hardware Resources

## Flight Computers

* Teensy 4.0 and 4.1: 600Mhz Cortex-M7 with boatloads of RAM, Storage, and IO
  pins. Programmable with the standard Arduino-IDE and C++ code.
  <https://www.pjrc.com/teensy/>

* BeagleBone Black: Linux companion computer, low power draw (especially
  compared to a modern raspberry pi), but still highly capable and well
  supported.  Marmot-1.7 boards directly plug in and have connected UART and
  several port pass throughs.
  <https://www.beagleboard.org/boards/beaglebone-black>

## Flight Computers, Sensors (GPS, Air Data), Cables and Connectors

* Bolder Flight Systems: Offers a teensy "compatible" based flight management
  computer/board with a 30x30mm footprint.  They may also continue to offer
  their marmot board which has a footprint that plugs directly into a beaglebone
  black.
  <https://bolderflight.com/>

## Sensors (GPS, Air Data), Cables and Connectors

* 3DR (Jordie) sells a variety of sensors boards, and cables.  For things like
  gps, airdata, and radio modems, this is the goto place.  Note, this project
  uses a teensy as a flight controller, not a pixhawk-based board, so the 3dr
  flight controllers are not compatible here.
  <https://store.3dr.com/>

## Cables and connectors

* JST-GH precrimped wires (combined with connectors you can build your own custom cables)
  <https://www.digikey.com/en/products/detail/jst-sales-america-inc/AGHGH28K51/6009448>
