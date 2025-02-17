# Hardware-in-the-loop Testing

Notes for setting up HIL testing

## Run Simluator

    Simulator$ ./run_jsbsim.py --model Rascal110 --takeoff S10:02

## Run the Out-the-window Visuals

Note: the first time you run the visuals in a new area, allow some time (a
minute or two) for the tile builder to download the SRTM terrain data and begin
building the top level tiles.  The simulator will wait and not do the initial
trim until it receives a valid ground elevation from the visuals.

    Simulator$ ./run_visuals.py

## Turn on the FMU Hardware

* Plug in your teensy flight controller via USB to your host computer.
* Power on the teensy.
* If you have integrated the RC receiver and set that up, power up your RC
  transmitter.

Go to examples/skywalker/config.json

* HIL_testing
  * enable = "True"
  * inceptors = "rc" (transmitter) or "sim" (joystick)

Copy this config file to: Files / Teensy MTP Disk / SD Card / config.json
(Sorry, teensy MTP doesn't support editing the config file in place.)

(and power cycle after changing the config.)

## Run the Ground Station Server

You may need to run dmesg or look at your /dev/tty* files to determine
the correct serial port (for linux at least.)

    GCS/nsLink$ ./nslink.py --serial <port>

## Open up the GCS web pages (in your favorite web browser)

If you forget the url, the GCS server pops up a small window with the top level
link.

## Launch

From a terminal run:

    $ telnet 5050
    > send task Launch
