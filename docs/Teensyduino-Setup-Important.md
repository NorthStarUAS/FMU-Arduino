# IMPORTANT ARDUINO SETUP /REQUIRED/ FOR CORRECT OPERATION

By required I mean REQUIRED and NOT optional!

Issue: serial port comms can push more bytes into the serial port buffers than
can be serviced at 50 or 100hz.  This leads to lost bytes and degraded
communication.  This can appear as spotty gps messages (like one update every
couple seconds even with a 3d fix and 20 sats.)  And it can lead to lost or
failed ground station comms.

## Future consideration

teensyduino also has addMemoryForRead() /addMemoryForWrite()  This might be
preferable, but you have to make sure to tell it the write buffer and length.
And I believe it leaves the stock buffer abandoned, but that is only 40 + 64
bytes so probably not a big deal.

Weirdmess though: somehow the code appears to add this buffer and keep the
original buffer and some how juggle the two non-contiguous buffers?  I don't see
how that gets handled in the code so I'm concerned and want to learn more before
venturing down this path.

## Fix Option 1

\#define new buffer sizes in setup_board.h -- but so far I haven't reliably
unraveled the \#include order to make this work right.

These probably have to be defined at the IDE level (global build environment,
-D_VARIABLE_=VALUE) and couldn't be set in some #include file in the project
source code.

## Fix Option 2

Edit Teensyduino Core files to increase hard coded buffer sizes

In the spirit of minimizing changes to the stock arduino/teensyduino
installation, I only edit the files I absolutely have to edit:

My installation is under ~/.arduino15, your mileage may vary on different OS's
or versions.

1. GPS port (Marmot v1.7)
    * ~/.arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy3/serial3.c
    * ~/.arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy4/HardwareSerial3.c

    SERIAL3_TX_BUFFER_SIZE 255
    SERIAL3_RX_BUFFER_SIZE 255

2. radio modem port (comms with host) (Marmot v1.7)
    * ~/.arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy3/serial4.c
    * ~/.arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy4/HardwareSerial4.c

    SERIAL4_TX_BUFFER_SIZE 512
    SERIAL4_RX_BUFFER_SIZE 512
