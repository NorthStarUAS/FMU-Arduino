# BUILD NOTES

* Select Arduino -> Tools -> USB Type -> "Serial + MTP Disk (Experimental)"
* Select Arduino -> Tools -> Optimize -> Smallest Code with LTO

## Notes on optimize for smallest size

Teensyduino loses the ability to print floats from printf()

<https://forum.pjrc.com/index.php?threads/float-and-double.31233/>

Oct 30, 2015 #11 Paul Stoffrogen:

Please keep in mind Serial.print() only accepts 32 bit float.

To see the full accuracy, use Serial.printf(). Tools > CPU Speed has to be set
to one of the optimize options (which is the default). If you change to reduce
code size, printf() becomes a size optimized version which doesn't support any
floats.

Serial.print() still is able to print floating point numbers so all is not lost!
