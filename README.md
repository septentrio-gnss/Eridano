# Septentrio Arduino Library

This Arduino library has been created to make Septentrio GNSS easier to use with Arduino hardware. <br>
/!\ Make sure that all baudrate match to use this library (keep in mind that the default baudrate for Septentrio is 115200 but it is not supported by AltSoftSerial used in the examples).

## Examples
This library's examples have been tested with a Mosaic-go and Arduino Uno.  
It uses the __AltSoftSerial__ library with its __default pins__ (9 for tx and 8 for rx) at a baudrate of __9600__. This is not mandatory to use the library and was only done for testing purposes.

* read_PVTGeodetic : 
    * tested in real time at 0.5 Hz and 0.2 Hz 
    * parsing function tested on log files
* read_GGA :
    * not yet tested
* write_command:
    * tested in real time with the commands such as "grc", "setSBFOutput, Stream1, USB2, PVTGeod, sec2" and "setSBFOutput, Stream1, none, none, off"
* log_PVTGeodetic : 
    * not yet tested

## Credit 
I would like to thank Nathan Seith and the Sparkfun team for allowing to use their [library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/) for their u-blox modules as a inspiration for this library. 
