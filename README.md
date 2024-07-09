# Septentrio Arduino Library

This Arduino library has been created to make Septentrio GNSS easier to use with Arduino hardware. <br>
/!\ Make sure that all baudrate match to use this library (keep in mind that the default baudrate for Septentrio is 115200 but it is not supported by AltSoftSerial used in the examples).

## Content of the library

Two classes are defined in this library:
* **SEPTENTRIO_GNSS** for input and ouput limited to the receiver. It needs a serial connection to be initialized and work correctly. 
* **SEPTENTRIO_NTRIP** for all operations related to NTRIP (see the NTRIP Client example's folder for more explanations). A caster and port must be provided when initializing. It can function with NTRIP 1 and 2 and HTTP protocol.

A debug mode can be enabled for both by usingthe _enableDebug_ function and providing a port capable of displaying the messages.<br>

## Examples
This library's examples have been tested with a Mosaic-go and Arduino Uno and Arduino R4 WiFi for the NTRIP features. <br>  
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
