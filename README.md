# <center> Septentrio Arduino Library </center>

<img align=right src=images/logo.png width=250 height=250>

This Arduino library has been created to make Septentrio GNSS easier to use with Arduino hardware. <br>
**Make sure that both the Arduino's and the receiver's baudrate match to use this library (keep in mind that the default baudrate for Septentrio is 115200 but it is not supported by AltSoftSerial used in some examples).**  <br> 
\
Author : chiara de Saint Giniez

## Septentrio resources

| Septentrio website                                    | Contact                                                          | Resources |
| :-:                                                   | :-:                                                              | :-: |
| [Septentrio home page](https://www.septentrio.com/en) | [Septentrio contact page](https://www.septentrio.com/en/contact) | [Septentrio resource page](https://www.septentrio.com/en/support) |


### Table of contents
* [Content of the library](#content-of-the-library)
* [Examples](#examples)
* [Installation](#installation)
* [Credits](#credits)

## <center> Content of the library

Two classes are defined in this library:
* **SEPTENTRIO_GNSS** for input and ouput limited to the receiver. 
* **SEPTENTRIO_NTRIP** for all operations related to NTRIP (see the NTRIP Client example's folder for more explanations). A caster and port must be provided when initializing. It can function with NTRIP 1 and 2, and HTTP protocol.
Both need a **two-way serial connection** to the receiver to be initialized and work correctly. 

A debug mode can be enabled for both by using the _enableDebug_ function and providing a port capable of displaying the messages (for instance the serial monitor).<br>

## <center> Examples </center>
This library's examples have been tested with a Mosaic-go and Arduino Uno, and Arduino R4 WiFi for the NTRIP features. <br>  
The examples use the Serial monitor but can be adapted to work without easily.  
Some example use the __AltSoftSerial__ library with its __default pins__ (9 for tx and 8 for rx) at a baudrate of __9600__. This is not mandatory to use the library and was only done for testing purposes. However, you always need a **two-way serial connection** between Arduino and receiver.

* read_PVTGeodetic : 
    * display the PVTGeodetic data (see SBF block) in the serial monitor.
    * This example can be easily adapted for all SBF blocks (see [this page](https://customersupport.septentrio.com/s/article/What-is-SBF-and-where-can-I-find-more-information-about-it) for reference)
* read_GGA :
    * Display GGA data in the serial monitor
* write_command:
    * Send commands to the receiver from the Serial monitor
    * Can be used to send or get configuration or check that the receiver works correctly
* log_PVTGeodetic : 
    * not yet tested
* ntrip_client : 
    * creates an NTRIP client from the Arduino board and sends correction data to the receiver through serial
    * This example requiers a WiFi connection to work

For more information on each example, see their respective README files.

## <center> Installation </center>
This library is available in the official Arduino IDE. It can also be downloaded from this GitHub repository.

## <center> Credits </center>
I would like to thank Nathan Seith and the Sparkfun team for allowing to use their [library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/) for their u-blox modules as a inspiration for this library. 
