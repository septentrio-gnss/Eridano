# <center> Developper guide 
---
## Content of the library

Two classes are defined in this library:
* **SEPTENTRIO_GNSS** for generalinput and ouput limited to the receiver. 
* **SEPTENTRIO_NTRIP** for all operations related to NTRIP (see the NTRIP Client example's folder for more explanations). A caster and port must be provided when initializing. It can function with NTRIP 1 and 2, and HTTP protocol.
Both need a **two-way serial connection** to the receiver to be initialized and work correctly. 

Different objects are defined in a separate file *septentrio_structs.h*:
* **sbfBuffer_t** contains all that is needed to store and process an SBF message
* **nmeaBuffer_t** contains all that is needed to store and process an NMEA message
* **tempBuffer_t** is a buffer for the header part of SBF messages (see [documentation](TODO) for what it entails)
* **ntripTempBuffer_t** is a buffer for the header part of incoming NTRIP response, compatible with NTRIP v1 and v2
* **ntripBuffer_t** is a simple buffer to store NTRIP data
* **ntripProperties_t** is a structure to store the data concerning the NTRIP caster 

## SEPTENTRIO_GNSS class

This class contains:
* the different buffer for SBF and NMEA 
* the necessary functions to setup the receiver
* the conversion functions for all types defined in [SBF documentation](TODO)
* the functions to test the integrity and validity of messages
* the functions for processing the data and buffers
* the functions to enable and disable debug mode

Here is how messages are processed in checkNewByte:
![SBF flow process](TODO)
![NMEA flow process](TODO)

#### How it works
This example is composed of two files:
* .ino : containing the program sent to the Arduino
* secrets.h : containing the different data needed for the connection (at the very minimum, the WiFi IP and password) which can be freely modified by the user.  
This example creates an NTRIP client connected to WiFi provided by the user and connects to an NTRIP caster, sending authentification, NMEA and/or another custom field if given by the user.  
Once connected, the program will process data depending on their type:
* gnss/data will be passed directly to the receiver for correction (in chunks if it is how the messages are being sent)
* sourcetable will be used to verify and potentially correct the known NTRIP properties
* other type of data will be ignored 

This program can also be run with a debug mode to use with a monitor (ex. the Serial Monitor) which will display : 
* A successfull connection with the corresponding code **200**
* A failed connection will display the error type as well as the header(s) corresponding
The most common NTRIP errors have special instruction but otherwise a list of the error codes can be found online [here](https://receiverhelp.trimble.com/r750-gnss/ioConfig.html)

## NTRIP connection

The NTRIP example is the most intricate of the library. You can take a look at the [user manual](https://github.com/septentrio-gnss/Septentrio_Arduino_library/blob/main/ressources/user_manual.md) for a general explanation of NTRIP. This part aims to explain the workflow of the example, starting with a general overview:<br>
<img align=center src="https://github.com/septentrio-gnss/Septentrio_Arduino_library/blob/main/images/ntrip_general_workflow.jpg"height="75%" width="75%"> <br> <br>
This example uses the HTTP protocol. The user first has to provide different inputs to fill the request message. Connection is done thanks to the WiFi library of Arduino and needs to be adapted for each boards. Once connection has been established, the request buffer is sent. <br>
Once the response has been received, the program first checks for a success code ('200'); if it finds it, it then checks for NTRIP version through the syntax of the 200 response and extract the relevant data from the header; otherwise, if debug is enabled the error code is printed and no further process is done.