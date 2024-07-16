
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
The most common NTRIP errors have special instruction but otherwise a list of the error codes can be found [here](https://receiverhelp.trimble.com/r750-gnss/ioConfig.html)