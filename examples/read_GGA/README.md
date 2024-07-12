# Example : Reading GNGGA #

## What this example does ##

This examples sets up a connection between the Arduino board and the Septentrion GNSS, and displays it in the serial monitor. 
It test the ID and checksum and displays the raw message with empty fields left if they are correct
The message ID *msgId* can be fully explicit or use '*' for any character (for instance : 'G*GGA' to allow 'GP', 'GN'... ).

## The NMEA message ##

The structure of the GGA sentence is the following:  

| Field name  | Description |
| :---------: | :-----------| 
| 0           | Message ID: $GNGGA |
| 1           | UTC of position fix |
| 2           | Latitude in degrees minute |
| 3           | Direction of latitude : <br> N : North <br> S : South |
| 4           | Longitude in degrees minute |
| 5           | Direction of longitude : <br> E : East <br> W : West |
| 6           | GPS quality indicator : <ul> <li> 0: Fix not valid </li> <li> 1: GPS fix </li> <li> 2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode </li> <li> 3: PPS fix </li> <li> 4: RTK fix </li> <li> 5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX </li> <li> 6: INS Dead reckoning </li> <li> 7 : Manual input mode </li> <li> 8 : Simulation mode </li> </ul> |
| 7           | Number of sattelites in use |
| 8           | Horizontal Dilution Of Precision (HDOP) |
| 9           | Orthometric height |
| 10          | M for meter, unit of previous field |
| 11          | Geoid separation |
| 12          | M for meter, unit of previous field |
| 13          | Age of correction data for differential GPS fix or RTK, in seconds |
| 14          | Correction station ID number |
| 15          | Checksum delimiter '*' and checksum value as two hex characters|