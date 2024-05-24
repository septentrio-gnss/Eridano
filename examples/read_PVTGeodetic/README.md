# Example : Reading PVTGeodetic #

## What this example does ##

This examples sets up a connection between the Arduino board and the Septentrion GNSS, parses the incoming data and displays it in the serial monitor. It takes do-not-use values into account and displays them as INVALID. Raw bit value are not translated to their meaning.

## The PVTGeodetic block ##

The structure of the SBF blobk is the following:  

| Byte number | Field name | Unit and scale   |Type                      | Description |
| :---------: | :--------: | :--------------: | :----------------------: | :-----------| 
| 0           | Sync 1     |                  | 1 byte character         | First half of the synch field with value '$' or hex 0x24. <br>Marks the beginning of the SBF block |
| 1           | Sync 2     |                  | 1 byte character         | First half of the synch field with value '@' or hex 0x40. <br>Marks the beginning of the SBF block |
| 2           | CRC        |                  | 2 bytes unsigned integer | 16-bit CRC of all the bytes in the SBF block, from and including the IF field to the last byte of the block. <br> The generator polynomial is the so-called CRC-CCIT : x^{16} + x^{12} + x^{5} + x^{0}. It is computed using a seed of 0, no reverse and no final XOR. |
| 4           | Length     | 1 byte           | 4 bytes unsigned integer | The size of the SBF block, including the header, in byte. It is always a multiple of four. |
| 6           | TOW        | 0.001s           | 4 bytes unsigned integer | Time of week. Time-tag expressed in whole millisecond since the beginning of the GPS week. <br> DO-NOT-USE value is : **4294967295**|
| 10          | WNc        | 1 week           | 2 bytes unsigned integer | GPS week number associated with the TOW, WNc is a continuous week count. It is not affected by GPS rollovers which occur every 1024 weeks. <br> DO-NOT-USE value is : **65535**|
| 14          | Mode       |                  | 1 byte unsigned integer  | Bit field indicating the PVT mode, as follows : <ul> <li> Bits 0-3 : types of PVT solution <br> <ul> <li> 0 : no GNSS PVT available </li> <li> 1 : Stand-Alone PVT</li> <li> 2 : Differential PVT </li> <li> 3 : Fixed location </li> <li> 4 : RTK with fixed ambiguities </li> <li> 5 : RTK with float ambiguities </li> <li> 6 : SBAS aided PVT</li> <li> 7 : Moving-base RTK with fixed ambiguities </li> <li> 8 : Moving-base RTK with float ambiguities </li> <li> 9 : Reserved </li> <li> 10 : Precise Point Positioning (PPP) </li> <li> 11 : Reserved </li> </ul> </li> <li> Bits 4-5 : Reserved </li> <li> Bit 6 : Set if the user has entered the command **setPVTMode, Static, , auto** and the receiver is still in the process of deermining its fixed position </li> <li> Bit 7 : 2D/3D flag : set in 2D mode (height assumed constant and not computed) </li> </ul> |
| 15           | Error    |                   | 2 bytes unsigned integer | PVT error code : <ul> <li> 0 : No error </li> <li> 1 : Not enough measurement </li> <li> 2 : Not enough ephemerides available </li> <li> 3 : DOP too large (larger than 15) </li> <li> 4 : Sum of squared residuals too large </li> <li> 5 : No convergence </li> <li> 6 : Not enough measurements after outlier rejections </li> <li>  7 : Position output prohibited due to export laws <li> <li> 8 :  Base station coordinates unavailable </li> <li> 10 : Ambiguities not fixed and user requested only to output RTK-fixed positions |
| 16          | Latitude   | 1 rad            | 8 bytes IEEE float       | Latitude, from -&pi/2 to +&pi/2, positive North of Equator. <br> DO-NOT-USE value is : **-2.10^{-2}** |
| 24          | Longitude  | 1 rad            | 8 bytes IEEE float       | Longitude, from -&pi to +&pi, positive East of Greenwich. <br> DO-NOT-USE value is : **-2.10^{-2}** . |
| 32          | Height     | 1 m              | 8 bytes IEEE float       | Ellipsoidal height, with respect to the ellipsoid specified by _*Datum*_ <br> DO-NOT-USE value is : **-2.10^{-2}** . |
| 40          | Undulation | 1 m              | 4 bytes IEEE float       | Geoid undulation. <br> DO-NOT-USE value is : **-2.10^{-2}** .|
| 44          | V_n        | 1 m/s            | 4 bytes IEEE float       | Velocity in the North direction. <br> DO-NOT-USE value is : **-2.10^{-2}** .|
| 48          | V_e        | 1 m/s            | 4 bytes IEEE float       | Velocity in the East direction. <br> DO-NOT-USE value is : **-2.10^{-2}** .| 
| 52          | V_u        | 1 m/s            | 4 bytes IEEE float       | Velocity in the 'Up' direction. <br> DO-NOT-USE value is : **-2.10^{-2}** .|
| 56          | COG        | 1 degree         | 4 bytes IEEE float       | Course Over Ground : angle of the vehicle with respect to the local level north, ranging from 0 to 360, and increasing toward the east. Set to the do-not-use value when speed is lower than 0.1 m/s. <br> DO-NOT-USE value is : **-2.10^{-2}** .|
| 60          | RxClkBias  | 1 ms             | 8 bytes IEEE float       | Receiver clock bias relative to the system time reported in the *_TimeSystem*_ field. Positive when the receiver time is ahead of the system time. <br> DO-NOT-USE value is : **-2.10^{-2}** .|
| 68          | RxClkDrift | 1 ppm            | 4 bytes IEEE float       | Receiver clock bias relative to the GNSS system time (relative frequency error). Positive when the receiver clock runs faster than the system time. <br> DO-NOT-USE value is : **-2.10^{-2}** .|
| 72          | TimeSystem |                  | 1 byte unsigned integer  | Time system to which the offset is provided in this sub-block. <ul> <li> 0 : GPS time </li> <li> 1 : Galileo time <li> <li> 3 : GLONASS time </li> <li> 4 : Beidou time </li> <li> 5 : QZSS time </li> <li> 100 : Fugro AtomiChron time </li> </ul> <br> DO-NOT-USE value is : **255** .|
| 73          | Datum      |                  | 1 byte unsigned integer  | This field defines in which datum the coordinates are expressed : <ul> <li> 0 : WGS84/ITRS </li> <li> 19 : Datum equal to that used by  the DGNSS/RTK base station </li> <li> 30 : ETRS89 (ETRF2000 serialization) </li> <li> 31 : NAD83(2011), North american datum (2011)</li> <li> 32 : NAD83(PA11), North american datum, Pacific plate </li> <li> 33 : NAD83(MA11), North american datum, Marianas plate </li> <li> 34 : GDA94(2010), Geocentric datum of Australia (2010) </li> <li> GDA2020, Geocentric datum of Australia (2020) </li> 36 : JGD2011, Japanese geodetic datum 2011 </li> <li> 250 : First user-defined datum </li> <li> 251 : Second user-defined datum </li> </ul> <br> DO-NOT-USE value is : **255** .|
| 74          | NrSv       |                  | 1 byte unsigned integer  | Total number of satelites used in the PVT computation. <br> DO-NOT-USE value is : **255** .|
| 75          | WaCorrInfo |                  | 1 byte unsigned integer  | Bit field providing information about which wide area corrections have been applied: <ul> <li> Bit 0: set if orbit and satellite clock correction information is used </li> <li> Bit 1: set if range correction information is used </li> <li> Bit 2: set if ionospheric information is used </li> <li> Bit 3: set if orbit accuracy information is used (UERE/SISA) </li> <li> Bit 4: set if DO229 Precision Approach mode is active </li> <li> Bits 5-6: Type of RTK corrections: <ul> <li> 0: Unknown or not in RTK mode </li> <li> 1: RTK with physical base </li> <li> 2: RTK with virtual base </li> <li> 3: RTK from SSR corrections (PPP-RTK) </li> <li> Bit 7: Reserved </li> </ul> </li> </ul> <br> DO-NOT-USE value is : **0** .|
| 76          | referenceID|                  | 2 bytes unsigned integer | This field indicates the reference ID of the differential information used. <br> In case of DGNSS or RTK operation, this field is to be interpreted as the base station identifier. In SBAS operation, this field is to be interpreted as the PRN of the geostationary satellite used (from 120 to 158). <br> If multiple base stations or multiple geostationary satellites are used the value is set to DO-NOT-USE value : **65534**. |
| 78          | meanCorrAge| 0.01 s           | 2 bytes unsigned integer | In case of DGNSS or RTK, this field is the mean age of the differential corrections. <br> In case of SBAS operation, this field is the mean age of the ’fast corrections’ provided by the SBAS satellites. <br> In case of PPP, this is the age of the last received clock or orbit correction message. <br> DO-NOT-USE value is : **65534**. |
| 80          | SignalInfo |                  | 4 bytes unsigned integer | Bit field indicating the type of GNSS signals having been used in the PVT computations. If a bit *i* is set, the signal type having index *i* has been used. <br> DO-NOT-USE value is : **0**|
| 84          | AlertFlag  |                  | 1 byte unsigned integer  | Bit field indicating integrity related information: <ul> <li> Bits 0-1: RAIM integrity flag: <ul> <li> 0: RAIM not active (integrity not monitored) </li> <li> 1: RAIM integrity test successful </li> <li> 2: RAIM integrity test failed </li> <li> 3: Reserved </li> </ul> <li> Bit 2: set if integrity has failed as per Galileo HPCA (HMI Probability Computation Algorithm) </li> <li> Bit 3: set if Galileo ionospheric storm flag is active </li> <li> Bit 4: Reserved </li> <li> Bits 5-7: Reserved </li> </ul>|

Revision 1 :
| Byte number | Field name | Unit and scale   |Type                      | Description |
| :---------: | :--------: | :--------------: | :----------------------: | :-----------| 
| 85          | NrBases    |                  | 1 byte unsigned integer  | Number of bases used in the PVT computation. <br> DO-NOT-USE value is : **0** .|
| 86          | PPPInfo    | 1 s              | 2 bytes unsigned integer | Bit field containing PPP-related information: <ul> <li> Bits 0-11: Age of the last seed, in seconds. The age is clipped to 4091s. This field must be ignored when the seed type is 0 (see bits 13-15 below). </li> <li> Bit 12: Reserved </li> <li> Bits 13-15: Type of last seed: <ul> <li> 0: Not seeded or not in PPP positioning mode </li> <li> 1: Manual seed <li> <li> 2: Seeded from DGNSS </li> <li> 3: Seeded from RTKFixed </li> </ul> </ul> <br> DO-NOT-USE value is : **0**|

Revision 2 :
| Byte number | Field name | Unit and scale   |Type                      | Description |
| :---------: | :--------: | :--------------: | :----------------------: | :-----------| 
| 87          | Latency    | 0.0001 s         | 2 bytes unsigned integer | Time elapsed between the time of applicability of the position fix and the generation of this SBF block by the receiver. This time includes the receiver processing time, but not the communication latency. <br> DO-NOT-USE value is : **65535**|
| 89          | HAccuracy  | 0.01 m           | 2 bytes unsigned integer | 2DRMS horizontal accuracy: twice the root-mean-square of the horizontal distance error. The horizontal distance between the true position and the computed position is expected to be lower than HAccuracy with a probability of at least 95%. The value is clipped to 65534 =655.34m. <br> DO-NOT-USE value is : **65535** |
| 91          | VAccuracy  | 0.01 m           | 2 bytes unsigned integer | 2-sigma vertical accuracy. The vertical distance between the true position and the computed position is expected to be lower than VAccuracy with a probability of at least 95%. The value is clipped to 65534 =655.34m. <br> DO-NOT-USE value is : **65535**|
| 93          | Misc       |                  | 1 byte unsigned integer  | Bit field containing miscellaneous flags: <ul> <li> Bit 0: In DGNSS or RTK mode, set if the baseline points to the base station ARP. Unset if it points to the antenna phase center, or if unknown. </li> <li> Bit 1: Set if the phase center offset is compensated for at the rover, unset if not or unknown. </li> <li> Bit 2: Proprietary. </li> <li> Bit 3: Proprietary. </li> <li> Bits 4-5: Proprietary. </li> <li> Bits 6-7: Flag indicating whether the marker position reported in this block is also the ARP position (i.e. whether the ARP-to- marker offset provided with the setAntennaOffset command is zero or not) <ul> <li> 0: Unknown </li> <li> 1: The ARP-to-marker offset is zero </li> <li> 2: The ARP-to-marker offset is not zero </li> </ul> </li> </ul> 
| 94          | Padding   |                   | n byte unsigned integer  | Padding bytes |