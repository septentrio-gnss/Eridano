#include "Septentrio_Arduino_driver.h"

static const uint16_t CRC_SBF_LookUp[256] =
{
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};
uint16_t compute_CRC_16CCITT(const sbfBuffer_t* buf,const int buf_length)
{
    /**Taken from sbf2ascii, may not be optimal**/
    uint16_t crc = 0;
    for (int i=0;i<buf_length;i++)
    {
        crc = (crc << 8) ^ CRC_SBF_LookUp[(crc >> 8) ^ buf->data[i]];
    }
    return crc;
}

SEPTENTRIO_GNSS::SEPTENTRIO_GNSS()
{
    tempBuffer = new tempBuffer_t;
}
SEPTENTRIO_GNSS::~SEPTENTRIO_GNSS()
{
    if (SBFBuffer!=nullptr)
    {
        delete[] SBFBuffer;
    }
    if (NMEABuffer!=nullptr)
    {
        delete[] NMEABuffer;
    }
}

            //Communication
bool SEPTENTRIO_GNSS::begin(Stream &serialPort, const uint16_t maxWait)
{
    /**
     * @brief Ensures the connection to GNSS through serial and initializes the temporary buffer
     * @param serialPort The port through which data will be exchanged 
     * @param maxWait The maximum amount of time to test before timing-out
     * @return Whether the connection with the serial port was successfull
     **/
    SEPTENTRIO_GNSS::_serialPort = &serialPort;
    bool connected=false;

    while (SEPTENTRIO_GNSS::_serialPort->available())
    {
        SEPTENTRIO_GNSS::_serialPort->read(); //empty data til tx buffer empty
    }
    connected=isConnected(maxWait);
    if (!connected)
    {
        connected=isConnected(maxWait);
    }
    tempBuffer = new tempBuffer_t;
    return connected;
}
bool SEPTENTRIO_GNSS::isConnected(const uint16_t maxWait)
{
    /**
     * @brief Checks the connection to the GNSS through response to a command
     * @param maxWait The maximum amount of time to test before timing-out
     * @return Whether the test was successfull
     **/
    while (SEPTENTRIO_GNSS::_serialPort->available()>0)
    {
        SEPTENTRIO_GNSS::_serialPort->read();
    }
    int byteSent=0;
    int counter=0;
    auto start=millis();
    SEPTENTRIO_GNSS::_serialPort->write("SSSSSSSSSS\n"); //allow for checking temporarily using command
    while (SEPTENTRIO_GNSS::_serialPort->availableForWrite()<3 && (millis()-start)<maxWait) 
    {
        if (SEPTENTRIO_GNSS::_serialPort->available()>0)
        {
            if (SEPTENTRIO_GNSS::_serialPort->peek()=='$')
            {
                return true;
            }
        }
        if (counter%5==0)
        {
            auto byteSent=_serialPort->write("grc");
        }
        counter++;
    }
    return (false);
}
bool SEPTENTRIO_GNSS::canSendCommand(Stream &serialport, const String comPort, const uint16_t maxWait) //add cd variable -> diff is we set to only cmd input
{
    /**
     * @brief Checks the connection and ensures that commands can always be sent (WIP)
     * @param serialPort The port through which the command and response will be sent 
     * @param comPort The port connected to the Arduino, through which the command and response are sent
     * @param maxWait The maximum number of tries before Timing-out
     * @return Whether the test was successfull
     **/
    int offset=0;
    int counter=0;
    auto start=millis();
    while (SEPTENTRIO_GNSS::_serialPort->available()>0)
    {
        SEPTENTRIO_GNSS::_serialPort->read();
    }
    while (SEPTENTRIO_GNSS::_serialPort->availableForWrite()<3 && (millis()-start)<maxWait) //find better way to get out
    {
        if (counter%5==0)
        {
            if (counter>15)
            {
                SEPTENTRIO_GNSS::_serialPort->write("SSSSSSSSSS\n");
            }
            auto byteSent=SEPTENTRIO_GNSS::_serialPort->write("grc");
        }
        
        while (_serialPort->available()>0)
        {
            offset = checkCommandResponse(SEPTENTRIO_GNSS::_serialPort->read(), offset);
            if (offset==3)
            {
                SEPTENTRIO_GNSS::_serialPort->write(("setDataInOut, "+comPort+", +CMD").c_str()); //TODO: allow for different port
                return true;
            }
            if (offset==-1)
            {
                //there would be an issue with the command sent but the response means the connection is valid
                //think about that case
                return true;
            }
        }
        counter++;
    }
    return false;
}
int SEPTENTRIO_GNSS::checkCommandResponse(const uint8_t byte, int offset) //change return value
{
    /**
     * @brief Checks the received bytes are a response to command
     * @param byte The incoming byte to process
     * @param offset The character offset in the response sentence
     **/
    switch (offset)
    {
    case 0:
        if (byte=='$')
        {
            offset++;
        }
        break;
    case 1:
        if (byte=='R')
        {
            offset++;
        }
        break;
    case 2:
        if (byte==':')
        {
            offset++;
            if (SEPTENTRIO_GNSS::_printDebug)
            {
                SEPTENTRIO_GNSS::_debugPort->println(F("received response"));
            }
        }
        else if (byte=='?')
        {
            offset=-1;
            if (SEPTENTRIO_GNSS::_printDebug)
            {
                SEPTENTRIO_GNSS::_debugPort->println(F("Invalid command"));
            }
        }
        break;
    default:
        offset=0;
        break;
    }
    return offset;
}
bool SEPTENTRIO_GNSS::enableSBF(Stream &serialPort, const String &portName, const uint16_t maxWait) //TODO : check
{
    /**
     * @brief Enables SBF output through port portName, SBF must not be already enabled
     * @param serialPort The serial port through which to send command and receive response
     * @param portName The port through which SBF data will be sent (GNSS port) 
     * @param maxWait Maximum wait-time for the response
     * @return Whether the command was successfull
     **/
    bool success=false;
    if (canSendCommand(serialPort, portName, 5000))
    {
        bool gotResponse=false;
        uint8_t finalCmd[cmdMaxSize];
        finalCmd[0]='s';
        finalCmd[1]='d';
        finalCmd[2]='i';
        finalCmd[3]='o';
        int index=3;
        serialPort.print("getDataInOut, "+portName);
        bool secondLine=false;
        auto start = millis();
        while (!gotResponse || (millis()-start)<maxWait)
        {
            if (serialPort.available())
            {
                /**copy all data after the first comma**/
                if (secondLine)
                {
                    if (index==3) /**skip beginning of response ("DataInOut")**/
                    {
                        for (int i=0;i<8;i++)
                        {
                            serialPort.read();
                        }
                    }
                    if (serialPort.peek()=='(') /**if find "(", stop and put "SBF, " before ->can it not be in parenthesis?**/
                    {
                        finalCmd[index++]='S';
                        finalCmd[index++]='B';
                        finalCmd[index++]='F';
                        finalCmd[index++]=',';
                        finalCmd[index++]=' ';
                    }

                    finalCmd[index++]=serialPort.read();
                    if (finalCmd[index]=='\r') /**get end of message if it exist**/
                    {
                        gotResponse=true;
                        break;
                    }
                }
                else if (serialPort.read()=='\r') /**skip first line as it is just an echo ie. wait until '\r'**/
                {
                    secondLine=true;
                }
            }
        }
        if (gotResponse)
        {
            for (int i=0; i<index; i++)
            {
                serialPort.print(finalCmd[i]);
            }
            success=true;
        }
    }
    return success;
}
bool SEPTENTRIO_GNSS::disableSBF(Stream &serialPort, const String &portName, const uint16_t maxWait) //TODO : check
{
    /**
     * @brief Disables SBF output through port portName
     * @param serialPort The serial port through which to send command and receive response
     * @param portName The port through which SBF data will be snippet 
     * @param maxWait The maximum wait-time for the response
     * @return Whether the command was successfull
     **/
    bool success=false;
    bool secondLine=false;
    auto start=millis();
    if (canSendCommand(serialPort, portName, 50))
    {
        uint8_t finalCmd[100];
        finalCmd[0]='s';
        finalCmd[1]='d';
        finalCmd[2]='i';
        finalCmd[3]='o';
        int index=3;
        serialPort.print("getDataInOut, "+portName);
        bool gotResponse=false;
        while (!gotResponse || (millis()-start)<maxWait)
        {
            if (secondLine)
            {
                if (index==3) /**skip beginning of response (DataInOut)**/
                {
                    for (int i=0;i<8;i++)
                    {
                        serialPort.read();
                    }
                }
                else if (serialPort.read()=='S') /**remove "SBF, "**/
                {
                    if (serialPort.peek()=='B')
                    {
                        for (int i=0;i<5; i++)
                        {
                            serialPort.read();
                            index++;
                        }
                    }
                    else
                    {
                        finalCmd[index++]='S';
                        finalCmd[index++]=serialPort.read();
                    }
                }
            }
            else if (serialPort.read()=='\r')
            {
                secondLine=true;
            }
        }
    }
}

            // -- Conversion functions -- //
uint16_t SEPTENTRIO_GNSS::u2Conv(const sbfBuffer_t *buffer, const uint16_t offset) 
{
    /**
     * @brief Extracts 2-byte unsigned integer (named u2 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte
     * @return The corresponding 2-bytes unsigned integer
     **/
    return (buffer->data[offset]| buffer->data[offset+1]<<8);
}
uint32_t SEPTENTRIO_GNSS::u4Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief Extracts 4-bytes unsigned integer (named u4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte
     * @return The corresponding 4-bytes unsigned integer
     **/
    return (buffer->data[offset]| buffer->data[offset+1]<<8 | static_cast<uint32_t>(buffer->data[offset+2])<<16 | static_cast<uint32_t>(buffer->data[offset+3])<<24);
}
uint64_t SEPTENTRIO_GNSS::u8Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief Extracts 8-bytes unsigned integer (named u8 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte in the buffer
     * @return The corresponding 8-bytes unsigned integer
     **/
    return (buffer->data[offset] | buffer->data[offset+1]<<8 | (static_cast<uint32_t>(buffer->data[offset+2]))<<16 | (static_cast<uint32_t>(buffer->data[offset+3]))<<24 | (static_cast<uint64_t>(buffer->data[offset+4]))<<32 | (static_cast<uint64_t>(buffer->data[offset+5]))<<40 | (static_cast<uint64_t>(buffer->data[offset+6]))<<48 | (static_cast<uint64_t>(buffer->data[offset+7]))<<56);
}
uint16_t SEPTENTRIO_GNSS::i2Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 2-bytes signed integer (named i2 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte in the buffer
     * @return The corresponding 2-bytes signed integer
     **/
    return (buffer->data[offset]|(buffer->data[offset+1])<<8);
}
uint32_t SEPTENTRIO_GNSS::i4Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief Extracts 4-bytes signed integer (named i4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte in the buffer
     * @return The corresponding 4-bytes signed integer
     **/
    return (buffer->data[offset] | buffer->data[offset+1]<<8 | (static_cast<uint32_t>(buffer->data[offset+2]))<<16 | (static_cast<uint32_t>(buffer->data[offset+3]))<<24);
}
uint64_t SEPTENTRIO_GNSS::i8Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief Extracts 8-bytes signed integer (named i8 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte in the buffer
     * @return The corresponding 8-bytes signed integer
     **/
    return buffer->data[offset] | buffer->data[offset+1]<<8 | (static_cast<uint32_t>(buffer->data[offset+2]))<<16 | (static_cast<uint32_t>(buffer->data[offset+3]))<<24 | (static_cast<uint64_t>(buffer->data[offset+4]))<<32 | (static_cast<uint64_t>(buffer->data[offset+5]))<<40 | (static_cast<uint64_t>(buffer->data[offset+6]&0xFF))<<48 | (static_cast<uint64_t>(buffer->data[offset+7]&0xFF))<<56;
}
float SEPTENTRIO_GNSS::f4Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief Extracts 4-bytes float (named f4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte in tbe buffer
     * @return The corresponding 4-bytes float
     **/
    int sign; 
    ((buffer->data[offset+3]&0x80)>>7 == 1) ? sign=-1 : sign=1;
    int exponent = ((buffer->data[offset+3]&0xFF)<<1)|(buffer->data[offset+2]>>7);
    uint64_t significant = (uint32_t)(buffer->data[offset+2])<<16 | buffer->data[offset+1]<<8|buffer->data[offset];
    double decScientific = 1.;                          
    for (int i=0;i<23;i++)
    {
        decScientific+=(uint8_t)((significant>>(22-i))&0x01)*pow(2,-1-i);
    }
    return sign*decScientific*pow(2,exponent-127);
}
double SEPTENTRIO_GNSS::f8Conv(const sbfBuffer_t *buffer,const  uint16_t offset)
{
    /**
     * @brief Extracts 8-bytes float (named f4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset The offset of the first byte in the buffer
     * @return The corresponding 8-bytes float
     **/
    int sign;
    ((buffer->data[offset+7]&0x80)>>7 == 1) ? sign=-1 : sign=1;
    int exponent = ((buffer->data[offset+7]&0x0FF)<<4)|(buffer->data[offset+6]>>4);
    uint64_t significant = (uint64_t)(buffer->data[offset+6]&0x0F)<<48 | (uint64_t)buffer->data[offset+5]<<40 | (uint64_t)buffer->data[offset+4]<<32 | (uint64_t)buffer->data[offset+3]<<24 | (uint64_t)buffer->data[offset+2]<<16 | (uint64_t)buffer->data[offset+1]<<8 | (uint64_t)buffer->data[offset];
    double decScientific = 1.;                         
    for (int i=0;i<52;i++)
    {
        decScientific+=(uint8_t)((significant>>(51-i))&0x01)*pow(2,-1-i);
    }
    return sign*decScientific*pow(2,exponent-1023);
}

    // -- Integrity functions -- //
bool SEPTENTRIO_GNSS::isSBFValid(sbfBuffer_t *buffer, const uint8_t desiredID)
{
    /**
     * @brief Checks the validity flag of the buffer and the ID
     * @param buffer The buffer containing the data
     * @param desiredID The ID of the SBF block
     * @return Whether the characteristics are valid
     **/
    return checkId(buffer,desiredID)*SBFBuffer->properties.bits.lengthValid*SBFBuffer->properties.bits.timeValid;
}
bool SEPTENTRIO_GNSS::checkId(sbfBuffer_t *buffer, const uint8_t desiredID)
{
    /**
     * @brief Checks the ID and version of the message
     * @param buffer The buffer containing the data
     * @param desiredID The expected ID of the message
     * @return Whether the ID matches and the version doesn't exceed the maximum one
     **/
    buffer->properties.bits.type=(buffer->data[4]&0xFF) | ((buffer->data[5]&0x1F)<<8);
    buffer->properties.bits.ver = (buffer->data[5]&0xe0)>>5; 
    return ((buffer->properties.bits.type==desiredID) && (buffer->properties.bits.ver<maxVersion)); 
}
bool SEPTENTRIO_GNSS::checkCRC(sbfBuffer_t *buffer) //TODO : test
{
    /**
     * @brief Verifies the received CRC and computed ones correspond (for SBF)
     * @param buffer The buffer containing the data
     * @return Whether the CRCs correpond
     */
    buffer->properties.bits.crcValid=(u2Conv(buffer, 2)==compute_CRC_16CCITT(buffer,buffer->offset));
    return (buffer->properties.bits.crcValid);
}
bool SEPTENTRIO_GNSS::checkChecksum(nmeaBuffer_t *buffer)
{
    /**
     * @brief Verifies the checksum of the NMEA
     * @param buffer The buffer containing the NMEA message
     * @return Whether the checksum corresponds to the calculated one
    **/
    uint16_t checksum=0;
    for (int i=0; i<(buffer->msgSize-5);i++)
    {
        checksum ^= buffer->data[i];
    }
    uint8_t CS1=checksum>>4;
    CS1 = (CS1<=0x09) ? CS1 + '0' : CS1 - 10 + 'A';
    uint8_t CS2=checksum&0x0F;
    CS2 = (CS2<=0x09) ? CS2 + '0' : CS2 - 10 + 'A';
    if (CS1==buffer->data[buffer->msgSize-4] && CS2==buffer->data[buffer->msgSize-3])
    {
        buffer->correctChecksum=true;
    }
    return buffer->correctChecksum;
}
bool SEPTENTRIO_GNSS::checkId(nmeaBuffer_t *buffer)
{
    /**
    * @brief Generic function to check for GNSS and INS ID
    * @param buffer The buffer containing the ID that need to be checked
    * @return Whether the buffer's ID is valid or not
    **/
    bool validId=false;
    if (buffer->data[1]=='G' || buffer->data[1]=='*')
    {
        if ((buffer->data[2]=='P' || buffer->data[2]=='N' || buffer->data[2]=='L' || buffer->data[2]=='A' || buffer->data[2]=='B' || buffer->data[2]=='I' || buffer->data[2]=='Q') || buffer->data[2]==)
        {
            if (buffer->data[3]=='G' && buffer->data[4]=='G' && buffer->data[5]=='A')
            {
                validId=true;
            }
        }
    }
    else if(buffer->data[1]=='B' && buffer->data[2]=='D') 
    {
        if (buffer->data[3]=='G' && buffer->data[4]=='G' && buffer->data[5]=='A')
        {
            validId=true;
        }
    }
    else if (buffer->data[1]=='I' && buffer->data[2]=='N')
    {
        if (buffer->data[3]=='G' && buffer->data[4]=='G' && buffer->data[5]=='A')
        {
            validId=true;
        }
    }
    return validId;
}
bool SEPTENTRIO_GNSS::checkCustomId(const nmeaBuffer_t *buffer, const char *customId)
{
    /**
    * @brief Function to check for any specific chosen ID 
    * @param buffer The buffer containing the ID that need to be checked
    * @param customId The ID to check for, need to be terminated with '\0' if it is smaller than 5 charatcers 
    * @return Whether the buffer's ID is valid or not
    **/
    int idSize;
    for (idSize=0;idSize<5;idSize++)
    {
        if (customId[idSize]=='\0')
        {
            break;
        }
    }
    bool validId=true;
    for (int i=0;idSize; i++)
    {
        validId*=(buffer->data[i+1]==customId[i] || customId[i]=='*');
    }
    return validId;
}
bool SEPTENTRIO_GNSS::correctFloat(const double f)
{
    /**
     * @brief Checks that the float is not a DO-NOT-USE value
     * @param f The float to check
     * @return Whether the float is a valid one
     */
    return (f!=doNotUseFloat);
};

    //  --  Buffer --  //
void SEPTENTRIO_GNSS::fillBuffer(sbfBuffer_t *buffer, const uint8_t incomingByte)
{
    /**
     * @brief fills the input buffer with the incoming byte and ajust the offset for the next byte
     * @param buffer The SBF buffer to fill
     * @param incomingByte The incoming byte to add
     **/
    buffer->data[buffer->offset]=incomingByte;
    buffer->offset++;
}
void SEPTENTRIO_GNSS::fillBuffer(nmeaBuffer_t *buffer, const uint8_t incomingByte)
{
    /**
     * @brief fills the input buffer with the incoming byte and ajust the offset for the next byte
     * @param buffer The NMEA buffer to fill
     * @param incomingByte The incoming byte to add
     **/
    buffer->data[buffer->offset]=incomingByte;
    buffer->offset++;
}
bool SEPTENTRIO_GNSS::checkNewByte(tempBuffer_t *tempBuffer, const uint8_t incomingByte, const uint16_t messageId)
{
    /**
     * @brief For SBF messages : checks new byte and stores it or not according to the data's validity and if the message is the desired type
     * @param tempBuffer A temporary buffer that holds data until it can be decided whether to keep or discard it
     * @param incomingByte The incoming byte to process
     * @param messageId The type of message to keep (only for SBF for now)
     * @return Whether the dedicated buffer contains the expected data size
     **/
    bool bufferFilled = false;
    if (incomingByte=='$') /**start of sentence**/
    {
        tempBuffer->headerCount=0;
    }
    if (incomingByte=='@' &&  tempBuffer->headerCount==0) /**set message type to SBF**/
    {    
        tempBuffer->sentenceType=SBF_SENTENCE;
        tempBuffer->data[0]=0x24;
        tempBuffer->data[1]=incomingByte;
        tempBuffer->headerCount++;
        if (tempBuffer->properties.newData==false) /**in case the previous message wasn't finished**/
        {
            tempBuffer->properties.newData=true;
            if (SBFBuffer!=nullptr && SEPTENTRIO_GNSS::_printDebug) //if found when the previous message isn't finished can send error message if debug was enabled
            {
                SEPTENTRIO_GNSS::_debugPort->println(F("WARNING : new message incoming, previous message will be discarded"));
                SEPTENTRIO_GNSS::_debugPort->println(F("WARNING : new message incoming, previous message will be discarded"));
                SEPTENTRIO_GNSS::_debugPort->print(F("Expected size : "));
                SEPTENTRIO_GNSS::_debugPort->print((int)SBFBuffer->msgSize);
                SEPTENTRIO_GNSS::_debugPort->print(F(" VS previous message sizes : "));
                SEPTENTRIO_GNSS::_debugPort->println((int)SBFBuffer->offset+1);
                
            }
        }
    }
    else if ((tempBuffer->headerCount==0) && (incomingByte!='@' || incomingByte!='G')) /** check for 'false positive' of new message while still processing a sentence, continue as before**/
    {
        if (tempBuffer->sentenceType==SBF_SENTENCE)
        {
            tempBuffer->headerCount=9;
        }
        else if (tempBuffer->sentenceType=SENTENCE_TYPE_UNDETERMINED)
        {
            tempBuffer->headerCount=-1;
        }
    }

    if (tempBuffer->sentenceType==SBF_SENTENCE) /**Process SBF messages**/
    {
        if (SBFBuffer!=nullptr && tempBuffer->headerCount>7) /**Header finished : fill the buffer with the info for the size of message**/
        {
            if (SBFBuffer->offset<SBFBuffer->msgSize-2)
            {
                fillBuffer(SBFBuffer, incomingByte);
            }
            else if (SBFBuffer->offset==SBFBuffer->msgSize-2) /**Fill last byte and "reset" the temporary buffer**/
            {
                fillBuffer(SBFBuffer, incomingByte);
                bufferFilled=true;
                fillSBFProperties(SBFBuffer, messageId);
                tempBuffer->properties.newData=true;
                tempBuffer->sentenceType=SENTENCE_TYPE_UNDETERMINED;
            }  
        }
        else if (tempBuffer->headerCount<headerSize && tempBuffer->headerCount>0) /**Check the header data and fill the temporary buffer**/
        {
            switch (tempBuffer->headerCount)
            {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
                tempBuffer->data[tempBuffer->headerCount] = incomingByte;
                tempBuffer->headerCount++;
                break;
            case 7:
                tempBuffer->data[tempBuffer->headerCount] = incomingByte;
                tempBuffer->headerCount++;
                if ((tempBuffer->data[4] | (tempBuffer->data[5]&0x1F)<<8) == messageId)     /**Check it's the type of message we want**/ 
                {
                    if (SBFBuffer==nullptr) 
                    { 
                        SBFBuffer = new sbfBuffer_t;  /**Create generic SBF buffer if needed**/
                    } 
                    if (SBFBuffer!=nullptr)
                    {
                        SBFBuffer->msgSize=(tempBuffer->data[6] | tempBuffer->data[7]<<8);
                        if (SBFBuffer->data!=nullptr)
                        {
                            SBFBuffer->data=nullptr; /**avoid memory overflow**/
                        }
                        SBFBuffer->data = new uint8_t[SBFBuffer->msgSize]; /**Give the correct size to data buffer, padding included**/
                        SBFBuffer->offset=0;
                        SBFBuffer->properties.bits.alreadyRead=0;
                        for (int i=0;i<8;i++)
                        {
                            fillBuffer(SBFBuffer, tempBuffer->data[i]);
                        }
                        tempBuffer->properties.newData=false; /**Indicate a message is being processed**/
                    }
                }
                else /**Discard if not of the chosen ID**/
                {
                    tempBuffer->sentenceType=SENTENCE_TYPE_UNDETERMINED;
                    tempBuffer->properties.newData=true;
                    tempBuffer->headerCount=0;
                }
                break;
            default:
                break;
            }
        }
    }
    return bufferFilled;
}
bool SEPTENTRIO_GNSS::checkNewByte(tempBuffer_t *tempBuffer, const uint8_t incomingByte, const char* messageId="*****")
{
    /**
     * @brief For NMEA messages : checks new byte and stores it or not according to the data's validity and if the message is the desired type
     * @param tempBuffer A temporary buffer that holds data until it can be decided whether to keep or discard it
     * @param incomingByte The incoming byte to process
     * @param messageId The type of message to keep (only for SBF for now)
     * @param customId Whether to messageId is a custom one, used to call validCustomId for non-generic talker id
     * @return Whether the dedicated buffer contains the exepected data size
     **/
    bool bufferFilled = false;
    if (incomingByte=='$') /**start of sentence**/
    {
        tempBuffer->headerCount=0;
    }
    else if ((tempBuffer->headerCount==0) && (incomingByte=='G' || incomingByte=='B' || incomingByte=='I')) 
    {
        tempBuffer->sentenceType==NMEA_SENTENCE;
        tempBuffer->data[0]=0x24; 
        tempBuffer->data[1]=incomingByte;
        tempBuffer->headerCount++;
        if (tempBuffer->properties.newData==false) //if previous message wasn't finished
        {
            tempBuffer->properties.newData=true;
            if (SEPTENTRIO_GNSS::_printDebug)
            {
                SEPTENTRIO_GNSS::_debugPort->println(F("WARNING : new message incoming, previous message will be discarded"));
                if (NMEABuffer!=nullptr)
                {
                    if (NMEABuffer->msgSize==0)
                    {
                        SEPTENTRIO_GNSS::_debugPort->println(F("Checksum wasn't reached"));
                    }
                    else
                    {
                        SEPTENTRIO_GNSS::_debugPort->println(F("Checksum is incomplete"));
                    }
                }
            }
        }     
    }
    else if ((tempBuffer->headerCount==0) && !(incomingByte=='G' || incomingByte=='B' || incomingByte=='I')) /** check for 'false positive' of new message while still processing a sentence, continue as before**/
    {
        if (tempBuffer->sentenceType==NMEA_SENTENCE)
        {
            tempBuffer->headerCount=9;
        }
        else if (tempBuffer->sentenceType=SENTENCE_TYPE_UNDETERMINED)
        {
            tempBuffer->headerCount=-1;
        }
    }
    if (tempBuffer->sentenceType==NMEA_SENTENCE) /**Process NMEA message in nmea buffer**/
    {
        if (NMEABuffer!=nullptr) /**Fill the NMAE buffer**/
        {
            if (NMEABuffer->msgSize==0 || (NMEABuffer->offset<NMEABuffer->msgSize))
            {
                fillBuffer(NMEABuffer, incomingByte);
                if (incomingByte=='*') /**Allows for setting the final length of message**/
                {
                    NMEABuffer->msgSize=NMEABuffer->offset+4;
                }
                else if (NMEABuffer->offset=(NMEABuffer->msgSize-1)) /**End of the message **/
                {
                    tempBuffer->properties.newData=true;
                    tempBuffer->sentenceType=SENTENCE_TYPE_UNDETERMINED;
                    NMEABuffer->correctChecksum=false;
                    bufferFilled=true;
                    if (messageId=="*****")
                    {
                        if (checkId(NMEABuffer))
                        {
                            NMEABuffer->alreadyRead=0;
                            NMEABuffer->correctChecksum=checkChecksum(NMEABuffer);
                        }
                    }
                    else
                    {
                        if(checkCustomId(NMEABuffer, messageId))
                        {
                            NMEABuffer->alreadyRead=0;
                            NMEABuffer->correctChecksum=checkChecksum(NMEABuffer);
                        }
                    }
                    tempBuffer->headerCount=-1;
                }
            }
        }
        else
        {
            if (NMEABuffer->data!=nullptr) /**Avoid memory overflow, may not be best practice*/
            {
                NMEABuffer->data=nullptr;
                NMEABuffer=nullptr;
            }
            NMEABuffer = new nmeaBuffer_t;
            NMEABuffer->data = new uint8_t[nmeaMaxSize]; 
            NMEABuffer->data[0]=tempBuffer->data[0];
            NMEABuffer->data[1]=tempBuffer->data[1];
        }
    }
    return bufferFilled;
}
bool SEPTENTRIO_GNSS::fullBuffer(const sbfBuffer_t *tempBuffer)
{
    /**
     * @brief Checks whether the SBF buffer was filled
     * @param buffer The buffer to check
     * @return Whether the buffer is full
     */
    return (tempBuffer->offset+1)==tempBuffer->msgSize;
}
void SEPTENTRIO_GNSS::fillSBFProperties(sbfBuffer_t *buffer, const uint16_t messageId)
{
    /**
     * @brief Fills the properties field of the SBFBuffer_t structure (ie. validity flags, type) and indicates that the message hasn't been read
     * @param buffer The buffer which will be filled
     * @param messageId The type of message to check for (using Septentrio ID)
     **/
    buffer->properties.bits.crcValid = (u2Conv(buffer, 2)==compute_CRC_16CCITT(buffer,buffer->offset));
    buffer->properties.bits.type = (buffer->data[4]&0xFF) | ((buffer->data[5]&0x1F)<<8);
    buffer->properties.bits.ver = (buffer->data[5]&0xe0)>>5; 
    SBFBuffer->properties.bits.timeValid = ((f4Conv(buffer, 8)!=doNotUseTOW) && (u2Conv(buffer, 12)!=doNotUseLong));
    buffer->properties.bits.type==messageId;
    buffer->properties.bits.alreadyRead=0;
}

    //  --  NTRIP  --   //
SEPTENTRIO_NTRIP::SEPTENTRIO_NTRIP()
{
    ntripProperties = new ntripProperties_t;
}
SEPTENTRIO_NTRIP::~SEPTENTRIO_NTRIP()
{
    if (ntripRequestBuffer!=nullptr)
    {
        delete[] ntripRequestBuffer;
    }
    if (ntripTempBuffer!=nullptr)
    {
        delete[] ntripTempBuffer;
    }
    if (ntripTempBuffer!=nullptr)
    {
        delete[] ntripTempBuffer;
    }
    if (ntripBuffer!=nullptr)
    {
        delete[] ntripBuffer;
    }
    delete[] ntripProperties;
}
bool SEPTENTRIO_NTRIP::connectToReceiver(Stream &receiverPort, int receiverTimeOut)
{
    /**
     * @brief Establish a connection to the receiver and allows it to receive commands and corrections on the parameter serial port
     * @param receiverPort The serial port from the Arduino to the receiver
     * @param receiverTimeOut The maximum amount of time for the receiver to respond (in ms)
     * @return Whether the receiver succesfully responded
     */
    Stream *thePort = &receiverPort;
    bool receiverSetup=false;
    char portName[5]={'\0'};
    char msgBuffer[29];
    int start=millis();
    while ((millis()-start)<receiverTimeOut && !receiverSetup)
    {
        int written = receiverPort.write("SSSSSSSSSS"); //force commands
        delay(100);
        if (receiverPort.available()>0)
        {
            for (int i=0;i<4;i++)
            {
                portName[i]=receiverPort.read();
            }
            while (receiverPort.available()){receiverPort.read();}
            snprintf(msgBuffer, 29, "sdio, %s, auto, SBF+NMEA\r\n", portName); //command for the receiver can accept commands and correction
            receiverPort.write(msgBuffer);
            delay(100);
            if (receiverPort.read()=='$' && receiverPort.read()=='@' &&  receiverPort.read()==':')
            {
                receiverSetup=true;
            }
            else 
            {
                while (receiverPort.available()>0){receiverPort.read();}
            }
        }
    }
    return receiverSetup;
}

bool SEPTENTRIO_NTRIP::resetNtripMsg()
{
    /**
     * @brief "reset" the ntripRequestBuffer
     * @return whether the ntripRequestBUffer is not a null pointer
     */
    if (ntripRequestBuffer!=nullptr)
    {
        ntripRequestBuffer=nullptr;
    }
    int ntripRequestMaxSize=ntripRequestMaxSize=requestStandardMaxSize+ (ntripProperties->userCredent64!=nullptr)*RequestAuthMaxSize+ (ntripProperties->nmeaData!=nullptr)*nmeaMaxSize+ (ntripProperties->custom!=nullptr)*RequestCustomMaxSize;
    ntripRequestBuffer = new char[ntripRequestMaxSize];
    return ntripRequestBuffer!=nullptr;
}
bool SEPTENTRIO_NTRIP::createRequestMsg(const char* userAgent)
{
    /**
     * @brief Create the ntrip HTTP request for NTRIP 1 or 2
     * @param userAgent the name of the user to send in the message
     * @return Whether the message totally filled the buffer (meaning bytes were probably left out)
     */
    bool response=false;
    if (resetNtripMsg())
    {
        if (ntripProperties->ntripVer<2)
        {
            response=requestMsgVer1(userAgent);
        }
        else
        {
            response=requestMsgVer2(userAgent);
        }
    }
    return response;
}
bool SEPTENTRIO_NTRIP::requestMsgVer1(const char* userAgent)
{
    /**
    * @brief Sends a message to the NTRIP v1 caster for data
    * @param userAgent The name of the user to send in the message
    * @return Whether the message totally filled the buffer (meaning bytes were probably left out)
    **/
    int ntripRequestMaxSize=requestStandardMaxSize+ (ntripProperties->userCredent64!=nullptr)*RequestAuthMaxSize+ (ntripProperties->nmeaData!=nullptr)*nmeaMaxSize+ (ntripProperties->custom!=nullptr)*RequestCustomMaxSize;
    int writtenBytes = snprintf(ntripRequestBuffer, ntripRequestMaxSize, 
    "GET /%s HTTP/1.0\r\n"
    "Host: %s\r\n"
    "Ntrip-Version: Ntrip/%s\r\n"
    "User-Agent: %s\r\n"
    "%s%s%s"
    "Connection: close\r\n"
    "%s"
    "\r\n%s%s",
    ntripProperties->caster.Mountpoint ? ntripProperties->caster.Mountpoint : "", 
    ntripProperties->caster.Host, 
    ntripProperties->ntripVer==0 ? "1.0" : "1.1",
    userAgent,
    ntripProperties->userCredent64 ? "Authorization: Basic " : "",  ntripProperties->userCredent64 ? ntripProperties->userCredent64 : "", ntripProperties->userCredent64 ? "\r\n" : ":",
    ntripProperties->custom ? ntripProperties->custom : "",
    ntripProperties->nmeaData ? ntripProperties->nmeaData : "", ntripProperties->nmeaData ? "\r\n" : "");
    return (writtenBytes<ntripRequestMaxSize);
}
bool SEPTENTRIO_NTRIP::requestMsgVer2(const char* userAgent)
{
    /**
    * @brief Sends a message to the NTRIP v2 caster for sourcetable or data
    * @param userAgent The name of the user to send in the message
    * @return Whether the message totally filled the buffer (meaning bytes were probably left out)
    **/
    int ntripRequestMaxSize=requestStandardMaxSize+ (ntripProperties->userCredent64!=nullptr)*RequestAuthMaxSize+ (ntripProperties->nmeaData!=nullptr)*nmeaMaxSize+ (ntripProperties->custom!=nullptr)*RequestCustomMaxSize;
    int writtenBytes = snprintf(ntripRequestBuffer, ntripRequestMaxSize, 
    "GET /%s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Ntrip-Version: Ntrip/%s\r\n"
    "User-Agent: %s\r\n"
    "%s%s%s"
    "Connection: close\r\n"
    "%s%s%s"
    "%s\r\n",
    ntripProperties->caster.Mountpoint ? ntripProperties->caster.Mountpoint : "", 
    ntripProperties->caster.Host, 
    ntripProperties->ntripVer==2 ? "2.0" : "2.1",
    userAgent, 
    ntripProperties->nmeaData ? "Ntrip-GGA: " : "", ntripProperties->nmeaData ? ntripProperties->nmeaData : "", ntripProperties->nmeaData ? "\r\n" : "",
    ntripProperties->userCredent64 ? "Authorization: Basic " : "",  ntripProperties->userCredent64 ? ntripProperties->userCredent64 : "", ntripProperties->userCredent64 ? "\r\n" : "",
    ntripProperties->custom ? ntripProperties->custom : "");
    return (writtenBytes<ntripRequestMaxSize);
}
bool SEPTENTRIO_NTRIP::processAnswer(ntripTempBuffer_t *ntripTempBuffer, const uint8_t incomingByte)
{
    /**
     * @brief Processing the bytes of the caster's answer to request messages (NTRIP v1 or v2)
     * @param ntripTempBuffer The buffer containing the headers
     * @param incomingByte The byte to process
     * @return Whether the message is finished being processed (code 200)
     */
    bool msgDone;
    if (ntripTempBuffer->msgValid && ntripBuffer!=nullptr)
    {
        ntripBuffer->data[ntripBuffer->offset]=incomingByte;
        ntripBuffer->offset++;
        if (incomingByte=='\n' && ntripBuffer->data[ntripBuffer->offset-3]=='\n') //end of message
        {
            msgDone=true;
            if (ntripBuffer->contentType==sourcetable)
            {
                extractSentence(ntripBuffer);
                msgDone=true;
                //reset buffer
                ntripTempBuffer->offset=0; 
                ntripTempBuffer->msgValid=false;
                ntripTempBuffer->contentType=unknown;
                ntripTempBuffer->msgSize=0;
                ntripTempBuffer->data=nullptr;
                if (_printSerial)
                {
                    _debugPort->println(F("Sourcetable end"));
                }
            }
            if (ntripBuffer->contentType==text_html) //process non-text message
            {
                if (_printDebug)
                {
                    for (int i=0;i<ntripBuffer->offset;i++)
                    {
                        _debugPort->print(ntripBuffer->data[i]);
                    }
                }
                msgDone=true;
                //reset buffer
                ntripTempBuffer->offset=0; 
                ntripTempBuffer->msgValid=false;
                ntripTempBuffer->contentType=unknown;
                ntripTempBuffer->msgSize=0;
                ntripTempBuffer->data=nullptr;
            }
        }
    }
    else //process header
    {
        if (ntripProperties->ntripVer<2)
        {
            msgDone=processHeaderVer1(ntripTempBuffer, incomingByte);
        }
        else
        {
            msgDone=processHeaderVer2(ntripTempBuffer, incomingByte);
        }
    }
    return msgDone;
}
bool SEPTENTRIO_NTRIP::processHeaderVer1(ntripTempBuffer_t *ntripTempBuffer, const uint8_t incomingByte)
{
    /**
    * @brief Processes of header one incoming byte at a time according to NTRIP v1
    * @param ntripTempBuffer The buffer for storing the header and keeping track of the message's properties 
    * @param incomingByte The byte to process
    * @return Whether the message is recognized (code 200)
    **/
    bool msgDone=false;
    ntripTempBuffer->data[ntripTempBuffer->offset]=incomingByte;
    ntripTempBuffer->offset++;
    if (incomingByte=='\n') //end of one header
    {
        if (!ntripTempBuffer->msgValid)
        {
            if (ntripTempBuffer->offset<35) //35 should be the max of the 1st line 
            {
                if (strncmp((const char*)ntripTempBuffer->data,"ICY 200 OK\r\n", 17)==0) //Correct answer : ntrip connexion succesfull
                {
                    ntripTempBuffer->msgValid=true;
                    ntripProperties->connected=1;
                    if (_printDebug) 
                    {
                        _debugPort->println(F("The caster sent back the correct response : 200"));
                    }
                }
                else if (strncmp((const char*)ntripTempBuffer->data,"SOURCETABLE 200 OK\r\n", 17)==0) //Incorrect answer : ntrip connexion possible but connexion parameters are wrong
                {
                    ntripTempBuffer->msgValid=true;
                    ntripProperties->connected=0;
                    if (_printDebug)
                    {
                        _debugPort->println(F("The caster sent back the response for an incorrect request, a sourcetable"));
                    }
                }
                else //Error message
                { 
                    ntripProperties->connected=false;
                    if (_printDebug) 
                    {
                        for (int i=0;i<ntripTempBuffer->offset && ntripTempBuffer->data[ntripTempBuffer->offset]!='\n';i++)
                        {
                            _debugPort->print(ntripTempBuffer->data[ntripTempBuffer->offset]); //print error type 
                        }
                    }
                    ntripTempBuffer->offset=0; 
                    ntripTempBuffer->data=nullptr;
                    ntripTempBuffer->data = new uint8_t[ntripHeaderMaxSize]; 
                }
            }
            else //not the first header or valid message
            {
                ntripTempBuffer->data=nullptr;
                ntripTempBuffer->data=new uint8_t[ntripHeaderMaxSize]; 
            }
            ntripTempBuffer->contentType==unknown;
        }
        else if (ntripTempBuffer->data[ntripTempBuffer->offset-3]=='\n' && ntripTempBuffer->msgValid) //End of the headers
        {
            ntripTempBuffer->contentType = getContentType(ntripTempBuffer); 
            ntripTempBuffer->msgSize=getContentLengh(ntripTempBuffer);
            if (ntripTempBuffer->contentType==text_plain || ntripTempBuffer->contentType==text_html) 
            {
                if (ntripBuffer==nullptr)
                {
                    ntripBuffer = new ntripBuffer_t;
                }
                ntripBuffer->contentType=sourcetable;
                if (ntripBuffer->data==nullptr)
                {
                    ntripBuffer->data = new uint8_t[ntripTempBuffer->msgSize+1]; //TODO:add space for '\0' to indicate end
                }
                ntripTempBuffer->offset=0;
            }    
            else if (ntripTempBuffer->contentType==unknown && _printDebug)
            {
                _debugPort->println(F("Impossible content type, make sure you're not losing data in transmission"));
            }        
        }
    }
    return ntripTempBuffer->contentType!=unknown;
}
bool SEPTENTRIO_NTRIP::processHeaderVer2(ntripTempBuffer_t *ntripTempBuffer, const uint8_t incomingByte)
{
    /**
    * @brief Processes of header one incoming byte at a time according to NTRIP v2
    * @param tempBuffer The buffer for storing the header and keeping track of the message's properties 
    * @param incomingByte The byte to process
    * @return Whether the message is recognized (code 200)
    **/
     bool msgDone=false;
    ntripTempBuffer->data[ntripTempBuffer->offset]=incomingByte;
    ntripTempBuffer->offset++;
    if (incomingByte=='\n') 
    {
        if (!ntripTempBuffer->msgValid) 
        {
            if (ntripTempBuffer->offset<35) //35 max 1st line size?
            {
                if (strstr((const char*)ntripTempBuffer->data,"HTTP")!=nullptr && strstr((const char*)ntripTempBuffer->data,"200 OK\r\n")!=nullptr) 
                {
                    ntripTempBuffer->msgValid=true;
                    ntripProperties->connected=1;
                    if (_printDebug) 
                    {
                        _debugPort->println(F("The caster sent back the correct response : 200"));
                    }
                }
                else if (strncmp((const char*)ntripTempBuffer->data,"ICY 200 OK\r\n", 17)==0 )
                {
                    ntripTempBuffer->msgValid=true;
                    ntripProperties->ntripVer=1; //NTRIPv2->v1
                    ntripProperties->connected=1;
                    if (_printDebug)
                    {
                        _debugPort->println(F("The caster sent back the response for an incorrect request, switched from NTRIP 1 to NTRIP"));
                    }
                }
                else 
                {
                    if (_printDebug)
                    {
                        _debugPort->println(F("First line header indicates error:"));
                        for (int i=0;i<ntripTempBuffer->offset && ntripTempBuffer->data[i]!='\n';i++)
                        {
                            _debugPort->write(ntripTempBuffer->data[i]); //print error type 
                        }
                        _debugPort->write('\n'); //print error type 
                    }
                    ntripTempBuffer->offset=0;
                    ntripProperties->connected=-1;
                }
            }
            else //not the first line of headers, delete
            {
                ntripTempBuffer->data=nullptr;
                ntripTempBuffer->data = new uint8_t[ntripHeaderMaxSize]; 
            }
            ntripTempBuffer->contentType=unknown;
        }
        else if (ntripTempBuffer->data[ntripTempBuffer->offset-3]=='\n' && ntripTempBuffer->msgValid)
        {
            ntripTempBuffer->contentType=getContentType(ntripTempBuffer);
            ntripTempBuffer->msgSize=getContentLengh(ntripTempBuffer);
            ntripProperties->transferEncoding=getTransferEncoding(ntripTempBuffer);
            if (ntripTempBuffer->contentType!=unknown || ntripTempBuffer->contentType!=gnss_data) 
            {
                //TODO check for non gnss data
                if (ntripBuffer==nullptr)
                {
                    ntripBuffer = new ntripBuffer_t;
                }
                ntripBuffer->contentType=ntripTempBuffer->contentType;
                if (ntripBuffer->data==nullptr)
                {
                    ntripBuffer->data = new uint8_t[ntripTempBuffer->msgSize+1]; //add space for '\0' to indicate end
                }
                ntripTempBuffer->offset=0;
            }
            else if ((ntripTempBuffer->contentType==unknown) && _printDebug)
            {
                _debugPort->println(F("Impossible content type, make sure no data was lost in transmission"));
            }        
        }
    }
    return ntripTempBuffer->contentType!=unknown;
}
bool SEPTENTRIO_NTRIP::getTransferEncoding(ntripTempBuffer_t *ntripTempBuffer)
{
    /**
     * @brief check if the data will be transfered in chunks
     * @param ntripTempBuffer the buffer containing the header to search
     * @return if the data will be given in chunks
     */
    return strstr(reinterpret_cast<char*>(ntripTempBuffer->data), "Transfer-Encoding: chunked")!=nullptr;
}
CONTENT_TYPE_t SEPTENTRIO_NTRIP::getContentType(const ntripTempBuffer_t *ntripTempBuffer)
{
    /**
    * @brief finds the type of the incoming data, compatible with both NTRIP 1 and 2 
    * (text/plain for NTRIP 1 source table is converted to NTRIP 2 gnss/sourcetable)
    * @param ntripTempBuffer : the buffer containing the header's data
    * @return the type of data received
    **/
    CONTENT_TYPE_t contentType=unknown;
    if (strstr(reinterpret_cast<char*>(ntripTempBuffer->data), "gnss/data")!=nullptr)
    {
        contentType=gnss_data;
    }
    else if (strstr(reinterpret_cast<char*>(ntripTempBuffer->data), "gnss/sourcetable")!=nullptr)
    {
        contentType=sourcetable;
    }
    else if (strstr(reinterpret_cast<char*>(ntripTempBuffer->data), "text/html")!=nullptr)
    {
        contentType=text_html;
    }
    else if (strstr(reinterpret_cast<char*>(ntripTempBuffer->data), "text/plain")!=nullptr)
    {
        contentType=text_plain;
    }
    return contentType;
}
int SEPTENTRIO_NTRIP::getContentLengh(const ntripTempBuffer_t *ntripTempBuffer)
{
    /**
    * @brief Extracts the length of the sourcetable from the header
    * @param ntripTempBuffer : the buffer for the header
    * @return the length of the sourcetable in byte (excluding ENDSOURCETABLE)
    **/
    int contentLength=17;
    char contentLine[contentLength]="Content-Length: "; 
    int i;
    int j=0;
    for (i=0; i<ntripTempBuffer->offset && j!=contentLength-1; i++)
    {
        for (j=0;j<contentLength && ntripTempBuffer->data[i+j]==contentLine[j];j++)
        {;}
    }
    i--;
    if (i!=ntripTempBuffer->offset)
    {
        int length=0;
        char lengthy[7];
        for (int j=0; j<7 && ntripTempBuffer->data[i+(contentLength-1)+j]!='\r';j++)
        {
            lengthy[j]=ntripTempBuffer->data[i+j+(contentLength-1)];
        }
        return atoi(lengthy); 
    }
    else
    {
        return 0;
    }
}
void SEPTENTRIO_NTRIP::extractSentence(const ntripBuffer_t *ntripBuffer)
{
    /**
    * @brief Identifies, extract and processes sentence types from a source table
    * @param ntripBuffer : the buffer in which the sourcetable's data is stored
    */
    uint8_t subSentence[200]; //TODO
    int sentenceSize;
    int i=0;
    for (i=0;i<ntripBuffer->offset;i++)
    {
        sentenceSize=0;
        while (ntripBuffer->data[i+sentenceSize]!='\r' && ntripBuffer->data[i+sentenceSize+1]!='\n' && (i+sentenceSize)<ntripBuffer->offset)
        {
            sentenceSize++;
        }
        sentenceSize+=2; //+2 for \r and \n;
        memcpy(subSentence, &ntripBuffer->data[i], sentenceSize);
        //process sentence
        if (subSentence[0]=='S' && subSentence[1]=='T' && subSentence[2]=='R')
        {
            if (_printDebug)
            {processSTR(subSentence);}
            if (strstr(reinterpret_cast<const char*>(subSentence), reinterpret_cast<const char*>(ntripProperties->caster.Mountpoint))!=nullptr)
            {
                casterRequirement(subSentence);
            }
        }
        else if (subSentence[0]=='N' && subSentence[1]=='E' && subSentence[2]=='T')
        {
            if (_printDebug)
            {processNET(subSentence);}
        }
        else if (subSentence[0]=='C' && subSentence[1]=='A' && subSentence[2]=='S')
        {
            if (_printDebug)
            {processCAS(subSentence);}
        }
        i+=sentenceSize;
    }
}
void SEPTENTRIO_NTRIP::processCAS(const uint8_t *buffer, const int customField=0)
{
    /**
     * @brief Processing of CAS sentence from sourcetable, gets the fallback info
     * In debug mode, it prints all elements of the sentence
     * @param buffer The buffer containing the data
     * @param customField [Optional] number of custom field if known
     */
    if (_printDebug)
    {
        int i=4; //skip "CAS,"
        _debugPort->print(F("HHostname : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //caster internet host
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        _debugPort->print(F("Port number : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //port number
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        _debugPort->print(F("Identifier : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //caster identifier
        {
            _debugPort->print(buffer[i]);
            i++;
        }

        i++; //skip comma
        _debugPort->print(F("Operator name : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //name of agency operating the caster
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        _debugPort->print(F("NMEA needed : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //capacity of receiving nmea
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        _debugPort->print(F("Country : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //country code, should be 3 char
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        _debugPort->print(F("Latitude : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //latitude, north
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        _debugPort->print(F("Longitude : "));
        while (buffer[i]!=';' && buffer[i-1]!='"') //longitude, east
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip comma
        if (buffer[i]!='\r' || buffer[i]!='\n')
        {
            _debugPort->print(F("Fallback host : "));
            while ((buffer[i]!='\r' && buffer[i+1]!='\n') && (buffer[i]!=';' && buffer[i-1]!='"'))//fallback caster IP
            {
                _debugPort->print(buffer[i]);
                i++;
            }
            _debugPort->println("");
            i++; //skip comma
        }
        if (buffer[i]!='\r' && buffer[i]!='\n')
        {
            _debugPort->print(F("Fallback port : "));
            while (buffer[i]!=';' && buffer[i-1]!='"' && ((buffer[i]!='\r' &&  buffer[i+1]!='\n'))) //fallback port number
            {
                _debugPort->print(buffer[i]);
                i++;
            }
            _debugPort->println("");
            i++; //skip comma
        }
        for (int j=0;j<customField;j++) //custom fields
        {
            _debugPort->print(F("Custom field "));
            _debugPort->print(j);
            _debugPort->print(" : ");
            while ((buffer[i]!=';' &&  buffer[i-1]!='"') && (buffer[i]!='\r' &&  buffer[i+1]!='\n') )
            {
                _debugPort->print(buffer[i]);
                i++;
            }
            _debugPort->println("");
            i++; //skip comma
        }
        if (buffer[i]!='\r' && buffer[i]!='\n')
        {
            _debugPort->print("Misc : ");
            while ((buffer[i]!=';' && buffer[i-1]!='"') && (buffer[i]!='\r' &&  buffer[i+1]!='\n'))
            {
                _debugPort->print(buffer[i]);
                i++;
            }
            _debugPort->println("");
        }
    }
    index=0; //get fallback info
    for (int i=0;i<10;i++) 
    {
        for (int j=0; buffer[j]!=','; j++){index++;}
    }
    for (int i=0; buffer[index]!=','&&i<128; i++)
    {
        ntripProperties->fallback.Host[i]=buffer[index];
        index++;
    }
    char tempString[12];
    for (int i=0;i<12 && buffer[index]!=','; i++)
    {
        tempString[i]=static_cast<char>buffer[index+i];
    }
    ntripProperties->fallback.Port=atoi(tempString);
}
void SEPTENTRIO_NTRIP::processNET(const uint8_t *buffer, int customField=0)
{
    /**
     * @brief Processing of NET sentence from sourcetable, only display info in debug mode
     * @param buffer The buffer containing the data
     * @param customField [Optional] number of custom field if known
     */
    if (_printDebug)
    {
        int i=4;
        _debugPort->print(F("networkID : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        _debugPort->print(F("Operator name : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        _debugPort->print(F("Authentification : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        _debugPort->print(F("Fee : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        _debugPort->print(F("WebNet : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        _debugPort->print(F("webStr : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        _debugPort->print(F("WebReg : "));
        while (buffer[i]!=';' && buffer[i-1]!='"')
        {
            _debugPort->print(buffer[i]);
            i++;
        }
        _debugPort->println("");
        i++; //skip semi-colon
        for (int j=0;j<customField;j++)
        {
            while (buffer[i]!=';' && buffer[i-1]!='"')
            {
                _debugPort->print(buffer[i]);
            }
            i++; //skip semi-colon
            _debugPort->println("");
        }
        if (buffer[i]!='\r' && buffer[i]!='\n')
        {
            _debugPort->print("Misc : ");
            while ((buffer[i]!=';' && buffer[i-1]!='"') && (buffer[i]!='\r' && buffer[i+1]!='\n'))
            {
                _debugPort->print(buffer[i]);
                i++;
            }
            i++; //skip semi-colon
        }
    }
    
}
void SEPTENTRIO_NTRIP::processSTR(const uint8_t *buffer, int customField=0)
{
    /**
     * @brief Processes STR sentence, only display info in debug mode
     * @param buffer The buffer containing the data
     * @param customField [Optional] number of custom field if known
     */
    int i=4;
    _debugPort->print(F("Mountpoint : "));
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print(F("Identifier : "));
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print(F("Data format : "));
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print(F("Format details : "));
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print(F("Carrier : "));
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
   _debugPort->print("Navigation system : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Network : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Country : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Latitude : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Longitude : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Solution : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Generator : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
     _debugPort->println("");
    i++; //skip semi-colon*/
    _debugPort->print("Compression-Encryption algorithm : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Authentification : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Authentification : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    _debugPort->print("Bitrate : ");
    while (buffer[i]!=';' && buffer[i-1]!='"')
    {
        _debugPort->print(buffer[i]);
        i++;
    }
    _debugPort->println("");
    i++; //skip semi-colon
    if (buffer[i]!='\r' && buffer[i]!='\n')
    {
        for (int j=0; j<customField; j++)
        {
            _debugPort->print("Custom field ");
            _debugPort->print(j);
            _debugPort->print(" : ");
            while ((buffer[i]!=';' && buffer[i-1]!='"') && (buffer[i]!='\r' && buffer[i+1]!='\n'))
            {
                _debugPort->print(buffer[i]);
            }
            i++; //skip semi-colon
            _debugPort->println("");
        }
    }
    if (buffer[i]!='\r' && buffer[i]!='\n')
    {
        _debugPort->print("Misc : ");
        while ((buffer[i]!=';' && buffer[i-1]!='"') && (buffer[i]!='\r' && buffer[i+1]!='\n'))
        {
            _debugPort->print(buffer[i]);
            i++;
        }
    }
}
void SEPTENTRIO_NTRIP::casterRequirement(const uint8_t buffer[200])
{
    /**
     * @brief creates buffers for authentication or nmea if needed (mind authentication will not be filled if nothing was given)
     * @param buffer the buffer containing the data for the caster
     */
    int commaCount=0;
    int index=0;
    if (_printDebug)
    {
        _debugPort->println("Found the caster's mountpoint");
    }
    while (commaCount<11)
    {
        if (buffer[index]==',')
        {
            index++;
            commaCount++;
        }
    }
    if (buffer[index]=='1' && ntripProperties->nmeaData==nullptr)
    {
        ntripProperties->nmeaData = new char[nmeaMaxSize];
        ntripProperties->nmeaData[0]='\0';
        if (_printDebug)
        {
            _debugPort->println("In need of NMEA, receiver will be asked for GGA");
        }
        char temp[22];
        snprintf(temp, 22, "exeNMEAOnce,%s,GGA\r\n", _portName);
        SEPTENTRIO_NTRIP::_serialPort->write("SSSSSSSSSS");
        SEPTENTRIO_NTRIP::_serialPort->write(temp);
        delay(100);
        auto start=millis();
        index=0;
        while (SEPTENTRIO_NTRIP::_serialPort->available()>0 && (millis()-start)<3000)
        {
            while (SEPTENTRIO_NTRIP::_serialPort->peek()!='$')
            {
                ntripProperties->nmeaData[index]=_serialPort->read();
                index++;
            }
            if (_serialPort->read()=='$' && _serialPort->read()=='@' && _serialPort->read()=='?')
            {
                if (_printDebug)
                {
                    _debugPort->println("Asking for GGA not wokring");
                }
            }
        }
    }
    while (commaCount<14)
    {
        if (buffer[index]==',')
        {
            index++;
            commaCount++;
        }
    }
    if (buffer[index]=='B' && ntripProperties->userCredent64==nullptr)
    {
        ntripProperties->userCredent64 = new char[RequestAuthMaxSize];
        ntripProperties->userCredent64[0]='\0';
        if (_printDebug)
        {
            _debugPort->println("In need of basic authentication, please provide some and restart the program");
        }
    }
}

bool SEPTENTRIO_NTRIP::rtpConnectToCasterMsg(const char *userHost, const char *userAgent, const char* localPort, const char *userMountpoint=nullptr)
{
    int writtenBytes = snprintf(ntripRequestBuffer, sizeof(ntripRequestBuffer), 
    "SETUP rtsp://%s/%s RTSP/1.0\r\n"
    "CSeq: 1\r\n"
    "Ntrip-Version: Ntrip/%s\r\n"
    "Ntrip-Component: Ntripclient\r\n"
    "User-Agent: %s\r\n"
    "Transport: RTP/GNSS;unicast;client_port=%s\r\n"
    "%s%s%s"
    "%s%s%s\r\n",
    userHost, userMountpoint ? userMountpoint : "", 
    ntripProperties->ntripVer==2 ? "2.0" : "2.1",
    ntripProperties->userCredent64 ? "Authorization: Basic " : "",  ntripProperties->userCredent64 ? ntripProperties->userCredent64 : "", ntripProperties->userCredent64 ? "\r\n" : "",
    userAgent,
    localPort,
    ntripProperties->nmeaData ? "Ntrip-GGA: " : "", ntripProperties->nmeaData ? ntripProperties->nmeaData : "", ntripProperties->nmeaData ? "\r\n" : "");
    return writtenBytes<sizeof(ntripRequestBuffer);
}
bool SEPTENTRIO_NTRIP::rtpStartTransferToCastersMsg(char* sessionNum, const char *userHost, const char *userMountpoint=nullptr)
{
    /**
     * @brief builds the message to start the data transfer with the NTRIP caster
     * @return whether the buffer is full (probably meaning some bytes were left out)
     */
    int ntripRequestMaxSize=requestStandardMaxSize+ (ntripProperties->userCredent64!=nulpptr)*RequestAuthMaxSize+ (ntripProperties->nmeaData!=nullptr)*nmeaMaxSize+ (ntripProperties->custom!=nullptr)*RequestCustomMaxSize;
    char msgBuffer[ntripRequestMaxSize]; 
    int writtenBytes = snprintf(ntripRequestBuffer, sizeof(ntripRequestBuffer), 
    "PLAY rtsp://%s/%s RTSP/1.0\r\n"
    "CSeq: 2\r\n"
    "Session : %s\r\n"
    "\r\n",
    userHost, userMountpoint ? userMountpoint : "",
    sessionNum);
    return writtenBytes<sizeof(ntripRequestBuffer);
}
bool SEPTENTRIO_NTRIP::rtpEndTransferToCasterMsg(const char* userHost, const char *sessionNum, const char *userMountpoint=nullptr)
{
    /**
     * @brief buids message to end the connexion to the NTRIP caster (NTRIP 2)
     * @return whether the buffer is full (probably meaning some bytes were left out)
     */
    int ntripRequestMaxSize=ntripRequestMaxSize=requestStandardMaxSize+ (ntripProperties->userCredent64!=nulpptr)*RequestAuthMaxSize+ (ntripProperties->nmeaData!=nullptr)*nmeaMaxSize+ (ntripProperties->custom!=nullptr)*RequestCustomMaxSize;
    char msgBuffer[ntripRequestMaxSize]; 
    int writtenBytes = snprintf(ntripRequestBuffer, sizeof(ntripRequestBuffer),
    "TEARDOWN rtsp://%s/%s RTSP/1.0\r\n"
    "CSeq: 3\r\n"
    "Session: %s\r\n"
    "\r\n",
    userHost, userMountpoint ? userMountpoint : "",
    sessionNum
    );
    return writtenBytes<sizeof(ntripRequestBuffer);
}

    //  --  Debugging  --  //
void SEPTENTRIO_GNSS::enableDebug(Stream &debugPort)
{
    /**
     * @brief enable debug message on the chosen debugPort
     * @param debugPort the port where the debug messages will be printed
     **/
    SEPTENTRIO_GNSS::_debugPort = &debugPort;
    SEPTENTRIO_GNSS::_printDebug=true;
}
void SEPTENTRIO_GNSS::disableDebug()
{
    /**
     * @brief disables debug messages
     */
    SEPTENTRIO_GNSS::_printDebug=false;
}
void SEPTENTRIO_NTRIP::enableDebug(Stream &debugPort)
{
    /**
     * @brief enable debug message on the chosen debugPort
     * @param debugPort the port where the debug messages will be printed
     **/
    SEPTENTRIO_NTRIP::_debugPort = &debugPort;
    SEPTENTRIO_NTRIP::_printDebug=true;
    if (_printDebug)
    {
        _debugPort->println(F("Debug enabled"));
    }
}
void SEPTENTRIO_NTRIP::disableDebug()
{
    /**
     * @brief disables debug messages
     */
    SEPTENTRIO_NTRIP::_printDebug=false;
}