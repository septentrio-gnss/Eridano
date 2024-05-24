#include <Septentrio_Arduino_driver.h>

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
     * @brief ensures the connection to GNSS through serial and initializes the temporary buffer
     * @param serialPort The Port through which data will be exchanged 
     * @param maxWait The maximum amount of time to test before timing-out
     * @return Whether the connection with the serial port was successfull
     **/
    SEPTENTRIO_GNSS::_serialPort = &serialPort;
    SEPTENTRIO_GNSS::_signsOfLife = false;
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
     * @brief checks the connection to the GNSS through response to a command
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
    while (SEPTENTRIO_GNSS::_serialPort->availableForWrite()<3 && (millis()-start)<maxWait) //find better way to get out
    {
        if (SEPTENTRIO_GNSS::_serialPort->available()>0)
        {
            if (SEPTENTRIO_GNSS::_serialPort->peek()=='$')
            {
                return true;
            }
        }
        if (counter%5==0) //send regularly until response
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
                auto cmd="setDataInOut, "+comPort+", CMD";
                SEPTENTRIO_GNSS::_serialPort->write(("setDataInOut, "+comPort+", CMD").c_str()); //TODO: allow for different port
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
     * @brief enables SBF output through port portName, SBF must not be already enabled
     * @param serialPort The serial port through which to send command and receive response
     * @param portName The port through which SBF data will be sent (GNSS port) 
     * @param maxWait Maximum wait-time for the response
     * @return Whether the command was successfull
     **/
    bool success=false;
    if (canSendCommand(serialPort, portName, 5000))
    {
        bool gotResponse=false;
        uint8_t finalCmd[msgMaxSize];
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
     * @brief disnables SBF output through port portName
     * @param serialPort The serial port through which to send command and receive response
     * @param portName The port through which SBF data will be snippet 
     * @param maxWait Maximum wait-time for the response
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
     * @brief extracts 2-byte unsigned integer (named u2 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
     * @return The corresponding 2-bytes unsigned integer
     **/
    return (buffer->data[offset]| buffer->data[offset+1]<<8);
}
uint32_t SEPTENTRIO_GNSS::u4Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 4-bytes unsigned integer (named u4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
     * @return The corresponding 4-bytes unsigned integer
     **/
    return (buffer->data[offset]| buffer->data[offset+1]<<8 | static_cast<uint32_t>(buffer->data[offset+2])<<16 | static_cast<uint32_t>(buffer->data[offset+3])<<24);
}
uint64_t SEPTENTRIO_GNSS::u8Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 8-bytes unsigned integer (named u8 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
     * @return The corresponding 8-bytes unsigned integer
     **/
    return (buffer->data[offset] | buffer->data[offset+1]<<8 | (static_cast<uint32_t>(buffer->data[offset+2]))<<16 | (static_cast<uint32_t>(buffer->data[offset+3]))<<24 | (static_cast<uint64_t>(buffer->data[offset+4]))<<32 | (static_cast<uint64_t>(buffer->data[offset+5]))<<40 | (static_cast<uint64_t>(buffer->data[offset+6]))<<48 | (static_cast<uint64_t>(buffer->data[offset+7]))<<56);
}
uint16_t SEPTENTRIO_GNSS::i2Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 2-bytes signed integer (named i2 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
     * @return The corresponding 2-bytes signed integer
     **/
    return (buffer->data[offset]|(buffer->data[offset+1])<<8);
}
uint32_t SEPTENTRIO_GNSS::i4Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 4-bytes signed integer (named i4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
     * @return The corresponding 4-bytes signed integer
     **/
    return (buffer->data[offset] | buffer->data[offset+1]<<8 | (static_cast<uint32_t>(buffer->data[offset+2]))<<16 | (static_cast<uint32_t>(buffer->data[offset+3]))<<24);
}
uint64_t SEPTENTRIO_GNSS::i8Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 8-bytes signed integer (named i8 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
     * @return The corresponding 8-bytes signed integer
     **/
    return buffer->data[offset] | buffer->data[offset+1]<<8 | (static_cast<uint32_t>(buffer->data[offset+2]))<<16 | (static_cast<uint32_t>(buffer->data[offset+3]))<<24 | (static_cast<uint64_t>(buffer->data[offset+4]))<<32 | (static_cast<uint64_t>(buffer->data[offset+5]))<<40 | (static_cast<uint64_t>(buffer->data[offset+6]&0xFF))<<48 | (static_cast<uint64_t>(buffer->data[offset+7]&0xFF))<<56;
}
float SEPTENTRIO_GNSS::f4Conv(const sbfBuffer_t *buffer, const uint16_t offset)
{
    /**
     * @brief extracts 4-bytes float (named f4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
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
     * @brief extracts 8-bytes float (named f4 in documentation) from buffer
     * @param buffer The buffer containing the data
     * @param offset offset of the first byte
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
     * @brief check the validity flag of the buffer and the ID
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
//generic function, checks for the most common IDs
bool SEPTENTRIO_GNSS::checkId(nmeaBuffer_t *buffer)
{
    /**
    * @brief Generic function to check for GNSS and INS ID
    * @param buffer The buffer containing the ID that need to be checked
    * @return Whether the buffer's ID is valid or not
    **/
    bool validId=false;
    if (buffer->data[1]=='G')
    {
        if (buffer->data[2]=='P' || buffer->data[2]=='N' || buffer->data[2]=='L' || buffer->data[2]=='A' || buffer->data[2]=='B' || buffer->data[2]=='I' || buffer->data[2]=='Q')
        {
            validId=true;
        }
    }
    else if(buffer->data[1]=='B' && buffer->data[2]=='D')
    {
        validId=true;
    }
    else if (buffer->data[1]=='I' && buffer->data[2]=='N')
    {
        validId=true;
    }
    return validId;
}
//more efficient but need to know the ID
bool SEPTENTRIO_GNSS::checkCustomId(const nmeaBuffer_t *buffer, const char *customId)
{
    /**
    * @brief Function to check for chosen ID (custom or not)
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
        validId*=(buffer->data[i+1]==customId[i]);
    }
    return validId;
}
bool SEPTENTRIO_GNSS::correctFloat(const double f)
{
    /**
     * @brief Checks that the float is not a DO-NOT-USE value
     * @param f The float to check
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
                if ((tempBuffer->data[4] | (tempBuffer->data[5]&0x1F)<<8) == messageId)     /**Check it's the type of message we want**/ //TODO: cast to uint16_t
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
                            delete[] SBFBuffer->data; /**avoid memory overflow**/
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
        //else if (SBFBuffer!=nullptr && tempBuffer->headerCount==8) /**end of header reached**/
        /*{
            tempBuffer->headerCount++;
        }*/
    }
    return bufferFilled;
}
bool SEPTENTRIO_GNSS::checkNewByte(tempBuffer_t *tempBuffer, const uint8_t incomingByte, const char* messageId, bool customId)
{
    /**
     * @brief For NMEA messages : checks new byte and stores it or not according to the data's validity and if the message is the desired type
     * @param tempBuffer A temporary buffer that holds data until it can be decided whether to keep or discard it
     * @param incomingByte The incoming byte to process
     * @param messageId The type of message to keep (only for SBF for now)
     * @param customId Whether to messageId is a custom one, used to call validCustomId for non-generic talker id
     **/
    bool bufferFilled = false;
    if (incomingByte=='$') /**start of sentence**/
    {
        tempBuffer->headerCount=0;
    }
    else if ((incomingByte==messageId[0]) && (tempBuffer->headerCount==0)) 
    {
        tempBuffer->sentenceType==NMEA_SENTENCE;
        tempBuffer->data[0]=0x24; //check correct type
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
    else if ((tempBuffer->headerCount==0) && incomingByte!=messageId[0]) /** check for 'false positive' of new message while still processing a sentence, continue as before**/
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
                    NMEABuffer->correctChecksum=checkChecksum(NMEABuffer);
                    bufferFilled=true;
                    if (!customId)
                    {
                        if (checkId(NMEABuffer))
                        {
                            NMEABuffer->alreadyRead=0;
                        }
                    }
                    else
                    {
                        if(checkCustomId(NMEABuffer, messageId))
                        {
                            NMEABuffer->alreadyRead=0;
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
                delete[] NMEABuffer->data;
                delete[] NMEABuffer;
            }
            NMEABuffer = new nmeaBuffer_t;
            NMEABuffer->data = new uint8_t[nmeaMaxSize]; //TODO:check ok with all nmea messages
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
     */
    return tempBuffer->offset+1==tempBuffer->msgSize;
}
void SEPTENTRIO_GNSS::fillSBFProperties(sbfBuffer_t *buffer, const uint16_t messageId)
{
    /**
     * @brief Fills the properties field of the SBFBuffer_t structure ie. validity flags, type and indicate message hasn't been read
     * @param buffer The buffer whose field will be filled
     * @param messageId The type of message to check for 
     **/
    buffer->properties.bits.crcValid = (u2Conv(buffer, 2)==compute_CRC_16CCITT(buffer,buffer->offset));
    buffer->properties.bits.type = (buffer->data[4]&0xFF) | ((buffer->data[5]&0x1F)<<8);
    buffer->properties.bits.ver = (buffer->data[5]&0xe0)>>5; 
    SBFBuffer->properties.bits.timeValid = ((f4Conv(buffer, 8)!=doNotUseTOW) && (u2Conv(buffer, 12)!=doNotUseLong));
    buffer->properties.bits.type==messageId;
    buffer->properties.bits.alreadyRead=0;
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