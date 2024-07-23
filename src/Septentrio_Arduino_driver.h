#ifndef SEPTENTRIO_ARDUINO_DRIVER_H
#define SEPTENTRIO_ARDUINO_DRIVER_H

#include <Arduino.h>
#include "septentrio_structs.h"

constexpr int headerSize = 8;
constexpr int cmdMaxSize = 100; //TODO: check really relevant
constexpr int nmeaMaxSize = 82; //TODO: check really relevant


//NTRIP Request sizes
constexpr int ntripHeaderMaxSize = 300; //TODO
constexpr int requestStandardMaxSize=500;
constexpr int RequestAuthMaxSize=50; //TODO: check
constexpr int RequestCustomMaxSize=100; //Added for potential other requirements of the ntrip caster

//Do-not-use values
constexpr long doNotUseTOW = 4294967295;
constexpr int doNotUseLong = 65535;
constexpr int doNotUseInt = 255;
constexpr float doNotUseFloat = -2e10;

//maximum possible revision of SBF data (as of July 2024)
constexpr int maxVersion = 3;

class SEPTENTRIO_GNSS
{
    private:
        Stream *_serialPort;
        Stream *_debugPort;
        bool _printDebug=false;

    public:
        //constructor
        SEPTENTRIO_GNSS();
        //destructor
        virtual ~SEPTENTRIO_GNSS();

        //receiver setup
        bool begin(Stream &serialport, const uint16_t maxWait=5000);
        bool isConnected(const uint16_t maxWait);
        bool canSendCommand(Stream &serialport, const String comPort, const uint16_t maxWait=5000);
        int checkCommandResponse(const uint8_t byte, int offset);
        bool enableSBF(Stream &serialPort, const String &portName, const uint16_t maxWait);
        bool disableSBF(Stream &serialPort, const String &portName, const uint16_t maxWait=5000);

        //conversion functions (see Septentrio documentation for more information)
        uint16_t u2Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        uint32_t u4Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        uint64_t u8Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        uint16_t i2Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        uint32_t i4Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        uint64_t i8Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        float f4Conv(const sbfBuffer_t *buffer, const uint16_t offset);
        double f8Conv(const sbfBuffer_t *buffer, const uint16_t offset);

        //Integrity functions
        bool isSBFValid(sbfBuffer_t *buffer, const uint8_t desiredID);
        bool checkId(sbfBuffer_t *buffer, const uint8_t desiredID=0);
        bool checkCRC(sbfBuffer_t *buffer);
        bool checkChecksum(nmeaBuffer_t *buffer);
        bool checkId(nmeaBuffer_t *buffer);
        bool checkCustomId(const nmeaBuffer_t *buffer, const char *customId);
        bool correctFloat(const double f);
    
        //Buffer
        tempBuffer_t *tempBuffer = nullptr;
        ntripBuffer_t *ntripBuffer = nullptr;
        nmeaBuffer_t *NMEABuffer = nullptr;
        sbfBuffer_t *SBFBuffer = nullptr;
        void fillBuffer(sbfBuffer_t *buffer, const uint8_t incomingByte);
        void fillBuffer(nmeaBuffer_t *buffer, const uint8_t incomingByte);
        bool checkNewByte(tempBuffer_t *tempBuffer, const uint8_t incomingByte, const uint16_t messageID);
        bool checkNewByte(tempBuffer_t *tempBuffer, const uint8_t incomingByte, const char* messageId, bool customId=false);
        bool fullBuffer(const sbfBuffer_t *buffer);
        void fillSBFProperties(sbfBuffer_t *buffer, const uint16_t messageID);

        //debugging
        void enableDebug(Stream &debugPort=Serial);
        void disableDebug();
};

class SEPTENTRIO_NTRIP
{
    private :
        Stream *_debugPort;
        bool _printDebug=false;

    public:
        //constructor
        SEPTENTRIO_NTRIP();
        //destructor
        virtual ~SEPTENTRIO_NTRIP();
        //connection
        bool connectToReceiver(Stream &receiverPort, int receiverTimeOut=5000);

        //NTRIP
        char* ntripRequestBuffer=nullptr;
        ntripTempBuffer_t *ntripTempBuffer=nullptr;
        ntripBuffer_t *ntripBuffer = nullptr;
        ntripProperties_t *ntripProperties=nullptr;
        bool resetNtripMsg();
        bool createRequestMsg(const char* userAgent);
        bool requestMsgVer1(const char* userAgent);
        bool requestMsgVer2(const char* userAgent);
        bool processAnswer(ntripTempBuffer_t *ntripTempBuffer, const uint8_t incomingByte);
        bool processHeaderVer1(ntripTempBuffer_t *ntripTempBuffer, const uint8_t incomingByte);
        bool processHeaderVer2(ntripTempBuffer_t *ntripTempBuffer, const uint8_t incomingByte);
        CONTENT_TYPE_t getContentType(const ntripTempBuffer_t *ntripTempBuffer);
        bool getTransferEncoding(ntripTempBuffer_t *ntripTempBuffer);
        int getContentLengh(const ntripTempBuffer_t *ntripTempBuffer);
        void extractSentence(const ntripBuffer_t *ntripBuffer);
        bool processSourcetable(ntripBuffer_t *ntripBuffer, const uint8_t incomingByte);
        void processCAS(const uint8_t buffer[200], const int customField=0);
        void processNET(const uint8_t buffer[200], const int customField=0);
        void processSTR(const uint8_t buffer[200], const int customField=0);
        void casterRequirement(const uint8_t buffer[200]);

        bool rtpConnectToCasterMsg(const char *userHost, const char *userAgent, const char* localPort, const char *userMountpoint);
        bool rtpStartTransferToCastersMsg(char* sessionNum, const char *userHost, const char *userMountpoint);
        bool rtpEndTransferToCasterMsg(const char* userHost, const char *sessionNum, const char *userMountpoint);
       
        //debugging
        void enableDebug(Stream &debugPort=Serial);
        void disableDebug();
};

#endif 
