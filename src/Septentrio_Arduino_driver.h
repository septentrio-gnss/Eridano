#ifndef SEPTENTRIO_ARDUINO_DRIVER_H
#define SEPTENTRIO_ARDUINO_DRIVER_H

#include "Arduino.h" 
#include "septentrio_structs.h"

constexpr int headerSize = 8;
constexpr int msgMaxSize = 100; //TODO: check really relevant
constexpr int nmeaMaxSize = 82; //TODO: check really relevant

constexpr long doNotUseTOW = 4294967295;
constexpr int doNotUseLong = 65535;
constexpr int doNotUseInt = 255;
constexpr float doNotUseFloat = -2e10;

constexpr int maxVersion = 3;

class SEPTENTRIO_GNSS
{
    private:
        Stream *_serialPort;
        Stream *_debugPort;
        bool _signsOfLife;
        bool _printDebug=false;

    public:
        //constructor
        SEPTENTRIO_GNSS();
        //destructor
        virtual ~SEPTENTRIO_GNSS();

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
        uint16_t i4Conv(const sbfBuffer_t *buffer, const uint16_t offset);
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

#endif 
