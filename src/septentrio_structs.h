#ifndef __septentrio_structs_h__
#define __septentrio_structs_h__

#include "Septentrio_Arduino_driver.h"

struct sbfBuffer_t{
    union 
    {
        uint16_t fullByte;
        struct
        {
            uint16_t crcValid : 1;
            uint16_t timeValid : 1;
            uint16_t lengthValid : 1;
            uint16_t alreadyRead : 1;
            uint16_t type : 6;
            uint16_t ver : 2;
        }bits; 
    }properties;
    uint8_t *data=NULL;
    uint8_t msgSize=0; //Expected size, might be more than what 1 byte can contain
    uint8_t offset=0; //number of byte actually filled, might be more than what 1 byte can contain
};

struct nmeaBuffer_t{
    bool correctChecksum=false;
    uint8_t *data;
    uint8_t msgSize=0;
    uint8_t offset=0;
    bool alreadyRead=1;
};

typedef enum {SENTENCE_TYPE_UNDETERMINED, SBF_SENTENCE, NMEA_SENTENCE} SENTENCE_TYPE_t;
struct tempBuffer_t{
    uint8_t data[22]; //8 bytes header for sbf, [] bytes for nmea
    int headerCount=-1;
    SENTENCE_TYPE_t  sentenceType=SENTENCE_TYPE_UNDETERMINED;
    struct 
    {
        uint8_t full : 1; //not used
        uint8_t newData : 1;
    }properties;
};

typdef enum {unknown, gnss_data, sourcetable, text_html,text_plain} CONTENT_TYPE_t;
struct ntripBuffer_t{
    CONTENT_TYPE_t contentType=unknown; 
    int offset=0;
    uint8_t *data=nullptr;
};

typedef enum {HTTP, RTP, NTRIP1} NTRIP_PROTOCOL_t;
struct ntripProperties_t{
    NTRIP_PROTOCOL_t protocol=HTTP;
    uint8_t ntripVer=3; //3=ntrip 2.1; 2=ntrip 2.0; 1=ntrip 1.1; 0=ntrip 1.0;
    char *nmeaData=nullptr;
    char *Auth64=nullptr;
    struct {
        uint8_t Host[128];
        int Port=0;
    }fallback;
};

#endif
