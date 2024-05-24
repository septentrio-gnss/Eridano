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
    uint8_t data[10]; //8 bytes header for sbf, [] bytes for nmea
    int headerCount=-1;
    SENTENCE_TYPE_t  sentenceType=SENTENCE_TYPE_UNDETERMINED;
    struct 
    {
        uint8_t full : 1; //not used
        uint8_t newData : 1;
    }properties;
};

#endif
