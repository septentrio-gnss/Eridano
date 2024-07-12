/**
*/

#include <RingBuffer.h>
#include <SoftwareSerial.h>
#include <Septentrio_Arduino_driver.h>
#include <septentrio_structs.h>
#include <arduino_base64.hpp>
#include "secrets.h"
#include "WiFiS3.h"

#define baudrate 9600
#define rtcmTimeOut 5000 //in ms
#define CLIENT_BUFFER_SIZE 500 //should have to be changed
#define RTCM_MAX_SIZE 512*4 //TODO: copied from sparkfun
#define NMEA_MAX_SIZE 100 //82 is standard but allows more for custom messages ->put in c++
#define AUTH_MAX_SIZE 100 //TODO ->put in c++
#define replyTimeout 5000 //for the first connection

const byte rxPin = 13;
const byte txPin = 12;

//WiFi stuff
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;

WiFiClient ntripClient;
SEPTENTRIO_NTRIP myNTRIP;
SoftwareSerial mySerial(rxPin, txPin);

unsigned long start=millis();

void setup() {
  Serial.begin(baudrate);
  mySerial.begin(baudrate);
  while (!Serial){;} 

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    delay(3000); // wait for 3 seconds so as not to pound
  }
  Serial.println("Connected to WiFi");

  //myNTRIP.enableDebug(Serial);
  myNTRIP.connectToReceiver(mySerial);
  if (casterUser!="")
  {
    myNTRIP.ntripProperties->userCredent64 = new char[AUTH_MAX_SIZE];
    myNTRIP.ntripProperties->userCredent64= new char[AUTH_MAX_SIZE];
    myNTRIP.ntripProperties->userCredent64[0]='\0'; // '/0' used as marker of empty, you can also just comment this line and the previous one
  }
  myNTRIP.ntripProperties->caster.Host = casterHost;
  myNTRIP.ntripProperties->caster.Mountpoint=mountPoint;
  myNTRIP.ntripTempBuffer = new ntripTempBuffer_t;
  myNTRIP.ntripTempBuffer->data = new uint8_t[ntripHeaderMaxSize];
  myNTRIP.ntripProperties->nmeaData = new char[nmeaMaxSize];
  int ntripRequestMaxSize=requestStandardMaxSize+ (myNTRIP.ntripProperties->userCredent64!=nullptr)*RequestAuthMaxSize+ (myNTRIP.ntripProperties->nmeaData!=nullptr)*nmeaMaxSize+ (myNTRIP.ntripProperties->custom!=nullptr)*RequestCustomMaxSize;
  myNTRIP.ntripRequestBuffer = new char[ntripRequestMaxSize];
  strncpy(myNTRIP.ntripProperties->nmeaData, "$GPGGA,070803.453,5050.913,N,00443.930,E,1,12,1.0,0.0,M,0.0,M,,*62", nmeaMaxSize); //back-up nmea
  
  //Optionnal:
  //pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (!myNTRIP.ntripProperties->connected) //initialize connection message
  {
    /*if (!getCurrentGGA(5000))
    {
      Serial.println("WARNING : couldn't get the NMEA in time. The default ones will be used.");
    }*/
    myNTRIP.ntripProperties->connected = setupNtripClient();
    
  }
  if (Serial)
  {
    if (!ntripClient.connected())
    {
      ntripClient.connect(casterHost, casterPort); //connects to caster
    }
    if ((millis()-start)>rtcmTimeOut)
    {
      ntripClient.write(myNTRIP.ntripRequestBuffer, strlen(myNTRIP.ntripRequestBuffer));  //sends request for correction in case of timeout
      delay(100);
    } 
    if (myNTRIP.ntripProperties->transferEncoding) //if transfer in chunks
    {
      while (ntripClient.available()>0)
      {
        int chunkSize=getChunkSize();
        /*Serial.print("Chunk : ");
        Serial.println(chunkSize);*/
        if (chunkSize!=0)
        {
          for (int i=0; i<chunkSize;i++)
          {
            mySerial.write(ntripClient.read());
          }
          ntripClient.read(); //clear '\r'
          ntripClient.read();  //clear '\n'
          start=millis();
        }
      }
    }
    else 
    {
      while (ntripClient.available()>0)
      {
        mySerial.write(ntripClient.read());
      }
    }
    //if (Serial) {Serial.println("End of transmission");}
    ntripClient.flush(); //flush all data to send only full messages
    while (ntripClient.available()){ntripClient.read();}
  }
  delay(100);
}

bool setupNtripClient()
{
  bool connectionSuccess=false;
  if (!myNTRIP.ntripProperties->connected)
  {
    if (!ntripClient.connect(casterHost, casterPort))
    {
      Serial.println(F("Failed to connect to caster, will retry"));
      //DIGITAL_WRITE(LED_BUILTIN, LOW);
    }
    else
    {
      Serial.print(F("Connected through "));
      Serial.print(casterHost);
      Serial.print(F(":"));
      Serial.println(casterPort);
      Serial.print(F("Will use the mountpoint : "));
      Serial.println(mountPoint);
      if (myNTRIP.ntripProperties->userCredent64!=nullptr)
      {
        Serial.print(F("Using credentials : "));
        Serial.print(casterUser);
        Serial.print(F(" and "));
        Serial.println(casterUserPwd);

        //create credential string and fill it
        uint8_t userCredentials[sizeof(casterUser)+sizeof(casterUserPwd)-1];
        snprintf((char*)userCredentials, sizeof(casterUser)+1+sizeof(casterUserPwd), "%s:%s", casterUser, casterUserPwd);
        //encode through base64
        base64::encode(userCredentials, sizeof(userCredentials), myNTRIP.ntripProperties->userCredent64);
      }

      myNTRIP.createRequestMsg(USER_AGENT);
      Serial.println("Request message: ");
      Serial.println(myNTRIP.ntripRequestBuffer);

      //sending the request
      while (ntripClient.available()>0) {ntripClient.read();}
      ntripClient.write(myNTRIP.ntripRequestBuffer, strlen(myNTRIP.ntripRequestBuffer));

      //wait for reply
      unsigned long start=millis();
      while (!ntripClient.available())
      {
        if (millis() - start > replyTimeout)
        {
          Serial.println(F("Caster timed-out"));
          return false;
        }
      }
      start=millis();
      while (ntripClient.available()>0 && !connectionSuccess && (millis()-start)<replyTimeout)
      {
        connectionSuccess=myNTRIP.processAnswer(myNTRIP.ntripTempBuffer, ntripClient.read());
      }
      //failed to connect
      if (!connectionSuccess)
      {
        if (Serial)
        {
          Serial.println(F("Failed to connect to caster"));
        }
      }
      else 
      {
        //DIGITAL_WRITE(LED_BUILTIN, HIGH);
        if (Serial)
        {
            Serial.println(F("Successfully connected to caster"));         
        }
        if (myNTRIP.ntripBuffer->contentType==sourcetable)
        {
          getmountPointRequierements(myNTRIP.ntripProperties->caster.Mountpoint);
        }
      }
    }
  }
  else 
  {
    connectionSuccess=true;
    if (Serial)
    {
      Serial.println(F("Already connected to NTRIP"));
    }
  } 
  return connectionSuccess;
}

int getChunkSize()
{
  int chunksize=0;
  bool sizeInProgress=true;
  while (sizeInProgress)
  {
    uint8_t byte = ntripClient.read();
    if(byte >= '0' && byte <= '9') 
    {
      chunksize = chunksize*16+byte-'0';
    }
    else if(byte >= 'a' && byte <= 'f')
    {
      chunksize = chunksize*16+byte-'a'+10;
    }
    else if(byte >= 'A' && byte <= 'F')
    {
      chunksize = chunksize*16+byte-'A'+10;
    }
    else if(byte == '\r')
    {
      if (ntripClient.read()!='\n')
      {
        chunksize=0;
      }
      sizeInProgress=false;
    }
    else if (byte == ';')
    {
      while (ntripClient.read()!='\r'){;}
      if (ntripClient.read()!='\n')
      {
        chunksize=0;
      }
      sizeInProgress=false;
    }
    else
    {
      chunksize=0;
      sizeInProgress=false;
    }
  }
  return chunksize;

}

void getmountPointRequierements(const char *mountPoint) //myNTRIP.ntripProperties->caster.Mountpoint)
{
  char buffer[strlen(myNTRIP.ntripRequestBuffer)];
  snprintf(buffer, strlen(buffer), "GET / HTTP/1.1\r\n"
                  "Host: %s\r\n"
                  "Ntrip-Version: Ntrip/%s\r\n"
                  "User-Agent: %s\r\n"
                  "Connection: close\r\n\r\n",
                  myNTRIP.ntripProperties->caster.Host,
                  myNTRIP.ntripProperties->ntripVer==0 ? "1.0" : (myNTRIP.ntripProperties->ntripVer==1 ? "1.1" : (myNTRIP.ntripProperties->ntripVer==2 ? "2.0" : "2.1")),
                  USER_AGENT);
  ntripClient.write(buffer, strlen(buffer));
  delay(100);
  while (ntripClient.available()>0)
  {
    myNTRIP.processAnswer(myNTRIP.ntripTempBuffer, ntripClient.read());
  }
}

bool getCurrentGGA(unsigned int nmeaTimeOut)
{
  char msgBuffer[24];
  bool nmeaAcquired=false;
  int index=0;
  unsigned int start;

  mySerial.write("SSSSSSSSSS"); //get port and force commands
  delay(100);
  strncpy(msgBuffer, "exeNMEAOnce, ", 13); //create message
  for (int i=0;i<4;i++)
  {
    msgBuffer[13+i]=mySerial.read();
  }
  while (Serial.available()>0) {Serial.read();}
  strncpy(msgBuffer+13+3, ", GGA\r\n", 7);
  Serial.println(msgBuffer);
  while ((millis()-start)<nmeaTimeOut && !nmeaAcquired)
  {
    mySerial.write(msgBuffer); //send request
    delay(100);
    while (mySerial.available()>0 && !nmeaAcquired)
    {
      while (mySerial.read()!='\n'){;} //skip repeat in the first line

      while (mySerial.read()!=','){;} //skip header

      if (mySerial.peek()!=',') //make sure we have a valid nmea by checking time is not empty
      {
        strncpy(myNTRIP.ntripProperties->nmeaData, "$GPGGA,", 7);
        int index=7;
        while (mySerial.peek()!='\r')
        {
          myNTRIP.ntripProperties->nmeaData[index]=mySerial.read();
        }
        nmeaAcquired=true;
      }
      else
      {
        delay(250); //wait a bit for NMEA data to be computed
      }
    }
  }
  return nmeaAcquired;  
}