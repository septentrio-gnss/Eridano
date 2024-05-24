//Parsing example of a Septentrio SBF block : PVTGeodetic
//This code only displays data, it does not keep it in memory
//
//This code was tested with an Arduino with only one serial port and used AltSoftSerial to remedy:
//If you have more than 1 serial port, you do not need this library 

#include <AltSoftSerial.h>
#include <Septentrio_Arduino_driver.h>
#include <septentrio_structs.h>

//Uncomment if you want to use another way to get a second serial port
//#define txPin 9
//#define rxPin 8
#define baudrate 9600 //make sure it is coherent with the GNSS baudrate

AltSoftSerial mySerial;
SEPTENTRIO_GNSS myGNSS;

int revision = 0;

void setup() {
  Serial.begin(baudrate);
  mySerial.begin(baudrate);
  Serial.println("Connections initiated");
  if (myGNSS.begin(mySerial)==false) //The serial port need to be able to send and receive data to work properly
  {
    Serial.println("Failed to connect to GNSS");
  }
  Serial.println("Connected to GNSS");
}

void loop() {
  while (mySerial.available()>0)
  {
    myGNSS.checkNewByte(myGNSS.tempBuffer, mySerial.read(), SBF_SENTENCE, 4007);
  }
  revision=myGNSS.SBFBuffer->data[5]>>5;

  if (Serial && myGNSS.SBFBuffer->properties.bits.alreadyRead==0)
  {
    Serial.print("Time of week : "); 
    uint64_t TOW=myGNSS.u4Conv(myGNSS.SBFBuffer, 8);
    if (TOW!=4294967295) {Serial.print(TOW*0.001);}
    else {Serial.print("INVALID");}
    Serial.println(" sec");

    Serial.print("Continuous week number : ");
    uint16_t WNc=myGNSS.u2Conv(myGNSS.SBFBuffer, 12);
    if (WNc!=doNotUseLong) {Serial.print(myGNSS.u2Conv(myGNSS.SBFBuffer, 12));}
    else {Serial.print("INVALID");}
    Serial.println(" weeks");
    
    Serial.print("PVT Solution : "); 
    Serial.println(myGNSS.SBFBuffer->data[14]&0x0F);
    Serial.print("Autobase : ");
    Serial.println(myGNSS.SBFBuffer->data[14]&0x40);
    Serial.print( "2D Flag : "); 
    Serial.println(myGNSS.SBFBuffer->data[14]&0x80);

    Serial.print( "Error : "); 
    Serial.println(myGNSS.SBFBuffer->data[15]);

    Serial.print("Latitude : ");
    Serial.print(myGNSS.f8Conv(myGNSS.SBFBuffer, 16), 12);
    Serial.println(" rad");
    Serial.print("Longitude : "); 
    Serial.print(myGNSS.f8Conv(myGNSS.SBFBuffer, 24), 12);
    Serial.println(" rad");
    Serial.print("Height : ");
    Serial.print(myGNSS.f8Conv(myGNSS.SBFBuffer, 32), 6);
    Serial.println(" m");
    Serial.print("Undulation : "); 
    Serial.print(myGNSS.f8Conv(myGNSS.SBFBuffer, 40), 6);
    Serial.println(" m");
    Serial.print("Vn : ");
    Serial.print(myGNSS.f4Conv(myGNSS.SBFBuffer, 44), 3);
    Serial.println(" m/s");
    Serial.print("Ve : ");
    Serial.print(myGNSS.f4Conv(myGNSS.SBFBuffer, 48), 3);
    Serial.println(" m/s");
    Serial.print("Vu : ");
    Serial.print(myGNSS.f4Conv(myGNSS.SBFBuffer, 52), 3);
    Serial.println(" m/s");
    Serial.print("COG : "); 
    Serial.print(myGNSS.f4Conv(myGNSS.SBFBuffer, 56), 3);
    Serial.println(" deg");
    Serial.print("Rx Clock Bias : ");
    Serial.print(myGNSS.f8Conv(myGNSS.SBFBuffer, 60), 6);
    Serial.println(" ms");
    Serial.print("Rx Clock Drift : "); 
    Serial.print(myGNSS.f4Conv(myGNSS.SBFBuffer, 68),3);
    Serial.println(" ppm");

    Serial.print("Time system : ");
    int TimeSystem = (int)myGNSS.SBFBuffer->data[72];
    if (TimeSystem!=doNotUseInt) {Serial.println(TimeSystem);}
    else {Serial.println("INVALID");}

    Serial.print("Datum : "); 
    int datum=myGNSS.SBFBuffer->data[73];
    if (datum!=doNotUseInt) {Serial.println(datum);}
    else {Serial.println("INVALID");}

    Serial.print("Number of services : ");
    uint8_t NrSv=(int)myGNSS.SBFBuffer->data[74];
    if (NrSv!=doNotUseInt) {Serial.println(NrSv);}
    else {Serial.println("INVALID");}

    Serial.print("WA Correction Info : ");
    uint8_t waCorrInfo=myGNSS.SBFBuffer->data[75];
    if (waCorrInfo!=0) {Serial.println(waCorrInfo);}
    else {Serial.println("INVALID");}

    Serial.print("Reference ID : "); 
    uint16_t refID=myGNSS.u2Conv(myGNSS.SBFBuffer, 76);
    if (refID!=doNotUseLong){Serial.println(refID);}
    else {Serial.println("INVALID");}

    Serial.print("Mean correction age : "); 
    uint16_t meanCorrAge=myGNSS.u2Conv(myGNSS.SBFBuffer, 78);
    if (meanCorrAge!=doNotUseLong){Serial.println(meanCorrAge);}
    else {Serial.println("INVALID");}

    Serial.print("Signal info : ");
    uint64_t signalInfo=myGNSS.u4Conv(myGNSS.SBFBuffer, 80);
    //if (signalInfo!=0) {Serial.println(TOW*1);}
    //else {Serial.println("INVALID");}

    Serial.print("Alert flag : "); 
    uint8_t alertFlag=myGNSS.SBFBuffer->data[84];
    if (signalInfo!=0) {Serial.println(alertFlag);}
    else {Serial.println("INVALID");}

    if (revision>=1)
    {
      Serial.print("Number of bases : "); 
      uint8_t nrBases=myGNSS.SBFBuffer->data[85];
      if (signalInfo!=0) {Serial.println(nrBases);}
      else {Serial.println("INVALID");}

      Serial.print("Signal info : ");
      uint16_t signalInfo=myGNSS.u2Conv(myGNSS.SBFBuffer,86);
      if (signalInfo!=0) {Serial.println(signalInfo);}
      else {Serial.println("INVALID");}
    }

    if (revision>=2)
    {
      Serial.print("Latency : "); 
      uint16_t latency=myGNSS.u2Conv(myGNSS.SBFBuffer,88);
      if (latency!=doNotUseLong){Serial.println(latency*0.0001);}
      else {Serial.println("INVALID");}

      Serial.print("HAccuracy : ");
      uint16_t hAccuracy=myGNSS.u2Conv(myGNSS.SBFBuffer,90);
      if (hAccuracy!=doNotUseLong){Serial.println(hAccuracy*0.01);}
      else {Serial.println("INVALID");}

      Serial.print("VAccuracy : ");
      uint16_t vAccuracy=myGNSS.u2Conv(myGNSS.SBFBuffer,92);
      if (vAccuracy!=doNotUseLong){Serial.println(vAccuracy*0.01);}
      else {Serial.println("INVALID");}
      Serial.print("Misc : ");
      Serial.println(myGNSS.SBFBuffer->data[95]);

    myGNSS.SBFBuffer->properties.bits.alreadyRead=1;
    }
    Serial.println();
  }
  //written as not to uselessly test if conditions while there is no new message
  delay(250); //Change depending on your message frequency
}
