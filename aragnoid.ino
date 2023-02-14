#include <Arduino.h>
#include "wiring_private.h"

#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>
/* Aragnoid: 
Tame that tractor project
Code to interface RTKsimple2b to an ARAG 400 to provide RTK precision GPS to a closed ecosystem sprayer
*/


/* Create an rtc object */
RTCZero rtc;

//create a serial 2 on pin 1 and 0
Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

#define STRINGSINGLE 20
#define MAXMESSAGESIZE 128

//global coordinates
//store long and lat as 2 char strings, one for units and another for decimals, we will not do calculations
//with these numbers and we need to preserve a number of digits greater than Arduino double (=float) can store
//this also solves the fact that Arduino has difficulty in dealing with floats in printf and scanf
char Time[STRINGSINGLE];
char Latitude[STRINGSINGLE];
char NSchar;
char Longitude[STRINGSINGLE];
char EWchar;
int Quality,NSat;
char HDOP[STRINGSINGLE];
char Altitude[STRINGSINGLE];
char unitchar;
char HWGS84[STRINGSINGLE];
char DGPSupdate[STRINGSINGLE];
char DGPSid[STRINGSINGLE];
//VTG
char Tracktrue[STRINGSINGLE];
char Trackmag[STRINGSINGLE];
char Knots[STRINGSINGLE];
char Speed[STRINGSINGLE];
char Speedunits='K';
char Modechar='D';
//ZDA
int Day=0;
int Month=0;
int Year=0;

bool toggle=true;
bool ready=false;
int cnt=0;
unsigned long lastTime=0;
int cntzda=0;

char msg[MAXMESSAGESIZE]; //a buffer that will get the msg contents
char buffer[MAXMESSAGESIZE]; //a buffer to hold incoming serial messages from RTKsimple


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //usb uart, will spit out debug messages as RTKsimple data is being parsed
  while (!Serial);
  rtc.begin(); // initialize RTC
  Serial1.begin(115200,SERIAL_8E1); //RX and TX on pin 13/14 , this is the connection to ARAG

  mySerial.begin(115200); //RX and TX on PIN 1/0, this will become the connection to RTKsimple
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  rtc.setHours(12);
  rtc.setMinutes(0);
  rtc.setSeconds(0);
  updatetime(); //only for debug in reality we should get this from rtksimple from the atomic clocks of the GPS satelites


  int n=0;
  //test parsing and fill global coordinates
  const char* testGPGGA= "$GPGGA,142435.90,5056.7191170,N,00446.6237262,E,1,14,0.8,25.571,M,45.50,M,,*63";
  n=parseGPGGA(testGPGGA);
  Serial.println(n);

  const char* testGNVTG= "$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E";
  n=parseGNVTG(testGNVTG);
  Serial.println(n);

  const char* testGPZDA= "$GPZDA,142436.00,05,02,2023,,*64";
  n=parseGPZDA(testGPZDA);
  Serial.println(n);

  Serial.println("Aragnoid started"); 

}

void loop() {

    //read incoming stream from rtksimple
    if (ready)
    {
        //did we get a ASCII message?
        if (buffer[0]=='$'){
          if (parseGPGGA(buffer)>1){
            Serial.println("GPGGA parsed");
           
          }
          else if (parseGNVTG(buffer)>1){
            Serial.println("GNVTG parsed");
            
          }
          else if (parseGPZDA(buffer)>1){
            Serial.println("GPZDA parsed");
            
          }
          else {
            Serial.println("Unable to parse message:");
            Serial.println(buffer);
          }
        }
        else {
          Serial.println("A binary message, ignoring");

        }
        ready = false;
    } else while (mySerial.available()) //read from simpleRTK
    {
        char c = mySerial.read();
        buffer[cnt++] = c;
        if ((c == '\n') || (cnt == sizeof(buffer)-1))
        {
            buffer[cnt] = '\0';
            cnt = 0;
            ready = true;
        }
    }


    //send to ARAG messages based on the global coordinates that we filled in from the parsed data
    if ( millis() - lastTime > 100){
      GPGGA(msg);
      sendnmea(msg);
      //sendnmea("test"); //10Hz
      GNVTG(msg);
      sendnmea(msg); //10Hz
      lastTime=millis();
      cntzda++;
      if (cntzda>9){
          GPZDA(msg);
          sendnmea(msg); //1 Hz
          cntzda=0;
          digitalWrite(LED_BUILTIN, toggle); 
          toggle=!toggle;
      }
    }
    

    

    //Serial1.println("$GPGGA,142513.00,5056.7191290,N,00446.6237365,E,1,14,0.8,25.600,M,45.50,M,,*61");
    //delay(dt1);
    //Serial1.println("$GNVTG,235.738,T,235.738,M,0.000,N,0.000,K,A*3D");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.20,5056.7191154,N,00446.6237248,E,1,14,0.8,25.578,M,45.50,M,,*6F");
    //delay(dt1);
    //Serial1.println("$GNVTG,238.603,T,238.603,M,0.001,N,0.003,K,A*3F");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.30,5056.7191167,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*6F");
    //delay(dt1);
    //Serial1.println("$GNVTG,14.744,T,14.744,M,0.006,N,0.012,K,A*38");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.40,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*67");
    //delay(dt1);
    //Serial1.println("$GNVTG,14.745,T,14.745,M,0.000,N,0.000,K,A*3D");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.50,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*66");
   // delay(dt1);
    //Serial1.println("$GNVTG,14.744,T,14.744,M,0.000,N,0.000,K,A*3D");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.60,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*65");
    //delay(dt1);
    //Serial1.println("$GNVTG,149.847,T,149.847,M,0.000,N,0.000,K,A*3D");
    //delay(dt);
    //Serial1.println("$GPGGA,142513.60,5056.7191269,N,00446.6237367,E,1,14,0.8,25.601,M,45.50,M,,*62");
    //delay(dt1);
    //Serial1.println("$GNVTG,14.739,T,14.739,M,0.000,N,0.000,K,A*3D");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.70,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*64");
    //delay(dt1);
    //Serial1.println("$GNVTG,14.739,T,14.739,M,0.000,N,0.000,K,A*3D");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.80,5056.7191167,N,00446.6237265,E,1,14,0.8,25.576,M,45.50,M,,*64");
    //delay(dt1);
    //Serial1.println("$GNVTG,110.297,T,110.297,M,0.000,N,0.001,K,A*3C");
    //delay(dt);
    //Serial1.println("$GPGGA,142435.90,5056.7191170,N,00446.6237262,E,1,14,0.8,25.571,M,45.50,M,,*63");
    //delay(dt1);
    //Serial1.println("$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E");
    //delay(dt4);
    //Serial1.println("$GPZDA,142436.00,05,02,2023,,*64");

    //binary message, lsb first, see NOVATEL description of NMEA protocol
  /*
    byte binmsg[]= {0xAA,0x44,0x12,0x1C,0xC4,0x04,0x02,0x20, //sync, length of header 0x1c=16+12=28,message id=0xC404=1220=???, msg type=0x02=original message, source 2, binary, port adress=20=com1
                    0x38,0x00,0x00,0x00,0x6E,0xB4,0xC8,0x08, //msg length=0x3800=56 bytes, sequence id=0, idle time=0x6E, time status 0xB4=180=FINESTEERING, gps ref week=0xC808                   0xF0,0xD6,0x17,0x03,0x00,0x00,0x00,0x00,
                    0xF3,0x16,0x83,0x25,0x00,0x00,0x00,0x00, //GPSec=0xF3168325, 
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x4A,0xFA,0x2C,0x3F}; //last 4 bytes is CRC32 checksum
    Serial1.write(binmsg,sizeof(binmsg)); 
    delay(dt4);
*/
    digitalWrite(LED_BUILTIN, toggle); 
    toggle=!toggle;
    //Serial.println("arag...");
    //delay(1000);
    //send out to ARAG
    //sendnmea(GPGGA(msg));
    //sendnmea(GNVTG(msg));
    //sendnmea(GPZDA(msg));
  
    //update time
    //updatetime(); //only for debug in reality we should get this from rtksimple from the atomic clocks of the GPS satelites
}

void updatetime(){
  //fill time from a local real time clock running on Arduino
  //in real operation we want the clock to be given by the gps and NOT by the inacurate rtc
  //use only for debugging purposes
  sprintf(Time,"%2d%2d%2d.00",rtc.getHours(),rtc.getMinutes(),rtc.getSeconds()); //no millisecond in rtc
  Serial.print("Time=");
  Serial.println(Time); 
  Day=rtc.getDay();
  Month=rtc.getMonth();
  Year=rtc.getYear();
}
void parseARAGcommands(const char* msg){
  //react to ARAG commands
  if (strcmp(msg,"\xAAD\x12\x1C\x04")) {
    Serial.println("Receive binary message from Arag, ignoring it");
    //"\xAAD\x12\x1C\x04\0\0\xC0 \0\0\0\x90\xE4\xB7A\x98#S\00\x12\x12\0(       \x12\0\x01\0\0\0\0\xC2\x01\0\x01\0\0\0\x08\0\0\0\x01\0\0\0\0\0\0\0\0\0\0\0\x01\0\0\0j\x82Dx":
    //sync, length of header 0x1c=16+12=28,message id=0x0400=4=???, msg type=0x02=original message, source 2, binary, port adress=20=com1
    
    // statements
  }
  else if (strcmp(msg,"log versiona once")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial1.println("#VERSIONA,COM1,0,56.0,FINESTEERING,2248,51929.543,00000000,3681,9603;3,GPSCARD,\"N1GA\",\"DEL13280066\",\"MCAGTP-3.01-22B\",\"3.906\",\"3.002\",\"2013/Mar/14\",\"14:22:01\",DB_USERAPPAUTO,\"SmartAg\",\"0\",\"\",\"1.101\",\"\",\"2011/Sep/29\",\"17:13:55\",USERINFO,\"No BT\",\"\",\"\",\"\",\"\",\"\",\"\"*0cd69629");
    Serial.println("Receive version request from Arag");
    // statements
  }
  else if (strcmp(msg,"unlogall com1 true") or strcmp(msg,"unlogall com2 true")){
    //do nothing
    delay(1);
  }
  else if (strcmp(msg,"nmeatalker auto")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive nmeatalker auto from Arag");
  }
  else if (strcmp(msg,"log com1 gpggalong ontime 0.1")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive log gpggalong from Arag");
  }
  else if (strcmp(msg,"log com1 gpvtg ontime 0.1")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive log gpvtg from Arag");
  }
  else if (strcmp(msg,"log com1 gpzda ontime 1")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive log gpzda from Arag");
  }
  else if (strcmp(msg,"log com1 tiltdatab ontime 1")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive log tiltdatab from Arag");
  }
  else if (strcmp(msg,"$PMDT,u,,,,0.0*7A")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial1.println("\r\n$PMDT,<,Tilt sensor not installed\r\n");
    Serial.println("Receive log PMDT from Arag");
  }
  else if (strcmp(msg,"pdpfilter enable")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive pdpfilter enable from Arag");
  }
  else if (strcmp(msg,"pdpmode relative auto")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive pdpmode relative from Arag");
  }
  else if (strcmp(msg,"sbascontrol disable")){
    Serial1.println("\r\n<OK\r\n[COM1]");
    Serial.println("Receive sbascontrol disable from Arag");
  }
  else{
    Serial.println("Received unknown message from Arag:");
     Serial.println(msg);
  }
}
int parseGPGGA(const char * m)
{
  int chk=0;
  int n=sscanf(m,"$GPGGA,%20[^,],%20[^,],%c,%20[^,],%c,%d,%d,%20[^,],%20[^,],%c,%20[^,],%c,%20[^,],%20[^*]*%d",
        Time,
        Latitude,
        &NSchar,
        Longitude,
        &EWchar,
        &Quality,
        &NSat,
        HDOP,
        Altitude,
        &unitchar,
        HWGS84,
        &unitchar,
        DGPSupdate,
        DGPSid,
        &chk
        );
    if (n!=15) {
      Serial.println("parsing failed to retrieve all 15 variables");
      //12 is also good if DGPS is not on
    }
    //check if all could be read
    //check if checksum was correct?
  return n;
}
void GPGGA(char * m)
{
  //prepare GPGGA msg depending on global position variables 
  
  sprintf(m, "GPGGA,%s,%s,%c,%s,%c,%d,%d,%s,%s,%c,%s,%c,%s,%s", Time,
                                                              Latitude,
                                                              NSchar,
                                                              Longitude,
                                                              EWchar,
                                                              Quality,
                                                              NSat,
                                                              HDOP, 
                                                              Altitude,
                                                              unitchar,
                                                              HWGS84,
                                                              unitchar,
                                                              DGPSupdate,
                                                              DGPSid);
  
  //202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR"; //checksum should be 61
  //mySerial.println(msg);
}

int parseGNVTG(const char * m)
{
  int chk=0;
  int n=sscanf(m,"$GNVTG,%20[^,],T,%20[^,],M,%20[^,],N,%20[^,],%c,%c*%d",
        Tracktrue,
        Trackmag,
        Knots,
        Speed,
        &Speedunits,
        &Modechar,
        &chk
        );
    if (n!=6) {
      Serial.println("GNVTG parsing failed to retrieve all 6 variables");
    }
    //check if all could be read
    //check if checksum was correct?
    //Serial.print(n);
  return n;
}

void GNVTG(char * m){
  //prepare GNVTG string
  //multi constellation
  //example $GNVTG,139.969,T,139.969,M,0.007,N,0.013,K,D*3D
  
  sprintf(m, "GNVTG,%s,T,%s,M,%s,N,%s,%c,%c",Tracktrue,Trackmag,Knots,Speed,Speedunits,Modechar);
}

int parseGPZDA(const char * m)
{
  int chk=0;
  int n=sscanf(m,"$GPZDA,%20[^,],%2d,%2d,%4d,,*%d",
        Time,
        &Day,
        &Month,
        &Year,
        &chk
        );
    if (n!=5) {
      Serial.println("GPZDA parsing failed to retrieve all 5 variables");
    }
    //check if all could be read
    //check if checksum was correct?
    //Serial.print(n);
  return n;

}


void GPZDA(char * m){
  //prepare GPZDA string
  //$GPZDA,204007.00,13,05,2022,,*62
  sprintf(m, "GPZDA,%s,%2.2d,%2.2d,%4d,,", Time, Day,Month,Year);
}


void sendnmea(const char * m)
{
  //send to ARAG
  Serial1.print('$'); //prequel
  Serial1.print(m); //msg
  Serial1.print('*'); //sequal
  Serial1.println(checksum(m), HEX); //convert checksum byte to hex

  //and also to USB for monitoring
  Serial.print('$'); //prequel
  Serial.print(m); //msg
  Serial.print('*'); //sequal
  Serial.println(checksum(m), HEX); //convert checksum byte to hex



  //showtext(msg); //and show on display
}

char checksum(const char* m)
{
  // calc checksum by XOR all bytes, but don't do the null termination character
  byte xorTemp = 0x00;
  // process all characters in char string
  for(int i = 0; m[i] != '\0'; i++){
    xorTemp ^= byte(m[i]);
    }
  return xorTemp;
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler()
{
  mySerial.IrqHandler();
}