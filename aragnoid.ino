#include <Arduino.h>
#include "wiring_private.h"
#include <Arduino_CRC32.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>
/* Aragnoid: 
Tame that tractor project
Code to interface RTKsimple2b to an ARAG 400 to provide RTK precision GPS to a closed ecosystem sprayer
*/


/* Create an rtc object */
RTCZero rtc;

Arduino_CRC32 crc32;

//create a serial 2 on pin 1 and 0
Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

#define STRINGSINGLE 20
#define MAXMESSAGESIZE 128

//comment any of these if you don't want this function
//#define DEBUG //print parsing details                             JUY
#define NMEAUSB //copy nmea messages also on usb uart 
#define NORTK //drop RTK specifics to resemble more the novatel original..changed that quality is maintained
#define GYRO //use gyro attached to arduino for tilt


#define CRC32_POLYNOMIAL 0xEDB88320L //needed for novatel crc32 implementation
#define HEADER 28 //size of binary header in bytes


//global coordinates
//store long and lat as char strings
//we will not do calculations with these numbers and we need to preserve a number of digits greater than Arduino double (=float) can store
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
char xchar='P'; 
//ZDA
int Day=0;
int Month=0;
int Year=0;
//PUBX04
char Hms[STRINGSINGLE];
char Dmy[STRINGSINGLE];
char UTC_TOW[STRINGSINGLE];
unsigned long UTC_ms = 0;
int UTC_Week;
char Reserved[STRINGSINGLE];
char Clk_B[STRINGSINGLE];
char Clk_D[STRINGSINGLE];
int PG;

//GYRO
double AccX=0.0; //acceleration in X direction m/s^2
double AccY=0.0; //acceleration in Y direction m/s^2
double TiltX=0.0; //tilt in X direction in degrees
double TiltY=0.0; //tilt in Y direction in degrees
bool gyro=false;


bool toggle=true;
bool ready=false;
bool receivedarag=false;
bool receivedaragbinary=false;
bool binmode=false;

int cnt=0;
int cntarag=0;

unsigned long lastTime=0;
int cntzda=0;

char msg[MAXMESSAGESIZE]; //a buffer that will get the msg contents
char buffer[MAXMESSAGESIZE]; //a buffer to hold incoming serial messages from RTKsimple
char bufferarag[MAXMESSAGESIZE]; //a buffer to hold incoming serial messages from RTKsimple



/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //usb uart, will spit out debug messages as RTKsimple data is being parsed
  while (!Serial);
  rtc.begin(); // initialize RTC
  Serial1.begin(115200,SERIAL_8E1); //RX and TX on pin 13/14 , this is the connection to ARAG
  while (!Serial1);
//Serial1.begin(9600,SERIAL_8E1); //RX and TX on pin 13/14 , this is the connection to ARAG

  mySerial.begin(115200); //RX and TX on PIN 1/0, this is the connection to RTKsimple
  while (!mySerial);
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  rtc.setHours(12);
  rtc.setMinutes(0);
  rtc.setSeconds(0);
  updatetime(); //only for debug in reality we should get this from rtksimple from the atomic clocks of the GPS satelites

  debugtest();

  //start the gyro if needed
  #ifdef GYRO
  if(bno.begin())
  {
    Serial.println("BNO055 detected");
    delay(1000);
    bno.setExtCrystalUse(true);
    gyro=true;
  }else
  {
    Serial.println("No gyro found, continuiing without");
    gyro=false;
  }
  #endif


  Serial.println("Aragnoid started"); 
}

void loop() {

    //read incoming stream from rtksimple
    if (ready)
    {
        //did we get a ASCII message?
        if (buffer[0]=='$'){
          //Serial.println(strlen(buffer));
          //Serial.println(buffer);

          //Serial.println("\n\n buffer input");
          //Serial.println(buffer);

          if(buffer[3] == 'G'){
              parseGPGGA(buffer);            
          }else if(buffer[3] == 'V'){
              parseGNVTG(buffer);
          }else if(buffer[3] == 'Z'){
              parseGPZDA(buffer);
          }else if(buffer[3] == 'B'){
              parsePUBX(buffer);
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
        if (c == '\n')
        {   
            if (cnt>2){
              buffer[cnt-2] = '\0'; //get rid of /r/n from the serial message
              ready = true;
            }
            else{
              buffer[0]='\0'; //meaningless ascii without /r/n ending, ignoring
              ready = false;
            }
            cnt = 0;
            
        }
        else if (cnt == sizeof(buffer)-1){
          cnt=0;
          buffer[0]='\0';
          ready=false;
        }
    }

//listen to ARAG messages during startup
  if (receivedarag)
    {
      Serial.println("parsing ascii Arag commands");
      //did we get a ASCII message? 
      parseARAGcommands(bufferarag);
      receivedarag = false;
    } 
  else if (receivedaragbinary)
    {
      Serial.print("got a binary arag command of 64 bytes");
      Serial.println("returning it to arag to signal that we understood it");
      Serial1.write(bufferarag,64);
    //"\xAAD\x12\x1C\x04\0\0\xC0 \0\0\0\x90\xE4\xB7A\x98#S\00\x12\x12\0(       \x12\0\x01\0\0\0\0\xC2\x01\0\x01\0\0\0\x08\0\0\0\x01\0\0\0\0\0\0\0\0\0\0\0\x01\0\0\0j\x82Dx":
    //sync, length of header 0x1c=16+12=28,message id=0x0400=4=???, msg type=0x02=original message, source 2, binary, port adress=20=com1
      receivedaragbinary = false;
  }
  else while (Serial1.available()) //read from ARAG
  //but try to make the difference between a digital message or an ascii one
  //in ascii several characters are forbidden like 0xAA which signals the start of a binary message
  //on the other hand, received serial ascii strings are ending with \r\n while this could be a valid binary part of the data
    {
        char c = Serial1.read();
        bufferarag[cntarag++] = c;
        if (cntarag == sizeof(bufferarag)-1)
        {
          //buffer overflow, starting from scratch and ignore this message which is apparently wrong
          cntarag = 0;
          bufferarag[0] = '\0'; //empty string
          receivedarag = false;
          receivedaragbinary=false;
          binmode=false;
        }
        else if (c == '\xAA'){
          //start of a new binary message, but what does this mean about the message that is now in the buffer?
          //if the existing message was ascii than it was terminated with \n and then we already handled it in the previous character
          bufferarag[0] = c;
          cntarag=1; //we already received the first byte
          receivedaragbinary=false;
          binmode=true;
          receivedarag = false;
        }
        else if ((binmode==true) && (cntarag>63)){
          //end of a binary message
          receivedaragbinary=true;
          cntarag=0;
          receivedarag = false;
          binmode=false;
        }
        else if ((c=='\n') && (binmode==false))
        {
          //end of an ascii command but only if we are in ascii mode
          
          //it could work with [cntarag-1]='\0' novatel ends ascii string with \r\n
          //-1 worked in the Wokwi simulator, but real response on HW may differ
          if (cntarag>2){
            bufferarag[cntarag-2] = '\0'; //end with null char to make it a valid string
            receivedarag = true;
          }
          else{
            bufferarag[0]='\0';
            receivedarag = false;
          }
          cntarag = 0;
          receivedaragbinary=false;
        }
    }





    //send to ARAG messages based on the global coordinates that we filled in from the parsed data
    if ( millis() - lastTime > 100){
      GNVTG(msg);
      sendnmea(msg); //10Hz
      GPGGA(msg);
      sendnmea(msg);
      #ifdef GYRO
      if (gyro) {
        readgyro();
      }
      #endif
      
      lastTime=millis();
      cntzda++;
      if (cntzda>9){
          GPZDA(msg);
          sendnmea(msg); //1 Hz
          sendbinary(); //note at the moment contains wrong time, maybe this binary isnt even nessecary as it is an empty tiltdata message with empty contents
          cntzda=0;
          digitalWrite(LED_BUILTIN, toggle); 
          toggle=!toggle;
      }
    }
    
    //updatetime(); //only for debug in reality we should get this from rtksimple from the atomic clocks of the GPS satelites
}

void sendbinary(){
  //binary message, lsb first, see NOVATEL description of NMEA protocol
  //Novatel is little endian, lsb first
  byte binmsg[]= {  
    0xAA,0x44,0x12,0x1C,0xC4,0x04,0x02,0x20,//sync, length of header 0x1c=16+12=28,message id=0xC404=1220=Tiltdata, msg type=0x02=original message, source 2, binary, port adress=20=com1
    0x38,0x00,0x00,0x00,0x6B,0xB4,0xC8,0x08,//msg length=0x3800=56 bytes, sequence id=0, idle time=0x6E, time status 0xB4=180=FINESTEERING, gps ref week=0xC808
    0xA0,0x8A,0x18,0x03,0x00,0x00,0x00,0x00,//GPSec=0xAO8A1803,
    0xF3,0x16,0x83,0x25,0x00,0x00,0x00,0x00,//reserved-what is this? stays the same on novatel 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //all data is zero, meaning novatel does not send calculated tilt data as it doesnt have the sensors for that
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xC6,0x43,0x16,0x5F}; //last 4 bytes is CRC32 checksum

  //fill in reference week 2 byte , little endian
  binmsg[14]=UTC_Week & 0x00FF;
  binmsg[15]=UTC_Week>>8 & 0x00FF;
  #ifdef DEBUG                               
  Serial.print("ref week binary:");
  Serial.print(binmsg[14],HEX);
  Serial.print(binmsg[15],HEX);
  Serial.println();
  #endif 
  //fill in GPSms ms since ref week 4byte, little endian
  binmsg[16]=UTC_ms & 0x000000FF;
  binmsg[17]=UTC_ms>>8 & 0x000000FF;
  binmsg[18]=UTC_ms>>16  & 0x000000FF;
  binmsg[19]=UTC_ms>>24     & 0x000000FF;
  #ifdef DEBUG
  Serial.print("ref UTC_ms binary:");
  Serial.print(binmsg[16],HEX);
  Serial.print(binmsg[17],HEX);
  Serial.print(binmsg[18],HEX);
  Serial.print(binmsg[19],HEX);
  Serial.println();
  #endif 

#ifdef GYRO
  //convert double to bytes
  byte* AccXpointer = (byte*)&AccX;
  byte* AccYpointer = (byte*)&AccY;
  byte* TiltXpointer = (byte*)&TiltX;
  byte* TiltYpointer = (byte*)&TiltY;
  //AccX 8 byte double, float and double  are little endian on arduino
  size_t id=HEADER+4;
  binmsg[id++]=AccXpointer[0];
  binmsg[id++]=AccXpointer[1];
  binmsg[id++]=AccXpointer[2];
  binmsg[id++]=AccXpointer[3];
  binmsg[id++]=AccXpointer[4];
  binmsg[id++]=AccXpointer[5];
  binmsg[id++]=AccXpointer[6];
  binmsg[id++]=AccXpointer[7];
  //and AccY
  binmsg[id++]=AccYpointer[0];
  binmsg[id++]=AccYpointer[1];
  binmsg[id++]=AccYpointer[2];
  binmsg[id++]=AccYpointer[3];
  binmsg[id++]=AccYpointer[4];
  binmsg[id++]=AccYpointer[5];
  binmsg[id++]=AccYpointer[6];
  binmsg[id++]=AccYpointer[7];
  //and Tiltx
  id=HEADER+28;
  binmsg[id++]=TiltXpointer[0];
  binmsg[id++]=TiltXpointer[1];
  binmsg[id++]=TiltXpointer[2];
  binmsg[id++]=TiltXpointer[3];
  binmsg[id++]=TiltXpointer[4];
  binmsg[id++]=TiltXpointer[5];
  binmsg[id++]=TiltXpointer[6];
  binmsg[id++]=TiltXpointer[7];
  //Tilty
  binmsg[id++]=TiltYpointer[0];
  binmsg[id++]=TiltYpointer[1];
  binmsg[id++]=TiltYpointer[2];
  binmsg[id++]=TiltYpointer[3];
  binmsg[id++]=TiltYpointer[4];
  binmsg[id++]=TiltYpointer[5];
  binmsg[id++]=TiltYpointer[6];
  binmsg[id++]=TiltYpointer[7];
#endif

#ifdef DEBUG
#endif


  //and recalc the CRC32 
  //uint32_t const crc32_res = crc32.calc((uint8_t const *)binmsg, sizeof(binmsg)-4);

  uint32_t const crc32_res=CalculateBlockCRC32(sizeof(binmsg)-4, binmsg );

  binmsg[84]=crc32_res      & 0x000000FF;
  binmsg[85]=crc32_res>>8   & 0x000000FF;
  binmsg[86]=crc32_res>>16  & 0x000000FF;
  binmsg[87]=crc32_res>>24  & 0x000000FF;
  #ifdef DEBUG
  Serial.print("crc32 binary:");
  Serial.print(binmsg[84],HEX);
  Serial.print(binmsg[85],HEX);
  Serial.print(binmsg[86],HEX);
  Serial.print(binmsg[87],HEX);
  Serial.println();
  #endif 
  Serial1.write(binmsg,sizeof(binmsg)); 
}
void updatetime(){
  //fill time from a local real time clock running on Arduino
  //in real operation we want the clock to be given by the gps and NOT by the inacurate rtc
  //use only for debugging purposes
  sprintf(Time,"%2d%2d%2d.00",rtc.getHours(),rtc.getMinutes(),rtc.getSeconds()); //no millisecond in rtc
  Serial.print("Time=");


  Day=rtc.getDay();
  Month=rtc.getMonth();
  Year=rtc.getYear();
}

void sendOK(){
  Serial1.println(); //\r\n
  Serial1.println("<OK"); //end with \r\n
  Serial1.print("[COM1]"); //no ending with \r\n
}
void parseARAGcommands(const char* msg){
  //react to ARAG commands
  
  if (strcmp(msg,"log versiona once")==0){
    sendOK();
    Serial1.println("#VERSIONA,COM1,0,56.0,FINESTEERING,2248,51929.543,00000000,3681,9603;3,GPSCARD,\"N1GA\",\"DEL13280066\",\"MCAGTP-3.01-22B\",\"3.906\",\"3.002\",\"2013/Mar/14\",\"14:22:01\",DB_USERAPPAUTO,\"SmartAg\",\"0\",\"\",\"1.101\",\"\",\"2011/Sep/29\",\"17:13:55\",USERINFO,\"No BT\",\"\",\"\",\"\",\"\",\"\",\"\"*0cd69629");
    Serial.println("Received version request from Arag");
    // statements
  }
  else if (strcmp(msg,"unlogall com1 true")==0 or strcmp(msg,"unlogall com2 true")==0){
    sendOK();
  }
  else if (strcmp(msg,"nmeatalker auto")==0){
    sendOK();
    Serial.println("Receive nmeatalker auto from Arag");
  }
  else if (strcmp(msg,"log com1 gpggalong ontime 0.1")==0){
    sendOK();
    Serial.println("Receive log gpggalong from Arag");
  }
  else if (strcmp(msg,"log com1 gpvtg ontime 0.1")==0){
    sendOK();
    Serial.println("Receive log gpvtg from Arag");
  }
  else if (strcmp(msg,"log com1 gpzda ontime 1")==0){
    sendOK();
    Serial.println("Receive log gpzda from Arag");
  }
  else if (strcmp(msg,"log com1 tiltdatab ontime 1")==0){
    sendOK();
    Serial.println("Receive log tiltdatab from Arag");
  }
  else if (strcmp(msg,"$PMDT,u,,,,0.0*7A")==0){
    sendOK();
    Serial1.println();
    Serial1.println("$PMDT,<,Tilt sensor not installed");
    Serial.println("Receive log PMDT from Arag");
  }
  else if (strcmp(msg,"pdpfilter enable")==0){
    sendOK();
    Serial.println("Receive pdpfilter enable from Arag");
  }
  else if (strcmp(msg,"pdpmode relative auto")==0){
    sendOK();
    Serial.println("Receive pdpmode relative from Arag");
  }
  else if (strcmp(msg,"sbascontrol disable")==0){
    sendOK();
    Serial.println("Receive sbascontrol disable from Arag");
  }
  else{
    Serial.println("Received unknown message from Arag:");
     Serial.println(msg);
  }
}
int parsePUBX(const char * m)
{
  //example
  //$PUBX,04,073731.00,091202,113851.00,1196,113851.00,1930035,-2660.664,43*3C<CR><+LF>
  //$PUBX,04,hhmmss.ss,ddmmyy,UTC_TOW,week,reserved,Clk_B,Clk_D,PG*hh<CR><LF>
  int chk=0;
  int n=sscanf(m,"$PUBX,04,%20[^,],%20[^,],%20[^,],%d,%20[^,],%20[^,],%20[^,],%d*%x",
        Hms,
        Dmy,
        UTC_TOW,
        &UTC_Week,
        Reserved,
        Clk_B,
        Clk_D,
        &PG,
        &chk
        );
    if (n!=9 ) {
      Serial.print("parsing failed to retrieve all 9 variables, only received: ");
      Serial.println(n);     
    }
  //convert UTC_TOW to integer ms since ref week
  int sec,ms;
  int m2=sscanf(UTC_TOW,"%d.%d",&sec,&ms);
  if (m2==2){
    UTC_ms=sec*1000+ms*10; //assuming 2 digit
  }
  
  #ifdef DEBUG
    Serial.println("result pubx04 parsing :" );
    Serial.print("Time : ");
    Serial.println(Hms);
    Serial.print("Dmy : ");
    Serial.println(Dmy);
    Serial.print("UTC_TOW : ");
    Serial.println(UTC_TOW);
    Serial.print("UTC_ms converted from UTC_TOW : ");
    Serial.println(UTC_ms);
    Serial.print("UTC_Week : ");
    Serial.println(UTC_Week);
    Serial.print("Reserved : ");
    Serial.println(Reserved);
    Serial.print("Clk_B : ");
    Serial.println(Clk_B);
    Serial.print("Clk_D : ");
    Serial.println(Clk_D);
    Serial.print("PG : ");
    Serial.println(PG);
    Serial.print("chk : ");
    Serial.println(chk);
    #endif

    //check if all could be read
    //check if checksum was correct?
  return n;
}


int parseGPGGA(const char * m)
{
  int chk=0;
  int n=sscanf(m,"$GPGGA,%20[^,],%20[^,],%c,%20[^,],%c,%d,%d,%20[^,],%20[^,],%c,%20[^,],%c,%20[^,],%20[^*]*%x",
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
    if (n!=15 ) {
      Serial.print("parsing failed to retrieve all 15 variables, only received: ");
      Serial.println(n);     
      //12 is also good if DGPS is not on
    }
  #ifdef DEBUG
    Serial.println("result gpggga parsing :" );
    Serial.print("Time : ");
    Serial.println(Time);
    Serial.print("Latitude : ");
    Serial.println(Latitude);
    Serial.print("&NSchar : ");
    Serial.println(&NSchar);
    Serial.print("Longitude : ");
    Serial.println(Longitude);
    Serial.print("EWchar : ");
    Serial.println(&EWchar);
    Serial.print("Quality : ");
    Serial.println(Quality);
    Serial.print("NSat : ");
    Serial.println(NSat);
    Serial.print("HDOP : ");
    Serial.println(HDOP);
    Serial.print("Altitude : ");
    Serial.println(Altitude);
    Serial.print("unitchar : ");
    Serial.println(&unitchar);
    Serial.print("HWGS84 : ");
    Serial.println(HWGS84);
    Serial.print("unitchar : ");
    Serial.println(&unitchar);
    Serial.print("DGPSupdate : ");
    Serial.println(DGPSupdate);
    Serial.print("DGPSid : ");
    Serial.println(DGPSid);
    Serial.print("chk : ");
    Serial.println(chk);
    #endif

    //check if all could be read
    //check if checksum was correct?
  return n;
}
void GPGGA(char * m)
{
  //prepare GPGGA msg depending on global position variables 
  
  //make it look more like novatel by supressing rtk specific parts
  #ifdef NORTK
  //Quality=1; //in case ARAG doesnt like 4
  strcpy(DGPSupdate,""); //novatel has these empty as there is not differential GPS
  strcpy(DGPSid,"");
  #endif

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
  int n=sscanf(m,"$G%cVTG,%20[^,],T,,M,%20[^,],N,%20[^,],%c,%c*%x",
        &xchar,
        Tracktrue,
        //Trackmag,
        Knots,
        Speed,
        &Speedunits,
        &Modechar,
        &chk
        );
    if (n!=7) {
      Serial.print("GNVTG parsing failed to retrieve all 7 variables, only got:");
      Serial.println(n); 
    }
    strcpy(Trackmag,Tracktrue);

    #ifdef DEBUG
    Serial.println("result GNVTG parsing :" );
    Serial.print("xchar : ");
    Serial.println(xchar);
    Serial.print("Tracktrue : ");
    Serial.println(Tracktrue);
    Serial.print("Trackmag : ");
    Serial.println(Trackmag);
    Serial.print("Knots : ");
    Serial.println(Knots);
    Serial.print("Speed : ");
    Serial.println(Speed);
    Serial.print("Speedunits : ");
    Serial.println(Speedunits);
    Serial.print(" Modechar : ");
    Serial.println( Modechar);
    Serial.print("chk : ");
    Serial.println(chk);
    #endif
    //check if all could be read
    //check if checksum was correct?
    //Serial.print(n);
  return n;
}

void GNVTG(char * m){
  //prepare GNVTG string
  //multi constellation
  //example $GNVTG,139.969,T,139.969,M,0.007,N,0.013,K,D*3D

  //add 3rd digit
  //Tracktrue[6]='0';
  //Tracktrue[7]='\0';
  //Trackmag[6]='0';
  //Trackmag[7]='\0';

  
  sprintf(m, "GNVTG,%s,T,%s,M,%s,N,%s,%c,%c",Tracktrue,Trackmag,Knots,Speed,Speedunits,Modechar);
}

int parseGPZDA(const char * m)
{

  int chk=0;
  #ifdef DEBUG
  Serial.println("ZDA Input string");
  Serial.println(m);
  #endif
  int n=sscanf(m,"$GPZDA,%20[^,],%2d,%2d,%4d,00,00*%x",
        Time,
        &Day,
        &Month,
        &Year,
        &chk
        );
    if (n!=5) {
      Serial.print("GPZDA parsing failed to retrieve all 5 variables, only got:");
      Serial.println(n);
    }
    #ifdef DEBUG
    Serial.println("result ZDA parsing :" );
    Serial.print("Time : ");
    Serial.println(Time);
    Serial.print("Day : ");
    Serial.println(Day);
    Serial.print("Month : ");
    Serial.println(Month);
    Serial.print("Year : ");
    Serial.println(Year);
    Serial.print("chk : ");
    Serial.println(chk);
    #endif
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
  #ifdef NMEAUSB
  Serial.print('$'); //prequel
  Serial.print(m); //msg
  Serial.print('*'); //sequal
  Serial.println(checksum(m), HEX); //convert checksum byte to hex
  #endif


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

void debugtest()
{
  Serial.println("Testing functionality"); 
  int n=0;
  

  Serial.println("Testing functionality with ardusimple messages"); 
  //now with ardusimple messages
  const char* atestGPGGA= "$GPGGA,152027.40,5056.7186002,N,00446.6230636,E,4,12,0.55,18.106,M,46.220,M,0.4,3174*44";
  n=parseGPGGA(atestGPGGA);
  Serial.println(n);
  
  const char* atestGNVTG= "$GPVTG,335.788,T,,M,0.019,N,0.036,K,D*2B";
  n=parseGNVTG(atestGNVTG);
  Serial.println(n);

  const char* atestGPZDA= "$GPZDA,152027.40,05,02,2023,00,00*65";
  n=parseGPZDA(atestGPZDA);
  Serial.println(n);

  //test parsing and fill global coordinates, these are from Novatel valid messages
  const char* testGPGGA= "$GPGGA,142518.90,5056.7191279,N,00446.6237381,E,1,14,0.8,25.593,M,45.50,M,,*67";
  n=parseGPGGA(testGPGGA);
  Serial.println(n);

  //const char* testGNVTG= "$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E";
  const char* testGNVTG= "$GNVTG,335.788,T,,M,0.001,N,0.002,K,A*3E";//without magnetic heading to not upset the parser
  
  n=parseGNVTG(testGNVTG);
  Serial.println(n);

  const char* testGPZDA= "$GPZDA,142436.00,05,02,2023,,*64";
  n=parseGPZDA(testGPZDA);
  Serial.println(n);

  const char* testPUBX= "$PUBX,04,073731.00,091202,113851.00,1196,113851.00,1930035,-2660.664,43*3C";
  n=parsePUBX(testPUBX);
  Serial.println(n);


  Serial.println("Testing arag parsing functionality");
  const char* testARAG= "log versiona once";
  parseARAGcommands(testARAG);

  
  const char* bintestARAG= "\xAAD\x12\x1C\x04\0\0\xC0 \0\0\0\x90\xE4\xB7A\x98#S\00\x12\x12\0(       \x12\0\x01\0\0\0\0\xC2\x01\0\x01\0\0\0\x08\0\0\0\x01\0\0\0\0\0\0\0\0\0\0\0\x01\0\0\0j\x82Dx";
  Serial.println("binary message");  
  Serial.write(bintestARAG,64);
  Serial.println();

  Serial.println("end of test code");

  
}

/* --------------------------------------------------------------------------

Calculate a CRC value to be used by CRC calculation functions.

-------------------------------------------------------------------------- */

unsigned long CRC32Value(int i) {
     int j;
     unsigned long ulCRC;
     ulCRC = i;
     for ( j = 8 ; j > 0; j-- ) {
          if ( ulCRC & 1 )
               ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
          else
               ulCRC >>= 1;
     }
     return ulCRC;
}

 

/* --------------------------------------------------------------------------

Calculates the CRC-32 of a block of data all at once

ulCount - Number of bytes in the data block

ucBuffer - Data block

-------------------------------------------------------------------------- */

unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer ) {
     unsigned long ulTemp1;
     unsigned long ulTemp2;
     unsigned long ulCRC = 0;
     while ( ulCount-- != 0 ) {
          ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
          ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xFF );
          ulCRC = ulTemp1 ^ ulTemp2;
     }
     return( ulCRC );
} 

void readgyro(){
  

  //read Euler tilt angles
  
  //read BNO055 gyroscope
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */
  #ifdef DEBUG
  Serial.print("tiltX: ");
  Serial.print(euler.x());
  Serial.print(" tiltY: ");
  Serial.print(euler.y());
  Serial.print(" tiltZ: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  #endif

  //read acceleration vector
  
  #ifdef DEBUG
  Serial.print("aX: ");
  Serial.print(accel.x());
  Serial.print(" aY: ");
  Serial.print(accel.y());
  Serial.print(" aZ: ");
  Serial.print(accel.z());
  Serial.print("\t\t");
  #endif

  //fill in global coordinates
  AccX=accel.x();
  AccY=accel.y();
  TiltX=euler.x();
  TiltY=euler.y();
}