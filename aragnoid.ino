#include <Arduino.h>
#include "wiring_private.h"

#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>

/* Create an rtc object */
RTCZero rtc;

//create a serial 2 on pin 1 and 0
Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

//global coordinates
//store long and lat as 2 char strings, one for units and another for decimals, we will not do calculations
//with these numbers and we need to preserve a number of digits greater than Arduino double (=float) can store
//this also solves the fact that Arduino has difficulty in dealing with floats in printf and scanf
char Time_units[20];
char Time_dec[20];
char Latitude_units[20];
char Latitude_dec[20]; //Latitude decimals
char NSchar;
char Longitude_units[20];
char Longitude_dec[20]; //Longitude decimals
char EWchar;
uint Quality,NSat;
char HDOP_unit[20];
char HDOP_dec[20];
char Altitude_units[20];
char Altitude_dec[20];
char unitchar;
char HWGS84_units[20];
char HWGS84_dec[20];
int DGPSupdate;
char DGPSid[20];
//VTG
char Tracktrue_units[20];
char Tracktrue_dec[20];
char Trackmag_units[20];
char Trackmag_dec[20];
char Knots_units[20];
char Knots_dec[20];
char Speed_units[20];
char Speed_dec[20];
char Modechar='D';
//ZDA
int Day=0;
int Month=0;
int Year=0;



int dt1=5;
int dt=80;
int dt3=60;
int dt4=17;
int dt5=8;
bool toggle=true;
char msg[128]; //a buffer that will get the msg contents



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //usb uart
  rtc.begin(); // initialize RTC
  Serial1.begin(115200,SERIAL_8E1); //RX and TX on pin 13/14 , this is the connection to ARAG

  mySerial.begin(115200); //RX and TX on PIN 1/0, this will become the connection to RTKsimple
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  rtc.setHours(12);
  rtc.setMinutes(0);
  rtc.setSeconds(0);
  
  //test parsing and fill global coordinates
  char* testmsg;
  testGPGGA= "$GPGGA,142435.90,5056.7191170,N,00446.6237262,E,1,14,0.8,25.571,M,45.50,M,,*63";
  parseGPGGA(testGPGGA);

  testGNVTG= "$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E";
  parseGPGGA(testGNVTG);

  testGPZDA= "$GPZDA,142436.00,05,02,2023,,*64";
  parseGPGGA(testGPZDA);

}

void loop() {
    // put your main code here, to run repeatedly:
    Serial.println("Hello, hello again..."); 
    //Serial1.println("Hello serial1..."); 
    Serial1.println("$GPGGA,142513.00,5056.7191290,N,00446.6237365,E,1,14,0.8,25.600,M,45.50,M,,*61");
    delay(dt1);
    Serial1.println("$GNVTG,235.738,T,235.738,M,0.000,N,0.000,K,A*3D");
    delay(dt);
    Serial1.println("$GPGGA,142435.20,5056.7191154,N,00446.6237248,E,1,14,0.8,25.578,M,45.50,M,,*6F");
    delay(dt1);
    Serial1.println("$GNVTG,238.603,T,238.603,M,0.001,N,0.003,K,A*3F");
    delay(dt);
    Serial1.println("$GPGGA,142435.30,5056.7191167,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*6F");
    delay(dt1);
    Serial1.println("$GNVTG,14.744,T,14.744,M,0.006,N,0.012,K,A*38");
    delay(dt);
    Serial1.println("$GPGGA,142435.40,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*67");
    delay(dt1);
    Serial1.println("$GNVTG,14.745,T,14.745,M,0.000,N,0.000,K,A*3D");
    delay(dt);
    Serial1.println("$GPGGA,142435.50,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*66");
    delay(dt1);
    Serial1.println("$GNVTG,14.744,T,14.744,M,0.000,N,0.000,K,A*3D");
    delay(dt);
    Serial1.println("$GPGGA,142435.60,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*65");
    delay(dt1);
    Serial1.println("$GNVTG,149.847,T,149.847,M,0.000,N,0.000,K,A*3D");
    delay(dt);
    Serial1.println("$GPGGA,142513.60,5056.7191269,N,00446.6237367,E,1,14,0.8,25.601,M,45.50,M,,*62");
    delay(dt1);
    Serial1.println("$GNVTG,14.739,T,14.739,M,0.000,N,0.000,K,A*3D");
    delay(dt);
    Serial1.println("$GPGGA,142435.70,5056.7191168,N,00446.6237257,E,1,14,0.8,25.577,M,45.50,M,,*64");
    delay(dt1);
    Serial1.println("$GNVTG,14.739,T,14.739,M,0.000,N,0.000,K,A*3D");
    delay(dt);
    Serial1.println("$GPGGA,142435.80,5056.7191167,N,00446.6237265,E,1,14,0.8,25.576,M,45.50,M,,*64");
    delay(dt1);
    Serial1.println("$GNVTG,110.297,T,110.297,M,0.000,N,0.001,K,A*3C");
    delay(dt);
    Serial1.println("$GPGGA,142435.90,5056.7191170,N,00446.6237262,E,1,14,0.8,25.571,M,45.50,M,,*63");
    delay(dt1);
    Serial1.println("$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E");
    delay(dt4);
    Serial1.println("$GPZDA,142436.00,05,02,2023,,*64");
    byte binmsg[]= {0xAA,0x44,0x12,0x1C,0xC4,0x04,0x02,0x20,
                    0x38,0x00,0x00,0x00,0x6E,0xB4,0xC8,0x08,
                    0xF0,0xD6,0x17,0x03,0x00,0x00,0x00,0x00,
                    0xF3,0x16,0x83,0x25,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x4A,0xFA,0x2C,0x3F};
    Serial1.write(binmsg,sizeof(binmsg)); 
    delay(dt4);

    digitalWrite(LED_BUILTIN, toggle); 
    toggle=!toggle;
    
    //send out to ARAG
    sendnmea(GPGGA(msg));
    sendnmea(GNVTG(msg));
    sendnmea(GPZDA(msg));
  
    //update time
    updatetime(); //only for debug in reality we should get this from rtksimple from the atomic clocks of the GPS satelites
}

void updatetime(){
  //fill time from a local real time clock running on Arduino
  //in real operation we want the clock to be given by the gps and NOT by the inacurate rtc
  //use only for debugging purposes
  sprintf(Time_units,"%2d%2d%2d",rtc.getHours(),rtc.getMinutes(),rtc.getSeconds()); //no millisecond in rtc
  Serial.print("Time=");
  Serial.println(Time_units); 
  Day=rtc.getDay();
  Month=rtc.getMonth();
  Year=rtc.getYear();
}

void parseGPGGA(const char * msg)
{
  int chk=0;
  int n=sscanf(msg,"$GPGGA,%s.%s,%s.%s,%c,%s.%s,%c,%d,%d,%s.%s,%s.%s,%c,%s.%s,%c,%d,%4s*%d",
        Time_units,
        Time_dec, //problem, leading zeros matter and an integer doesnt care for them...how to solve this?
        Latitude_units,
        Latitude_dec,
        &NSchar,
        Longitude_units,
        Longitude_dec,
        &EWchar,
        &Quality,
        &NSat,
        HDOP_unit,
        HDOP_dec,
        Altitude_units,
        Altitude_dec,
        &unitchar,
        HWGS84_units,
        HWGS84_dec,
        &unitchar,
        &DGPSupdate,
        DGPSid,
        &chk
        );
    if (n!=21) {
      Serial.println("parsing failed to retrieve all 21 variables");
    }
    //check if all could be read
    //check if checksum was correct?
    Serial.print(n);
}

void parseGNVTG(const char * msg)
{
  int chk=0;
  int n=sscanf(msg,"$GNVTG,%s.%s,T,%s.%s,M,%s.%s,N,%s.%s,%c*%d",
        Tracktrue_units,
        Tracktrue_dec,
        Trackmag_units,
        Trackmag_dec,
        Knots_units,
        Knots_dec,
        Speed_units,
        Speed_dec,
        &Modechar,
        &chk
        );
    if (n!=9) {
      Serial.println("GNVTG parsing failed to retrieve all 9 variables");
    }
    //check if all could be read
    //check if checksum was correct?
    Serial.print(n);
}

const char * GNVTG(char * msg){
  //prepare GNVTG string
  //multi constellation
  //example $GNVTG,139.969,T,139.969,M,0.007,N,0.013,K,D*3D
  char tracktruestring[20];
  getstring(tracktruestring,Tracktrue_units,Tracktrue_dec,3);
  char trackmagstring[20];
  getstring(trackmagstring,Trackmag_units,Trackmag_dec,3);
  char knotsstring[20];
  getstring(knotsstring,Knots_units,Knots_dec,3);
  char speedstring[20];
  getstring(speedstring,Speed_units,Speed_dec,3);
  
  sprintf(msg, "GNVTG,%s,T,%s,M,%s,N,%s,K%c", tracktruestring,trackmagstring,knotsstring,speedstring,Modechar);
  return msg;
}

void parseGPZDA(const char * msg)
{
  int chk=0;
  int n=sscanf(msg,"$GPZDA,%s.%s,%2d,%2d,%4d,,*%d",
        Time_units,
        Time_dec,
        &Day,
        &Month,
        &Year,
        &chk
        );
    if (n!=6) {
      Serial.println("GPZDA parsing failed to retrieve all 6 variables");
    }
    //check if all could be read
    //check if checksum was correct?
    Serial.print(n);
}


const char * GPZDA(char * msg){
  //prepare GPZDA string
  //$GPZDA,204007.00,13,05,2022,,*62
  char timestring[20];
  getstring(timestring,Time_units,Time_dec,2);

  sprintf(msg, "GPZDA,%s,%2d,%2d,%4d,,,", timestring, Day,Month,Year);
  return msg;
}

const char * GPGGA(char * msg)
{
  //prepare GPGGA msg depending on global position variables 
  char timestring[20];
  getstring(timestring,Time_units,Time_dec,2);
  char latitudestring[20];
  getstring(latitudestring,Latitude_units,Latitude_dec,4);
  char longitudestring[20];
  getstring(longitudestring,Longitude_units,Longitude_dec,4);
  char hdopstring[20];
  getstring(hdopstring,HDOP_unit,HDOP_dec,1);
  char altitudestring[20];
  getstring(altitudestring,Altitude_units,Altitude_dec,2);
  char  HWGS84string[20];
  getstring(HWGS84string,HWGS84_units,HWGS84_dec,2);
  sprintf(msg, "GPGGA,%s,%s,%c,%s,%c,%d,%d,%s,%s,%c,%s,%c,%d,%s", timestring,
                                                              latitudestring,
                                                              NSchar,
                                                              longitudestring,
                                                              EWchar,
                                                              Quality,
                                                              NSat,
                                                              hdopstring, 
                                                              altitudestring,
                                                              unitchar,
                                                              HWGS84string,
                                                              unitchar,
                                                              DGPSupdate,
                                                              DGPSid);
  
  //202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR"; //checksum should be 61
  //mySerial.println(msg);
  return msg;
}

void getstring(char * str,const char* unit,const char* dec,const int digits)
{
  int origdigit=strlen(dec); //original number of digits
  char* decdig="00000000000000";
  int maxdigit=strlen(decdig);
  strcpy(decdig, dec);
  if (origdigit>digits){
    //clip 
    decdig[origdigit]='\0'; 
  }
  else if (digits<maxdigit){
    //pad zeros, by removing \0 character and put it at higher point in the string
    decdig[origdigit]='0';
    decdig[digits]='\0';
  }

  sprintf(str,"%s.%s",unit,decdig);
}

void sendnmea(const char * msg)
{
  //temporary on Serial for debugging
  Serial.print('$'); //prequel
  Serial.print(msg); //msg
  Serial.print('*'); //sequal
  Serial.println(checksum(msg), HEX); //convert checksum byte to hex
  //showtext(msg); //and show on display
}

char checksum(const char* msg)
{
  // calc checksum by XOR all bytes, but don't do the null termination character
  byte xorTemp = 0x00;
  // process all characters in char string
  for(int i = 0; msg[i] != '\0'; i++){
    xorTemp ^= byte(msg[i]);
    }
  return xorTemp;
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler()
{
  mySerial.IrqHandler();
}