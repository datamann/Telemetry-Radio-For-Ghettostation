/*
 * Written by Stig B. Sivertsen
 * sbsivertsen@gmail.com
 * https://github.com/datamann/GPSConfigurator
 * 06.09.2019
*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "LoRa.h"
RH_RF95 rf95;

typedef struct baudRateHelper {
   String txt;
   uint8_t* arr;
};

#define SET_FACTORY_DEFAULT         // Activating factory default GPS configuration

// Only activate one satellite system!
#define USE_DEFAULT     // GPS,Galileo,QZSS,Glonass
//#define USE_GPS_QZSS  // Only GPS & QZSS. By UBLOX NMEA documentation, It's recomended to both.
//#define USE_GALILEO   // Only Galileo
//#define USE_GLONASS   // Only Glonass
//#define USE_BEIDOU    // Only BeiDou
//#define USE_ALL       // GPS,SBAS,Galileo,IMES,QZSS,Glonass

#define DATARATE_1_HZ
//#define DATARATE_5_HZ
//#define DATARATE_10_HZ

boolean USE_DEFAULT_BAUDRATE = false;  // Configure GPS to higher baudrate.
boolean BAUDRATE_OK;

//static const uint32_t BAUDRATE = 4800;
static const uint32_t BAUDRATE = 9600;
//static const uint32_t BAUDRATE = 19200;
//static const uint32_t BAUDRATE = 38400;
//static const uint32_t BAUDRATE = 57600;
//static const uint32_t BAUDRATE = 115200;

static const int RXPin = 3, TXPin = 4;      // Used when RX/TX are not used.
static const uint32_t GPSBaud = BAUDRATE;
static const uint32_t SerialBaud = 19200;
SoftwareSerial nss(RXPin, TXPin);

#include "gps.h"
#include "nmea.h"
#include "ublox.h"
#include "gpsStartup.h"
#include "code.h"
 
void setup() {
 
  // Start up serial ports
  nss.begin(GPSBaud);
  Serial.begin(SerialBaud); // used for debug ouput
  delay(2000); // Give the GPS time to come boot

  checkConnectivity();

  if ( BAUDRATE_OK )
  {
    #ifdef SET_FACTORY_DEFAULT
      Serial.print(txtToDisplayFD);
      sendUBX(revertDefault, sizeof(revertDefault)/sizeof(uint8_t));
      getUBX_ACK(revertDefault);
      USE_DEFAULT_BAUDRATE = false;
    #endif

    if ( USE_DEFAULT_BAUDRATE ) //&& BAUDRATE != 9600
    {
      Serial.print("Trying to configure new baud setting...");
      uint8_t baudRate = setBaudRate(BAUDRATE);
      sendUBX(baudRate, sizeof(baudRate)/sizeof(uint8_t));
      getUBX_ACK(baudRate);

      if ( !BAUDRATE_OK )
      {
        checkConnectivity(); 
      }
      else
      {
        Serial.print(String("GPS is now configured to use baud setting: ") + BAUDRATE);
      }
    }

    Serial.print(txtToDisplay);
    sendUBX(activateSats, sizeof(activateSats)/sizeof(uint8_t));
    getUBX_ACK(activateSats);
  }    
}

void loop() {

  static unsigned long lastSendTime = 0;
  unsigned long now = millis();
  
  while ( nss.available() > 0 ) // && now - lastSendTime > 500)
  {
    char c = nss.read();
    Serial.print(c);

    //lastSendTime = now; 
  }
}

void checkConnectivity()
{
  Serial.println();
  Serial.print("Checking connectivity...");
  Serial.println();
  uint8_t test[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x01,0xFB,0x10};
  sendUBX(test, sizeof(test)/sizeof(uint8_t));
  getUBX_ACK(test);
  
  while ( !BAUDRATE_OK )
  {
    autoBaud();
    
    if ( BAUDRATE_OK ){
      break;
    }
  }
}

void autoBaud()
{
  long int baudRate[6] = { 4800,9600,19200,38400,57600,115200 };
  while ( !BAUDRATE_OK )
  {
    for ( int i=0; i<6; i++ )
    {
      // Switch baud rates on the software serial
      Serial.println(String("Switching to ") + baudRate[i] + String(" for GPS port."));
      nss.begin(baudRate[i]);
      delay(1000);

      // Used for testing GPS configuration changes. If successfull we have the correct baud settings.
      uint8_t test[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x01,0xFB,0x10};
      sendUBX(test, sizeof(test)/sizeof(uint8_t));
      getUBX_ACK(test);
      
      if ( BAUDRATE_OK ){
        break;
      }
    }
  }
}
 
// Send a byte array of UBX protocol to the GPS
void sendUBX ( uint8_t *MSG, uint8_t len )
{
  for ( int i=0; i<len; i++ )
  {
    nss.write(MSG[i]);
    //Serial.print(MSG[i], HEX);
  }
  Serial.println();
}
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK ( uint8_t *MSG )
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for ( uint8_t i=2; i<8; i++ )
  {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while ( 1 )
  { 
    // Test for success
    if ( ackByteID > 9 )
    {
        // All packets in order!
        Serial.println(" (SUCCESS!)");
        BAUDRATE_OK = true;
        return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if ( millis() - startTime > 3000 )
    { 
      Serial.println(" (FAILED!)");
      BAUDRATE_OK = false;
      return false;
    }
 
    // Make sure data is available to read
    if ( nss.available() )
    {
      b = nss.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if ( b == ackPacket[ackByteID] )
      { 
        ackByteID++;
        //Serial.print(b, HEX);
      }
      else
      {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}

// Send a byte array of NMEA protocol to the GPS
void sendNMEA ( char s[] )
{
  //char s[]= { "$PAMTC,EN,HDG,1,10*HH\r\n" } ;
  
  int cpos = 1 ;
  uint8_t cs = 0 ;
  
  while ( true )
  {
      cs = cs^s[cpos];
      cpos++;
      if ( s[cpos] == '*' )
      {
           char css[3];
           sprintf ( css,"%02X", cs);
           s[cpos+1] = css[0];
           s[cpos+2] = css[1];
           break;
      }
  }
    Serial.println( nss.print(s) );
    
    while ( nss.available() )
    {
      char c = nss.read();
      Serial.print(c);
    }
    Serial.println();
}
