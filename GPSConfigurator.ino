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

// Activating factory default GPS configuration
//#define SET_FACTORY_DEFAULT

// Only activate one!
#define USE_DEFAULT     // GPS,Galileo,QZSS,Glonass
//#define USE_GPS_QZSS  // Only GPS & QZSS. By UBLOX NMEA documentation, It's recomended to both.
//#define USE_GALILEO   // Only Galileo
//#define USE_GLONASS   // Only Glonass
//#define USE_BEIDOU    // Only BeiDou
//#define USE_ALL       // GPS,SBAS,Galileo,IMES,QZSS,Glonass
#include "gps.h"
#include "nmea.h"
#include "ublox.h"
#include "gpsStartup.h"
#include "code.h"

RH_RF95 rf95;
static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;
SoftwareSerial nss(RXPin, TXPin);
byte navmode = 99;
 
void setup() {
 
  // Start up serial ports
  nss.begin(GPSBaud);
  Serial.begin(19200); // used for debug ouput
  delay(2000); // Give the GPS time to come boot

  // Switch baud rates on the software serial
  /*Serial.println("Switching to 9600b GPS serial");
  nss.begin(9600);
  delay(1000);*/  

  Serial.println(HEADER1 + "-" + HEADER2);

  #ifdef SET_FACTORY_DEFAULT
     Serial.print(txtToDisplayFD);
    sendUBX(revertDefault, sizeof(revertDefault)/sizeof(uint8_t));
    getUBX_ACK(revertDefault);  
  #endif 

  //Turn on satellites
  Serial.print(txtToDisplay);
  sendUBX(activateSats, sizeof(activateSats)/sizeof(uint8_t));
  getUBX_ACK(activateSats);
}

void loop() {

  static unsigned long lastSendTime = 0;
  unsigned long now = millis();
  
  while(nss.available() > 0) // && now - lastSendTime > 500)
  {
    char c = nss.read();
    Serial.print(c);
    //lastSendTime = now; 
  }
}
 
// Send a byte array of UBX protocol to the GPS
void sendUBX ( uint8_t *MSG, uint8_t len )
{
  for ( int i=0; i<len; i++ )
  {
    nss.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  Serial.println();
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
        return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if ( millis() - startTime > 3000 )
    { 
      Serial.println(" (FAILED!)");
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
        Serial.print(b, HEX);
      }
      else
      {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}
