/*
 * Written by Stig B. Sivertsen
 * sbsivertsen@gmail.com
 * https://github.com/datamann/GPSConfigurator
 * 11.09.2019
 * @see The GNU Public License (GPL) Version 3
*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_RF95.h>
RH_RF95 rf95;

#define DEBUG

/*#############################################- LoRA DEFINES - READ CAREFULLY -######################################################*/
static const uint32_t FREQ = 433.00;
//static const uint32_t FREQ = 868.00;
static const uint32_t MODEMCONFIG = "RH_RF95::Bw31_25Cr48Sf512";
//static const uint32_t MODEMCONFIG = "RH_RF95::Bw125Cr48Sf4096";
static const uint32_t RFPOWER = 13;
/*###############################################################################################################################*/

/*#############################################- GPS DEFINES - READ CAREFULLY -######################################################*/
// Activating factory default GPS configuration
//#define SET_FACTORY_DEFAULT

// Only activate one satellite system!
//#define USE_DEFAULT     // GPS,Galileo,QZSS,Glonass
#define USE_GPS_QZSS  // Only GPS & QZSS. By UBLOX NMEA documentation, It's recomended to both.
//#define USE_GALILEO   // Only Galileo
//#define USE_GLONASS   // Only Glonass
//#define USE_BEIDOU    // Only BeiDou
//#define USE_ALL       // GPS,SBAS,Galileo,IMES,QZSS,Glonass

// Only activate one!
#define NMEA_Ghettostation              // For Ghettostation - $GPGGA : Global Positioning System Fix Data - $GPVTG : Ttack and Ground Speed
//#define turn_On_Default_NMEA_Packets  // Turn on default NMEA packets
//#define turn_Off_All_NMEA_Packets     // Turn off all NMEA packets - It is possible to finetune packets
//#define turn_On_All_NMEA_Packets      // Turn on all NMEA packets - It is possible to finetune packets

// Configure GPS datarate.
// Only activate one!
#define DATARATE_1_HZ
//#define DATARATE_5_HZ   // May be unstable on Beitian BN-880. If connectivity is lost, power off the GPS and wait for a few minutes.
//#define DATARATE_10_HZ  // May be unstable on Beitian BN-880. If connectivity is lost, power off the GPS and wait for a few minutes.

// Configure GPS baudrate.
boolean USE_DEFAULT_BAUDRATE = false;
/*###############################################################################################################################*/

//static const uint32_t BAUDRATE = 4800;
static const uint32_t BAUDRATE = 9600;
//static const uint32_t BAUDRATE = 19200;
//static const uint32_t BAUDRATE = 38400;
//static const uint32_t BAUDRATE = 57600;
//static const uint32_t BAUDRATE = 115200;

static const int RXPin = 3, TXPin = 4;      // Used when RX/TX are not used.
static const uint32_t GPSBaud = BAUDRATE;
static const uint32_t SerialBaud = 9600;
SoftwareSerial nss(RXPin, TXPin);

boolean BAUDRATE_OK;

#include "gps.h"
#include "nmea.h"
#include "ublox.h"
#include "gpsStartup.h"
#include "code.h"

void setup() {
 
  // Start up serial ports
  nss.begin(GPSBaud);
  Serial.begin(SerialBaud); // used for debug ouput
  delay(2000);              // Give the GPS time to come boot

  checkConnectivity();

  if ( BAUDRATE_OK )
  {
    #ifdef DATARATE_1_HZ
      #ifdef DEBUG
        Serial.print(txtToDisplayDR);
      #endif      
      sendUBX(setRate, sizeof(setRate)/sizeof(uint8_t));
      getUBX_ACK(setRate);
    #endif
    #ifdef DATARATE_5_HZ
      #ifdef DEBUG
        Serial.print(txtToDisplayDR);
      #endif
      sendUBX(setRate, sizeof(setRate)/sizeof(uint8_t));
      getUBX_ACK(setRate);
    #endif
    #ifdef DATARATE_10_HZ
      #ifdef DEBUG
        Serial.print(txtToDisplayDR);
      #endif
      sendUBX(setRate, sizeof(setRate)/sizeof(uint8_t));
      getUBX_ACK(setRate);
    #endif

    if ( USE_DEFAULT_BAUDRATE )
    {
      #ifdef DEBUG
        Serial.print("Trying to configure new baud setting...");
      #endif      
      uint8_t baudRate = setBaudRate(BAUDRATE);
      sendUBX(baudRate, sizeof(baudRate)/sizeof(uint8_t));
      getUBX_ACK(baudRate);

      if ( !BAUDRATE_OK )
      {
        checkConnectivity(); 
      }
      else
      {
        #ifdef DEBUG
          Serial.print(String("GPS is now configured to use baud setting: ") + BAUDRATE);
        #endif        
      }
    }

    #ifdef DEBUG
      Serial.print(txtToDisplay);
    #endif    
    sendUBX(activateSats, sizeof(activateSats)/sizeof(uint8_t));
    getUBX_ACK(activateSats);

    #ifdef NMEA_Ghettostation
      turnOnNMEAGhettostation();
    #endif

    #ifdef turn_On_Default_NMEA_Packets
      turnOnNMEADefaultSet();
    #endif

    #ifdef turn_Off_All_NMEA_Packets
      turnOffNMEA();
    #endif

    #ifdef turn_On_All_NMEA_Packets
      turnOnNMEA();
    #endif

    #ifdef SET_FACTORY_DEFAULT
      #ifdef DEBUG
        Serial.print(txtToDisplayFD);
      #endif      
      sendUBX(revertDefault, sizeof(revertDefault)/sizeof(uint8_t));
      getUBX_ACK(revertDefault);
      USE_DEFAULT_BAUDRATE = false;
    #endif
  }
  
  if ( !rf95.init() )
  {
    #ifdef DEBUG
      Serial.println("Init sender failed");
    #endif
  }
  else
  {
    #ifdef DEBUG
      Serial.println("Init sender succeeded");
    #endif
    rf95.setFrequency(433.00);
    //rf95.setFrequency(868.00);
    //rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);  // Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range.
    //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);   // Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range.
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);    // Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.
    //rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);      // Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range.
    rf95.setTxPower(13, false);                        // With useRFO false, valid values are from +5 to +23, 13 = default
    
    uint8_t data[] = "Sender started";
    rf95.send(data, sizeof(data));  
    rf95.waitPacketSent();
  }
}

#define GPS_BUFFERSIZE 74
boolean GPS_checksum_calc = false;
char c;
char buffer[GPS_BUFFERSIZE];
int numc;
int i;
int bufferidx;
uint8_t GPS_checksum;
boolean a = true, b = true;

void loop() {

  //while (nss.available())
    //c = nss.read();Serial.print(c);
  
  numc = nss.available();
  if (numc > 0)
  for (i=0;i<numc;i++){
    c = nss.read();
    if (c == '$'){                      // NMEA Start
      bufferidx = 0;
      buffer[bufferidx++] = c;
      GPS_checksum = 0;
      GPS_checksum_calc = true;
      continue;
    }
    if (c == '\r'){                     // NMEA End
      buffer[bufferidx++] = c;
      buffer[bufferidx++] = 0;

      if ( strncmp(buffer,"$GPGGA",6) == 0 && a)
      {
        a = false;
        b = true;
        
        //Serial.println("$GPGGA");
        #ifdef DEBUG
          String myString = String((char *)buffer);
          Serial.println(myString);
        #endif
        
        rf95.send((uint8_t *)&buffer, sizeof(buffer));
        rf95.waitPacketSent();
      }
      else if ( strncmp(buffer,"$GPVTG",6) == 0 && b)
      {
        b = false;
        a = true;
        
        //Serial.println("$GPVTG");
        #ifdef DEBUG
          String myString = String((char *)buffer);
          Serial.println(myString);
        #endif
        
        rf95.send((uint8_t *)&buffer, sizeof(buffer));
        rf95.waitPacketSent();
        delay(200);
      }
      else
      {
        Serial.println("No packets");
      }
      
      /*#ifdef DEBUG
        String myString = String((char *)buffer);
        Serial.println(myString);
      #endif*/
      
      //rf95.send((uint8_t *)&buffer, sizeof(buffer));
      //rf95.waitPacketSent();
    }
    else {
      if (bufferidx < (GPS_BUFFERSIZE-1)){
        if (c == '*')
          GPS_checksum_calc = false;    // Checksum calculation end
          buffer[bufferidx++] = c;
          if (GPS_checksum_calc)
            GPS_checksum ^= c;          // XOR 
      }
      else
      bufferidx=0;   // Buffer overflow : restart
    }
  }
} // Loop
