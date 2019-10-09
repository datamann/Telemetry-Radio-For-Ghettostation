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

// Only activate one
#define UBX_Ghettostation               // For Ghettostation - Active messages : NAV-POSLLH Geodetic Position Solution, NAV-VELNED Velocity Solution in NED, NAV-STATUS Receiver Navigation Status or NAV-SOL Navigation Solution Information
//#define turn_On_All_UBX_Packages      // Turns on all UBX packets.

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
SoftwareSerial gps(RXPin, TXPin);

boolean BAUDRATE_OK;

#include "gps.h"
#include "ublox.h"
#include "gpsStartup.h"
#include "code.h" // Temp code space

void setup() {
 
  // Start up serial ports
  gps.begin(GPSBaud);
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
        Serial.print(F("Trying to configure new baud setting..."));
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
          Serial.print(F("GPS is now configured to use baud setting: "));
          Serial.print(BAUDRATE);
        #endif        
      }
    }

    #ifdef DEBUG
      Serial.print(txtToDisplay);
    #endif    
    sendUBX(activateSats, sizeof(activateSats)/sizeof(uint8_t));
    getUBX_ACK(activateSats);

    #ifdef UBX_Ghettostation
      turnOffUBX();               // Turns off all UBX packages.
      turnOnUBXGhettostation();   // Turns on only UBX packages that is needed for Ghettostation.
    #endif
    #ifdef turn_On_All_UBX_Packages
      turnOffUBX();               // Turns off all UBX packages.
      turnOnUBX();                // Turns on all UBX packages.
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
      Serial.println(F("Init sender failed"));
    #endif
  }
  else
  {
    #ifdef DEBUG
      Serial.println(F("Init sender succeeded"));
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

void loop() {
  ubx();
}

#define MAX_UBLOX_PAYLOAD_SIZE 256
#define UBLOX_BUFFER_SIZE MAX_UBLOX_PAYLOAD_SIZE

void ubx(){
  byte data;
  int numc;
  bool _skip_packet;
  uint8_t _class;
  uint8_t _ck_a;
  uint8_t _ck_b;
  uint8_t _msg_id;
  uint16_t _payload_length;
  uint16_t _payload_counter;
  uint8_t bytes[UBLOX_BUFFER_SIZE];
  uint8_t bytesToSend[UBLOX_BUFFER_SIZE];
  uint8_t PREAMBLE1 = 0xB5;
  uint8_t PREAMBLE2 = 0x62;
  int bufferidx = 0;
  int _step = 0;
  
  numc = gps.available();
  if (numc > 0)
  {
    for (int i=0;i<numc;i++)  // Process bytes received
    {
      // Page 96:
      // file:///C:/Users/sbsiv/Desktop/GPS/u-blox6_ReceiverDescrProtSpec_(GPS.G6-SW-10018)_Public.pdf
      // UBX_class = 0x01
      // Check INAV: gps_ublox.c line: 605 gpsNewFrameUBLOX()

      // Check: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/blob/092d489ca7bb8590a76be8e93d5511314b20487d/src/SparkFun_Ublox_Arduino_Library.h
      // Check: https://github.com/loginov-rocks/UbxGps/blob/master/src/UbxGps.h
    
      data = gps.read();
      switch (_step) {
        case 0: // Sync char 1
          
          Serial.print(F("Step: "));
          Serial.println(_step);
          
          if(PREAMBLE1 != data){
            _step = 0;
            bufferidx = 0;
            break;
          }

          Serial.print(F("Sync byte 1: "));
          Serial.println(data);
          
          _skip_packet = false;
          bytesToSend[bufferidx++] = data;
          _step++;

          Serial.print(F("Step: "));
          Serial.println(_step);
          
          break;
        case 1: // Sync char 2
          if(PREAMBLE2 != data){
            _step = 0;
            bufferidx = 0;
            break;
          }

          Serial.print(F("Sync byte 2: "));
          Serial.println(data);
          
          bytesToSend[bufferidx++] = data;
          _step++;
          break;
        case 2: // Class
          _class = data;
          bytesToSend[bufferidx++] = _class;
          _step++;
          _ck_b = _ck_a = data;   // reset the checksum accumulators

          Serial.print(F("Class: "));
          Serial.println(_class);
          
          break;
        case 3: // Id
          _msg_id = data;
          bytesToSend[bufferidx++] = _msg_id;
          _step++;
          _ck_b += (_ck_a += data);       // checksum byte

          Serial.print(F("ID: "));
          Serial.println(_msg_id);
          
          break;
        case 4: // Lenght byte 1
          _payload_length = data;
          bytesToSend[bufferidx++] = _payload_length;
          _step++;
          _ck_b += (_ck_a += data);       // checksum byte

          Serial.print(F("Payload length byte 1: "));
          Serial.println(_payload_length);

          break;
        case 5: // Lenght byte 2
           _payload_length |= (uint16_t)(data << 8);
           bytesToSend[bufferidx++] = _payload_length;
           _step++;
           _ck_b += (_ck_a += data);       // checksum byte

           Serial.print(F("Payload length byte 2: "));
           Serial.println(_payload_length);

           if (_payload_length > MAX_UBLOX_PAYLOAD_SIZE) {
                // we can't receive the whole packet, just log the error and start searching for the next packet.
                _step = 0;
                bufferidx = 0;

                Serial.print(F("Case 5 - To long payload length!!!: "));
                Serial.println(_payload_length);

                break;
            }
            
            // prepare to receive payload
            _payload_counter = 0;
            if (_payload_length == 0) {
                _step = 7;
                Serial.print(F("Case 5 - Payload length is 0 !!!: "));
                Serial.println(_payload_length);
            }
          break;
        case 6: // Payload
          _ck_b += (_ck_a += data);       // checksum byte
          
          Serial.print(F("Case 6: Payload: "));
          Serial.println(_payload_counter);
          
          if (_payload_counter < MAX_UBLOX_PAYLOAD_SIZE) {
            
              Serial.print(F("Case 6, Adding payload to buffer "));
              Serial.println(_payload_counter);
              
              bytes[_payload_counter] = data;
              bytesToSend[bufferidx++] = data;
          }else{
            Serial.print(F("Case 6, Max payload size reached!!!"));
            Serial.println(_payload_counter);
          }
          // NOTE: check counter BEFORE increasing so that a payload_size of 65535 is correctly handled.  This can happen if garbage data is received.
          if (_payload_counter == _payload_length - 1) {
              _step++;
              Serial.print(F("Case 6, Payload counter is equal to payload length, let's move on... "));
              Serial.println(_payload_counter);              
          }else{
            _payload_counter++;
          }
          Serial.print(F("Case 6 - Step: "));
          Serial.println(_step);
          break;
        case 7: // Checksum byte 1
          Serial.print(F("Case 7 - Step: "));
          Serial.println(_step);
          
          if (_ck_a != data) {
                _skip_packet = true;          // bad checksum
                _step = 0;
                bufferidx = 0;
                Serial.println(F("Case 7 - Bad checksum, skipping packet!!!: "));
                break;
          }
          Serial.println(F("Case 7 - Checksum OK!, Moving ON..."));
          
          bytesToSend[bufferidx++] = data;
          _step++;
          break;
        case 8: // Checksum byte 2
          _step = 0;
          bytesToSend[bufferidx++] = data;
          bufferidx = 0;
          
          if (_ck_b != data) {
                Serial.println(F("Case 8 - Checksum byte 2 error, cancelling!!!: "));
                break;              // bad checksum
          }
          if (_skip_packet) {
              Serial.println(F("Case 8 - Checksum byte 1 error, skipping packet, cancelling!!!: "));
              break;
          }

          /*for ( int i=0; i<sizeof(bytesToSend); i++ )
          {
            Serial.println(bytesToSend[i]);
          }*/

          Serial.println(F("Case 8 - Checksum OK!, Packet complete!"));
          //break;
      } // End Switch... 
    } // End for...
  } // End if numc
} // End function
