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
#define UBX_MAXPAYLOAD 256

uint8_t ck_a;     // Packet checksum
uint8_t ck_b;

void ubx(){
  byte data;
  int numc;
  /*bool _skip_packet;
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
  int _step = 0;*/

  uint8_t UBX_step;
  uint8_t UBX_class;
  uint8_t UBX_id;
  uint8_t UBX_payload_length_hi;
  uint8_t UBX_payload_length_lo;
  uint8_t UBX_payload_counter;
  uint8_t UBX_buffer[UBX_MAXPAYLOAD];
  uint8_t UBX_ck_a;
  uint8_t UBX_ck_b;
  uint8_t PrintErrors;
  static unsigned long GPS_timer = 0;
  
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

      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5){  // UBX sync char 1
          
          Serial.print(F("Step: "));
          Serial.println(UBX_step);

          Serial.print(F("Sync byte 1: "));
          Serial.println(data);
          
          UBX_step++;   //OH first data packet is correct, so jump to the next step
        }else{
          
          Serial.print(F("Error on sync byte 1.... exiting!!! "));
          Serial.println(data);
          
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
          break;
        }
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
        {  
          Serial.print(F("Step: "));
          Serial.println(UBX_step);

          Serial.print(F("Sync byte 2: "));
          Serial.println(data);
          
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        }else{
          
          Serial.print(F("Error on sync byte 2.... exiting!!! "));
          Serial.println(data);
          
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
          break;
        }
      case 2:
        UBX_class = data;
        ubx_checksum(UBX_class);
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);

        Serial.print(F("Class: "));
        Serial.println(UBX_class);
        
        UBX_step++;
        break;
      case 3:
        UBX_id = data;
        ubx_checksum(UBX_id);
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);

        Serial.print(F("ID: "));
        Serial.println(UBX_id);
        
        UBX_step++;
        break;
      case 4:
        UBX_payload_length_hi = data;
        ubx_checksum(UBX_payload_length_hi);
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);

        Serial.print(F("Payload length byte 1: "));
        Serial.println(UBX_payload_length_hi);
        
        UBX_step++;
        // We check if the payload lenght is valid...
        if (UBX_payload_length_hi >= UBX_MAXPAYLOAD)
        {
          //if (PrintErrors){
            Serial.println(F("ERR:GPS_BAD_PAYLOAD_LENGTH!!"));
            UBX_step=0;   //Bad data, so restart to step zero and try again.     
            ck_a=0;
            ck_b=0;
          //}
        }
        break;
      case 5:
        UBX_payload_length_lo = data;
        ubx_checksum(UBX_payload_length_lo);
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);

        Serial.print(F("Payload length byte 2: "));
        Serial.println(UBX_payload_length_lo);
        
        UBX_step++;
        UBX_payload_counter = 0;
        break;
      case 6:         // Payload data read...
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);

        //Serial.print(F("Case 6: Payload: "));
        //Serial.println(UBX_payload_counter);
          
        if (UBX_payload_counter < UBX_payload_length_hi)  // We stay in this state until we reach the payload_length
        {
          UBX_buffer[UBX_payload_counter] = data;
          
          Serial.print(F("Case 6, Adding payload to buffer "));
          Serial.println(UBX_payload_counter);
          
          ubx_checksum(data);
          UBX_payload_counter++;
          
          if (UBX_payload_counter == UBX_payload_length_hi){
            
            Serial.print(F("Case 6, Payload counter is equal to payload length, let's move on... "));
            Serial.println(UBX_payload_counter);
            
            UBX_step++;
          }
        }
        break;
      case 7:
        UBX_ck_a = data;   // First checksum byte
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);
        
        UBX_step++;
        break;
      case 8:
        
        Serial.print(F("Step: "));
        Serial.println(UBX_step);
        
        UBX_ck_b = data;   // Second checksum byte
       
        // We end the GPS read...
        if(( ck_a == UBX_ck_a ) && ( ck_b == UBX_ck_b )){   // Verify the received checksum with the generated checksum..
          
          Serial.println(F("Case 8 - Checksum OK!, Packet complete!"));
          
        }else{
          //if (PrintErrors)
            Serial.println(F("Case 8 - Checksum byte error, skipping packet, cancelling!!!: "));
          //}
          // Variable initialization
          UBX_step=0;
          ck_a=0;
          ck_b=0;
          GPS_timer = millis(); //Restarting timer...
          break;
        }
      } // End switch...      
    } // End for...
  } // End if numc
} // End function

// Ublox checksum algorithm
void ubx_checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}
