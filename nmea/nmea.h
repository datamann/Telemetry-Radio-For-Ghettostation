/*
 * Written by Stig B. Sivertsen
 * sbsivertsen@gmail.com
 * https://github.com/datamann/GPSConfigurator
 * 11.09.2019
 * @see The GNU Public License (GPL) Version 3
*/

void turnOnNMEADefaultSet(){
  // Enable default NMEA packets
  #ifdef DEBUG
    Serial.print(F("Switching on default NMEA packets: "));
  #endif  
  uint8_t setDefaultOn[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x01,0xFB,0x10, //(GxGGA)
                            0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x01,0xFC,0x12, //(GxGLL)
                            0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x01,0xFD,0x14, //(GxGSA)
                            0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x01,0xFE,0x16, //(GxGSV)
                            0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x01,0xFF,0x18, //(GxRMC)
                            0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x01,0x00,0x1A  //(GxVTG)
                            };
  sendUBX ( setDefaultOn, sizeof(setDefaultOn)/sizeof(uint8_t) );
  getUBX_ACK ( setDefaultOn );
}

void turnOffNMEA(){
  #ifdef DEBUG
    Serial.print(F("Switching off all NMEA packets: "));
  #endif  
  uint8_t setALLOff[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F, //(GxGGA)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11, //(GxGLL)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13, //(GxGSA)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15, //(GxGSV)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17, //(GxRMC)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19, //(GxVTG)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0A,0x00,0x04,0x23, //(GxDTM)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x09,0x00,0x03,0x21, //(GxGBS)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0D,0x00,0x07,0x29, //(GxGNS)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x06,0x00,0x00,0x1B, //(GxGRS)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x07,0x00,0x01,0x1D, //(GxGST)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0F,0x00,0x09,0x2D, //(GxVLW)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x08,0x00,0x02,0x1F, //(GxZDA)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x00,0x00,0xFB,0x12, //(PUBX-00)
                         //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x01,0x00,0xFC,0x14, //(PUBX-01) Failed on Beitian BN-880
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x03,0x00,0xFE,0x18, //(PUBX-03)
                         0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x04,0x00,0xFF,0x1A  //(PUBX-04)
                         //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x05,0x00,0x00,0x1C, //(PUBX-05) Failed on Beitian BN-880
                         //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x06,0x00,0x01,0x1E  //(PUBX-06) Failed on Beitian BN-880
                        };
  sendUBX(setALLOff, sizeof(setALLOff)/sizeof(uint8_t));
  getUBX_ACK(setALLOff);
}

void turnOnNMEA(){
  // Enable NMEAs
  #ifdef DEBUG
    Serial.print(F("Switching on all NMEA packets: "));
  #endif
  uint8_t setALLOn[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x01,0xFB,0x10, //(GxGGA)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x01,0xFC,0x12, //(GxGLL)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x01,0xFD,0x14, //(GxGSA)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x01,0xFE,0x16, //(GxGSV)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x01,0xFF,0x18, //(GxRMC)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x01,0x00,0x1A, //(GxVTG)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0A,0x01,0x05,0x24, //(GxDTM)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x09,0x01,0x04,0x22, //(GxGBS)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0D,0x01,0x08,0x2A, //(GxGNS)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x06,0x01,0x01,0x1C, //(GxGRS)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x07,0x01,0x02,0x1E, //(GxGST)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0F,0x01,0x0A,0x2E, //(GxVLW)
                        0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x08,0x01,0x03,0x20  //(GxZDA)
                        //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x00,0x01,0xFC,0x13, //(PUBX-00)
                        //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x01,0x01,0xFD,0x15, //(PUBX-01) Failed on Beitian BN-880
                        //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x03,0x01,0xFF,0x19, //(PUBX-03) ?
                        //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x04,0x01,0x00,0x1B  //(PUBX-04)
                        //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x05,0x01,0x01,0x1D, //(PUBX-05) Failed on Beitian BN-880
                        //0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x06,0x01,0x02,0x1F  //(PUBX-06) Failed on Beitian BN-880
                        };
  sendUBX ( setALLOn, sizeof(setALLOn)/sizeof(uint8_t) );
  getUBX_ACK ( setALLOn );
}
   
void turnOnNMEAGhettostation()
{
  turnOffNMEA();
  #ifdef DEBUG
    Serial.print(F("Switching on NMEA packets for Ghettostation: "));
  #endif
  uint8_t turnOnNMEAGths[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x01,0xFB,0x10, // $GPGGA : Global Positioning System Fix Data
                              0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x01,0x00,0x1A  // $GPVTG : Ttack and Ground Speed
                             };
  sendUBX(turnOnNMEAGths, sizeof(turnOnNMEAGths)/sizeof(uint8_t));
  getUBX_ACK(turnOnNMEAGths);
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
    #ifdef DEBUG
      Serial.println( gps.print(s) );
    #endif   
    
    while ( gps.available() )
    {
      char c = gps.read();
      #ifdef DEBUG
        Serial.print(c);
      #endif
    }
    #ifdef DEBUG
      Serial.println();
    #endif
}

void turnOffUBX()
{
  #ifdef DEBUG
    Serial.print(F("Switching off UBLOX: "));
  #endif  
  uint8_t setUBXOff[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x07,0x00,0x12,0x50,  //NAV-PVT off
                         0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x00,0x0D,0x46,  //NAV-POSLLH off
                         0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x00,0x0E,0x48,  //NAV-STATUS off
                         0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x04,0x00,0x0F,0x4A,  //NAV-DOP off
                         0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x00,0x11,0x4E,  //NAV-SOL off
                         0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x00,0x1D,0x66   //NAV-VELNED off
                         };
  sendUBX(setUBXOff, sizeof(setUBXOff)/sizeof(uint8_t));
  getUBX_ACK(setUBXOff);  
}

// Switch off GLL
  /*Serial.print("Switching off NMEA GLL: ");
  uint8_t setGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
  sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
  getUBX_ACK(setGLL);*/
 
  // Switch off GSA
  /*Serial.print("Switching off NMEA GSA: ");
  uint8_t setGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
  sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
  getUBX_ACK(setGSA);*/
 
  // Switch off GSV
  /*Serial.print("Switching off NMEA GSV: ");
  uint8_t setGSV[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
  sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
  getUBX_ACK(setGSV);*/
 
  // Switch off RMC
  /*Serial.print("Switching off NMEA RMC: ");
  uint8_t setRMC[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40 };
  sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
  getUBX_ACK(setRMC);*/
