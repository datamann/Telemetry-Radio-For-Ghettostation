/*
 * Written by Stig B. Sivertsen
 * sbsivertsen@gmail.com
 * https://github.com/datamann/GPSConfigurator
 * 11.09.2019
 * @see The GNU Public License (GPL) Version 3
*/

#ifdef DATARATE_1_HZ
  // Set Datarate to 1HZ
  String txtToDisplayDR = "Setting datarate to 1HZ...";
  uint8_t setRate[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39}; //(1Hz)
#endif

#ifdef DATARATE_5_HZ
  // Set Datarate to 5HZ
  String txtToDisplayDR = "Setting datarate to 5HZ...";
  uint8_t setRate[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};  //(5Hz)
#endif

#ifdef DATARATE_10_HZ
  // Set Datarate to 5HZ
  String txtToDisplayDR = "Setting datarate to 10HZ...";
  uint8_t setRate[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12}; //(10Hz)
#endif

// Revert to default configuration
#ifdef SET_FACTORY_DEFAULT
  String txtToDisplayFD = "Revert to default configuration: ";
  uint8_t revertDefault[] = {0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x03,0x1B,0x9A};
#endif

//(GPS,Galileo,QZSS,Glonass)
#ifdef USE_DEFAULT
String txtToDisplay = "Turn on most used satellites: ";
uint8_t activateSats[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
                          0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
                          0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,
                          0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01,
                          0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,
                          0x2F,0x81};
#endif

//(GPS & QZSS)
#ifdef USE_GPS_QZSS
String txtToDisplay = "Turn on GPS & QZSS: ";
uint8_t activateSats[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
                          0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
                          0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x02,0x04,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01,
                          0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01,
                          0x2D,0x59};
#endif

//(Galileo)
#ifdef USE_GALILEO
String txtToDisplay = "Turn on Galileo: ";
uint8_t activateSats[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
                          0x00,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,
                          0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01,
                          0x2C,0x3D};
#endif

//(Glonass)
#ifdef USE_GLONASS
String txtToDisplay = "Turn on Glonass: ";
uint8_t activateSats[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
                          0x00,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x02,0x04,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,
                          0x2C,0x1D};
#endif

//(BeiDou)
#ifdef USE_BEIDOU
String txtToDisplay = "Turn on BeiDou: ";
uint8_t activateSats[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
                          0x00,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x02,0x04,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
                          0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,
                          0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,
                          0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01,
                          0x2C,0x35};
#endif

//(GPS,SBAS,Galileo,IMES,QZSS,Glonass)
#ifdef USE_ALL
String txtToDisplay = "Turn on all satellites (GPS,SBAS,Galileo,IMES,QZSS,Glonass): ";
uint8_t activateSats[] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
                          0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
                          0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01,
                          0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,
                          0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
                          0x04,0x00,0x08,0x00,0x01,0x00,0x01,0x01,
                          0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01,
                          0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,
                          0x31,0xC1
                          };
#endif

// Send a byte array of UBX protocol to the GPS
void sendUBX ( uint8_t *MSG, uint8_t len )
{
  for ( int i=0; i<len; i++ )
  {
    gps.write(MSG[i]);
  }
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK ( uint8_t *MSG )
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  
  #ifdef DEBUG
    Serial.print(" * Reading ACK response: ");
  #endif
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;    // header
  ackPacket[1] = 0x62;    // header
  ackPacket[2] = 0x05;    // class
  ackPacket[3] = 0x01;    // id
  ackPacket[4] = 0x02;    // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;       // CK_A
  ackPacket[9] = 0;       // CK_B
 
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
        #ifdef DEBUG
          Serial.println(" (SUCCESS!)");  
        #endif        
        BAUDRATE_OK = true;
        return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if ( millis() - startTime > 3000 )
    {
      #ifdef DEBUG
        Serial.println(" (FAILED!)");
      #endif
      BAUDRATE_OK = false;
      return false;
    }
 
    // Make sure data is available to read
    if ( gps.available() )
    {
      b = gps.read();
 
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

void autoBaud()
{
  long int baudRate[6] = { 4800,9600,19200,38400,57600,115200 };
  while ( !BAUDRATE_OK )
  {
    for ( int i=0; i<6; i++ )
    {
      // Switch baud rates on the software serial
      #ifdef DEBUG
        Serial.println(String("Switching to ") + baudRate[i] + String(" for GPS port."));
      #endif      
      gps.begin(baudRate[i]);
      delay(2000);

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

void checkConnectivity()
{
  #ifdef DEBUG
    Serial.println();
    Serial.print("Checking connectivity...");
    Serial.println();
  #endif
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