/*static const char * baudInitDataNMEA[GPS_BAUDRATE_COUNT] = {
    "$PUBX,41,1,0003,0001,115200,0*1E\r\n",     // GPS_BAUDRATE_115200
    "$PUBX,41,1,0003,0001,57600,0*2D\r\n",      // GPS_BAUDRATE_57600
    "$PUBX,41,1,0003,0001,38400,0*26\r\n",      // GPS_BAUDRATE_38400
    "$PUBX,41,1,0003,0001,19200,0*23\r\n",      // GPS_BAUDRATE_19200
    "$PUBX,41,1,0003,0001,9600,0*16\r\n"        // GPS_BAUDRATE_9600
};*/
 
 //Serial.println("Turn on GGA & RMC: ");
  //char nmea[] = "$PMTK314,1,1,1,1,1,5,1,1,1,1,1,1,0,1,1,1,1,1,1*2C\r\n";
  //char nmea[] = "$PMTK314,-1*04\r\n";                                       // Restore system default
  //char nmea[] = "$PMTK251,38400*27\r\n";                                  // Set Baudrate to 38400
  //char nmea[] = "$PMTK605*31\r\n";                                        // Query Firmware info
  //char nmea[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";  // Disable all messages except GGA and RMC
  //sendNMEA(nmea);

/*#define GPS_BUFFERSIZE 300//83
byte recvBuffer[GPS_BUFFERSIZE];
boolean GPS_checksum_calc = false;
byte recvIdx;
char c;

if ( c == '$' )
    {
      GPS_checksum_calc = false;
      recvIdx = 0;
      recvBuffer[recvIdx++] = c;
    }
    else if ( c == '\r' || c == '\n' )            // || c == '\r\n'
    {
      recvBuffer[recvIdx++] = 0;
      if ( GPS_checksum_calc )
      {  
        rf95.send((uint8_t *)&recvBuffer, sizeof(recvBuffer));
        rf95.waitPacketSent();
        lastSendTime = now;
        String myString = String((char *)recvBuffer);
        Serial.println(myString);
      }
      continue;
    }
    else
    {
      if ( recvIdx < (GPS_BUFFERSIZE - 1) )
      {
        if ( c == '*' )
        {
          GPS_checksum_calc = true;
        }
        recvBuffer[recvIdx++] = c;
      }
      else
      {
        recvIdx = 0;   // Buffer overflow : restart
        gps.flush();
        Serial.println("Buffer overflow!");
      }
    }



/*
static const char * nmeaPackages[13] = {
  "$GNDTM",
  "$GNRMC",
  "$GNVTG",
  "$GNGNS",
  "$GNGSA",
  "$GNGGA",
  "$GPGSV",
  "$GNGLL",
  "$GLGSV",
  "$GNZDA",
  "$GNGST",
  "$GNGBS",
  "$GNVLW"
  };*/
