/*#define GPS_BUFFERSIZE 300//83
byte recvBuffer[GPS_BUFFERSIZE];
boolean GPS_checksum_calc = false;
byte recvIdx;
char c;

//
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


  /*while ( nss.available() > 0 && now - lastSendTime > 1000 )
  {
    c = nss.read();
    if ( c == '$' )
    {
      GPS_checksum_calc = false;
      recvIdx = 0;
      recvBuffer[recvIdx++] = c;
    }
    /*else if ( recvIdx == 6 )
    {
      int len = ( sizeof (nmeaPackages) / sizeof (*nmeaPackages) ); // how many elements in array
      int x; // generic loop counter
      char buffer[128];
      char* PauseStr;
      PauseStr = (char*)recvBuffer;
      
      for ( x = 0; x < len; x++ )
      {
        sprintf ( buffer, "Looking for %s (at index %d)...", (char*)recvBuffer, x );
        //Serial.print (buffer);
        
        if ( strcmp ((char*)recvBuffer, nmeaPackages[x]) == 0 )
        { // this is the key: a match returns 0
          recvBuffer[recvIdx++] = c;
          sprintf ( buffer, "Found %s in array at index %d!\r\n", nmeaPackages[x], x );
          //Serial.print (buffer);
        }
        else
        {
          recvIdx = 0;   // Lets throw this package away
          nss.flush();
          sprintf ( buffer, "Fail. %s is not %s at index %d!\r\n", (char*)recvBuffer, nmeaPackages[x], x );
          //Serial.print (buffer);
        }
      }
    }
    else if ( c == '\r' || c == '\n' )            // || c == '\r\n'
    {
      recvBuffer[recvIdx++] = 0;
      if ( GPS_checksum_calc )
      {
        //float data1=3.14159f;
        //rf95.send((uint8_t*)&data1, sizeof(data1));
        
        //uint8_t data[] = "Sender started";
        //rf95.send(data, sizeof(data));  
        //rf95.waitPacketSent();
  
        /*rf95.send((uint8_t *)&recvBuffer, sizeof(recvBuffer));
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
        nss.flush();
        Serial.println("Buffer overflow!");
      }
    }
  }*/
