// Modified by Stig Sivertsen
#include <SPI.h>
#include <RH_RF95.h>
RH_RF95 rf95;

void setup() 
{ 
  Serial.begin(19200);
  
  //while (!Serial){
    if (!rf95.init()){
      Serial.println("Init failed");
    } else {
      Serial.println("Init succeeded");
    }
  //}
  
    rf95.setFrequency(433.00);
    //rf95.setFrequency(868.00);
    //rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);  // Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range.
    //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);   // Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range.
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);    // Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.
    //rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);      // Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range.
    rf95.setTxPower(13, false);                        // with useRFO false, valid values are from +5 to +23, 13 = default
}

/*uint8_t buffer[] = {'$','G','P','G','G','A',',','1','2','3','5','1','9',
                    ',','4','8','0','7','.','0','3','8',',','N',',','0',
                    '1','1','3','1','.','0','0','0',',','E',',','1',',',
                    '0','8',',','0','.','9',',','5','4','5','.','4',',',
                    'M',',','4','6','.','9',',','M',',',',','*','4','7','\n',
                    '$','G','P','V','T','G',',','3','6','0','.','0',',',
                    'T',',','3','4','8','.','7',',','M',',','0','0','0',
                    '.','0',',','N',',','0','0','0','.','0',',','K','*',
                    '4','3','\n'};*/

#define GPS_BUFFERSIZE 120
byte recvBuffer[GPS_BUFFERSIZE];
unsigned long lastSendTime = 0;
int i = 0;

void loop() {

  unsigned long now = millis();
  if ( now - lastSendTime >= 1000 )
  {
    if ( rf95.available() )
    {        
      // Should be a message for us now   
      uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(data);

      if ( rf95.recv(data, &len) )
      {
        for( i = 0; i < len; i++ ){        
          // Writing using UART
          Serial.write(data[i]);
          delay(05); // Need delay to be able to receive properly on the recevie side.
        }
      }
      else
      {
        Serial.println("recv failed");
      }
    }
    else
    {
      Serial.println("RF95 not available!");
    }    
    lastSendTime = now;
  }
}
