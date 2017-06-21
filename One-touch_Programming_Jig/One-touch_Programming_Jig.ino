
#include <util/crc16.h>



void setup() {

// Pins A0, A1, and A2 used to power the attached TPR
  
  DDRC  = 0b00000111;       // Output
  PORTC = 0b00000111;       // Drive output pins high
  
  Serial.begin(9600);
  while (! Serial); // Wait until Serial is ready 

}


/**
 * 
 *  TPR EEPROM programming protocol
 *  ===============================
 *  
 *  TPR knows it is in programming mode when it sees 5V on Vcc. 
 *  
 *  Batteries can only make <4V (even Energizer Ultimate Lithium only 1.8V each, so 2xAA =3.6V), 
 *  so promamming mode can only be  *  entered when external power source- like programmer.
 *  
 *  Once TPR see the 5V, it starts listening for data bits. 
 *  
 *  Becuase of streched out signals due to the decoupling cap, the comunication channel is all about 
 *  high to low transitions which happen quickly. This gives a variable ammount of for the voltage to 
 *  rise back up again. If we were to sample at fixed times, maybe the voltage was too slow to rise, or
 *  too quick and we would get wrong samples. 
 *  
 *  Each bit is a sync pulse followed by optional data pulse.
 *  
 *  Each pulse is a dip in voltage lasting for about 1ms. 
 *  
 *  A full pulse cycle takes about 10ms to RX becuase of the recovery time of the 10uF cap. 
 *  This speed reqires using 2 IO pins to drive enough current. Make sure you
 *  switch both pins simultainiously using PORT assignmnets!
 *  
 *  When a sync is seen, TPR waits 7.5ms and starts looking for a 2nd pulse. 
 *  If a second pulse is seen in the next 5ms, then a 1 is recieved, otherwise a 0.
 *  
 *  Then we start looking for next sync pulse, which shoudl come in about 2.5ms.
 * 
 *  8 bits to a byte. TODO: Add parity?
 *  
 *  No break neede between bytes.
 *  
 *  Variable length frames have 50ms ide between them.
 *    
 *  Checksum handled by higher frame layer. 
 *  
 *  If you ever wait longer than 40ms for a pulse, then you abort the byte and frame and start seaching again.
 *  
 *  TODO: Add hystereis to voltage thresholds?
 *  TODO: Make voltage levels relative rather than absolutle?
 * 
 */


void sendbit(int b) {

    PORTC = 0b00000000;         // Send sync pulse
    delay(1);
    PORTC = 0b00000111;
    delay(9);

     
    if (b) {

      PORTC = 0b00000000;
      delay(1);
      PORTC = 0b00000111;
      delay(9);

    } else {

      delay(10);
     
    }

}


void sendbyte(uint8_t b) {

  //Serial.print("XMIT:");
  //Serial.print(b,HEX);
  //Serial.println("");

  int bitmask=0b10000000;

  while (bitmask) {

    if (bitmask&b) {
      sendbit(1);
    } else {
      sendbit(0);
    }

    bitmask >>=1;
   
  }

      

}

void sendpacket( uint16_t channel  , uint8_t band, uint8_t deemphassis , uint8_t spacing ) {

  uint16_t crc = 0x0000;

  sendbyte( channel >> 8 );
  
  crc = _crc16_update(crc, channel>>8 );   
            
  sendbyte( channel & 0xff );
  crc = _crc16_update(crc, channel & 0xff );    

  /*

  sendbyte( band );
  crc = _crc16_update(crc, band );    

  sendbyte( deemphassis );
  crc = _crc16_update(crc, deemphassis );    

  sendbyte( spacing );
  crc = _crc16_update(crc, spacing );    

  */

  sendbyte( crc >> 8 );
  sendbyte( crc & 0xff );
      
}

void sendprogramming( float station  , uint8_t band, uint8_t deemphassis , uint8_t spacing ) {

  const float base[]  = { 87.5, 76, 76};            // Base freqenecy based on band
  const float step[] = {  0.20 , 0.10 , 0.05 };     // Freqnecy step based on spacing

  Serial.print(" computed channel=");

  uint16_t channel  = (uint16_t) ((station - base[band]) / step[ spacing] );

  Serial.print( channel );
  Serial.print( "..." );

  sendpacket( channel , band, deemphassis , spacing ); 

}

int readLine(char *buffer , int size) {

    int len=0;

    while (1) {

      if (Serial.available()) {
  
        int c = Serial.read(); 
  
        if (c=='\n') { 

          //Serial.println("XXX");
                    
          buffer[len]=0x00; // Terminate string

          return(0); 
            
        } else {
  
          if (isprint(c)) {
  
            if (len<size) {

              Serial.print( (char) c);              
    
              buffer[len++] = c;
              
            }
          }
        }        
      }
    }
}

#define BUFFER_LEN 20

void loop() {

  //delay(500);

  float station=103.5;
  uint8_t band =0;
  uint8_t deemphassis=0;
  uint8_t spacing =0;
  

  while (1) {

    Serial.println("TPR one-touch programmer");       
    Serial.println("========================");
    
    Serial.print("S-Station    [");
    Serial.print(station);
    Serial.println("]");

    Serial.print("B-Band       [");
    Serial.print(band);
    Serial.println("]");

    Serial.print("D-Deemphassis[");
    Serial.print(deemphassis);
    Serial.println("]");

    Serial.print("P-sPacing    [");
    Serial.print(spacing);
    Serial.println("]");


    Serial.println("");
    Serial.println("ENTER-Program it!");
    
    char buffer[BUFFER_LEN];

    if (!readLine( buffer , BUFFER_LEN )){


      Serial.println("");


      switch (toupper(buffer[0])) {

        case 0x00:

          Serial.print("Programming...");
          sendprogramming( station  , band, deemphassis , spacing );
          Serial.println("done.");
          break;
          
        case 'S':
          station=String(buffer+1).toFloat();
          Serial.println("Station set.");
          break;

        case 'B':
          band=buffer[0]-'0';
          Serial.println("Band set.");
          break;

        case 'D':
          deemphassis=buffer[0]-'0';
          Serial.println("Deemphassis set.");
          break;

        case 'P':
          spacing=buffer[0]-'0';
          Serial.println("Spacing set.");
          break;


       default:
          Serial.print("Don't understand [");
          Serial.print(buffer);         
          Serial.println("]");
          break;
       
      }
    }
  }
}
