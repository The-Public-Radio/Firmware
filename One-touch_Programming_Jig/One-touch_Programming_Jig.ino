




void setup() {

// Pins A0, A1, and A2 used to power the attached TPR
  
  DDRC  = 0b00000111;       // Output
  PORTC = 0b00000111;       // Drive output pins high
  
  Serial.begin(9600);
  while (! Serial); // Wait until Serial is ready 
  Serial.println("Enter LED Number 0 to 7 or 'x' to clear"); 

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

void loop() {

  //delay(500);

  while (1) {

    if (Serial.available()) {
    
      uint8_t led = Serial.parseInt();
      //uint8_t led = ch - '0';    

      sendbyte(led);
      //sendbyte(led);

      Serial.println( led );
                
    }
  
    //pinMode(12,INPUT);  
    //delay(2000);
    

    
  }
}
