


/***

These functions implement programming config by modulating the Vcc voltage

This code is processor specific so may not work on other chips besides ATTINY25/45/85.

This code assumes default clock speed of 1MHZ.


***/

#if !defined( F_CPU )
    #error F_CPU must be definded before this header
#endif

#if F_CPU!=1000000
    #error You must edit the ADC prescaler to match F_CPU if it is not 1000000!
#endif

#include <avr/io.h>
#include <util/delay.h>

// Is the current Vcc higher than the programming voltage threshold?

uint8_t programmingVoltagePresent() {
    return( PROGRAM_V_TEST() );
}    

// Read a programming byte 
// Times out after 40ms
// Returns -1 if error
// Assumes we are >PROGRAM_V on entry

int readPbyte(void) {
    
    uint8_t bitmask=0b10000000;
    uint8_t byte=0;
    
    while (bitmask) {
        
        int bit = readPbit();
        
        if (bit<0) return( bit);        // Error;
        
        if (bit!=0) byte |= bitmask;
        
        bitmask >>= 1;
        
    }        
    
    return byte;
    
}




