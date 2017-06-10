


/***

These functions implement programming config by modulating the Vcc voltage

This code is processor specific so may not work on other chips besides ATTINY25/45/85.

This code assumes default clock speed of 1MHZ.


***/

#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>

#include "VccADC.h"
#include "VccProg.h"

#define PROGRAM_V (4.5)

// Must see >4.5V for at least 5ms to know we are in programming mode. 
// Can never happen on batteries since they only make at MOST 3.6V (with lithium 2xAAs)
// We look over 5 ms to make sure not a glitch


// Currently at or below programming voltage?

#define PROGRAM_V_TEST() ( !VCC_GT(PROGRAM_V ) )

// Is the current Vcc higher than the programming voltage threshold?

uint8_t programmingVoltagePresent() {
    return( PROGRAM_V_TEST() );
}    


// Read a programming bit
// Returns -1 if error
// Assumes we are >PROGRAM_V on entry

#define PP_TIMEOUT_US  40000       // Time to wait for initial sync pulse before timing out
#define PP_DP_DELAY_US 15000       // Delay from sync pulse to start looking for data pulse
#define PP_DP_SEACH_US 10000       // Time window to look for data pulse
#define PP_MARGIN_US    5000       // delay from end of data pulse window to start looking for next sync pulse



int readPbit(void) {
    
    uint16_t sp_countdown = PP_TIMEOUT_US / ADC_DELAY_US;      // Wait up to 20ms for a sync pulse
        
    while ( !PROGRAM_V_TEST() ) {
        
        sp_countdown--;
        
        if (!sp_countdown) {
         
            return(-1);         /// Timed out- waited more than 20ms for sync
            
        }                    
       
    }
            
    // just got beginning of sync
    // It take about 5ms for the voltage to rise back up to the threshold after the pulse
    // so we delay 7.5ms so we can be...
    // 2.5ms after we should be back above threshold, 
    // and 2.5ms before scheduled next data bit pulse...
                
    _delay_us( PP_DP_DELAY_US );          

           
    if ( PROGRAM_V_TEST() ) {      // we should have risen back above threshold 5ms ago...
        
        PORTB |= _BV(1); 
        _delay_ms(1);
        PORTB &= ~_BV(1);
        _delay_ms(1);
        PORTB |= _BV(1); 
        _delay_ms(1);
        PORTB &= ~_BV(1);
        _delay_ms(1);        
                
        return(-2);                             // if not, then something is wrong. 
                
    }                
                    
    uint8_t dp_countdown = PP_DP_SEACH_US/ADC_DELAY_US;      // window for the data pulse
            
    uint8_t bitflag = 0;                   // Did we get a data pulse? Is this a 1 bit?
    
    PORTB |= _BV(1); 

    while ( dp_countdown--  && !bitflag ) {
        
        if ( PROGRAM_V_TEST() ) {
            
            bitflag =1 ;
                        
        }        
                                        
    }                                    
    
    PORTB &= ~_BV(1);
        
    if (bitflag) {
        
        _delay_us( PP_DP_DELAY_US );        // This is a 1 bit, so wait for voltage to rise again. 
        
    }        
    
    // If a zero bit, we will just start searching sooner, but next bit will come soooner than timeout.
    
        
    if ( PROGRAM_V_TEST()  ) {                  // we should have risen back above threshold 5ms ago...
                
        return(-3);                             // if not, then something is wrong. 
                
    }                
       
    return bitflag;
                
}    

// Read a programming byte 
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


