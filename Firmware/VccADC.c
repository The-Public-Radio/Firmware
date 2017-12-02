


/***

These functions use the internal bandgap to measure the Vcc voltage

This code is processor specific so may not work on other chips besides ATTINY25/45/85.

This code assumes default clock speed of 1MHz.

More info here...
https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/


***/

#include <avr/io.h>

#define F_CPU 1000000

#include <util/delay.h>

// Enables ADC and sets to read the internal 1.1V bandgap voltage against Vcc scale

void adc_on(void) {
    
    // Select ADC inputs
    // bit    76543210 
    // REFS = 00       = Vcc used as Vref
    // MUX  =   100001 = Single ended, 1.1V (Internal Ref) as Vin
    
    // ADMUX = 0b00100001;
    ADMUX = _BV(MUX3) | _BV(MUX2); // For ATtiny85, Select Vbg as input

    /*
    By default, the successive approximation circuitry requires an input clock frequency between 50
    kHz and 200 kHz to get maximum resolution.
    */  
                
    // Enable ADC, set prescaller to /8 which will give a ADC clock of 1mHz/8 = 125kHz    
    ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);

    // Enable ADC, set prescaller to /2 which will give a ADC clock of 128khz/2 = 64kHz    
    //ADCSRA = _BV(ADEN);

    
    /*
        After switching to internal voltage reference the ADC requires a settling time of 1ms before
        measurements are stable. Conversions starting before this may not be reliable. The ADC must
        be enabled during the settling time.
    */
        
    _delay_ms(1);
                
    /*
        The first conversion after switching voltage source may be inaccurate, and the user is advised to discard this result.
    */
    
    ADCSRA |= _BV(ADSC);                // Start a conversion


    while( ADCSRA & _BV( ADSC) ) ;      // Wait for 1st conversion to be ready...
                                        //..and ignore the result        
    
}


    /*  
        Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
        mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
        sleep modes to avoid excessive power consumption.
    */

void adc_off(void) {
    
   ADCSRA &= ~_BV( ADEN );         // Disable ADC to save power
    
}        


uint16_t readADC(void) {
    
        
    ADCSRA |= _BV(ADSC);                // Start a conversion


    while( ADCSRA & _BV( ADSC) ) ;      // Wait for 1st conversion to be ready...
                                        //..and ignore the result                        
        
    /*
        After the conversion is complete (ADIF is high), the conversion result can be found in the ADC
        Result Registers (ADCL, ADCH).      
        
        When an ADC conversion is complete, the result is found in these two registers.
        When ADCL is read, the ADC Data Register is not updated until ADCH is read.     
    */
    
    // Note we could have used ADLAR left adjust mode and then only needed to read a single byte here
        
    uint16_t adc = ADC;;       // 0<= result <=1023
                
    return adc;
                    
}
