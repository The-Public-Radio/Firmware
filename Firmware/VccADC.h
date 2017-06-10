


/***

These functions use the internal bandgap to measure the Vcc voltage

This code is processor specific so may not work on other chips besides ATTINY25/45/85.

This code assumes default clock speed of 1MHZ.

More info here...
https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/


***/

#if !defined( F_CPU )
    #error F_CPU must be definded before this header
#endif

#if F_CPU!=1000000
    #error You must edit the ADC prescaler to match F_CPU if it is not 1000000!
#endif

#include <avr/io.h>
#include <util/delay.h>

// Enables ADC and sets to read the internal 1.1V bandgap voltage against Vcc scale

void adc_on(void);
void adc_off(void);


// This macro will give you the value returned from the ADC for a Vcc voltage of V
// Nice to have in a macro because for fixed voltages, you can precompute 
// rather than doing an expensive 16 bit multiply and divide at runtime. 

// Vcc   =  (1.1v * 1024) / ADC

#define VCC2A(v) ((1.1*1023.0)/v)


// ADC clock at 1mhz with /8 prescaller is 125Khz
// Takes ~125us 

#define ADC_DELAY_US 125 

uint16_t readADC(void);
