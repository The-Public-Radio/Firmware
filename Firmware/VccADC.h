


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


// ADC clock at 1mhz with /8 prescaller is 125Khz
// Takes ~125us 

#define ADC_DELAY_US 125 

uint16_t readADC(void);

// This macro gives you the current Vcc voltage for a given ADC value returned from readADC()

#define ADC2VCC(a) ( (1.1*1023) / a )

// This macro will give you the value returned from the ADC for a Vcc voltage of V
// Nice to have in a macro because for fixed voltages, you can precompute 
// rather than doing an expensive 16 bit multiply and divide at runtime. 

// Vcc   =  (1.1v * 1024) / ADC

#define VCC2ADC(v) ((uint16_t) ((1.1*1023.0)/v))

// This macro tests if the current Vcc is currently below the specified voltage
// Note that the `<` is reversed becuase larger values from the ADC corespond to smaller
// Vcc voltages when measuring the internal bandgap like we do here. 


// Note that this is a macro so the floating point math in VCC2ADC can be evaluated statically
// when V is const, which it should be. 

#define VCC_LESS_THAN(v) (readADC()>VCC2ADC(v))      // returns true if the Measured Vcc is than V
