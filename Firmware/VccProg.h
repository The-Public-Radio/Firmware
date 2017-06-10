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


// Read a programming byte 
// Times out after 40ms
// Returns -1 if error
// Assumes we are >PROGRAM_V on entry

int readPbyte();



// Is the current Vcc higher than the programming voltage threshold?

uint8_t programmingVoltagePresent();


