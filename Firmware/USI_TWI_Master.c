/*****************************************************************************
*
* Atmel Corporation
*
* File              : USI_TWI_Master.c
* Compiler          : AVRGCC Toolchain version 3.4.2
* Revision          : $Revision: 992 $
* Date              : $Date: 2013-11-07 $
* Updated by        : $Author: Atmel $
*
* Support mail      : avr@atmel.com
*
* Supported devices : All device with USI module can be used.
*                     The example is written for the ATmega169, ATtiny26 and ATtiny2313
*
* AppNote           : AVR310 - Using the USI module as a TWI Master
*
* Description       : This is an implementation of an TWI master using
*                     the USI module as basis. The implementation assumes the AVR to
*                     be the only TWI master in the system and can therefore not be
*                     used in a multi-master system.
* Usage             : Initialize the USI module by calling the USI_TWI_Master_Initialise() 
*                     function. Hence messages/data are transceived on the bus using
*                     the USI_TWI_Transceive() function. The transceive function 
*                     returns a status byte, which can be used to evaluate the 
*                     success of the transmission.
*
****************************************************************************/

#define	F_CPU	1000000UL	/* Hack until we fix things the right way (tm) */



#include <avr/io.h>
#include "USI_TWI_Master.h"
#include <util/delay.h>

#define BIT_TIME_US     (5)          // How long should should we wait between bit transitions?

#define SBI(port,bit) (port|=_BV(bit))
#define CBI(port,bit) (port&=~_BV(bit))
#define TBI(port,bit) (port&_BV(bit))


// These are open collector signals, so never drive high - only drive low or pull high

static inline void sda_drive_low(void) {
    CBI(PORT_USI, PIN_USI_SDA);      // Stop pulling high 
    SBI(DDR_USI , PIN_USI_SDA);      // Drive low
}    


static inline void sda_pull_high(void) {
    CBI(DDR_USI , PIN_USI_SDA);      // Stop driving low
    SBI(PORT_USI, PIN_USI_SDA);      // Pull high
}    

static inline void scl_drive_low(void) {
    CBI(PORT_USI, PIN_USI_SCL);      // Stop pulling high 
    SBI(DDR_USI , PIN_USI_SCL);      // Drive low
}    


static inline void scl_pull_high(void) {
    CBI(DDR_USI , PIN_USI_SCL);      // Stop driving low
    SBI(PORT_USI, PIN_USI_SCL);      // Pull high
}    

static inline uint8_t sda_read(void) {
    return TBI(PIN_USI , PIN_USI_SDA);
}    

/*---------------------------------------------------------------
 USI TWI single master initialization function
---------------------------------------------------------------*/
void USI_TWI_Master_Initialise( void )
{
        
  sda_pull_high();
  scl_pull_high();
  
  // This leaves us with both SCL and SDA high, which is an idle state  
}


// Write a byte out to the slave and look for ACK bit
// Assumes SCL low, SDA doesn't matter

// Returns 0=success, SDA high, SCL low. The ACK bit is still on SDA but will disapear on next SCL falling edge

static unsigned char USI_TWI_Write_Byte( unsigned char data ) {
    
    
    
    for( uint8_t bitMask=0b10000000; bitMask !=0; bitMask>>=1 ) {
        
        // setup data bit
                        
        if ( data & bitMask) {
            sda_pull_high();
        } else {
            sda_drive_low();
            
        }                
        
        // clock it out        

        _delay_us(BIT_TIME_US);
                
        scl_pull_high();           // Clock in the next address bit
        
        _delay_us(BIT_TIME_US);
        
        scl_drive_low();        
                        
    }        
            
    // The device acknowledges the address by driving SDIO
    // low after the next falling SCLK edge, for 1 cycle. 
        
    sda_pull_high();            // Pull SDA high so we can see if the salve is driving low
    _delay_us(BIT_TIME_US);     // Not needed, but so we can see what is happening on the scope
    scl_pull_high();
    _delay_us(BIT_TIME_US);     // TODO: Don't need all these delays
    
    uint8_t ret = sda_read();   // slave should be driving low now
    scl_drive_low();            // Slave release
    _delay_us(BIT_TIME_US);     
    
    // TODO: CHeck ret
        
    return(ret);        
        
}

// Read a byte from the slave and send ACK bit
// Assumes SCL low, returns with SCL low
// Assumed SDA pulled high

// Returns 0=success, SDA high, SCL high

static unsigned char USI_TWI_Read_Byte(void) {
          
    unsigned char data=0;
    
    for( uint8_t bitMask=0b10000000; bitMask !=0; bitMask>>=1 ) {
        
        // Clock in the address bits
        
        // TODO: Finish this
                                
        scl_pull_high();           // Clock in the next address bit
        
        _delay_us(BIT_TIME_US);
        
        if (sda_read()) {
            
            data |= bitMask;
            
        }            
        
        scl_drive_low();
        
        _delay_us(BIT_TIME_US);
                                        
    }      
    
    //After each byte of data is read,
    //the controller IC must drive an acknowledge (SDIO = 0)
    //if an additional byte of data will be requested. Data
    //transfer ends with the STOP condition.         

    sda_drive_low();            // Pull SDA high so we can see if the salve is driving low
    _delay_us(BIT_TIME_US);     // Not needed, but so we can see what is happening on the scope       
    scl_pull_high();            // Clock out the ACK bit    
    _delay_us(BIT_TIME_US);
    scl_drive_low();
    sda_pull_high();    
    _delay_us(BIT_TIME_US);
    
    return(data);            
         
}



// WriteFlag=0 leaves in read mode
// WriteFlag=1 leaves in write mode
// Returns 0 on success, 1 if no ACK bit received.
// Assumes bus idle on entry (SCL and SDA high) 
// Returns with SCL low

static unsigned char USI_TWI_Start( unsigned char addr , unsigned char readFlag) {

    // We assume that we enter in idle state since that is how all public functions leave us
    
    _delay_us(BIT_TIME_US);         // Make sure we have been in idle at least long enough to see the falling SDA

    // Data transfer is always initiated by a Bus Master device. A high to low transition on the SDA line, while
    // SCL is high, is defined to be a START condition or a repeated start condition.
       
    sda_drive_low();
    
    _delay_us(BIT_TIME_US);
    
    scl_drive_low();
        
                
    // A START condition is always followed by the (unique) 7-bit slave address (MSB first) and then w/r bit
        
    // The control word is latched internally on
    // rising SCLK edges and is eight bits in length, comprised
    // of a seven bit device address equal to 0010000b and a
    // read/write bit (write = 0 and read = 1).        
        
    uint8_t controlword = (addr << 1) | readFlag;     
    
    return USI_TWI_Write_Byte( controlword );
        
}    


// Write the bytes pointed to by buffer
// addr is the chip bus address
// assumes bus is idle on entry, Exists with bus idle
// Returns 0 on success

unsigned char USI_TWI_Write_Data(unsigned char addr, const uint8_t *buffer , uint8_t count)
{
        
    USI_TWI_Start( addr , 0 );      // TODO: check for error
    
    
    while (count--) {
        
        USI_TWI_Write_Byte( *buffer );
        
        buffer++;
        
    }
    
    // Data transfer ends with the STOP condition 
    // (rising edge of SDIO while SCLK is high). 
    
    // TODO: Is this Really needed? Can we just do repeat starts and save this code? Spec is vague if address is reset on start. 
    
    sda_drive_low();
    scl_pull_high();
    _delay_us(BIT_TIME_US);
    
    sda_pull_high();
    _delay_us(BIT_TIME_US);
        
    // End transaction with bus in idle
    
    return(0);
    
}


// Fill data buffer with bytes read from TWI
// addr is the chip bus address
// assumes bus is idle on entry, Exists with bus idle
// Returns 0 on success

unsigned char USI_TWI_Read_Data(unsigned char addr, uint8_t *buffer , uint8_t count)
{
    
    USI_TWI_Start( addr , 1 );      // TODO: check for error
    
    while (count--) {
        
        *buffer = USI_TWI_Read_Byte();
        
        buffer++;
        
    }
    
    // Data transfer ends with the STOP condition 
    // (rising edge of SDIO while SCLK is high). 
    
    // TODO: Is this Really needed? Can we just do repeat starts and save this code? Spec is vague if address is reset on start. 
    // TODO: Make stop function to save space
    
    sda_drive_low();
    scl_pull_high();
    _delay_us(BIT_TIME_US);
    
    sda_pull_high();
    _delay_us(BIT_TIME_US);
        
    // End transaction with bus in idle
    
    return(0);
    
}

// TODO: delete this placeholder

/*
unsigned char USI_TWI_Start_Transceiver_With_Data(unsigned char c, unsigned char *d, unsigned char l) {
    return(0);
} 
*/   

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be returned from the function.
---------------------------------------------------------------*/

/*
unsigned char USI_TWI_Master_Transfer( unsigned char temp )
{
  USISR = temp;                                     // Set USISR according to temp.
                                                    // Prepare clocking.
  temp  =  (0<<USISIE)|(0<<USIOIE)|                 // Interrupts disabled
           (1<<USIWM1)|(0<<USIWM0)|                 // Set USI in Two-wire mode.
           (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|     // Software clock strobe as source.
           (1<<USITC);                              // Toggle Clock Port.
  do
  {
    _delay_us( T2_TWI/4 );              
    USICR = temp;                          // Generate positve SCL edge.
    while( !(PIN_USI & (1<<PIN_USI_SCL)) );// Wait for SCL to go high.
    _delay_us( T4_TWI/4 );              
    USICR = temp;                          // Generate negative SCL edge.
  }while( !(USISR & (1<<USIOIF)) );        // Check for transfer complete.
  
  _delay_us( T2_TWI/4 );                
  temp  = USIDR;                           // Read out data.
  USIDR = 0xFF;                            // Release SDA.
  DDR_USI |= (1<<PIN_USI_SDA);             // Enable SDA as output.

  return temp;                             // Return the data from the USIDR
}


*/

/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/

/*
unsigned char USI_TWI_Master_Stop( void )
{
  PORT_USI &= ~(1<<PIN_USI_SDA);           // Pull SDA low.
  PORT_USI |= (1<<PIN_USI_SCL);            // Release SCL.
  while( !(PIN_USI & (1<<PIN_USI_SCL)) );  // Wait for SCL to go high.
  _delay_us( T4_TWI/4 );               
  PORT_USI |= (1<<PIN_USI_SDA);            // Release SDA.
  _delay_us( T2_TWI/4 );                
  
#ifdef SIGNAL_VERIFY
  if( !(USISR & (1<<USIPF)) )
  {
    USI_TWI_state.errorState = USI_TWI_MISSING_STOP_CON;    
    return (FALSE);
  }
#endif

  return (TRUE);
}
*/