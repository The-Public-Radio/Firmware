/*
 *
 * Initial firmware for Public Radio.
 *
 * Responsibilities include:
 *	- set up PB4 for PWM output, controlled by Timer1/OC1B
 *	- de-assertion of 4702 reset (drive PB1 high)
 *	- initialisation of 4072 receiver via i2c bus
 *		o muting of output
 *		o selection of correct operating mode (mono, etc)
 *		o output level selection
 *		o <other>
 *	- programming of receive frequency based on fields in EEPROM
 *	- unmute receiver
 *	- periodically poll RSSI and map onto OC1B output duty cycle
 *	- set up debounced button push for programming mode
 *	- consume as little power as possible when in normal operation
 *
 *	- enter manual programming mode via long button press (>= 2 sec)
 *	- led flashes 320ms on, 320mS off
 *	- interpret short button push as scan up command
 *	- automatically stop on next valid channel
 *	- wrap at band edges.
 *	- interpret long button press (>= 2 sec) as save memory command
 *	- led on solid as confirmation.
 *	- when in manual programming mode, interpret very long button
 *	  press (>= 4 sec) as factory reset request.
 *	- led flashes at 160mS on, 160mS off while waiting for comnfirmation.
 *	- confirmation is long button press (>= 2 sec)
 *	- led on solid as confirmation.
 *	- timeout of 10 secs of no button pressed aborts all tuning (will
 *	  stay tuned to new channel if in scan mode - but will power up
 *	  on the normal channel.)
 *
 * The cunning plan for Si4702 register access is as follows:
 *
 *   - remember that reads always start with register 0xA (status), and
 *     wrap around from register 0xf to 0x0.
 *   - writes on the other hand always start with register 0x2 (power config).
 *   - we should only ever need to write registers 0x2 thru 0x7
 *
 *   - We will *always* read all 16 registers, and store them in AVR RAM.
 *   - We will only ever write registers 2 - 7 (6 total), and we will write
 *     them from the same shadow storage in AVR RAM.
 *
 *   - To change a register, we will modify the shadow registers in place
 *     and then write back the entire bank of 6.
 *
 *   - The Tiny25 has limited resources, to preserve these, the shadow
 *     registers are not laid out from 0 in AVR memory, but rather from 0xA
 *     as follows:
 *
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 0
 *     | Register 0xA |===========|
 *     |              |  Low Byte |   <--- Byte offset: 1
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 2
 *     | Register 0xB |===========|
 *     |              |  Low Byte |   <--- Byte offset: 3
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 4
 *     | Register 0xC |===========|
 *     |              |  Low Byte |   <--- Byte offset: 5
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 6
 *     | Register 0xD |===========|
 *     |              |  Low Byte |   <--- Byte offset: 7
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 8
 *     | Register 0xE |===========|
 *     |              |  Low Byte |   <--- Byte offset: 9
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 10
 *     | Register 0xF |===========|
 *     |              |  Low Byte |   <--- Byte offset: 11
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 12
 *     | Register 0x0 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 13
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 14
 *     | Register 0x1 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 15
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 16
 *     | Register 0x2 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 17
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 18
 *     | Register 0x3 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 19
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 20
 *     | Register 0x4 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 21
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 22
 *     | Register 0x5 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 23
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 24
 *     | Register 0x6 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 25
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 26
 *     | Register 0x7 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 27
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 28
 *     | Register 0x8 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 29
 *     +--------------+-----------+
 *     |              | High Byte |   <--- Byte offset: 30
 *     | Register 0x9 |===========|
 *     |              |  Low Byte |   <--- Byte offset: 31
 *     +--------------+-----------+
 *
 *   - Because registers 2 thru 7 don't wrap in the buffer, we can use
 *     contiguous read *and* write function.
 *   - The goal here is to get the pre-processor to do as much of the work
 *     for us as possible, especially since we only ever need to access
 *     registers by name, not algorithmically.
 *   - Note that the endianness is opposite of the natural order for uint16_t
 *     on AVR, so a union overlaying 16 bit registers on the 8-bit array
 *     will not give us what we need.
 * 
 */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/crc16.h>

#define	F_CPU	1000000UL
#include <util/delay.h>



#include "USI_TWI_Master.h"
#include "VccADC.h"
#include "VccProg.h"

#define FMIC_ADDRESS        (0b0010000)                // Hardcoded for this chip, "a seven bit device address equal to 0010000"

#define LED_DRIVE_BIT       PB4
#define FMIC_RESET_BIT      PB1
#define FMIC_SCLK_BIT       PB2
#define FMIC_SDIO_BIT       PB0

#define BUTTON_INPUT_BIT    PB3
#define BUTTON_PCINT_BIT    PCINT3
#define LONG_PRESS_MS       (2000)      // Hold down button this long for a long press
#define BUTTON_DEBOUNCE_MS  (50)        // How long to debounce button edges

#define LOW_BATTERY_VOLTAGE (2.1)       // Below this, we will just blink LED and not turn on 

#define BREATH_COUNT_TIMEOUT (60)       // How many initial breaths should we take before going to sleep? Each breath currently about 2 secs.

#define SBI(port,bit) (port|=_BV(bit))
#define CBI(port,bit) (port&=~_BV(bit))
#define TBI(port,bit) (port&_BV(bit))

typedef enum {
	REGISTER_00 = 12,
	REGISTER_01 = 14,
	REGISTER_02 = 16,
	REGISTER_03 = 18,
	REGISTER_04 = 20,
	REGISTER_05 = 22,
	REGISTER_06 = 24,
	REGISTER_07 = 26,
	REGISTER_08 = 28,
	REGISTER_09 = 30,
	REGISTER_10 =  0,
	REGISTER_11 =  2,
	REGISTER_12 =  4,
	REGISTER_13 =  6,
	REGISTER_14 =  8,
	REGISTER_15 = 10,
} si4702_register;

#define EEPROM_BAND		    ((const uint8_t *)0)
#define EEPROM_DEEMPHASIS	((const uint8_t *)1)
#define EEPROM_SPACING		((const uint8_t *)2)
#define EEPROM_CHANNEL		((const uint16_t *)3)
#define EEPROM_VOLUME		((const uint8_t *)5)
#define	EEPROM_CRC16		((const uint16_t *)14)

#define EEPROM_PARAM_SIZE	(16)

#define	EEPROM_WORKING		((const uint8_t *)0)
#define EEPROM_FACTORY		((const uint8_t *)16)

const uint8_t last_resort_param[16] PROGMEM = {
	0x00,
	0x00,
	0x00,
	0x09, 0x00,
	0x0f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x6f, 0x6c,
} ;



/*
 * Setup Timer 1 to drive the LED using PWM on the OC1B pin (PB4)
 */


static void LED_PWM_init(void)
{
    
    /*    
        LSM: Low Speed Mode
        The high speed mode is enabled as default and the fast peripheral clock is 64 MHz, but the low speed mode can
        be set by writing the LSM bit to one. 
        Then the fast peripheral clock is scaled down to 32 MHz. The low speed mode
        must be set, if the supply voltage is below 2.7 volts, because the Timer/Counter1 is not running fast enough on low
        voltage levels. It is highly recommended that Timer/Counter1 is stopped whenever the LSM bit is changed.    
        
    */
    
	PLLCSR = _BV( LSM );            // Low speed mode. 
    
    // We are going to run PWM mode
    
    
	OCR1C = 255;
	OCR1B = 0;
    
}


static inline void LED_PWM_on(void) {

    /*
        PWM1B: Pulse Width Modulator B Enable
        When set (one) this bit enables PWM mode based on comparator OCR1B in Timer/Counter1 and the counter
        value is reset to $00 in the CPU clock cycle after a compare match with OCR1C register value.    
      
        Bits 5:4 – COM1B[1:0]: Comparator B Output Mode, Bits 1 and 0
        The COM1B1 and COM1B0 control bits determine any output pin action following a compare match with compare
        register B in Timer/Counter1. Since the output pin action is an alternative function to an I/O port, the corresponding
        direction control bit must be set (one) in order to control an output pin.
        In Normal mode, the COM1B1 and COM1B0 control bits determine the output pin actions that affect pin PB4
        (OC1B) as described in Table 12-6. Note that OC1B is not connected in normal mode.        
        
        COM1B1=1 0 Clear the OC1B output line
        
    */
    
	TCCR1 = 0x84;
    GTCCR = 0x60;    // PWM1B COM1B1 
       

}    


// Turn off the timer to save power when we sleep. Also turns off the LED becuase it is disconnected from the timer so controlled by PORT. 

static inline void LED_PWM_off(void) {
    
    TCCR1 = 0;              // Stop counter
    GTCCR = 0x00;           // Disconnect OCR2B
        
}    

static inline void setLEDBrightness( uint8_t b) {    
    OCR1B = b;
}    

// Breath data thanks to Lady Ada

const PROGMEM uint8_t breath_data[] =  {
        
 1, 1, 2, 3, 5, 8, 11, 15, 20, 25, 30, 36, 43, 49, 56, 64, 72, 80, 88, 97, 105, 114, 123, 132, 141, 150, 158, 167, 175, 183, 191, 199, 206, 212, 219, 225, 230, 235, 240, 244, 247, 250, 252, 253, 254, 255, 254, 253, 252, 250, 247, 244, 240, 235, 230, 225, 219, 212, 206, 199, 191, 183, 175, 167, 158, 150, 141, 132, 123, 114, 105, 97, 88, 80, 72, 64, 56, 49, 43, 36, 30, 25, 20, 15, 11, 8, 5, 3, 2, 1, 0
    
};


#define BREATH_LEN (sizeof( breath_data) / sizeof( *breath_data ))


uint8_t breath(uint8_t step) {

  uint8_t brightness = pgm_read_byte_near( breath_data + step);
  return brightness;
  
}


uint8_t shadow[32];

static inline uint16_t get_shadow_reg(si4702_register reg)
{
	return (shadow[reg] << 8) | (shadow[reg + 1]);
}

static inline void set_shadow_reg(si4702_register reg, uint16_t value)
{
	shadow[reg] = value >> 8;
	shadow[reg + 1] = value & 0xff;
}

// Read all the registers. The FM_IC starts reads at register 0x0a and then wraps around

void si4702_read_registers(void)
{
	USI_TWI_Start_Transceiver_With_Data(0x21, shadow, 32);
}

/*
 * Just write registers 2 thru 7 inclusive from the shadow array.
 */

void si4702_write_registers(void)
{
	//USI_TWI_Start_Transceiver_With_Data(0x20, &(shadow[REGISTER_02]), 12);
        
    // Only registers 0x02 - 0x07 are relevant for config, and each register is 2 bytes wide
    
    // Even though the datasheet says that the reserved bits of 0x07 must be Read before writing, 
    // The previous version of this software blindly wrote 0's and that seems to work fine. 
    
    
    USI_TWI_Start_Transceiver_With_Data(0x20, &(shadow[REGISTER_02]), (0x08 - 0x02) * 2);    
    //USI_TWI_Write_Data( FMIC_ADDRESS ,  &(shadow[REGISTER_02]) , (0x08 - 0x02) * 2 );
}

/*
 * tune_direct() -	Directly tune to the specified channel.
 */
void tune_direct(uint16_t chan)
{
	set_shadow_reg(REGISTER_03, 0x8000 | (chan & 0x01ff));

	si4702_write_registers();

	_delay_ms(160);

	set_shadow_reg(REGISTER_03, get_shadow_reg(REGISTER_03) & ~0x8000);
	si4702_write_registers();
}

uint16_t currentChanFromShadow(void) {
    return( get_shadow_reg(REGISTER_03 & 0x1ff));
}    

/*
 * update_channel() -	Update the channel stored in the working params.
 *			Just change the 2 bytes @ EEPROM_CHANNEL,
 *			recalculate the CRC, and then write that.
 */
void update_channel(uint16_t channel)
{
	uint16_t crc = 0x0000;
	const uint8_t *src;

	eeprom_write_word((uint16_t *)EEPROM_CHANNEL, channel);

	/*
	 * Spin through the working params, up to but not including the CRC
	 * calculating the new CRC and write that.
	 */
	for (src = EEPROM_WORKING; src < (const uint8_t *)EEPROM_CRC16; src++) {
		crc = _crc16_update(crc, eeprom_read_byte(src));
	}

	eeprom_write_word((uint16_t *)EEPROM_CRC16, crc);
    
}



/*
 * Set initial factory config. This becomes both the the active config and the config people will get when they do a factory reset. 
 */

void update_facotry_config(uint16_t channel , uint8_t band, uint8_t deemphassis , uint8_t spacing )
{
	uint16_t crc = 0x0000;
	

	eeprom_write_word((uint16_t *)EEPROM_CHANNEL,    channel);
	eeprom_write_byte((uint8_t *)EEPROM_BAND,       band);
	eeprom_write_byte((uint8_t *)EEPROM_DEEMPHASIS, deemphassis);
	eeprom_write_byte((uint8_t *)EEPROM_SPACING,    spacing);

	/*
	 * Spin through the working params, up to but not including the CRC
	 * calculating the new CRC and write that.
	 */
	for (const uint8_t *src = EEPROM_WORKING; src < (const uint8_t *)EEPROM_CRC16; src++) {
		crc = _crc16_update(crc, eeprom_read_byte(src));
	}

	eeprom_write_word((uint16_t *)EEPROM_CRC16, crc);
    
    /*
    // Now copy to factory default EEPROM
    
    const uint8_t *src= EEPROM_WORKING;
	uint8_t *dest = (uint8_t *)(EEPROM_FACTORY);
	uint8_t i;

	for (i = 0; i < sizeof(last_resort_param); i++, src++, dest++) {
		eeprom_write_byte(dest, eeprom_read_byte(src));
	}
    
    */
    
}


/*
 * check_param_crc() -	Check EEPROM_PARAM_SIZE bytes starting at *base,
 *			and return whether the crc (last 2 bytes) is correct
 *			or not.
 *			Return 0 if crc is good, !0 otherwise.
 */
static uint16_t check_param_crc(const uint8_t *base)
{
	uint16_t crc = 0x0000;
	uint8_t i;

	for (i = 0; i < EEPROM_PARAM_SIZE; i++, base++) {
		crc = _crc16_update(crc, eeprom_read_byte(base));
	}

	/*
	 * If CRC (last 2 bytes checked) is correct, crc will be 0x0000.
	 */
	return crc ;
}

/*
 * init_factory_param() -	Re-init the factory default area, using the
 *				16 bytes from last_resort_param[]. This really
 *				is, as the name suggests, the last resort.
 */
static void init_factory_param(void)
{
	uint8_t *dest = (uint8_t *)(EEPROM_FACTORY);
	uint8_t i;

	for (i = 0; i < sizeof(last_resort_param); i++, dest++) {
		eeprom_write_byte(dest, pgm_read_byte(&(last_resort_param[i])));
	}
}

/*
 * copy_factory_param() -	Copy the factory default parameters into the
 *				working param area  simple bulk copy of the
 *				entire 16 bytes. No check of the crc.
 */
static void  copy_factory_param(void)
{
	const uint8_t *src = EEPROM_FACTORY;
	uint8_t *dest = (uint8_t *)(EEPROM_WORKING);
	uint8_t i;

	for (i = 0; i < sizeof(last_resort_param); i++, src++, dest++) {
		eeprom_write_byte(dest, eeprom_read_byte(src));
	}
}

/*
 * check_eeprom() -	Check the CRC for the operational parameters in eeprom.
 *			If the CRC fails, check the factory parameters CRC.
 *			If that is good, copy into the operational area, if not
 *			good, then just pick some suitable defaults and write
 *			the same to both factory and operational areas.
 *
 *			The idea being that by the time this function returns,
 *			the eeprom has valid data in it that passes CRC.
 */

void check_eeprom(void)
{
	if (check_param_crc(EEPROM_WORKING)) {
		if (check_param_crc(EEPROM_FACTORY)) {
			init_factory_param();
		}
		copy_factory_param();
	}
}


void debugBlink( uint8_t b ) {
    
         
    for(int p=0; p<b; p++ ) {   
        SBI( PORTB , LED_DRIVE_BIT);
        _delay_ms(10);
        CBI( PORTB , LED_DRIVE_BIT);
        _delay_ms(200);            
    }
         
    _delay_ms(1000);            
            
}    

void debugshortblink(void) {
    SBI( PORTB , LED_DRIVE_BIT);
    _delay_ms(50);
    CBI( PORTB , LED_DRIVE_BIT);
    _delay_ms(50);                
}    


void binaryDebugBlink( uint16_t b ) {
    
    while (1) {
         
         for(uint16_t bitmask=0x8000; bitmask !=0; bitmask>>=1) {

            SBI( PORTB , LED_DRIVE_BIT);
            _delay_ms(200);            
             
             if (b & bitmask) {
                 _delay_ms(200);            
             }            
            CBI( PORTB , LED_DRIVE_BIT);
            
            _delay_ms(400);
            
         }
         
         _delay_ms(1000);            
            
    }        
}    

void si4702_init(void)
{
	/*
	 * Init the Si4702 as follows:
	 *
	 * Set up USI in I2C (TWI) mode
     * Enable pull-ups on TWI lines
	 * Enable the oscillator (by writing to TEST1 & other registers)
	 * Wait for oscillator to start
	 * Enable the IC, set the config, tune to channel, then unmute output.
	 */


    // Let's do the magic dance to awaken the FM_IC in 2-wire mode
    // We enter with RESET low (active)
        
    // Busmode selection method 1 requires the use of the
    // GPIO3, SEN, and SDIO pins. To use this busmode
    // selection method, the GPIO3 and SDIO pins must be
    // sampled low by the device on the rising edge of RST.
    // 
    // The user may either drive the GPIO3 pin low externally,
    // or leave the pin floating. If the pin is not driven by the
    // user, it will be pulled low by an internal 1 M? resistor
    // which is active only while RST is low. The user must
    // drive the SEN and SDIO pins externally to the proper
    // state.


    // Drive SDIO low (GPIO3 will be pulled low internally by the FM_IC) 
                
    SBI( PORTB , FMIC_RESET_BIT );     // Bring FMIC and AMP out of reset
                                       // The direct bit was set to output when we first started in main()
        
    _delay_ms(1);                      // When selecting 2-wire Mode, the user must ensure that a 2-wire start condition (falling edge of SDIO while SCLK is
                                       // high) does not occur within 300 ns before the rising edge of RST.
   
                                           
    // Enable the pull-ups on the TWI lines

	USI_TWI_Master_Initialise();

	
    // Reg 0x07 bit 15 - Crystal Oscillator Enable.
    // 0 = Disable (default).
    // 1 = Enable.
    
    // Reg 0x07 bits 13:0 - Reserved. If written, these bits should be read first and then written with their pre-existing values. Do not write during powerup.
    // BUT Datasheet also says Bits 13:0 of register 07h must be preserved as 0x0100 while in powerdown and as 0x3C04 while in powerup.
   // si4702_read_registers();  

	set_shadow_reg(REGISTER_07, 0x8100);

	si4702_write_registers();

    /*

        Wait for crystal to power up (required for crystal oscillator operation).
        Provide a sufficient delay (minimum 500 ms) for the oscillator to stabilize. See 2.1.1. "Hardware Initialization”
        step 5.    
    
    */

	_delay_ms(600);

	/*
	 * Register 02 default settings:
	 *	- Softmute enable
	 *	- Mute enable
	 *	- Mono
	 *	- Wrap on band edges during seek
	 *	- Seek up
	 */
        
	set_shadow_reg(REGISTER_02, 0xE201);

	si4702_write_registers();
    
    /*    
        Software should wait for the powerup
        time (as specified by Table 8, “FM Receiver
        Characteristics1,2,” on page 12) before continuing with
        normal part operation.  
        
        Powerup Time From powerdown
        (Write ENABLE bit to 1)
        max 110 ms        
          
    */    

	_delay_ms(110);

	/*
	 * Set deemphasis based on eeprom.
	 */
    
	set_shadow_reg(REGISTER_04, get_shadow_reg(REGISTER_04) | (eeprom_read_byte(EEPROM_DEEMPHASIS) ? 0x0800 : 0x0000));

	set_shadow_reg(REGISTER_05,
			(((uint16_t)(eeprom_read_byte(EEPROM_BAND) & 0x03)) << 6) |
			(((uint16_t)(eeprom_read_byte(EEPROM_SPACING) & 0x03)) << 4));


	/*
	 * It looks like the radio tunes to <something> once enabled.
	 * Make sure the STC bit is cleared by clearing the TUNE bit.
     *
     * JML - The clearing of TUNE bit does not seem to be nessisary, but the read_regs() is. Go figure, something must change somewhere.
     * TODO: Find out what is changed on this read and hardcode it so we can dump the whole read function.
	 */
    
	si4702_read_registers();
	set_shadow_reg(REGISTER_03, 0x0000);
	si4702_write_registers();

	tune_direct(eeprom_read_word(EEPROM_CHANNEL));
    //set_shadow_reg(REGISTER_03, 0x8050 );                   //for testing, frequency = 103.5 MHz = 0.200 MHz x 80 + 87.5 MHz). Write data 8050h.


    // Pump up the volume

	set_shadow_reg(REGISTER_05, (get_shadow_reg(REGISTER_05) & ~0x000f) |
				(eeprom_read_byte(EEPROM_VOLUME) & 0x0f));

	si4702_write_registers();
    
    /*
    
    Can you here the de-emphasis difference? I don't think so. 
    
    while (1) {

	    set_shadow_reg(REGISTER_04, get_shadow_reg(REGISTER_04) | 0x0800 );
        SBI( PORTB , LED_DRIVE_BIT );
        si4702_write_registers();
        _delay_ms(1000);
	    set_shadow_reg(REGISTER_04, get_shadow_reg(REGISTER_04) & ~0x0800 );
        CBI( PORTB , LED_DRIVE_BIT );
        si4702_write_registers();        
        _delay_ms(1000);
        
           
    } 
    
    */       
}


void debug_slowblink(void) {
    while (1) {
            
        SBI( PORTB , LED_DRIVE_BIT);
        _delay_ms(100);
        CBI( PORTB , LED_DRIVE_BIT);
        _delay_ms(900);
            
    }    
}    

void debug_fastblink(void) {
    while (1) {
            
        SBI( PORTB , LED_DRIVE_BIT);
        _delay_ms(100);
        CBI( PORTB , LED_DRIVE_BIT);
        _delay_ms(100);
            
    }    
}    


// Goto bed, will only wake up on button press interrupt (if enabled)

void deepSleep(void) {    
	set_sleep_mode( SLEEP_MODE_PWR_DOWN );
    sleep_enable();
    sleep_cpu();        // Good night    
}   


// Returns true if button is down

static inline uint8_t buttonDown(void) {
    return( !TBI( PINB , BUTTON_INPUT_BIT ));           // Button is pulled high, short to ground when pressed
}    

// Setup pin change interrupt on button 
// returns true of the button was down on entry

uint8_t initButton(void) 
{
 
     SBI( PORTB , BUTTON_INPUT_BIT);              // Enable pull up on button 
     
     uint8_t ret = buttonDown();
    
    /*
        PCIE: Pin Change Interrupt Enable
        When the PCIE bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt is
        enabled. Any change on any enabled PCINT[5:0] pin will cause an interrupt. The corresponding interrupt of Pin
        Change Interrupt Request is executed from the PCI Interrupt Vector.         
    */
    
    SBI( GIMSK , PCIE );   // turns on pin change interrupts
    
    /*
        PCINT[5:0]: Pin Change Enable Mask 5:0
        Each PCINT[5:0] bit selects whether pin change interrupt is enabled on the corresponding I/O pin. If PCINT[5:0] is
        set and the PCIE bit in GIMSK is set, pin change interrupt is enabled on the corresponding I/O pin. If PCINT[5:0] is
        cleared, pin change interrupt on the corresponding I/O pin is disabled.    
    */
    
    SBI( PCMSK , BUTTON_PCINT_BIT );
    
    sei();                 // enables interrupts
    
    return ret;
}    




// Called on button press pin change interrupt

ISR( PCINT0_vect )
{

    // Do nothing in ISR, just here so we can catch the interrupt and wake form deep sleep
    // ISRs are ugly syemantics with voltaile access and stuff, simple to handle in main thread. 
    
   return;
   
}

// Assumes button is actually down and LED_Timer is on
// Always waits for the debounced up before returning


void handleButtonDown(void) {
    
    setLEDBrightness(0);            // Led off when button goes down. Gives feedback if we are currently breathing otherwise benign
    
    _delay_ms( BUTTON_DEBOUNCE_MS );        // Debounce down
        
    uint16_t currentChan = currentChanFromShadow();

    uint16_t countdown = LONG_PRESS_MS;
    
    while (countdown && buttonDown()) {       // Spin until either long press timeout or they let go
        _delay_ms(1);          
        countdown--;
    }        
    
    if (countdown) {                            // Did not timeout, so short press
        
        // Advance to next station
            
        currentChan++;
            
        // Input Frequency fRF 76 — 108 MHz
            
        if (currentChan > (1080-760) ) {        // Wrap at top of band back down to bottom
                
            currentChan = 0; 
                
        }                
        
        tune_direct(  currentChan );
            
        // TODO: test this wrap (lots of button presses, so start high!)
        
        
    } else {            // Initiated a long-press save to EEPROM            


        // User feedback of long press with 200ms flash on LED
        
        setLEDBrightness(255);
        
            
        update_channel( currentChan );            
        _delay_ms(500);

        setLEDBrightness(0);            
           
    }    
        
    while (buttonDown());                   // Wait for button release
        
    _delay_ms( BUTTON_DEBOUNCE_MS );        // Debounce up
        
}



// Attempt to read a programming packet, act on any valid ones received

int readProgrammingPacket(void) {
        
    uint16_t crc = 0x0000;
            
    int d;
           
    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc = _crc16_update(crc, d );    
    
    uint16_t channel = d << 8;                 // Byte 1: channel high byte
    
    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc = _crc16_update(crc, d );    
    
    channel |= d;                              // Byte 2: channel low byte    


/*

    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc = _crc16_update(crc, d );    
    
    uint8_t deemphasis = d;

        
    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc = _crc16_update(crc, d );    
    
    uint8_t band = d;
    
    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc = _crc16_update(crc, d );    
    
    uint8_t spacing = d;

*/
    
    // Calculate the crc check
  
    uint16_t crc_rec = 0x0000;              // crc received in packet
    
           
    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc_rec = d << 8;                       // Byte 3: received crc high byte

    d = readPbyte();    
    if (d<0) return(d); // Check for error    
    crc_rec |= d;                           // Byte 4: received crc low byte
    
    if ( crc != crc_rec ) {                 // Compared received crc to calculated crc
        
        debugBlink(1);                      // Three short blinks = bad crc
        return(-1); 
        
    }        
    
    
    // TODO: Is this necessary?
    
    _delay_ms(50);      // Let capacitor charge up a bit
    
    //update_channel( channel );    
    //update_facotry_config(channel , band, deemphasis , spacing );
    
    update_facotry_config(channel , 0, 0 , 0 );
                
    debugBlink(2);          // two short blinks = programming accepted
    
    return(0); 
                    
}    


int main(void)
{
    
    // Set up the reset line to the FM_IC and AMP first so they are quiet. 
    // This eliminates the need for the external pull-down on this line. 
    
    
    SBI( DDRB , FMIC_RESET_BIT);    // drive reset low, makes them sleep            
        
    // TODO: Enable DDR on LED pin but for now just use pull-up
	SBI( DDRB , LED_DRIVE_BIT);    // Set LED pin to output, will default to low (LED off) on startup
            
    adc_on();
    
    if (!VCC_GT(LOW_BATTERY_VOLTAGE)) {
        
        adc_off();  // Mind as well save some power 
        
        // indicate dead battery with a 10%, 1Hz blink
        
        // TODO: Probably only need to blink for a minute and then go into deep sleep?
        
        // NOTE that here we are dring the LED directly full on rather than PWM
        // Since the battery is already low, this will give us max brightness.
         
         
        for( uint8_t i=120; i>0; i--) {             // Blink for 2 minutes (120 seconds)
            
            SBI( PORTB , LED_DRIVE_BIT);
            _delay_ms(100);
            CBI( PORTB , LED_DRIVE_BIT);
            _delay_ms(900);
            
            // Each cycle 100ms+900ms= ~1 second
            
        }
        
        // We are not driving any pins except the RESET to keep the FM_IC and AMP asleep
        // and the LED which is low, so no Wasted power
        
        // Note that we never turned on the timer so this should be a very low power sleep. 
        
        deepSleep();
        // Never get here since button int not enabled.
        
        // Confirmed power usage in low battery mode @ 2.1V:
        // 0.60mA while blinking
        // 0.06mA while sleeping - probably most due to amp and FMIC?
        
        // TODO: Should probably periodically wake up durring normal playing and check for low battery
        // so someone leaving it on does not run into the ground. 
        
        
    }   
    
    
    if (programmingVoltagePresent()) {          
        
        // Ok, we are currently being powered by a programmer since a battery could not make the voltage go this high
        
        // The VccProg stuff depends on the ADC,which is already on here. 
        
        // DDRB |= _BV(0);     // Show the search window for testing
        
        while (readProgrammingPacket());      // wait for a darn good programming packet
                    
    }     
    
    
    adc_off();      /// All done with the ADC, so same a bit of power      
    

    // Normal operation from here (good battery voltage, not connected to a programmer)
    
           
	check_eeprom();
    
    
    LED_PWM_init();          // Make it so we can pwm the LED for now on...
    
    LED_PWM_on(); 
            
    if (initButton()) {             // Was button down on startup?
                
        // TODO: How should this work?
        
        // I think only boot with factory params, but do not save - what if they are worse or button held accedentally?
        // Better to wait for release, and then a 2nd long press to save after you hear that it works. 
        // Definitely harder to implement because we need a working EEPROM image. 
        
        while (buttonDown()) {
            
            // Double blink while boot button down to indicate writing factory config
            
            setLEDBrightness(255);
            _delay_ms(100);
            setLEDBrightness(0);
            _delay_ms(100);
            setLEDBrightness(255);
            _delay_ms(100);
            setLEDBrightness(0);
            _delay_ms(900);
            
            // TODO: Test this and maybe fix this for better behaivor. 
            
        }            
        
        copy_factory_param();       // Revert to initial config
                        
    }   
    
	si4702_init();
    // Radio is now on and tuned

             
    // Breathe for a while so user knows we are alive in case not tuned to a good station or volume too low

    uint8_t countdown_s = BREATH_COUNT_TIMEOUT;
    
    while (countdown_s) {
        
        for(uint8_t cycle=0; cycle<BREATH_LEN; cycle++ ) { 
        
            setLEDBrightness( breath(cycle)  );
        
            _delay_ms(20);
            
            if (buttonDown()) {
             
                handleButtonDown(); 
                
                cycle=0;        // Start cycle over since it looks mucho más profesionales
                   
            }                
            
        }        
        
        countdown_s--;
        
    }        
    
        
    LED_PWM_off();       // Save power while sleeping (we don't need LED anymore unless we wake from button press)   
    
    while (1) {
        
        // Any button state change will wake us up
        
        deepSleep();
        
        if (buttonDown()) {         // We only care about presses, not lifts
            
            LED_PWM_on();           // We are gonna want the PWM here when we loose the current limiting resistor. 
            
            handleButtonDown();     // Always wait for the debounced up before returning
            
            LED_PWM_off();
            
        }            
        
    }        
    
    
    // Never get here...
    
    
}
