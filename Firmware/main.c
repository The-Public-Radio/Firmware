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

#define BREATH_COUNT_TIMEOUT_S (60)       // How many initial breaths should we take before going to sleep? Each breath currently about 2 secs.

// These are the error display codes
// When we run into problems, we blink the LED this many times to show the user
// Note that 1 blink is skipped intentionally for UX reasons. Is a single blink a single blink, or an infinite number of blinks? Think about it!

#define DIAGNOSTIC_BLINK_BADEEPROM     3            // EEPROM user settings failed CRC check on startup
#define DIAGNOSTIC_BLINK_LOWBATTERY    2            // Battery too low for operation. 
#define DIAGNOSTIC_BLINK_NONE          0

#define  DIAGNOSTIC_BLINK_TIMEOUT_S   120           // Show diagnostic blink at least this long before going to sleep
                                                    // Give user time to see it, but don't go too long because we will
                                                    // make crusty batteries. Must fit in unit8_t.

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
#define	EEPROM_CRC16		((const uint16_t *)14)      // Each block has an independent CRC-16

#define EEPROM_PARAM_BLOCK_SIZE	(16)

// Starting address of parameter blocks in EEPROM. Can't overlap and must match with other tools that make EEPROM images

#define	EEPROM_WORKING		((const uint8_t *) 0)
#define EEPROM_FACTORY		((const uint8_t *)16)


/*
 * Setup Timer 1 to drive the LED using PWM on the OC1B pin (PB4)
 */

// Assumes timer registers are still at reset-default values.

static inline void LED_PWM_init(void)
{
    
    // We are using simple PWM mode to drive OCR1B which is connected to the LED.
    // We are driving off the system clock, not the PLL
    // We are using non-inverting mode where OCR is set at 0 and cleared at match, so lower matches are less brightness
    // Note that if OCR is 0, then there i not pulse geenrated so LED stays off
    
    
    // We are going to run PWM mode
    
    
	OCR1C = 255;                //  In PWM mode, the Timer counter counts up to the value specified in the output compare register OCR1C and starts again from $00. 
    
    // OCR1B=0 here because it is the default startup value
    
    GTCCR =     
        _BV( PWM1B  ) |          // Enable PWM module B
        _BV( COM1B1 )            // OC1x cleared on compare match. Set when TCNT1 = $00.
    ;
    
    
	//TCCR1 = _BV( CS12 );        // Enable timer, PCK/8           
            
}


// Enter with OCR1B already set to desired duty cycle

static inline void LED_PWM_on(void) {

    
    // We could have saved a write here if the LED was attached to OCR1A instead of B since we could have
    // disabled the output pin and stopped the timer in a single register
            
    TCNT1 =254;                 // Next count will overflow and start a proper cycle based on OCR1B
                                // If we do not do this, then the output gets set high when we enable the PWM! Undocumented!
    
    GTCCR =     
        _BV( PWM1B  ) |          // Enable PWM module B
        _BV( COM1B1 ) |          // OC1x cleared on compare match. Set when TCNT1 = $00.
        _BV( FOC1B )  |          // Force compare match. Because of the output mode bits, this will clear the output.    
        _BV( PSR1 )              //  Prescaler Reset Timer/Counter1 - start with a clean slate on the precsaller too
    ;
    
    // 1Mhz clock / 256 step cycle / 8 prescaller = 500hkz pwm rate
    // SHould be fast enough for driving LED without current limiting resistor
    
      TCCR1 = _BV( CS12 );        // Enable timer, PCK/8           
            
}    


// Turn off the timer to save power when we sleep. 
// Assumes the brightness has already been set to zero. 

static inline void LED_PWM_off(void) {
    
    /*

        When OCR1A or OCR1B contain $00 or the top value, as specified in OCR1C register, the output PB1(OC1A) or
        PB4(OC1B) is held low or high according to the settings of COM1A1/COM1A0. This is shown in Table 12-2.
        In PWM mode, the Timer Overflow Flag - TOV1 is set when the TCNT1 counts to the OCR1C value and the
        TCNT1 is reset to $00. The Timer Overflow Interrupt1 is executed when TOV1 is set provided that Timer Overflow
        Interrupt and global interrupts are enabled. This also applies to the Timer Output Compare flags and interrupts.
        The frequency of the PWM will be Timer Clock 1 Frequency divided by (OCR1C value + 1). See the following
        equation:
        Resolution shows how many bits are required to express the value in the OCR1C register and can be calculated
        using the following equation:
        Table 12-2. PWM Outputs OCR1x = $00 or OCR1C, x = A or B
        COM1x1 COM1x0 OCR1x Output OC1x Output OC1x
        0 1 $00 L H    
    
    */
    
    // So, if the OCR is set to 0, then the LED should be off no matter what...    

    // Disable the PWM, disconnect the pin (will be driven by PORT which should be low)

    GTCCR = 0;                  // Disconnect output pin. 
    TCCR1 = 0;                  // Stop the clock. Save some power.
                
    // Note that we don't bother to disable the clock when LED is off because it will stop in sleep shutdown mode anyway
        
}  

static uint8_t currentLEDBrightness=0;

static void updateLEDcompensation(void) {

    if (currentLEDBrightness>0) {
        OCR1B = currentLEDBrightness;
        // TODO: Do voltage adjust        
    }    
}    


#define MIN(a,b) (((a)<(b))?(a):(b))

// 0=off, 255-brightest. Normalized for voltage.   

static inline void setLEDBrightness( uint8_t newBrightness ) {   
     
    OCR1B = MIN((newBrightness)/16,8);            //TODO:this is a hack, fix the lookup table to have correct values
    
    if ( newBrightness ) {              // faster to always blindly enable rather than test previous value
        LED_PWM_on();                   // Also forces the new brightness to take effect immediately - although not visible to human eye
    } else  {
        LED_PWM_off();                  // Turn off timer when LED off to save power
    }
    
    currentLEDBrightness = newBrightness;
                
}    

// Breath data thanks to Lady Ada

// MUST end with 0 brightness or else LED will stay on during sleep and kill battery.
// If you want to change to a pattern that does not end at 0, then add a line to manually set brightness to zero 
// when existing breathe mode.



const PROGMEM uint8_t breath_data[] =  {
        
 0, 1, 1, 2, 3, 5, 8, 11, 15, 20, 25, 30, 36, 43, 49, 56, 64, 72, 80, 88, 97, 105, 114, 123, 132, 141, 150, 158, 167, 175
    
};


#define BREATH_CYCLE_LEN ((sizeof( breath_data) / sizeof( *breath_data )) * 2)      // *2 because we walk the values up and then back down


static uint8_t breath(uint8_t step) {
    
  uint8_t lookup;         // Map  steps to 1/2 as many lookups (pattern is symmetric around center)
  
  if (step<(BREATH_CYCLE_LEN/2)) {
      
      lookup = step;
      
   } else {
       
       lookup = (BREATH_CYCLE_LEN-1) -  step ;
      
  }      

  uint8_t brightness = pgm_read_byte_near( breath_data + lookup );
  return brightness;
  
}

static uint8_t shadow[32];

static uint16_t get_shadow_reg(si4702_register reg)
{
	return (shadow[reg] << 8) | (shadow[reg + 1]);
}

// Making this a macro saves 20 bytes of precious flash

#define set_shadow_reg(reg,value) shadow[reg] = (value) >> 8; shadow[reg + 1] = (value) & 0xff

// Real code for above macro for clarity (but unused here to save space)

static inline void set_shadow_reg_non_macro(si4702_register reg, uint16_t value)
{
	shadow[reg] = value >> 8;
	shadow[reg + 1] = value & 0xff;
}

// Read all the registers. The FM_IC starts reads at register 0x0a and then wraps around

static void si4702_read_registers(void)
{
    USI_TWI_Read_Data( FMIC_ADDRESS , shadow , 0x10 * 2 );      // Total of 16 registers, each 2 bytes
}

/*
 * Just write registers 2 thru 7 inclusive from the shadow array.
 */

// Writes registers starting at 0x02 up to and including the specified upto_reg (max 0x09) 
// No reason to overwrite registers that we have not changed - especially 0x07 which has conflicting
// documentation about what to write there after powerup. Better to leave it be!

static void si4702_write_registers(unsigned upto_reg)
{
	//USI_TWI_Start_Transceiver_With_Data(0x20, &(shadow[REGISTER_02]), 12);
        
    // Only registers 0x02 - 0x07 are relevant for config, and each register is 2 bytes wide
    
    USI_TWI_Write_Data( FMIC_ADDRESS ,  &(shadow[REGISTER_02]) , ( upto_reg - REGISTER_02 + 2) );
}

// Blink a byte out pin 2 (normally button)

void debugBlinkByte( uint8_t data ) {

    CBI( PORTB , PB3 );    // TODO: TESTING
    SBI( DDRB , PB3 );// TODO: TESTING
    
    _delay_us(500);

    for( uint8_t bitMask=0b10000000; bitMask !=0; bitMask>>=1 ) {

        SBI( PORTB , PB3 );    // TODO: TESTING
        _delay_us(50);
        CBI( PORTB , PB3 );     // TODO: TESTING
        _delay_us(50);
        
        // setup data bit
                        
        if ( data & bitMask) {
            SBI( PORTB , PB3 );    // TODO: TESTING
        } else {
            CBI( PORTB , PB3 );     // TODO: TESTING
            
        }     
        
        _delay_us(900);           
        
   }
    CBI( DDRB , PB3 );// TODO: TESTING
    SBI( PORTB , PB3 );// TODO: TESTING
    
}    


/*
 * tune_direct() -	Directly tune to the specified channel.
 * Assumes chan < 0x01ff
 */
static void tune_direct(uint16_t chan)
{

	set_shadow_reg(REGISTER_03, 0x8000 | chan );

	si4702_write_registers( REGISTER_03 );
	_delay_ms(160);

    /*
        The tune operation begins when the TUNE bit is set high. The STC bit is set high
        when the tune operation completes. The STC bit must be set low by setting the TUNE
        bit low before the next tune or seek may begin.
    */

    // Clear the tune bit here so chip will be ready to tune again if user presses the button.
	set_shadow_reg(REGISTER_03,  chan );
	si4702_write_registers( REGISTER_03 );
}

static uint16_t currentChanFromShadow(void) {
    return( get_shadow_reg(REGISTER_03 & 0x1ff));
}    

/*
 * update_channel() -	Update the channel stored in the working params.
 *			Just change the 2 bytes @ EEPROM_CHANNEL,
 *			recalculate the CRC, and then write that.
 */
static void update_channel(uint16_t channel)
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
 * check_param_crc() -	Check EEPROM_PARAM_SIZE bytes starting at *base,
 *			and return whether the crc (last 2 bytes) is correct
 *			or not.
 *			Return 0 if crc is good, !0 otherwise.
 */

static uint16_t check_param_crc(const uint8_t *base)
{
	uint16_t crc = 0x0000;
	uint8_t i;

	for (i = 0; i < EEPROM_PARAM_BLOCK_SIZE; i++, base++) {
		crc = _crc16_update(crc, eeprom_read_byte(base));
	}

	/*
	 * If CRC (last 2 bytes checked) is correct, crc will be 0x0000.
	 */
	return crc ;
}

/*
 * copy_factory_param() -	Copy the factory default parameters into the
 *				working param area  simple bulk copy of the
 *				entire 16 bytes. 
 */

static void  copy_factory_param(void)
{
	const uint8_t *src = EEPROM_FACTORY;
	uint8_t *dest = (uint8_t *)(EEPROM_WORKING);
	uint8_t i;

	for (i = 0; i < EEPROM_PARAM_BLOCK_SIZE; i++, src++, dest++) {
		eeprom_write_byte(dest, eeprom_read_byte(src));
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



// Goto bed, will only wake up on button press interrupt (if enabled) or WDT

static void deepSleep(void) {    
	set_sleep_mode( SLEEP_MODE_PWR_DOWN );
    sleep_enable();
    sleep_cpu();        // Good night    
}   


// Show the user something went wrong and we are shutting down.
// Blink the LED _count_ times every second.
// Continue doing this for at least DIAGNOSTIC_BLINK_SECONDS
// then deep sleep forever.
// Assumes interrupts off.
// Assumes amp and FMIC in low power RESET
//
// Adjusts LED brightness based on Vcc voltage
// Assumes Timer is running to PWM the LED

static void shutDown(uint8_t blinkCount) {
    
        // SHow blink pattern fora while...
    
        for (uint8_t countdown=DIAGNOSTIC_BLINK_TIMEOUT_S; countdown>0; countdown--) {
            for(uint8_t p=blinkCount; p>0; p-- ) {   
                setLEDBrightness(255);
                _delay_ms(20);                  // TODO: What looks best here?
                setLEDBrightness(0);
                _delay_ms(200);            
            }
        
            _delay_ms(1000);    // 1 second pause between blink patterns
            
        }        
        
        // Currently only called form places where FMIC and AMP are already in reset    
             
        // TODO: Shutdown the amp and FM_IC chips by holding them in reset
        // TODO: Add a MOSFET so we can completely shut them off (they still pull about 25uA in reset)?
        
        // Interrupts are off, so this will be a sleep we never wake from...
        
        deepSleep();
}    


void debugshortblink(void) {
    SBI( PORTB , LED_DRIVE_BIT);
    _delay_ms(50);
    CBI( PORTB , LED_DRIVE_BIT);
    _delay_ms(50);                
}    


void binaryDebugBlink( uint8_t b ) {
    
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


#define REG_02_DSMUTE_BIT   15          // Softmute enable
#define REG_02_DMUTE_BIT    14          // Mute disable
#define REG_02_MONO_BIT     13          // Mono select
#define REG_02_ENABLE_BIT    1          // Powerup enable

#define REG_04_DE_BIT       11          // Deemphassis


static void si4702_init(void)
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
    
     // Assumes FMIC_RESET_BIT DDR is already set to output 
    


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
    
    // Note that Sparkfun library also does it this way, so should be ok. 
    // https://github.com/sparkfun/Si4703_FM_Tuner_Evaluation_Board/blob/V_H1.3_L1.2.0/Libraries/Arduino/src/SparkFunSi4703.cpp#L133

	set_shadow_reg(REGISTER_07, 0x8100);

	si4702_write_registers(REGISTER_07);
    
    /*

        Wait for crystal to power up (required for crystal oscillator operation).
        Provide a sufficient delay (minimum 500 ms) for the oscillator to stabilize. See 2.1.1. "Hardware Initialization
        step 5.    
    
    */

	_delay_ms(500);

    /*    
        Set the ENABLE bit high and the DISABLE bit low to
        powerup the device.    
        
    */
           
    // Note that if we unmute here, we get a "click" before the radio tunes 

    // Ok folks, likes like you *must* enable RDS in "Verbose" mode here or else 
    // some silicon (seems like older) will sometimes (not everytime) come up in a weird mode with a 
    // hum over the audio. WTF Si?
    
    set_shadow_reg(REGISTER_02, 0x4001 );                
       
	/*
	 * Set radio params based on eeprom...
	 */
    
	set_shadow_reg(REGISTER_04, (eeprom_read_byte(EEPROM_DEEMPHASIS) ? _BV( REG_04_DE_BIT ) : 0x0000));
    
    // TODO: These ANDs can go if we ever need room - if these bytes are not 0 padded correctly then something is very wrong. 

	set_shadow_reg(REGISTER_05,
			(((uint16_t)(eeprom_read_byte(EEPROM_BAND) & 0x03)) << 6) |
			(((uint16_t)(eeprom_read_byte(EEPROM_SPACING) & 0x03)) << 4) |
            (((uint16_t)(eeprom_read_byte(EEPROM_VOLUME) & 0x0f)))           
    );


    // If we Unmute here, then you hear a blip of music before a click. Arg. 

    // Note that this write looks like it must come before the tune.
    // If we try to batch them into one write then we get no audio. Hmmm. 
    
	si4702_write_registers( REGISTER_05 );


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
              
    uint16_t chan = eeprom_read_word(EEPROM_CHANNEL);       // Assumes this does not have bit 15 set. 
    
    //uint16_t chan = 0x0040;                                  // test with z100.
    //uint16_t chan = 0x0044;                                  // Test with  - cbs 101 fm
        
	set_shadow_reg(REGISTER_03, 0x8000 |  chan );

	si4702_write_registers( REGISTER_03 );
    
    /*   
        Seek/Tune Time8,11 SPACE[1:0] = 0x, RCLK
        tolerance = 200 ppm, (x = 0 or 1)
        60 ms/channel
        
   */
    
    // THis appears to be where the click happens. 
    // If this is 60ms, then we hear the station tune, then lick, then station again.
    // More where needed here. 
    
    // Even though only 60ms spec'ed, if we don't wait for 100ms before unmute then we hear a a blink of music before the pop. Arg. 
   // _delay_ms(100);
    
    // We should be all tuned up and ready to go when we get here, so setup auto and unmute and let the music play!
    set_shadow_reg(REGISTER_02,  _BV(REG_02_DMUTE_BIT) | (REG_02_MONO_BIT) | _BV(REG_02_ENABLE_BIT) );  

    // TODO: Play with this more. Can we get rid of the click here?    

    /*
        The tune operation begins when the TUNE bit is set high. The STC bit is set high
        when the tune operation completes. The STC bit must be set low by setting the TUNE
        bit low before the next tune or seek may begin.
    */

    // Clear the tune bit here so chip will be ready to tune again if user presses the button.
	set_shadow_reg(REGISTER_03,  chan );
	
	si4702_write_registers( REGISTER_03 );
        
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

// Blink  1 second

static void longBlink(void) {
    setLEDBrightness(255);
    _delay_ms(1000);
    setLEDBrightness(0);
}    



// Returns true if button is down

static inline uint8_t buttonDown(void) {
    return( !TBI( PINB , BUTTON_INPUT_BIT ));           // Button is pulled high, short to ground when pressed
}    

// Setup pin change interrupt on button 
// returns true of the button was down on entry
// Assumed button DDR is input (the default)

static uint8_t initButton(void) 
{
 
     SBI( PORTB , BUTTON_INPUT_BIT);              // Enable pull up on button 
     
     _delay_ms(1);                                // Give the little pull-up a chance to overcome pin capacatance
     
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
        
    return ret;
}    




// Called on button press pin change interrupt

ISR( PCINT0_vect )
{

    // Do nothing in ISR, just here so we can catch the interrupt and wake form deep sleep
    // ISRs are ugly semantics with volatile access and stuff, simpler to handle in main thread. 
    
   return;
   
}

// Assumes button is actually down and LED_Timer is on
// Always waits for the debounced up before returning
// Call from main assumes that LED will be off when this returns

static void handleButtonDown(void) {
    
    setLEDBrightness(0);                    // Led off when button goes down. Gives feedback if we are currently breathing otherwise benign
    
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
        // TODO: Adjust for band
            
        if (currentChan > (1080-760) ) {        // Wrap at top of band back down to bottom
                
            currentChan = 0; 
                
        }                
        
        tune_direct(  currentChan );
            
        // TODO: test this wrap (lots of button presses, so start high!)
        
        
    } else {            // Initiated a long-press save to EEPROM            

        // When we get here, the button has been down for LONG_PRESS_MS and is still down

        // User feedback of long press with long flash on LED
        
        setLEDBrightness(255);        
            
        update_channel( currentChan );      
              
        _delay_ms(500);

        setLEDBrightness(0);            
                
        while (buttonDown());           // Wait for them to finally release the button- hopefully after seeing the confirmation long blink
           
    }    

    _delay_ms( BUTTON_DEBOUNCE_MS );        // Debounce the most recent up
        
}




void timertest(void) {
    
    
    TCNT1=254;            // Ensure counter not at TOP value
    
	OCR1C = 255;        // Set TOP at MAX
    
    OCR1B = 15;         // No match
    
    GTCCR =     
        _BV( PWM1B  ) |          // Enable PWM module B
        _BV( COM1B1 )            // OC1x cleared on compare match. Set when TCNT1 = $00.
    ;
    
        
    _delay_ms(1000);
    
    TCCR1 = _BV( CS12 );        // Enable timer, PCK/8        
    
    while (1);
    
    //OCR1B = 16;

    CBI( GTCCR , PWM1B );    
    //CBI( GTCCR , COM1B1 );
    //SBI( GTCCR , COM1B1 );
    SBI( GTCCR , PWM1B );
    
    while (1);
    

    
    GTCCR =     
        _BV( PWM1B  ) |          // Enable PWM module B
        _BV( COM1B1 )            // OC1x cleared on compare match. Set when TCNT1 = $00.
    ;
    
    while (1);
    GTCCR =     
        _BV( PWM1B  ) |          // Enable PWM module B
        _BV( COM1B1 )            // OC1x cleared on compare match. Set when TCNT1 = $00.
    ;
    
    
    //TCCR1 = _BV( CS12 );        // Enable timer, PCK/8           
    
    //while (TCNT1 < 5); 
    
    //TCCR1 = 0;
    
        
}


int main(void)
{
    
    // Set up the reset line to the FM_IC and AMP first so they are quiet. 
    // This eliminates the need for the external pull-down on this line. 
        
    SBI( DDRB , FMIC_RESET_BIT);    // drive reset low, makes them sleep            
        
	SBI( DDRB , LED_DRIVE_BIT);    // Set LED pin to output, will default to low (LED off) on startup
                                   // Keeps input pin from floating and toggling unnecessarily and wasting power
         
                
    adc_on();
    
    if (!VCC_GT(LOW_BATTERY_VOLTAGE)) {
        
        adc_off();  // Mind as well save some power 
        
        // indicate dead battery with a 10%, 1Hz blink
        
        // TODO: Probably only need to blink for a minute and then go into deep sleep?
        
        // NOTE that here we are driving the LED directly full on rather than PWM
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
    
    
    // Since we know we have good voltage, we can use PWM on the LED for now on... 
        
    LED_PWM_init();          // Make it so we can pwm the LED for now on...
                                
    adc_off();      /// All done with the ADC, so same a bit of power      
    
    // TODO: Check battery voltage periodically and shutdown if it goes low durring operation.     

    // Normal operation from here (good battery voltage, not connected to a programmer)
    
                
    if (initButton()) {             // Was button down on startup?
                
        // TODO: How should this work?
        
        // I think only boot with factory params, but do not save - what if they are worse or button held accidentally?
        // Better to wait for release, and then a 2nd long press to save after you hear that it works. 
        // Definitely harder to implement because we need a working EEPROM image. 
        
        // Wait for release
        
        while (buttonDown());
                            
        copy_factory_param();       // Revert to initial config
        
        longBlink();
        
        // Factory config now loaded into working config. Continue as you were...        
                        
    }   
                          
    // Now lets check if the working EEPROM settings are corrupted
    // We do this *after* the factory reset test, see why?
    
    if (check_param_crc( EEPROM_WORKING )) {
        
        // Must be inside a nuclear power reactor...
        
        // Tell user we are in trouble and then go to sleep
        // This is nice because at least we get some feedback that EEPROMS are corrupting.
        // Do not try to rewrite EEPROM settings, let the user do that manually with a factory reset.
        
        shutDown( DIAGNOSTIC_BLINK_BADEEPROM );
        
    }        

    
	si4702_init();
    
    // Radio is now on and tuned
    
    // I know gotos are forboden in civil society, but can you think of a better way to express this that is not overly baroque?    
    // If so, please LMK! -josh
    
    startBreatheOver:
    
    // Breathe for a while so user knows we are alive in case not tuned to a good station or volume too low    
    
    for(uint8_t countdown = BREATH_COUNT_TIMEOUT_S ; countdown; countdown-- ) {
        
        for(uint8_t cycle=0; cycle<(BREATH_CYCLE_LEN); cycle++ ) { 
                   
            setLEDBrightness( breath(cycle)  );
        
            _delay_ms(20);
            
            if (buttonDown()) {
             
                handleButtonDown(); 
                                               
                goto startBreatheOver;      // Looks nice to have LED start cycle over after button press
                
                // I think this is right UX? Breathe lasts 2 mins since last interaction (power on or button press)
                   
            }                
            
        }        
                
    }        
    
    // LED brightness will always end up at 0  when we stop breathing. 
        
                
    while (1) {
        
        
        // Wake on...
        //      any button state change (but ignore any but down)
        //      periodic WDT


        // Assume LED off here
        
        sei();          // Wake on button change interrupt
        
        deepSleep();
        
        if (buttonDown()) {         // We only care about presses, not lifts            
            
            handleButtonDown();     // Always wait for the debounced up before returning
                        
        }            
        
    }        
    
    
    // Never get here...
    
    
}
