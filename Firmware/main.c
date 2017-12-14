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
#include <avr/wdt.h>

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


#define LOW_BATTERY_VOLTAGE_COLD (2.1)          // We need to see this at power up to start operation. 
#define LOW_BATTERY_VOLTAGE_WARM (1.80)         // If we get this low, we are not working anymore so user accidentally 
                                                // left power on. We should down to avoid battery blistering
                                             
#define LOW_BATTERY_VOLTAGE_WARM_COUNT  10      // We need to see this many consecutive low battery voltage readings before shutting down
                                                // this prevents us from shutting down based on seeing one sample that might have happened 
                                                // right when the amp was pulling a spike of current. 
                                                
                                                
// TODO: Empirically figure out optimal values for low battery voltages

#define LED_COUNT (2)       // How many initial breaths should we display? Resets on startup and after each button press. Each breath currently about 2 secs.

// These are the error display codes
// When we run into problems, we blink the LED this many times to show the user
// Note that 1 blink is skipped intentionally for UX reasons. Is a single blink a single blink, or an infinite number of blinks? Think about it!

#define DIAGNOSTIC_BLINK_BADEEPROM     3            // EEPROM user settings failed CRC check on startup
#define DIAGNOSTIC_BLINK_LOWBATTERY    2            // Battery too low for operation. 

#define DIAGNOSTIC_BLINK_TIMEOUT_S   120            // Show diagnostic blink at least this long before going to sleep
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
	REGISTER_0A =  0,
	REGISTER_0B =  2,
    

    // These not used in current implementation. 
	REGISTER_0C =  4,
	REGISTER_0D =  6,
	REGISTER_0E =  8,
	REGISTER_0F = 10,
        
} si4702_register;


// RF-IC Stuff below

#define REG_02_DSMUTE_BIT       15          // Softmute enable
#define REG_02_DMUTE_BIT        14          // Mute disable
#define REG_02_MONO_BIT         13          // Mono select
#define REG_02_RDSM_VERBOSE_BIT 11          // Old code had a comment that this must be enabled?
#define REG_02_SKMODE           10          // 0 = Wrap at the upper or lower band limit and continue seeking (default). 1 = Stop seeking at the upper or lower band limit.
#define REG_02_SEEKUP_BIT        9          // 0 = Seek down (default). 1 = Seek up.
#define REG_02__SEEK             8          // This is a command. 1 = Enable. A seek operation may be aborted by setting SEEK = 0.
#define REG_02_ENABLE_BIT        0          // Power up enable

#define REG_02_DEFAULT ( _BV( REG_02_DMUTE_BIT) |  _BV(REG_02_MONO_BIT) | _BV(REG_02_ENABLE_BIT) | _BV(REG_02_SEEKUP_BIT) )    // Used mostly when setting and clearing seek bit


#define REG_04_DE_BIT       11          // Deemphasis

#define REG_07_XOSCEN       15          // Enable crystal oscillator
#define REG_07_AHIZEN       14          // Audio high-Z enable


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


static inline void LED_on(void) {
   // Very quick blink LED twice
   SBI( PORTB , LED_DRIVE_BIT );
}    



static inline void LED_off(void) {
   CBI( PORTB , LED_DRIVE_BIT );        
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

// Read registers 0x0a and 0x0b from FM_IC.
// We really only care about 0x0b because this is where the current channel is saved after a seek.
// We have to read both because all TWO reads on this chip start at register 0x0a (they wraps around to at 0x0f) 
// We need 0x03 so we can check what station we landed on after a seek.

static void si4702_read_registers_upto_0B(void)
{
    USI_TWI_Read_Data( FMIC_ADDRESS , shadow , REGISTER_0B - REGISTER_0A + 2 );      // Total of 10 registers,  each 2 bytes
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



/*
 * Seek thresholds - see Appendix of SiLabs AN230
 * TODO: Set these according to AN284?
 */
#define	SEEK_RSSI_THRESHOLD	(10)
#define	SEEK_SNR_THRESHOLD	(2)
#define SEEK_IMPULSE_THRESHOLD	(4)




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

// Note: must read the registers from the FM_IC first with si4702_read_registers_upto_0B()
// THis gets the current channel after a seek.

/*

Register 0Bh. Read Channel

9:0 READCHAN[9:0] Read Channel.
READCHAN[9:0] provides the current tuned channel and is updated during a seek
operation and after a seek or tune operation completes. Spacing and channel are set
with the bits SPACE 05h[5:4] and CHAN 03h[9:0].

*/


static uint16_t currentSeekChanFromShadow(void) {
    return( get_shadow_reg(REGISTER_0B & 0x1ff));
}    

// read the current channel from the RF-IC and save to eeprom. 
// We need this on a long press to save a new station after a seek.

static void updateToCurrentChannel(void) {
    
    si4702_read_registers_upto_0B();
    
    update_channel( currentSeekChanFromShadow() ); 
    
}    



// Issue a seek, come what may.
// Note that state gets mussy here. We don't check for when the seek ends because it would use
// previous code space and also checking messes with the RFIC and can cause us to miss stations.
// We just blindly send the seek command even if there is already a seek in progress. It is going to be ok. 

// Seek settings taken from original version of this code.
// TODO: Should we adjust seek settings based on AN284 app note?

static void seekNext(void) {
                
        /* 
    
        "The STC and SF/BL bits must be set low by setting the SEEK bit low
        before the next seek or tune may begin/"        
        */
                
                                
    set_shadow_reg(REGISTER_02, REG_02_DEFAULT  );            
    
    si4702_write_registers( REGISTER_02 );
    
    // Empirically determined that we need this delay.
    // We we follow the stop with a start immediately, it does not work. 
    // 1ms was the 1st guess and it worked. 
   
    
    _delay_ms(1);
                
            
    // Set "SEEK" bit on - begins the seek
            
    set_shadow_reg(REGISTER_02, REG_02_DEFAULT | _BV(REG_02__SEEK) );            
    
    si4702_write_registers( REGISTER_02 );
                
    
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


// Goto bed, will only wake up on button press interrupt (if enabled) or WDT

static void deepSleep(void) {
	set_sleep_mode( SLEEP_MODE_PWR_DOWN );
    sleep_enable();
    sleep_cpu();        // Good night    
}  


// Don't do anything, just the interrupt itself will wake us up
// TODO: If we ever need more code space, this could be replaced by an IRET in the VECTOR table. 

EMPTY_INTERRUPT( WDT_vect );

#define HOWLONG_16MS   (_BV(WDIE) )
#define HOWLONG_32MS   (_BV(WDIE) | _BV( WDP0) )
#define HOWLONG_64MS   (_BV(WDIE) | _BV( WDP1) )
#define HOWLONG_125MS  (_BV(WDIE) | _BV( WDP1) | _BV( WDP0) )
#define HOWLONG_250MS  (_BV(WDIE) | _BV( WDP2) )
#define HOWLONG_500MS  (_BV(WDIE) | _BV( WDP2) | _BV( WDP0) )
#define HOWLONG_1S     (_BV(WDIE) | _BV( WDP2) | _BV( WDP1) )
#define HOWLONG_2S     (_BV(WDIE) | _BV( WDP2) | _BV( WDP1) | _BV( WDP0) )
#define HOWLONG_4S     (_BV(WDIE) | _BV( WDP3) )
#define HOWLONG_8S     (_BV(WDIE) | _BV( WDP3) | _BV( WDP0) )

// Goto sleep - get woken up by the watchdog timer or other interrupt like button
// This is very power efficient since chip is stopped except for WDT
// Note that if the timer was on entering here that it will stay on, so LED will still stay lit.

static void sleepFor( uint8_t howlong ) {
    
    wdt_reset();
    WDTCR =   howlong;              // Enable WDT Interrupt  (WDIE and timeout bits all included in the howlong values)
    
    sei();
    deepSleep();
    cli();
    
    WDTCR = 0;                      // Turn off the WDT interrupt (no special sequence needed here)
                                    // (assigning all bits to zero is 1 instruction and we don't care about the other bits getting clobbered
    
}    


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
    
   
   
                                           
    // Enable the pull-ups on the TWI lines

	USI_TWI_Master_Initialise();
	
    // Reg 0x07 bit 15 - Crystal Oscillator Enable.
    // 0 = Disable (default).
    // 1 = Enable.
    
    // Reg 0x07 bits 13:0 - Reserved. If written, these bits should be read first and then written with their pre-existing values. Do not write during powerup.
    // BUT Datasheet also says Bits 13:0 of register 07h must be preserved as 0x0100 while in powerdown and as 0x3C04 while in powerup.
    
    // Note that Sparkfun library also does it this way, so should be ok. 
    // https://github.com/sparkfun/Si4703_FM_Tuner_Evaluation_Board/blob/V_H1.3_L1.2.0/Libraries/Arduino/src/SparkFunSi4703.cpp#L133

    // Tested and setting AHIZEN here has no effect on enable click. 
    // Tested setting 07 to XOSCEND | 0x3c04 and chip does not function. 

	set_shadow_reg(REGISTER_07, 0x8100 );

	si4702_write_registers(REGISTER_07);
    
    /*

        Wait for crystal to power up (required for crystal oscillator operation).
        Provide a sufficient delay (minimum 500 ms) for the oscillator to stabilize. See 2.1.1. "Hardware Initialization
        step 5.    
    
    */
    
    // UPDATE 11-5-2017: Some units powering up with static in production, so increasing this from 500ms to 600ms
    // and also increasing powerup time in enable from 200ms to 350ms to try and get a margin past the problem.
    // TODO: Walk this back once problem cured to see if this delay was relevant. 

	_delay_ms(600);
    
}

// We break out init() and enable() into different functions so we can check the battery voltage 
// After the 550ms delay after startup.     
    
static void si4702_enable(void) {
    

    /*    
        Set the ENABLE bit high and the DISABLE bit low to
        powerup the device.    
        
        We leave mute on here so we don't hear static in the time between when the chip actually comes up
        and when we can get the seek in.
        
    */
               
//    set_shadow_reg(REGISTER_02, _BV( REG_02_DSMUTE_BIT ) |  _BV( REG_02_ENABLE_BIT ) );    // Setting DSMUTE here doesnt help with click


    set_shadow_reg(REGISTER_02,  _BV( REG_02_ENABLE_BIT ) );    // Enable chip


    // OK, CLICK DEFINATELY HAPPENS ON THIS ENABLE ACTION!!!!

	si4702_write_registers( REGISTER_02 );

       
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

    
	/*
	 * Set the seek SNR and impulse detection thresholds.
	 */
    
    // We are writing up to REGISTER_05 soon anyway, mind as well set 06 here too while we are up there
    // even though we may never need these set (only used if user presses button to seek)
    
	set_shadow_reg(REGISTER_06,
			(SEEK_SNR_THRESHOLD << 4) | SEEK_IMPULSE_THRESHOLD);
    

    // If we Unmute here, then you hear a blip of music before a click. Arg. 

    // Note that this write looks like it must come before the tune.
    // If we try to batch them into one write then we get no audio. Hmmm. 
    
	si4702_write_registers( REGISTER_06 );


    /*    
        Software should wait for the powerup
        time (as specified by Table 8, “FM Receiver
        Characteristics1,2,” on page 12) before continuing with
        normal part operation.  
        
        Powerup Time From powerdown
        (Write ENABLE bit to 1)
        max 110 ms        
          
    */    
    
    // This was 110ms as per spec, but caused a minority of units to come up
    // tuned to static. WTF Si?
    // 200ms next guess, seems to cure problem on unit tested. 
    // UPDATE 11-7-2017:  Maybe 200ms not enough either. Some units in production are still powering up to
    // static so upping this form 200ms to 350ms, and osc delay from 500ms to 600ms. 
    // TODO: Maybe try polling the chip to see exactly when it wakes up to shorten this?
        
    _delay_ms(350); 
    
}

/*
 * tune_direct() -	Directly tune to the specified channel.
 * Assumes chan < 0x01ff
 * Unneeded in current embodiment since we only use seek to change channels after startup
 
 */

static void si4702_tune(uint16_t chan)   {
                  
    
    //uint16_t chan = 0x0040;                                  // test with z100.
    //uint16_t chan = 0x0044;                                  // Test with  - cbs 101 fm
          
	set_shadow_reg(REGISTER_03, 0x8000 |  chan );

	si4702_write_registers( REGISTER_03 );
    
    // Ok, we should be all set up and tuned here, but still muted. 
           
    // Disable mute, set mono, seek up!
    
    set_shadow_reg(REGISTER_02,  REG_02_DEFAULT );       
    
    si4702_write_registers( REGISTER_02 );

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


/*

Old tunedirect()

{

	set_shadow_reg(REGISTER_03, 0x8000 | chan );

	si4702_write_registers( REGISTER_03 );
	_delay_ms(160);

    
    // Clear the tune bit here so chip will be ready to tune again if user presses the button.
	set_shadow_reg(REGISTER_03,  chan );
	si4702_write_registers( REGISTER_03 );
}

*/

// Blink  1 second

static void longBlink(void) {
    LED_on();
    _delay_ms(1000);
    LED_off();
}    

// Returns true if button is down

static inline uint8_t buttonDown(void) {
    return( !TBI( PINB , BUTTON_INPUT_BIT ));           // Button is pulled high, short to ground when pressed
}    

 
// Called on button press pin change interrupt
// Do nothing in ISR, just here so we can catch the interrupt and wake form deep sleep
// ISRs are ugly semantics with volatile access and stuff, simpler to handle in main thread. 

// TODO: If we ever need more code space, this could be replaced by an IRET in the VECTOR table. 


EMPTY_INTERRUPT( PCINT0_vect ); 


// Setup pin change interrupt on button 
// returns true of the button was down on entry
// Assumed button DDR is input (the default)

static uint8_t initButton(void) 
{
 
     SBI( PORTB , BUTTON_INPUT_BIT);              // Enable pull up on button 
     
     _delay_ms(1);                                // Give the little pull-up a chance to overcome pin capacitance
     
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




// Assumes button is actually down and LED_Timer is on
// Always waits for the debounced up before returning
// Call from main assumes that LED will be off when this returns

static void handleButtonDown(void) {
    
    LED_off();                    // Led off when button goes down. Gives feedback if we are currently breathing otherwise benign
    
    _delay_ms( BUTTON_DEBOUNCE_MS );        // Debounce down
        
    uint16_t countdown = LONG_PRESS_MS;
    
    while (countdown && buttonDown()) {       // Spin until either long press timeout or they let go
        _delay_ms(1);          
        countdown--;
    }        
    
    if (countdown) {                            // Did not timeout, so short press
        
        // Advance to next station
        
        // quick blink the LED to let the user know they did something 

        LED_on();
        _delay_ms(150);
        LED_off();
        
        seekNext();
                                
        // TODO: test this wrap (lots of button presses, so start high!)
        
        
    } else {            // Initiated a long-press save to EEPROM            

        // When we get here, the button has been down for LONG_PRESS_MS and is still down

        // User feedback of long press with long flash on LED
        
        LED_on();        
                            
        updateToCurrentChannel();      // TODO: This no longer works with seek rather than step.
                      
        _delay_ms(500);

        LED_off();            
                
                
        while (buttonDown());           // Wait for them to finally release the button- hopefully after seeing the confirmation long blink
                                        // Note that we are not checking for low battery voltage here. This means that a malicious user could
                                        // hold down the button for a few years and cause the battery to blister. Might need to add a warning sticker
                                        // above button saying "HOLDING BUTTON DOWN FOR MORE THAN 1 YEAR MAY CCAUSE BATTERY DAMAGE"
                           
    }    

    _delay_ms( BUTTON_DEBOUNCE_MS );        // Debounce the most recent up
        
}


// Show user bad eeprom was read with 3 blinks.
// Use the PWM on LED since the battery is good. 
// No way to wake up since the EEPROM will still be bad.

static void badEEPROMBlink(void) {

    // No need to disable FMIC and AMP becuase we didn't turn them on before the eeprom check 
      
    // No need to turn off adc since it was not enabled yet
       
    uint8_t blinkCountDown= DIAGNOSTIC_BLINK_TIMEOUT_S;
    
    while (blinkCountDown-- ) {          // Still blinking? Also, a button press will abort the blink cycle

        for( uint8_t c=3; c ; c--) {                // 3 blinks
            LED_on(); 
            sleepFor( HOWLONG_16MS);
            LED_off();
            sleepFor( HOWLONG_500MS );                // Space between blinks
        }        
        
        sleepFor( HOWLONG_2S );                // Space between blinks
        
    }

    // Interrupts are off, so this is forevah    
    deepSleep();
    // Never get here
    
}    


// Show the user something went wrong and we are shutting down.
// Blink the LED _count_ times every second.
// Continue doing this for at least DIAGNOSTIC_BLINK_SECONDS
// then go into a mode where we burn about 50uA forever.
// We need to do this to discharge the decoupling caps
// se the FM_IC will reset when we do a battery change. 

// It would be nice if pushing the button would just wake up back up, 
// but the FM_IC seems to need a full power cycle for that to work. 

static void lowBatteryShutdown(void) {
    
    LED_off();        // Turn off PWM, we will directly drive the LED from the pin output
    
    
    // Shutdown all peripherals so save power in deep sleep
    // Before 4uA
    // After 4uA
    // Most of this draw is likely from the amp and FM_IC in shutdown modes
    PRR = _BV( PRTIM1 ) | _BV( PRTIM0) | _BV( PRUSI ) | _BV(PRADC ); 
    
    
    while (1) { 
        
                
         
        // TODO: Test how much current actually used here. Can we lower it with peripheral power control register?
        // TODO: Add a MOSFET so we can completely shut them off (they still pull about 25uA in reset)?
    
        // Turn on interrupts. This will enable wake on button press
        // which gives uses a way to not have to wait for us to completely run down the
        // decoupling caps
    
        uint8_t blinkCountDown= DIAGNOSTIC_BLINK_TIMEOUT_S;
    
        while ( blinkCountDown-- &&  !buttonDown() ) {          // Still blinking? Also, a button press will abort the blink cycle
        
            // Note that here we directly turn the LED on and off using the pin rather than using
            // PWM. This is so we can get maximum brightness when the battery voltage is low.
        
            // Very quick blink LED twice
            SBI( PORTB , LED_DRIVE_BIT );
            sleepFor( HOWLONG_16MS);
            CBI( PORTB , LED_DRIVE_BIT );
            
            _delay_ms(250);                             // Space between blinks. Waste power. 
            //sleepFor( HOWLONG_250MS );                // Space between blinks
            SBI( PORTB , LED_DRIVE_BIT );
            sleepFor( HOWLONG_16MS);
            CBI( PORTB , LED_DRIVE_BIT );
            sleepFor( HOWLONG_1S );                // Space between blinks
        
        }
        
        
        while (1) {
            _delay_ms(10); // Burn power. Ouch this hurts. 
            sleepFor( HOWLONG_125MS );
        }
        

        /*    
        sei();          // Enable interrupts. This will wake us if the user presses the button to signal they replaced the battery.    
        deepSleep();
        cli(); 
    
        // Spin and blink while the button is down. We do this for a couple reasons.
        // 1. Give user feedback that we see the button.
        // 2. Make sure the button is up by the time we restart so we don't see it down and do a seek or save
        // 3. If the battery was not changed, this will really kill a dead battery and drop the voltage.
    
        while (buttonDown()) {

            SBI( PORTB , LED_DRIVE_BIT );
            _delay_ms(20);
            CBI( PORTB , LED_DRIVE_BIT );
            _delay_ms(20);
        
        }
        
        // Now we spend ~255*40ms= 10s blinking craziliy to try and run out the capactors so chip will reset on powerup. 
        
        uint8_t delay=255;
        
        while (delay--) {

            SBI( PORTB , LED_DRIVE_BIT );
            _delay_ms(20);
            CBI( PORTB , LED_DRIVE_BIT );
            _delay_ms(20);
            
        }
        
        // If by now we are still runiing, 
        
        */
    
    }    
    
    
    /*

    //Test to see if we get here

    
    while (1) {

        SBI( PORTB , LED_DRIVE_BIT );
        _delay_ms(200);
        CBI( PORTB , LED_DRIVE_BIT );
        _delay_ms(200);
        
    }
    
    
    */
        
}


static void debugBlinkDigit(uint8_t c) {
    
    while (c--) {
        
        LED_on();
        _delay_ms(200);
        LED_off();
        _delay_ms(200);
    }
    
    _delay_ms(400);     // Break between high and low digits
    
        
}    

// Blink out the current voltage on the LED for debugging
// volts...500ms...tenths of volts....1s...repeat

static void __attribute__ ((unused)) debugBlinkVoltage(void) {
    
    
    uint8_t Vcc = ADC2VCC( readADC() ) * 10.0;        // Vcc now *10, so 30 = 3.0v
        
    uint8_t VccT = Vcc/10;          // Break out high and low digits
    
    debugBlinkDigit( VccT );
    uint8_t VccB = Vcc%10;
        
    debugBlinkDigit(VccB);
    
    _delay_ms(1000);        // Break between readings
                
}  

void run(void) {
    SBI( PORTB , FMIC_RESET_BIT );     // Bring FMIC and AMP out of reset
    // The bit direction was set to output when we first started in main()
    
    _delay_ms(1);                      // When selecting 2-wire Mode, the user must ensure that a 2-wire start condition (falling edge of SDIO while SCLK is
                                       // high) does not occur within 300 ns before the rising edge of RST.
    
        
    si4702_init();
    
    // We do a check here because init on FM_IC takes 500ms, so by the time we get here
    // power has stabilized but we do want to check before the amp starts playing music.

    adc_on();
    
    if (VCC_LESS_THAN( LOW_BATTERY_VOLTAGE_COLD )) {
        

        // Disable FMIC and AMP
        CBI( PORTB , FMIC_RESET_BIT);    // drive reset low, makes them sleep. DDR is set output on start-up and never changed.
        
        adc_off();
        
        lowBatteryShutdown();
        
        return;
        
    }        
        
            
    si4702_enable();            // Finish bringing up the FM_IC (it will still be muted)
    
    uint16_t chan = eeprom_read_word(EEPROM_CHANNEL);       // Assumes this does not have bit 15 set.

    si4702_tune( chan );        // Tune up the programmed station and start playing
        
    // Radio is now on and tuned
    
    uint8_t ledCountdown = LED_COUNT;         // Count down how long LED is displayed. When this gets to 0, stop showing LED until a button push resets count
    
    uint8_t warm_low_count=0;                    // How many times in a row has the warm voltage been too low?
    
    while (1) {
        
        // This loop cycles every second 
                
        // Constantly check battery and shutdown if low
                
        if  (VCC_LESS_THAN( LOW_BATTERY_VOLTAGE_WARM )) {
            
            warm_low_count++;
            
            if (warm_low_count>=LOW_BATTERY_VOLTAGE_WARM_COUNT) {
                
                // Only shutdown if we see a consecutive series of low voltage samples to avoid
                // false alarm due to temp low voltage from a current spike.
                
                // Disable FMIC and AMP
                CBI( PORTB , FMIC_RESET_BIT);    // drive reset low, makes them sleep. DDR is set output on start-up and never changed.
                                               
                adc_off();
                
                LED_off();        // Turn off LED PWM
                
                lowBatteryShutdown();
                
                return;
                
            }                
                
            
        } else {
            
            // If we see even one good voltage sample, then we start counting over again
            // this helps make sure the battery is really low before we shut down
            
            warm_low_count=0;
            
        }
                
        
        if (buttonDown()) {
            
            handleButtonDown();
            
            // Every time the button is pressed we start the LED light countdown over
            // !! commenting out to avoid turning the LED on ever time we change stations
            
            // ledCountdown = LED_COUNT;
            
        }
        
        
        // Light LED so user knows we are alive in case not tuned to a good station or volume too low
        // Each pass though the while loop in this stage takes about 20ms

        
        if (ledCountdown) {
            
            LED_on();               // Blindly turn on even though might already be on becuase it takes longer to check than do
                        
            ledCountdown--;
            
            if (!ledCountdown) {
                
                LED_off();
                
            }  
            
        }               
            
        sleepFor( HOWLONG_1S );      // Do nothing for a while before checking low battery again (will wake instantly on button press) to save power
        // We actually do not need to check the battery this often, this is just the longest the watchdog timer can sleep for
        // The CPU only used a few microamps for this 8 seconds, which should help extend battery life.
            
                
    }
}    


int main(void) {
           
    // Set up the reset line to the FM_IC and AMP first so they are quiet. 
    // This eliminates the need for the external pull-down on this line. 
        
    SBI( DDRB , FMIC_RESET_BIT);    // drive reset low, makes them sleep            
        
	SBI( DDRB , LED_DRIVE_BIT);    // Set LED pin to output, will default to low (LED off) on startup
                                   // Keeps input pin from floating and toggling unnecessarily and wasting power
                                       
    _delay_ms(50);                 // Debounce the on switch
        
    // TODO: Test shutdown current on a new PCB that lets us hold the amp in reset
                                                                                                                            
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
        
        badEEPROMBlink();
                
    }        
    
    while (1) {        
        
        // In here, we try to run but if battery is low then we turn off radio and sleep until a button press and then return.
        run();        
    }        
    // Never get here...
       
}
