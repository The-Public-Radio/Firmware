# TPR Firmware notes

This firmware runs inside the ATTINY25/45/85 inside The Public Radio. 

More info at...
http://www.thepublicrad.io/

## TODO

Review factory revert behavior. 

Make LED brightness normalized for battery voltage.


## Quick start

Install two AA batteries.    

Turn the knob clockwise until it clicks to turn on. Turn more clockwise makes it louder.

You should hear your programmed station and see LED breathing. If not, see troubleshooting below.   

Turn the knob counter clockwise until clicks to turn off. 

 ### Troubleshooting

If you don't hear anything, check the LED. 

* If it is flashing two quick blinks every second, then your batteries are too low. replace them.
* If it is flashing three quick blinks every second, then your EEPROM is corrupt. This is very unlikely, so we want to talk to you!
* If you see nothing, then the batteries are dead or not installed. Fix that.
* If it is breathing, then either...
    * The volume is turned too low. Make it louder
    * The programmed station is not tuning in. Press and release the button until you get to a station that works (and you like), then press and hold the button for 2 seconds to save that station for next time.
    

## Changing the default programmed station 

Press and release the button until you find a new station that you like.  

To save the currently playing station as the new turn-on default, press and hold the button for about 2 seconds. You should see the LED blink on for about 1/2 a second telling you the save worked. Now that station will play the next time you turn on.

If you want to go back to the station your unit was programmed with when you got it, turn it off and then hold down the button while you turn it back on. Once it is on, release the button to restore the configuration (if you turn power off before releasing, then the initial configuration will not be saved).  You should see a 1/2 second flash indication that the restore worked, and then the TPR should start playing your original factory programmed station. 


## User interface

There are three user interaction elements...

* Volume knob
* Momentary push button
* Indication LED

When the knob is in the off position, the unit is unpowered. When the knob is on, adjusts the volume on the speaker.

Durring normal operation, the LED will "breathe" at about 1Hz to give feedback that the unit is powered up and running. This will contuine for about 2 minutes after the last interaction (either a power up or  button push).

### LED error indications

* 2 short blinks repeat every second -  Battery Too Low for Operation

Can occur at startup or durring operation.  Unit will stop blinking and go into deep sleep after about 2 minutes.

* 3 short blinks every second - Corrupt EEPROM checksum

Can only occur at startup. Unit will stop blinking and go into deep sleep after about 2 minutes.


### Low battery threshold voltages 

There are two low battery voltage thresholds - one for cold power up before audio has started playing, and a lower one for after audio has startred. This is to account for the lower battery voltage caused by the large current draw of the amplifier. 

### User configuration

The user can change the programmed station using the push button. Power must be on and the batteries must be good. 

A short press (<2 seconds) of the button advances the next channel. The advance happens on the button release.  Each press advances the tunes frequency by the `spacing` parameter which is either 100Khz or 200Khz depending on country. If you living the the USA with 100Khz spacing and are listening to 93.9Mhz and you short press once, you will be at 94.1Mhz. Use repeated presses to find the desired station. The station will wrap back to the bottom when it gets to the top of the valid frequency band.  

A long press (<2 seconds) of the button stores the current station into EEPROM. A long (0.5 second) flash on the LED confirms that the station was stored. This station will be loaded the next time the unit powers up. 

Note that if you advance the station with short presses and do not save with a long press, then the unit will revert to the previously stored station on next power up.

### Factory reset

To load the station programmed when the unit initially shipped... 
 1. Start with the unit powered off (knob all the way counter-clockwise) 
 2. Press and hold the button
 3. Turn on the power (turn knob clockwise until it clicks)
 4. Release the button

You should see a long blink (about 0.5 second) indicating that the unit was reset to the factory configuration. It will then start playing.

If you do not see the single blink when you release the button, check to make sure the batteries are good. 

## One touch programming feedback

You can update the factory configuration by sending a packet though the battery terminals. More information in the on-touch-programming folder. This is meant to be done at the factory.

When the unit receives a programming packet, it verifies the checksum.
 
* If no errors, the new config is saved in EEPROM as both WORKING and FACTORY configs and the LED make four quick flashes. 
* If the check fails, the packet is ignored and the LED makes one quick flash.

## Theory of operation

The ATTINY boots and...

1. Checks for sufficient voltage for operation. If battery is too low, then flashes an indication on the LED and goes to sleep.
2. Checks if we are connected to a programming station based on voltage (programming station supplies >4.5V). If so, accepts a configuration download and writes to EEPROM.
3. Checks if button is held down on startup. If so, reverts to the user's initial configuration on release. 
4. Configures and starts up the amp and radio chip and tunes to the programmed station.
5. Flashes an "I'm alive" breathing pattern on the LED for about 2 minute.
6. Goes to deep sleep, only to be woken on a button press.
7. On release of a short button press, advances to the next station on the dial. 
8. On long button (2+ seconds) press, stores the current station in EEPROM.


## TWI library
The TWI code here is custom written for this project. It differs from a a general purpose library in that...

1. **Internal pin pill-ups are used so no external resistors are needed.** This is a DFM optimization that theoretically could limit bandwidth, but in out case here (1) our ATTINY runs out of steam before then anyway, (2) we only need to send a dozen bytes, and only at startup and when changing channels, so performance doesn't matter anyway. 

1. **Only a single master is supported. ** We only have one anyway. 

1. **Everything is bitbanged, so both clock and data could be moved to any IO pin. **

## Notes

1. **Only TWI `write` is used, not `read`.** It is just a quirk that the config registers of FM chip used always default to `0`s and functionally the TPR never needs to read status info. There are some reserved bits in register 0x07 that say they must be read before being written, but we get around this thanks to the fact that 0x07 is the highest register we need to write to, so once we set it we are conservative when writing lower registers to never overwrite it again. Not including read code saves flash space. 
2. The band, spacing, and deemphassis are not user updatable. The user can only change the channel within the factory programmed locale. This is by design. 
