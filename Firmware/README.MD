# TPR Firmware notes

This firmware runs inside the ATTINY25/45/85 inside The Public Radio. 

More info at...
http://www.thepublicrad.io/

## TODO

Factory revert behavior not implemented pending review.

Removal of SDA pull-up resistor (and all associated code) not implemented pending getting logical analyzer back from Pete! 

Breath blink. 

Confirm 100Khz steps.

Confirm change de-empassis on wrap? Do we even need it at all?

Use lower band by default? Or maybe only on wrap? 



## Quick start

Install two AA batteries.    

Turn the knob clockwise until it clicks to turn on. Turn more clockwise makes it louder.

You should hear your programmed station. If not, see troubleshooting below.   

Turn the knob counter clockwise until clicks to turn off. 

 ### Troubleshooting

If you don't hear anything, check the LED. 

* If it is flashing, then your batteries are too low. replace them. 
* If you see nothing, then the batteries are dead or not installed. Fix that.
* If it is breathing, then either...
    * The volume is turned too low. Make it louder
    * The programmed station is not tuning in. Press and release the button until you get to a station that works (and you like), then press and hold the button for 2 seconds to save that station for next time.
    


## Changing the default programmed station 

Press and release the button until you find a new station that you like. Each time you will go up 0.1Mhz. 

To save the currently playing station as the new default, press and hold the button. You should see the LED blink after about 2 seconds. Now that station will play the next time you turn on.

If you want to go back to the station your unit was programmed with when you got it, turn it off and then hold down the button while you turn it back on. You should see the LED flash 3 times quickly. To save this as your default station, press and hold the button for 2 seconds until the LED blinks.  


## User interface

There are three user interaction elements...

* Volume knob
* Momentary push button
* Indication LED

When the knob is in the off position, the unit is unpowered. When the knob is on, adjusts the volume on the speaker.

### Low battery indication

On power up, if the battery voltage is too low for operation, the LED will short blink (about 100ms) twice. This blink pattern will repeat once per second for about 1 minute and then the unit will go into deep sleep. If this happens, replace the batteries with fresh ones. 

### User configuration

The user can change the programmed station using the push button. Power must be on and the batteries must be good. 

A short press (<2 seconds) of the button will advances the next frequency. The advance happens on the button release.  Each press advances the tunes frequency by 100Khz, so if you are listening to 93.9Mhz and you short press once, you will be at 94.0Mhz. Use repeated presses to find the desired station. The station will wrap back to the bottom when it gets to the top of the valid frequency band.  

A long press (<2 seconds) of the button stores the current station into EEPROM. A 0.5 second flash on the LED confirms that the station was stored. This station will be loaded the next time the unit powers up. 

Note that if you advance the station with short presses and do not save with a long press, then the unit will revert to the previously stored station on next power up.

To load the station programmed when the unit initially shipped to the user, press and hold the button during power-up. The unit will indicate that the initial station was loaded with 3 short flashes on the LED. To save this initial station to the EEPROM, release the button and then long press (see above). 

## Theory of operation

The ATTINY boots and...

1. Checks for sufficient voltage for operation. If battery is too low, then flashes an indication on the LED and goes to sleep.
2. Check if we are connected to a programming station. If so, accepts a configuration download and writes to EEPROM.
3. Checks if button is held down on startup. If so, reverts to the user's initial configuration.
4. Configures and starts up the amp and radio chip and tunes to the programmed station.
5. Flashes an "I'm alive" indication pattern on the LED for about 1 minute.
6. Goes to deep sleep, only to be woken on a button press.
7. On release of a short button press, advances to the next station on the dial. 
8. On release of a long button press, stores the current station in EEPROM.




## TWI library
The TWI code here is custom written for this project. It differs from a a general purpose library in that...

1. **Internal pin pill-ups are used so no external resistors are needed.** This is a DFM optimization that theoretically could limit bandwidth, but in out case here (1) our ATTINY runs out of steam before then anyway, (2) we only need to send a dozen bytes, and only at startup and when changing channels, so performance doesn't matter anyway. 

1. **Only a single master is supported. ** We only have one anyway. 

1. **Only `write` is implemented, no `read`.** It is just a quirk that the config registers of FM chip used always default to `0`s and functionally the TPR never needs to read status info. There are some reserved bits in register 0x07 that say they must be read before being written, but the previous R/W code just smashed the reserved bits with `0`'s anyway and that seems to work fine. 

1. **Everything is bitbanged, so both clock and data could be moved to any IO pin. **