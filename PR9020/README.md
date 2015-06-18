Firmware & support scripts for Public Radio.

The Makefile and scripts do not knowingly accomodate non-linux build platforms, or different programmers than an AVRISP2. Pull requests broadening that support welcome.

Firmware reads configuration settings from eeprom and programs the Si4702 accordingly. Also implements simple single button, single LED, user interface to allow the owner to reprogram the channel.

Eeprom layout is described here: https://docs.google.com/spreadsheets/d/1fg65g10DV3-tExtAQwt_fEo1chngjSt0LV546Z1wjgg/edit?usp=sharing

User interface as follows:

Tuning Functionality

-Radio ships pre-tuned, frequency stored in EEPROM 
-Tactile switch allows for manual retuning, saving and overwriting old station. 
-The valid frequency range is programmed during perso, based on destination region.

User Feedback Options
-Audio
        o Listening to a station as you tune to verify.
-LED
        o Provides feedback based on which tuning mode the user is relying on:
                * Manual Re-tune Mode
                * Scan Mode 
                * Factory Reset Mode
                * Save Mode 
                * Timed Out Mode

LED Displays

-Normal Mode = brightness dependant on RSSI
-Tune Mode = 320mS HIGH, 320mS LOW
-Factory Reset Mode = 160mS HIGH, 160mS LOW
-Save Mode = HIGH
-Timed Out = revert to Normal Mode

Tuning Modes

-Begin Manual re-tune
        o Initialized by a >= 2 second button depress.  
        o LED flashes 320mS on/off.
        o Single short (< 2 sec) button press seeks up to next valid station.
        o Channel step & band edges as per perso data in EEPROM


Contents:
   dump_eeprom.py - take intel hex dump of eeprom contents, decode and print in <sort of> human readable format.
   eeprom.py - create intel hex suitable for programming into eeprom, according to various comamnd line flags (--help to see)
   Makefile - makefile(duh)
   USI_TWI_Master.[ch] - Atmel reference code to implement TWI/i2c master functionality on the USI present in the ATTinyx5 chips.
   main.c - everything else, reasonably well documented.
