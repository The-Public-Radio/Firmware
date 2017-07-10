## EEPROM Utilities

* `dump_eeprom.py` - take intel hex dump of eeprom contents, decode and print in <sort of> human readable format.
* `eeprom.py` - create intel hex suitable for programming into eeprom, according to various comamnd line flags (--help to see)
* `maker.sh` - a bash script to make communicating with eeprom.py easier during testing

Note that these EEPROM utilities require outside libraries which can be installed by entering the commands...

    pip install intelhex
    pip install crcmod

### Example usages...

#### dump_eeprom.py

This will print out the contents of an EEPROM file in human readable format.

It reads from `stdin`, so you run like this...

`dump_eeprom.py <py-WAPP.hex`

...where `py-WAPP.hey` is an eeprom file.

Output looks like this...

```
dump_eeprom.py <py-WAPP-Z100.hex
NOTE:Eyecatcher text string not found
---Working
    Band:        00 = 87.5-108 MHz (USA, Europe)
    Deemphasis:  00 = 75 us. Used in USA
    Spacing:     00 = 200 kHz (USA, Australia)
    Channel:    080
    Volume:      15
    Freqency=103.50 Mhz (calculated)
---Factory
    Band:        00 = 87.5-108 MHz (USA, Europe)
    Deemphasis:  00 = 75 us. Used in USA
    Spacing:     00 = 200 kHz (USA, Australia)
    Channel:    064
    Volume:      15
    Freqency=100.30 Mhz (calculated)
SN:                  
WW: 255
YY: 255
Station:   
Campaign:              
```

The note about `Eyecatcher` not found indtactes that a special tag string was not in the EEPROM, but don't worry because this string does not seem to be in any files (or units) in practice.   


#### eeprom.py

To get a hex file for programming 103.5 Hot FM with other default parameters, enter..

    python eeprom.py -f 103.5

..which produces the output...

	:1000000000000050000F000000000000000040F160
	:1000100000000050000F000000000000000040F150
	:00000001FF 


    
