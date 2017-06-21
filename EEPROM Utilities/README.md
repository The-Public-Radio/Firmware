## EEPROM Utilities

* `dump_eeprom.py` - take intel hex dump of eeprom contents, decode and print in <sort of> human readable format.
* `eeprom.py` - create intel hex suitable for programming into eeprom, according to various comamnd line flags (--help to see)
* `maker.sh` - a bash script to make communicating with eeprom.py easier during testing

Note that these EEPROM utilities require outside libraries which can be installed by entering the commands...

    pip install intelhex
    pip install crcmod

### Example usage...

To get a hex file for programming 103.5 Hot FM with other default parameters, enter..

    python eeprom.py -f 103.5

..which produces the output...

	:1000000000000050000F000000000000000040F160
	:1000100000000050000F000000000000000040F150
	:00000001FF 


    
