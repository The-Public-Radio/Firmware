## EEPROM Utilities

* `dump_eeprom.py` - take intel hex dump of eeprom contents, decode and print in <sort of> human readable format.
* `eeprom.py` - create intel hex suitable for programming into eeprom, according to various comamnd line flags (--help to see)
* `maker.sh` - a bash script to make communicating with eeprom.py easier during testing

Note that these EEPROM utilities require the [IntelHex library](https://github.com/bialix/intelhex) which can be installed by entering...

    pip install intelhex
