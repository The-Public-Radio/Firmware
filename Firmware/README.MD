# TPR Firmware notes

This firmware runs inside the ATTINY25/45/85 inside The Public Radio. 

More info at...
http://www.thepublicrad.io/

## TWI library
The TWI code here is custom written for this project. It differs from a a general purpose library in that...

1. **Internal pin pill-ups are used so no external resistors are needed.** This is a DFM optimization that theoretically could limit bandwidth, but in out case here (1) our ATTINY runs out of steam before then anyway, (2) we only need to send a dozen bytes, and only at startup and when changing channels, so performance doesn't matter anyway. 

1. **Only a single master is supported. ** We only have one anyway. 

1. **Only `write` is implemented, no `read`.** It is just a quirk that the config registers of FM chip used always default to `0`s and functionally the TPR never needs to read status info. There are some reserved bits in register 0x07 that say they must be read before being written, but the previous R/W code just smashed the reserved bits with `0`'s anyway and that seems to work fine. 

1. **Everything is bitbanged, so both clock and data could be moved to any IO pin. **