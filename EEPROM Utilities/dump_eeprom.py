#!/usr/bin/python

#
# Utility to decode the contents of EEPROM read from a radio
#
# TODO:
#	- add command line args via getopt to tailor output and input
#	- decode fields into human readable form

from intelhex import IntelHex
from struct import unpack_from, unpack
from crcmod.predefined import Crc
import sys

eyecatcher='The Public Radio'

source = sys.stdin

base = {0: 87.5, 1: 76, 2: 76}
step = {0: 5, 1: 10, 2: 20}

# These are copy.pasted right form the data sheet

bandstr = {0:"87.5-108 MHz (USA, Europe)", 1:"76=108 MHz (Japan wide band)", 2:"76-90 MHz (Japan)", 3:"Reserved"}
spacestr= {0:"200 kHz (USA, Australia) ", 1:"100 kHz (Europe, Japan)", 2:" 50 kHz"}
dempstr = {0:"75 us. Used in USA", 1:"50 us. Used in Europe, Australia, Japan"}

def calc_freq(band, spacing, chan):
	b = base[band]
	s = step[spacing]
	return b + (float(chan) / s)

def dump_freq(info, crc):

	crc16 = Crc('crc-16')
	crc16.update(info)

	if (crc16.crcValue != crc):
		print "%s: CRC mismatch" % name
	else:
		(band, deemph, spacing, chan, vol) = unpack('<BBBHB8x', info)

		print "    Band:        %02x = %s" % (band,   bandstr[band])
		print "    Deemphasis:  %02x = %s" % (deemph, dempstr[deemph])
		print "    Spacing:     %02x = %s" % (spacing,spacestr[spacing])
		print "    Channel:    %.3d" % chan
		print "    Volume:      %02d" % vol

		try:
			freq = calc_freq(band, spacing, chan)
			print "    Freqency=%03.02f Mhz (calculated)" % freq
			
		except:	
			print "%s: Cannot decode frequency" % name
		
		
dump=IntelHex(source)

# Unpack fields
# 14 - String - Working station
#  2 = HEX    - Working CRC
# 14 - String - Factory station
#  2 = HEX    - Factory CRC

#payload = list(unpack_from('<14sH14sH17sBB2s13s17s', dump.tobinstr()))
payload = list(unpack_from('<14sH', dump.tobinstr()))

#if not (eyecatcher in payload[9]):
	#print "NOTE:Eyecatcher text string not found"


print "---Working"
dump_freq(payload[0], payload[1])
print "---Factory"
dump_freq(payload[2], payload[3])

print "SN: %s" % payload[4].strip('\000')
print "WW: %d" % payload[5]
print "YY: %d" % payload[6]
print "Station: %s" % payload[7]
print "Campaign: %s" % payload[8].strip('\000')
