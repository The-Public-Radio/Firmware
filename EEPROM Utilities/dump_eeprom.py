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

def calc_freq(band, spacing, chan):
	b = base[band]
	s = step[spacing]
	return b + (float(chan) / s)

def dump_freq(info, crc, name):
	crc16 = Crc('crc-16')
	crc16.update(info)

	if (crc16.crcValue != crc):
		print "%s: CRC mismatch" % name
	else:
		(band, deemph, spacing, chan, vol) = unpack('<BBBHB8x', info)
		try:
			freq = calc_freq(band, spacing, chan)
			print "%s: Freq %.2f MHz" % (name, freq)
		except:
			print "%s: Cannot decode frequency" % name

		print "    Band: %02x" % band
		print "    Deemphasis: %02x" % deemph
		print "    Spacing: %02x" % spacing
		print "    Channel: %d" % chan
		print "    Volume: %02x" % vol

dump=IntelHex(source)

payload = list(unpack_from('<14sH14sH17sBB2s13s17s', dump.tobinstr()))

if eyecatcher in payload[9]:
	dump_freq(payload[0], payload[1], "Working")
	dump_freq(payload[2], payload[3], "Factory")

	print "SN: %s" % payload[4].strip('\000')
	print "WW: %d" % payload[5]
	print "YY: %d" % payload[6]
	print "Station: %s" % payload[7]
	print "Campaign: %s" % payload[8].strip('\000')
else:
	sys.exit(1)
