#!/usr/bin/python

#
# Utility to create eeprom images for Public Radio.
#
# EEPROM consists of 3 areas:
#
# 1. running config. In practical terms the only attribute that changes
#    in normal use is the channel. However, we store all the soft attributes
#    (band, channel spacing, de-emphasis, volume) in addition to the channel
#    and protect the structure via CRC16. If CRC16 does not match, then entry
#    is considererd corrupt, and factory values will be used.
# 2. Factory config. Never written by the firmware. Used if the running config
#    is corrupted (e.g. written with a low/failing battery), or if the user
#    requests a factory reset.
# 3. Manufacturing data. Never accessed by firmware. Contains serial number,
#    ISO week & year of manufacture, production test fixture identifier and
#    field for any associated campaign (e.g. pledge drive, promotion, etc)
#
# Defaults to US settings unless otherwise specified.
#
# TODO: CLI parsing to set these.
#       Make this pythonic, not procedural, if anyone cares.
#
#
# NOTE: you will need to install the crcmod package from here:
#       https://pypi.python.org/pypi/crcmod
#
# Can be programmed either with the firmware, or independantly, examples:
#  - With the firmware:
#       avrdude -qq -P usb -c avrisp2 -p attiny45 -B 15 -e -U flash:w:pr.hex -U eeprom:w:XYZ-1234.hex
#  - Without the firmware:
#       avrdude -qq -P usb -c avrisp2 -p attiny45 -B 15 -U eeprom:w:XYZ-1236.hex
#    Note the absence of the "-e", or erase option in the second form.
#  

from intelhex import IntelHex
from datetime import date
from struct import pack
from math import modf
from crcmod.predefined import Crc
from getopt import getopt, GetoptError
import sys

#
# All the following to be able to be set with getopt.
# These defaults will do for testing purposes.
#
outfile=sys.stdout


# manufacturing data
#
manuf=False
sn=""
ts=""
campaign=""
eyecatcher='The Public Radio'

# tuning info
#
freq=0.0
band=0			# US 87.5 - 108
demphasis=0		# US 75uS
spacing=0		# US 200KHz
volume=0x0f		# max (0 dBFS)

#
# Create a manufacturing record.
#
def manuf_record(sn, ts, campaign):
	ww = int(date.today().strftime('%V'))
	yy = int(date.today().strftime('%g'))
	return pack('17sBB2s13s17s', sn[:16], ww, yy, ts[:2], campaign[:12], eyecatcher)

option_list='MS:T:C:f:b:d:s:v:'

def usage():
	print r'''Usage: eeprom -f <freq> [-b <n>] [-d <n>] [-s <n>] [-v <n>]
		[-m [-S <sn>] [-T <ts>] [-C <campaign>]]
		-f <n>	Specify the frequency in MHz
		-b <n>	Specify the band (0=87.5-108, 1=76-108, 2=76-90,
			default = 0)
		-d <n>	Specify the demphasis (0=75uS, 1=50uS, default = 0)
		-s <n>	Specify the channel spacing (0=200KHz, 1=100KHz,
			2=50KHz, default = 0)
		-v <n>	Specify max volume (0-15, default 15)
		-M	Append a manufacturing record (only to be used in
			production test fixtures)
		-S <sn>	Specify a serial number (<= 16 characters in length)
		-T <ts> Specify a test station identifier (<= 2 characters)
		-C <c>	Specify a campaign (<= 12 characters in length)'''
	sys.exit(1)

try:
	opts, args = getopt(sys.argv[1:], option_list)
except GetoptError as err:
	print str(err)
	usage()

for o, a in opts:
	if o == '-f':
		freq = float(a)
	elif o == '-b':
		band = int(a)
	elif o == '-d':
		demphasis = int(a)
	elif o == '-s':
		spacing = int(a)
	elif o == '-v':
		volume = int(a)
	elif o == '-M':
		manuf = True
	elif o == '-S':
		sn = a
	elif o == '-T':
		ts = a
	elif o == '-C':
		campaign = a
	else:
		usage()

if band < 0 or band > 2 or demphasis < 0 or demphasis > 1 or spacing < 0 or spacing > 2 or volume < 0 or volume > 15:
	usage()

if freq < 76.0 or freq > 108.0:
	print "Frequency unspecified or out of bounds"
	usage()

#
# calculate channel # based on freq, band & channel spacing.
# Note that we force the floating point arithmetic to round
# to a reasonable number of digits in order to avoid daft problems.

base = {0: 87.5, 1: 76, 2: 76}
step = {0: 5, 1: 10, 2: 20}

try:
	chan = round((freq - base[band]) * step[spacing], 4)
except:
	print "Illegal band and/or spacing"
	sys.exit(1)

if modf(chan)[0] != 0.0:
	print "Illegal freq/spacing combo"
	sys.exit(2)

chan = int(chan)

# First create the data without the checksum, note that we specify
# little-endianness for multi-byte values.

t = pack('<BBBHB8x', band, demphasis, spacing, chan, volume)

# Calculate and append a crc-16 checksum
crc16 = Crc('crc-16')
crc16.update(t)

t = t + pack('<H', crc16.crcValue)

#
# Simply create a hex file with the concatenation of two tuning structures (t)
# and optionally one manufacturing structure, then write the result.
#
eeprom = t + t

if manuf:
	eeprom = eeprom + manuf_record(sn, ts, campaign)

hexfile = IntelHex()
hexfile.puts(0, eeprom)
hexfile.write_hex_file(outfile)
