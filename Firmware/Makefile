#
# Select correct AVR part. Production units use ATTiny45.
#
PART=attiny45

OBJS=main.o USI_TWI_Master.o VccADC.o

OPTFLAGS=-Os

AVR_GCC=avr-gcc
AVR_CCFLAGS=-mmcu=$(PART) -Wall $(OPTFLAGS) -g --std=c99
AVR_LDFLAGS=-mmcu=$(PART) -g

AVR_OBJDUMP=avr-objdump

AVR_OBJCOPY=avr-objcopy
AVR_OBJCOPY_FLAGS=-j .text -j .data -O ihex

AVR_SIZE=avr-size

#PROGRAMMER=jtag2isp
PROGRAMMER=avrisp2

AVRDUDE=avrdude
AVRDUDE_FLAGS=-qq -P usb -c $(PROGRAMMER) -p $(PART)  -B 15

.PHONEY: all program read_fuses write_fuses clean reset clobber

all: pr.hex

%.o: %.c
	$(AVR_GCC) $(AVR_CCFLAGS) -c $<

%.lst: %.elf
	$(AVR_OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(AVR_OBJCOPY) $(AVR_OBJCOPY_FLAGS) $< $@

pr.elf: $(OBJS)
	$(AVR_GCC) $(AVR_LDFLAGS) -o $@ $(OBJS)

program: pr.hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$<

read_fuses:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h

write_fuses:
	false		# Fail until we know correct fuse byte values.

reset:
	$(AVRDUDE) $(AVRDUDE_FLAGS)

clean:
	rm -f $(OBJS)

clobber: clean
	rm -f pr.elf pr.lst pr.hex

USI_TWI_Master.o: USI_TWI_Master.c USI_TWI_Master.h
VccADC.o: VccADC.c VccADC.h
