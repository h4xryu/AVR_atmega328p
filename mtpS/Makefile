baud=115200
ProjectName = mtpS
src=~/avr_src/SimulIDE_1.0.0-SR1_Lin64/$(ProjectName)/$(ProjectName)
avrType=atmega328p
avrFreq=16000000 # 16MHz for accurate baudrate timing
programmerDev=/dev/ttyACM0
programmerType=arduino
CC=gcc
CPP=g++
CFILES=$(wildcard ./*.c)
CPPFILES=$(wildcard ./*.cpp)
OBJ=$(CFILES:.c=.o) $(CPPFILES:.cpp=.o)
hexFile = $(ProjectName).flash.hex

cflags=-g -DF_CPU=$(avrFreq) -Wall -Os -Wextra

memoryTypes=calibration eeprom efuse flash fuse hfuse lfuse lock signature application apptable boot prodsig usersig

.PHONY: backup clean disassemble dumpelf edit eeprom elf flash fuses help hex makefile object program

help:
	@echo 'backup       Read all kavr nown memory types from controller and write it into a file. Available memory types: $(memoryTypes)'
	@echo 'clean        Delete automatically created files.'
	@echo 'disassemble  Compile source code, then disassemble object file to mnemonics.'
	@echo 'dumpelf      Dump the contents of the .elf file. Useful for information purposes only.'
	@echo 'edit     Edit the .c source file.'
	@echo 'eeprom       Extract EEPROM data from .elf file and program the device with it.'
	@echo 'elf      Create $(src).elf'
	@echo 'flash        Program $(src).hex to controller flash memory.'
	@echo 'fuses        Extract FUSES data from .elf file and program the device with it.'
	@echo 'help     Show this text.'
	@echo 'hex      Create all hex files for flash, eeprom and fuses.'
	@echo 'object       Create $(src).o'
	@echo 'program      Do all programming to controller.'

edit:
	vim $(src).c

makefile:
	vim Makefile

default: $(OBJ)
	avr-ar rcs core.a $^

%.o : %.c
	$(CC) $< $(CFLAGS) -c -o $@

%.o : %.cpp
	$(CPP) $< $(CPPFLAGS) -c -o $@

#all: object elf hex

clean: 
	rm $(src).elf $(src).eeprom.hex $(src).fuses.hex $(src).lfuse.hex $(src).hfuse.hex $(src).efuse.hex $(src).flash.hex $(src).o
	date

object:
	avr-gcc $(cflags) -mmcu=$(avrType) -Wa,-ahlmns=$(src).lst -c -o $(src).o $(src).c 

elf: object
	avr-gcc $(cflags) -mmcu=$(avrType) -o $(src).elf $(src).o
	chmod a-x $(src).elf 2>&1

hex:    elf
	avr-objcopy -j .text -j .data -O ihex $(src).elf $(src).flash.hex
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $(src).elf $(src).eeprom.hex
	avr-objcopy -j .fuse -O ihex $(src).elf $(src).fuses.hex --change-section-lma .fuse=0
	#srec_cat $(src).fuses.hex -Intel -crop 0x00 0x01 -offset  0x00 -O $(src).lfuse.hex -Intel
	#srec_cat $(src).fuses.hex -Intel -crop 0x01 0x02 -offset -0x01 -O $(src).hfuse.hex -Intel
	#srec_cat $(src).fuses.hex -Intel -crop 0x02 0x03 -offset -0x02 -O $(src).efuse.hex -Intel
	
	#if you got an error : sudo apt install srecord
disassemble: elf
	avr-objdump -s -j .fuse $(src).elf
	avr-objdump -C -d $(src).elf 2>&1

eeprom: hex
	avrdude -p$(avrType) -c$(programmerType) -P$(programmerDev) -b$(baud) -v -U eeprom:w:$(src).eeprom.hex
	date

fuses: hex
	avrdude -p$(avrType) -c$(programmerType) -P$(programmerDev) -b$(baud) -v -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m

	date

dumpelf: elf
	avr-objdump -s -h $(src).elf

program: flash eeprom fuses

flash: hex
	avrdude -p$(avrType) -P$(programmerDev)  -c $(programmerType) -U flash:w:$(hexFile):i
	date
	
arduino-flash: hex
	avrdude -p$(avrType) -P$(programmerDev)  -c $(programmerType) -U flash:w:$(hexFile):i
	avrdude
	

backup:
	@for memory in $(memoryTypes); do \
        avrdude -p $(avrType) -c$(programmerType) -P$(programmerDev) -b$(baud) -v -U $$memory:r:./$(avrType).$$memory.hex:i; \
    done
