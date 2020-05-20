avr-objcopy.exe -I ihex ATA5702_eeprom.eep -O binary eeprom.bin
bin2h.exe eeprom.bin ATA5702_eeprom.h eeprom

avr-objcopy.exe -I ihex ATA5702_flash.hex -O binary flash.bin
bin2h.exe flash.bin ATA5702_flash.h flash

del eeprom.bin
del flash.bin