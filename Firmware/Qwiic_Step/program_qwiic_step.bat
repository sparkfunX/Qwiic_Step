@echo Programming the SparkX Qwiic Step. If this looks incorrect, abort and retry.
@pause
:loop

@echo Flashing bootloader and  firmware...
@avrdude -C avrdude.conf -pm328p -cusbtiny -e -Uefuse:w:0x05:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m -Uflash:w:Qwiic_Step.ino.with_bootloader.standard.hex:i

@echo Done programming! Move on to the next board.
@pause

goto loop