@echo Programming the SparkX Qwiic Step. If this looks incorrect, abort and retry.
@pause
:loop

@echo - 
@echo Flashing bootloader...
@avrdude -C avrdude.conf -v -patmega328p -cusbtiny -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m 

@timeout 1

@echo Flashing firmware...
@avrdude -C avrdude.conf -pm328p -cusbtiny -e -Uflash:w:Qwiic_Step.ino.standard.hex:i

@echo Done programming! Move on to the next board.
@pause

goto loop