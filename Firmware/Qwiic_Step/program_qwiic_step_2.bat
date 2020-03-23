@echo Programming the SparkX Qwiic Step. If this looks incorrect, abort and retry.
@pause
:loop

@echo Flashing firmware...
@avrdude -C avrdude.conf -pm328p -cusbtiny -e -Uflash:w:Qwiic_Step.ino.with_bootloader.standard.hex:i

@echo Done programming! Move on to the next board.
@pause

goto loop