@echo Programming the SparkFun Qwiic Step. If this looks incorrect, abort and retry.
@pause
:loop

@echo -
@echo Flashing firmware...

rem The -B1 option reduces the bitclock period (1us = 1MHz SPI), decreasing programming time
rem May increase verification errors

@echo Flashing bootloader and  firmware...
@avrdude -C avrdude.conf -v -V -patmega328p -carduino -PCOM5 -b115200 -D -Uflash:w:Qwiic_Step-v11.hex:i 

@echo Done programming! Move on to the next board.
@pause

goto loop