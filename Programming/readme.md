Programming Qwiic Step
===========================================================

The Qwiic Step uses the ATmega328P with the standard Arduino/STK500 bootloader. New firmware can be loaded with a USB to serial connection and a 6-pin header.

![Programming Qwiic Step via 6-pin Serial](https://github.com/sparkfunX/Qwiic_Step/blob/master/Programming/Images/Qwiic%20Step%20Bootloading%20-%201.jpg?raw=true)

1) Solder a 6-pin [right angle](https://www.sparkfun.com/products/553) (or [straight](https://www.sparkfun.com/products/116)) male header onto the 'Debug' footprint.
2) Connect a [USB to Serial](https://www.sparkfun.com/products/15096) adapter along with a [USB cable](https://www.sparkfun.com/products/15425) to a computer and verify the new COM port is displayed. For information about drivers or COM port locating, see [here](https://learn.sparkfun.com/tutorials/sparkfun-serial-basic-ch340c-hookup-guide).
3) Download the contents of the 'Programming' folder so that you have **avrdude.exe**, **avrdude.conf**, **program_qwiic_step.bat**, and **Qwiic-Step-vXX.hex** in a folder.
4) Modify 'program_qwiic_step.bat' to use the COM port that your computer uses (default is COM5).
5) Open a command line, navigate to 'program_qwiic_step.bat' and run. 
6) Press enter to begin programming.

![Output from avrdude](https://github.com/sparkfunX/Qwiic_Step/blob/master/Programming/Images/Qwiic%20Step%20Bootloading.jpg?raw=true)
