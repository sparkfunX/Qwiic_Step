#include <Wire.h>
#include <EEPROM.h>
#include "nvm.h"
#include "registers.h"

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0100
#define DEFAULT_I2C_ADDRESS 0x52

//Hardware connections

volatile memoryMap registerMap{
    DEVICE_ID,           //id
    FIRMWARE_VERSION,    //firmware
    {0, 0, 0},           //motorStatus {isRunning, isAccelerating, isDecelerating}
    {0, 0, 0, 0, 0},     //motorConfig {limitSwitch, disableStepper, microStep}
    {0, 0, 0, 0},        //motorControl {stop, runTo, runContinuous, sleep}
    0x00000000,          //currentPos
    0x00000000,          //moveTo
    0x00000000,          //maxSpeed
    0x00000000,          //acceleration
    0x00000000,          //setSpeed
    0x00,                //enableSetSpeedNVM
    0x0000,              //holdCurrent
    0x0000,              //runCurrent
    DEFAULT_I2C_ADDRESS, //i2cAddress
};

//This defines which of the registers are read-only (0) vs read-write (1)
memoryMap protectionMap = {
    0x00,            //id
    0x0000,          //firmware
    {1, 1, 1},       //motorStatus {isRunning, isAccelerating, isDecelerating}
    {1, 1, 1, 1, 1}, //motorConfig {limitSwitch, disableStepper, microStep}
    {1, 1, 1, 1},    //motorControl {stop, runTo, runContinuous, sleep}
    0xFFFFFFFF,      //currentPos
    0xFFFFFFFF,      //moveTo
    0xFFFFFFFF,      //maxSpeed
    0xFFFFFFFF,      //acceleration
    0xFFFFFFFF,      //setSpeed
    0xFF,            //enableSetSpeedNVM
    0xFFFF,          //holdCurrent
    0xFFFF,          //runCurrent
    0xFF,            //i2cAddress
}