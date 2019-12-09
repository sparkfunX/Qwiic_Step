#include <Wire.h>
#include <EEPROM.h>
// #include "nvm.h"    //DEBUG: don't know what this is yet...
#include "registers.h"

#include <avr/sleep.h> //Needed for sleep_mode

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0100
#define DEFAULT_I2C_ADDRESS 0x52

//Hardware connections
const uint8_t stp = A3;
const uint8_t dir = 6;
const uint8_t MS1 = 9;
const uint8_t MS2 = 8;
const uint8_t MS3 = 7;
const uint8_t addressPin = 11;
const uint8_t curr_ref_pwm = 5;
const uint8_t curr_sense = A6;   //DEBUG: right way to reference these pins?
const uint8_t a49885_reset = A7; //DEBUG: might not work... is pin only ADC input?
const uint8_t interrupt0 = 2;
const uint8_t interrupt1 = 3;
const uint8_t externalInterrupt = A1;

volatile memoryMap registerMap{
    DEVICE_ID,           //id
    FIRMWARE_VERSION,    //firmware
    0x00,                //interruptEnable
    {1, 1, 1},           //motorStatus {isRunning, isAccelerating, isDecelerating}
    {1, 1, 1, 1, 1},     //motorConfig {ms1, ms2, ms3, disableStepper, limitSwitch}
    {1, 1, 1, 1},        //motorControl {stop, runTo, runContinuous, sleep}
    0x00000000,          //currentPos
    0x00000000,          //moveTo
    0x00000000,          //maxSpeed
    0x12345678,          //acceleration
    0x00000000,          //setSpeed
    0x00,                //enableSetSpeedNVM
    0x0000,              //holdCurrent
    0x0000,              //runCurrent
    0x00000000,          //tempEmpty0
    0x00000000,          //tempEmpty1
    DEFAULT_I2C_ADDRESS, //i2cAddress
};

//This defines which of the registers are read-only (0) vs read-write (1)
memoryMap protectionMap = {
    0x00,               //id
    0x0000,             //firmware
    0xFF,               //interruptEnable      
    {1, 1, 1},          //motorStatus {isRunning, isAccelerating, isDecelerating}
    {1, 1, 1, 1, 1},    //motorConfig {limitSwitch, disableStepper, microStep}
    {1, 1, 1, 1},       //motorControl {stop, runTo, runContinuous, sleep}
    0xFFFFFFFF,         //currentPos
    0xFFFFFFFF,         //moveTo
    0xFFFFFFFF,         //maxSpeed
    0xFFFFFFFF,         //acceleration
    0xFFFFFFFF,         //setSpeed
    0xFF,               //enableSetSpeedNVM
    0xFFFF,             //holdCurrent
    0xFFFF,             //runCurrent
    0x00000000,         //tempEmpty0
    0x00000000,         //tempEmpty1
    0xFF,               //i2cAddress
};

//Cast 32bit address of the object registerMap with uint8_t so we can increment the pointer
uint8_t *registerPointer = (uint8_t *)&registerMap;
uint8_t *protectionPointer = (uint8_t *)&protectionMap;

volatile uint8_t registerNumber;  //Gets set when user writes an address. We then serve the spot the user requested.

//DEBUG: not sure how I'm using this yet...
//volatile boolean updateFlag = true; //Goes true when we recieve new bytes from the users. Causes things to update in main loop.

void setup(void)
{
    //Configure ATMega pins
    //Motor config pins are all outputs
    pinMode(stp, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(MS3, OUTPUT);

    pinMode(addressPin, INPUT_PULLUP);
    pinMode(curr_ref_pwm, OUTPUT);
    pinMode(curr_sense, INPUT);
    pinMode(a49885_reset, OUTPUT);

    //interrupt pins... all inputs?
    pinMode(interrupt0, INPUT_PULLUP);
    pinMode(interrupt1, INPUT_PULLUP);
    pinMode(externalInterrupt, INPUT_PULLUP);

    //DEBUG: do I need to disable ADC & Brown-out detect?!

    //Power down various bits of hardware to lower power usage
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    //Print info to Serial Monitor
    Serial.begin(115200);
    Serial.println("Qwiic Button");
    Serial.print("Address: 0x");
    Serial.println(registerMap.i2cAddress, HEX);
    Serial.print("Device ID: 0x");
    Serial.println(registerMap.id, HEX);

    //Determine the I2C address to be using and listen on I2C bus
    startI2C(&registerMap);
}

void loop(void){
 
}

void startI2C(memoryMap *map)
{
  uint8_t address;
  
  //DEBUG: need to handle address jumper here. What is it supposed to do?

  //check if the address stored in memoryMap is valid
  if (map->i2cAddress > 0x07 && map->i2cAddress < 0x78)
    address = map->i2cAddress;
  //if the value is illegal, default to the default I2C address for our platform
  else
    address = DEFAULT_I2C_ADDRESS;

  //save new address to the register map
  map->i2cAddress = address;

  Wire.end();
  Wire.begin(address);  //rejoin the I2C bus on new address

  //Connect receive and request events to the interrupts
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}
