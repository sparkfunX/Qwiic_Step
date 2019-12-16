#include <Wire.h>
#include <EEPROM.h>
// #include "nvm.h"    //DEBUG: don't know what this is yet...
#include "registers.h"
#include <AccelStepper.h>

#include <avr/sleep.h> //Needed for sleep_mode

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0100
#define DEFAULT_I2C_ADDRESS 0x52

//Hardware connections
//const uint8_t stp = A3;
//const uint8_t dir = 6;
//const uint8_t MS1 = 9;
//const uint8_t MS2 = 8;
//const uint8_t MS3 = 7;
const uint8_t stp = 2;
const uint8_t dir = 3;
const uint8_t Pin_MS1 = 4;
const uint8_t Pin_MS2 = 5;
const uint8_t Pin_MS3 = 6;
//const uint8_t addressPin = 11;
//const uint8_t curr_ref_pwm = 5;
//const uint8_t curr_sense = A6;   //DEBUG: right way to reference these pins?
//const uint8_t a49885_reset = A7; //DEBUG: might not work... is pin only ADC input?
//const uint8_t interrupt0 = 2;
//const uint8_t interrupt1 = 3;
//const uint8_t externalInterrupt = A1;

volatile memoryMap registerMap {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptEnable {requestedPosReached, limSwitchPressed}
  {0, 0, 0, 0, 0},     //motorStatus {isRunning, isAccelerating, isDecelerating, isLimited, isReached}
  {0, 0, 0, 0, 1},     //motorConfig {ms1, ms2, ms3, disableStepper, limitSwitch}
  {1, 1, 1, 1},        //motorControl {stop, runTo, runContinuous, sleep}
  0x00000000,          //currentPos
  0x00000000,          //distanceToGo
  0x00000000,          //move
  0x00000000,          //moveTo
  0x00000000,          //maxSpeed (float)
  0x00000000,          //acceleration (float)
  0x00000000,          //setSpeed (float)
  0x00,                //enableSetSpeedNVM
  0x0000,              //holdCurrent
  0x0000,              //runCurrent
  DEFAULT_I2C_ADDRESS, //i2cAddress
};

//This defines which of the registers are read-only (0) vs read-write (1)
memoryMap protectionMap = {
  0x00,               //id
  0x0000,             //firmware
  {1, 1},             //interruptEnable {requestedPosReached, limSwitchPressed}
  {1, 1, 1, 1, 1},    //motorStatus {isRunning, isAccelerating, isDecelerating, isLimited, isReached}
  {1, 1, 1, 1, 1},    //motorConfig {ms1, ms2, ms3, disableStepper, limitSwitch}
  {1, 1, 1, 1},       //motorControl {stop, runTo, runContinuous, sleep}
  0xFFFFFFFF,         //currentPos
  0x00000000,         //distanceToGo
  0xFFFFFFFF,         //move
  0xFFFFFFFF,         //moveTo
  0xFFFFFFFF,         //maxSpeed (float)
  0xFFFFFFFF,         //acceleration (float)
  0xFFFFFFFF,         //setSpeed (float)
  0xFF,               //enableSetSpeedNVM
  0xFFFF,             //holdCurrent
  0xFFFF,             //runCurrent
  0xFF,               //i2cAddress
};

//Cast 32bit address of the object registerMap with uint8_t so we can increment the pointer
uint8_t *registerPointer = (uint8_t *)&registerMap;
uint8_t *protectionPointer = (uint8_t *)&protectionMap;

volatile uint8_t registerNumber;  //Gets set when user writes an address. We then serve the spot the user requested.

volatile boolean updateFlag = false; //Goes true when we recieve new bytes from the users. Calls accelstepper functions with new registerMap values.

//Define a stepper and its pins
AccelStepper stepper(AccelStepper::DRIVER, stp, dir); //Stepper driver, 2 pins required

void setup(void)
{
  //Configure ATMega pins
  //Motor config pins are all outputs
  //    pinMode(stp, OUTPUT);
  //    pinMode(dir, OUTPUT);
  pinMode(Pin_MS1, OUTPUT);
  pinMode(Pin_MS2, OUTPUT);
  pinMode(Pin_MS3, OUTPUT);
  digitalWrite(Pin_MS1, LOW);
  digitalWrite(Pin_MS2, LOW);
  digitalWrite(Pin_MS3, LOW);
  
  //    pinMode(addressPin, INPUT_PULLUP);
  //    pinMode(curr_ref_pwm, OUTPUT);
  //    pinMode(curr_sense, INPUT);
  //    pinMode(a49885_reset, OUTPUT);

  //interrupt pins... all inputs?
  //    pinMode(interrupt0, INPUT_PULLUP);
  //    pinMode(interrupt1, INPUT_PULLUP);
  //    pinMode(externalInterrupt, INPUT_PULLUP);

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
  printState();
}

void loop(void) {
  if (updateFlag == true) {
    updateStepper();
    printState();
    //clear updateFlag
    updateFlag = false;
  }

  stepper.run();
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

void updateStepper(){
  //call accelstepper functions with the values in registerMap
  stepper.setCurrentPosition(registerMap.currentPos);
  stepper.move(registerMap.move);
  stepper.moveTo(registerMap.moveTo);
  stepper.setMaxSpeed(convertToFloat(registerMap.maxSpeed));
  stepper.setAcceleration(convertToFloat(registerMap.acceleration));
  stepper.setSpeed(convertToFloat(registerMap.setSpeed));

  //update the step mode by flipping pins MS1, MS2, MS3
  digitalWrite(Pin_MS1, registerMap.motorConfig.ms1);
  digitalWrite(Pin_MS2, registerMap.motorConfig.ms2);
  digitalWrite(Pin_MS3, registerMap.motorConfig.ms3);
}
void printState() {
  Serial.println();

  Serial.print("Register map id: 0x");
  Serial.println(*(registerPointer + 0), HEX);

  Serial.print("Firmware version: 0x");
  if (*(registerPointer + 2) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 2), HEX);
  if (*(registerPointer + 1) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 1), HEX);

  Serial.print("Interrupt enable: 0x");
  Serial.println(*(registerPointer + 3), HEX);

  Serial.print("Motor status: 0x");
  Serial.println(*(registerPointer + 4), HEX);

  Serial.print("Device config: 0x");
  Serial.println(*(registerPointer + 5), HEX);

  Serial.print("Device control: 0x");
  Serial.println(*(registerPointer + 6), HEX);

  Serial.print("Current position: 0x");
  if (*(registerPointer + 0xA) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0xA), HEX);
  if (*(registerPointer + 0x9) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x9), HEX);
  if (*(registerPointer + 0x8) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x8), HEX);
  if (*(registerPointer + 0x7) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x7), HEX);

  Serial.print("Distance to go: 0x");
  if (*(registerPointer + 0xE) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0xE), HEX);
  if (*(registerPointer + 0xD) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0xD), HEX);
  if (*(registerPointer + 0xC) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0xC), HEX);
  if (*(registerPointer + 0xB) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0xB), HEX);

  Serial.print("Move: 0x");
  if (*(registerPointer + 0x12) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x12), HEX);
  if (*(registerPointer + 0x11) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x11), HEX);
  if (*(registerPointer + 0x10) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x10), HEX);
  if (*(registerPointer + 0xF) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0xF), HEX);

  Serial.print("Move to: 0x");
  if (*(registerPointer + 0x16) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x16), HEX);
  if (*(registerPointer + 0x15) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x15), HEX);
  if (*(registerPointer + 0x14) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x14), HEX);
  if (*(registerPointer + 0x13) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x13), HEX);

  Serial.print("Max Speed: 0x");
  if (*(registerPointer + 0x1A) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1A), HEX);
  if (*(registerPointer + 0x19) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x19), HEX);
  if (*(registerPointer + 0x18) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x18), HEX);
  if (*(registerPointer + 0x17) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x17), HEX);

  Serial.print("Acceleration: 0x");
  if (*(registerPointer + 0x1E) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1E), HEX);
  if (*(registerPointer + 0x1D) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1D), HEX);
  if (*(registerPointer + 0x1C) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1C), HEX);
  if (*(registerPointer + 0x1B) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x1B), HEX);

  Serial.print("Set speed: 0x");
  if (*(registerPointer + 0x22) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x22), HEX);
  if (*(registerPointer + 0x21) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x21), HEX);
  if (*(registerPointer + 0x20) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x20), HEX);
  if (*(registerPointer + 0x1F) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x1F), HEX);

  Serial.print("Enable set speed: 0x");
  Serial.println(*(registerPointer + 0x23), HEX);

  Serial.print("Hold current: 0x");
  if (*(registerPointer + 0x25) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x25), HEX);
  if (*(registerPointer + 0x24) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x24), HEX);

  Serial.print("Run current: 0x");
  if (*(registerPointer + 0x27) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x27), HEX);
  if (*(registerPointer + 0x26) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x26), HEX);

  Serial.print("I2C address: 0x");
  Serial.println(*(registerPointer + 0x28), HEX);
}
