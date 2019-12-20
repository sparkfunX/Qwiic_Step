#include <Wire.h>
#include <EEPROM.h>
#include "nvm.h"    //DEBUG: don't know what this is yet...
#include "registers.h"
#include <AccelStepper.h>

#include <avr/sleep.h> //Needed for sleep_mode

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0100
#define DEFAULT_I2C_ADDRESS 0x52
#define TEST_SETUP 1

//Hardware connections
//test set-up pins
#if defined(TEST_SETUP)
const uint8_t stp = 7;
const uint8_t dir = 8;
const uint8_t pin_MS1 = 4;
const uint8_t pin_MS2 = 5;
const uint8_t pin_MS3 = 6;
const uint8_t pin_interrupt0 = 2;
const uint8_t pin_interrupt1 = 3;
const uint8_t pin_externalInterrupt = A1;
#elif
const uint8_t stp = A3;
const uint8_t dir = 6;
const uint8_t pin_MS1 = 9;
const uint8_t pin_MS2 = 8;
const uint8_t pin_MS3 = 7;
const uint8_t addressPin = 11;
const uint8_t curr_ref_pwm = 5;
const uint8_t curr_sense = A6;   //DEBUG: right way to reference these pins?
const uint8_t a49885_reset = A7; //DEBUG: might not work... is pin only ADC input?
const uint8_t pin_interrupt0 = 2;   //Limit Switch
const uint8_t pin_interrupt1 = 3;   //E-Stop
const uint8_t pin_externalInterrupt = A1;
#endif

volatile memoryMap registerMap {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptEnable {requestedPosReached, limSwitchPressed}
  {0, 0, 0, 0, 0},     //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {0, 0, 0, 0, 1},     //motorConfig {ms1, ms2, ms3, disableMotorPositionReached, stopOnLimitSwitchPress}
  {1, 1, 1, 1},        //motorControl {stop, runTo, runContinuous, sleep}
  0x00000000,          //currentPos
  0x00000000,          //distanceToGo
  0x00000000,          //move
  0x00,                //enableMoveNVM
  0x00000000,          //moveTo
  0x00000000,          //maxSpeed (float)
  0x00000000,          //acceleration (float)
  0x00000000,          //speed (float)
  0x00,                //enableSpeedNVM
  0x0000,              //holdCurrent
  0x0000,              //runCurrent
  DEFAULT_I2C_ADDRESS, //i2cAddress
};

//this memory map holds temporary "old" values so we don't call accelstepper functions an unnecessary amount of times
volatile memoryMap registerMapOld {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptEnable {requestedPosReached, limSwitchPressed}
  {0, 0, 0, 0, 0},     //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {0, 0, 0, 0, 1},     //motorConfig {ms1, ms2, ms3, disableMotorPositionReached, stopOnLimitSwitchPress}
  {1, 1, 1, 1},        //motorControl {stop, runTo, runContinuous, sleep}
  0x00000000,          //currentPos
  0x00000000,          //distanceToGo
  0x00000000,          //move
  0x00,                //enableMoveNVM
  0x00000000,          //moveTo
  0x00000000,          //maxSpeed (float)
  0x00000000,          //acceleration (float)
  0x00000000,          //speed (float)
  0x00,                //enableSpeedNVM
  0x0000,              //holdCurrent
  0x0000,              //runCurrent
  DEFAULT_I2C_ADDRESS, //i2cAddress
};

//This defines which of the registers are read-only (0) vs read-write (1)
memoryMap protectionMap = {
  0x00,               //id
  0x0000,             //firmware
  {1, 1},             //interruptEnable {requestedPosReached, limSwitchPressed}
  {1, 1, 1, 1, 1},    //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {1, 1, 1, 1, 1},    //motorConfig {ms1, ms2, ms3, disableMotorPositionReached, stopOnLimitSwitchPress}
  {1, 1, 1, 1},       //motorControl {stop, runTo, runContinuous, sleep}
  0xFFFFFFFF,         //currentPos
  0x00000000,         //distanceToGo
  0xFFFFFFFF,         //move
  0xFF,               //enableMoveNVM
  0xFFFFFFFF,         //moveTo
  0xFFFFFFFF,         //maxSpeed (float)
  0xFFFFFFFF,         //acceleration (float)
  0xFFFFFFFF,         //speed (float)
  0xFF,               //enableSpeedNVM
  0xFFFF,             //holdCurrent
  0xFFFF,             //runCurrent
  0xFF,               //i2cAddress
};

//Cast 32bit address of the object registerMap with uint8_t so we can increment the pointer
uint8_t *registerPointer = (uint8_t *)&registerMap;
uint8_t *protectionPointer = (uint8_t *)&protectionMap;

volatile uint8_t registerNumber;  //Gets set when user writes an address. We then serve the spot the user requested.

volatile boolean updateFlag = false; //Goes true when we recieve new bytes from the users. Calls accelstepper functions with new registerMap values.

//temp variable to hold previous speed of motor to calculate its acceleration status
float previousSpeed;

//Define a stepper and its pins
AccelStepper stepper(AccelStepper::DRIVER, stp, dir); //Stepper driver, 2 pins required

void setup(void)
{
  //Configure ATMega pins
  //Motor config pins are all outputs
  pinMode(pin_MS1, OUTPUT);
  pinMode(pin_MS2, OUTPUT);
  pinMode(pin_MS3, OUTPUT);
  //Default to full step mode
  digitalWrite(pin_MS1, LOW);
  digitalWrite(pin_MS2, LOW);
  digitalWrite(pin_MS3, LOW);
  
  //    pinMode(addressPin, INPUT_PULLUP);
  //    pinMode(curr_ref_pwm, OUTPUT);
  //    pinMode(curr_sense, INPUT);
  //    pinMode(a49885_reset, OUTPUT);

  //interrupt pins... all inputs?
  pinMode(pin_interrupt0, INPUT_PULLUP);
  pinMode(pin_interrupt1, INPUT_PULLUP);
  pinMode(pin_externalInterrupt, INPUT_PULLUP);

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

  readSystemSettings(); //Load all system settings from EEPROM
  
  //temporary variable to calculate acceleration
  previousSpeed = stepper.speed();

  //attach state-change of interrupt pins to corresponding ISRs
  attachInterrupt(digitalPinToInterrupt(pin_interrupt0), limitSwitchTriggered, LOW); 
  attachInterrupt(digitalPinToInterrupt(pin_interrupt1), eStopTriggered, LOW);
  
  //Determine the I2C address to be using and listen on I2C bus
  startI2C(&registerMap);
  printState();
}

void loop(void) {
  //update motor status
//  updateMotorStatus();
  
  //update external interrupt pin output
  //if an interrupt is triggered
  if ((registerMap.interruptEnable.requestedPosReachedEnable && registerMap.motorStatus.isReached) ||
      (registerMap.interruptEnable.limSwitchPressedEnable && registerMap.motorStatus.isLimited))
      {
        //pull pin low
        pinMode(pin_externalInterrupt, OUTPUT);
        digitalWrite(pin_externalInterrupt, LOW);
      }
      else
      {
        //go to high-impedance mode
        pinMode(pin_externalInterrupt, INPUT_PULLUP);
      }

  //Update accelstepper functions
  if (updateFlag == true) {
    updateStepper();
    printState();

    //Record anything new to EEPROM
    recordSystemSettings();
    //clear updateFlag
    updateFlag = false;
  }

  if (registerMap.motorStatus.eStopped == false)
  {
    stepper.run();
  }
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
  if (registerMapOld.maxSpeed != registerMap.maxSpeed){
    stepper.setMaxSpeed(convertToFloat(registerMap.maxSpeed));
    registerMapOld.maxSpeed = registerMap.maxSpeed;
  }
  if (registerMapOld.speed != registerMap.speed){
    stepper.setSpeed(convertToFloat(registerMap.speed));
    registerMapOld.speed = registerMap.speed; 
  }
  if (registerMapOld.acceleration != registerMap.acceleration){
    stepper.setAcceleration(convertToFloat(registerMap.acceleration));
    registerMapOld.acceleration = registerMap.acceleration;
  }
  stepper.move(registerMap.move);
  stepper.moveTo(registerMap.moveTo);

  //update the step mode by flipping pins MS1, MS2, MS3
  digitalWrite(pin_MS1, registerMap.motorConfig.ms1);
  digitalWrite(pin_MS2, registerMap.motorConfig.ms2);
  digitalWrite(pin_MS3, registerMap.motorConfig.ms3);
}

void recordSystemSettings()
{
  EEPROM.put(0x00, registerMap);
}

void readSystemSettings()
{
  //Read interrupt enable
  EEPROM.get(LOCATION_INTERRUPTENABLE, registerMap.interruptEnable.byteWrapped);
  if (registerMap.interruptEnable.byteWrapped == 0xFF){
    //default to no interrupts enabled
    registerMap.interruptEnable.byteWrapped = 0x00;
    EEPROM.put(LOCATION_INTERRUPTENABLE, registerMap.interruptEnable.byteWrapped);
  }

  //Read device configuration
  EEPROM.get(LOCATION_DEVICECONFIG, registerMap.motorConfig.byteWrapped);
  if (registerMap.motorConfig.byteWrapped == 0xFF){
    //default to stop on limit switch press
    registerMap.motorConfig.byteWrapped = 0x10;
    EEPROM.put(LOCATION_DEVICECONFIG, registerMap.motorConfig.byteWrapped);
  }

  //Read move if user has said to
  EEPROM.get(LOCATION_MOVE_ENABLENVM, registerMap.enableMoveNVM);
  if (registerMap.enableMoveNVM == 0x59){
    EEPROM.get(LOCATION_MOVE, registerMap.move);
    if (registerMap.move == 0xFFFFFFFF){
      //default to move of 0
      registerMap.move = 0x00000000;
      EEPROM.put(LOCATION_MOVE, registerMap.move);
    }
  }

  //Read max speed
  EEPROM.get(LOCATION_MAXSPEED, registerMap.maxSpeed);
  if (registerMap.maxSpeed == 0xFFFFFFFF){
    //default to max speed of 0
    registerMap.maxSpeed = 0x000000000;  
    EEPROM.put(LOCATION_MAXSPEED, registerMap.maxSpeed);
  }

  //Read acceleration
  EEPROM.get(LOCATION_ACCELERATION, registerMap.acceleration);
  if (registerMap.acceleration == 0xFFFFFFFF){
    //default acceleration to 0
    registerMap.acceleration = 0x00000000;
    EEPROM.put(LOCATION_ACCELERATION, registerMap.acceleration);
  }

  //Read speed if user has said to 
  EEPROM.get(LOCATION_SPEED_ENABLENVM, registerMap.enableSpeedNVM);
  if (registerMap.enableSpeedNVM == 0x47){
    EEPROM.get(LOCATION_SPEED, registerMap.speed);
    if (registerMap.speed == 0xFFFFFFFF){
      //default speed to 0
      registerMap.speed = 0x00000000;
      EEPROM.put(LOCATION_SPEED, registerMap.speed);
    }
  }

  //Read hold current
  EEPROM.get(LOCATION_HOLDCURRENT, registerMap.holdCurrent);
  if (registerMap.holdCurrent == 0xFF){
    //DEBUGGING: default to ??
    EEPROM.put(LOCATION_HOLDCURRENT, registerMap.holdCurrent);
  }

  //Read run current
  EEPROM.get(LOCATION_RUNCURRENT, registerMap.runCurrent);
  if (registerMap.runCurrent == 0xFF){
    //DEBUGGING: default to ??
    EEPROM.put(LOCATION_RUNCURRENT, registerMap.runCurrent);
  }

  //Read I2C address
  EEPROM.get(LOCATION_I2C_ADDRESS, registerMap.i2cAddress);
  if (registerMap.i2cAddress < 0x08 || registerMap.i2cAddress > 0x77){
    registerMap.i2cAddress = DEFAULT_I2C_ADDRESS;
    EEPROM.put(LOCATION_I2C_ADDRESS, registerMap.i2cAddress);
  }
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

  Serial.print("Enable move NVM: 0x");
  Serial.println(*(registerPointer + 0x13), HEX);
  
  Serial.print("Move to: 0x");
  if (*(registerPointer + 0x17) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x17), HEX);
  if (*(registerPointer + 0x16) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x16), HEX);
  if (*(registerPointer + 0x15) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x15), HEX);
  if (*(registerPointer + 0x14) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x14), HEX);

  Serial.print("Max Speed: 0x");
  if (*(registerPointer + 0x1B) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1B), HEX);
  if (*(registerPointer + 0x1A) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1A), HEX);
  if (*(registerPointer + 0x19) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x19), HEX);
  if (*(registerPointer + 0x18) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x18), HEX);

  Serial.print("Acceleration: 0x");
  if (*(registerPointer + 0x1F) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1F), HEX);
  if (*(registerPointer + 0x1E) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1E), HEX);
  if (*(registerPointer + 0x1D) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x1D), HEX);
  if (*(registerPointer + 0x1C) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x1C), HEX);

  Serial.print("Speed: 0x");
  if (*(registerPointer + 0x23) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x23), HEX);
  if (*(registerPointer + 0x22) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x22), HEX);
  if (*(registerPointer + 0x21) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x21), HEX);
  if (*(registerPointer + 0x20) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x20), HEX);

  Serial.print("Enable set speed: 0x");
  Serial.println(*(registerPointer + 0x24), HEX);

  Serial.print("Hold current: 0x");
  if (*(registerPointer + 0x26) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x26), HEX);
  if (*(registerPointer + 0x25) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x25), HEX);

  Serial.print("Run current: 0x");
  if (*(registerPointer + 0x28) < 0x10) Serial.print("0");
  Serial.print(*(registerPointer + 0x28), HEX);
  if (*(registerPointer + 0x27) < 0x10) Serial.print("0");
  Serial.println(*(registerPointer + 0x27), HEX);

  Serial.print("I2C address: 0x");
  Serial.println(*(registerPointer + 0x29), HEX);
}
