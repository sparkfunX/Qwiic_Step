/*
  Firmware to drive Qwiic Step
*/

#include <Wire.h>
#include <EEPROM.h>
#include <AccelStepper.h> //Click here to get the library: http://librarymanager/All#AccelStepper by Mike McCauley
#include "registers.h"

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0100
#define DEFAULT_I2C_ADDRESS 0x52

//Hardware connections
#if defined(__AVR_ATmega328P__) //Used for development
#define DEBUG_PIN 12
const uint8_t PIN_STEP = 7;
const uint8_t PIN_DIRECTION = 8;
const uint8_t PIN_MS1 = 4;
const uint8_t PIN_MS2 = 5;
const uint8_t PIN_MS3 = 6;
const uint8_t PIN_INTERRUPT0 = 2;   //E-stop
const uint8_t PIN_INTERRUPT1 = 3;   //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
#elif defined(__AVR_ATtiny84__) //Used in production
const uint8_t PIN_STEP = A3;
const uint8_t PIN_DIRECTION = 6;
const uint8_t PIN_MS1 = 9;
const uint8_t PIN_MS2 = 8;
const uint8_t PIN_MS3 = 7;
const uint8_t addressPin = 11;
const uint8_t curr_ref_pwm = 5;
const uint8_t curr_sense = A6;   //DEBUG: right way to reference these pins?
const uint8_t a49885_reset = A7; //DEBUG: might not work... is pin only ADC input?
const uint8_t PIN_INTERRUPT0 = 2;   //E-Stop
const uint8_t PIN_INTERRUPT1 = 3;   //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
#endif

volatile memoryMap registerMap {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0, 0},           //interruptConfig {requestedPosReachedEnable, requestedPosReachedIntTriggered, limSwitchPressedEnable}
  {0, 0, 0, 1, 0, 0},  //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {0, 0, 0, 0, 1},     //motorConfig {ms1, ms2, ms3, disableMotorPositionReached, stopOnLimitSwitchPress}
  {1, 0, 0, 0, 0},     //motorControl {run, runSpeed, runSpeedToPosition, stop, disableMotor}
  0x00000000,          //currentPos
  0x00000000,          //distanceToGo
  0x0FFFFFFF,          //move
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
  {0, 0, 0},           //interruptConfig {requestedPosReachedEnable, requestedPosReachedIntTriggered, limSwitchPressedEnable}
  {0, 0, 0, 1, 0, 0},  //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {0, 0, 0, 0, 1},     //motorConfig {ms1, ms2, ms3, disableMotorPositionReached, stopOnLimitSwitchPress}
  {1, 0, 0, 0, 0},     //motorControl {run, runSpeed, runSpeedToPosition, stop, disableMotor}
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
  {1, 1, 1},          //interruptConfig {requestedPosReachedEnable, requestedPosReachedIntTriggered, limSwitchPressedEnable}
  //DEBUGGING: should these all be read-only for motorStatus?
  //DEBUG: isLimited is R/W for sure
  {1, 1, 1, 1, 1, 1}, //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {1, 1, 1, 1, 1},    //motorConfig {ms1, ms2, ms3, disableMotorPositionReached, stopOnLimitSwitchPress}
  {1, 1, 1, 1, 1},    //motorControl {run, runSpeed, runSpeedToPosition, stop, disableMotor}
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
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIRECTION); //Stepper driver, 2 pins required

void setup(void)
{
#if defined(__AVR_ATmega328P__) //Used for development
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);
#endif

  //Motor config pins are all outputs

  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);

  //Default to full step mode
  digitalWrite(PIN_MS1, LOW);
  digitalWrite(PIN_MS2, LOW);
  digitalWrite(PIN_MS3, LOW);

  //    pinMode(addressPin, INPUT_PULLUP);
  //    pinMode(curr_ref_pwm, OUTPUT);
  //    pinMode(curr_sense, INPUT);
  //    pinMode(a49885_reset, OUTPUT);

  pinMode(PIN_INTERRUPT0, INPUT_PULLUP);    //E-Stop
  pinMode(PIN_INTERRUPT1, INPUT_PULLUP);    //Limit Switch
  pinMode(PIN_INT_OUTPUT, OUTPUT); //'INT' pin on board to indicate there is an interrupt

  //Print info to Serial Monitor
#if defined(__AVR_ATmega328P__) //Used for development
  Serial.begin(115200);
  Serial.println("Qwiic Step");
  Serial.print("Address: 0x");
  Serial.println(registerMap.i2cAddress, HEX);
  Serial.print("Device ID: 0x");
  Serial.println(registerMap.id, HEX);
#endif

  //  readSystemSettings(); //Load all system settings from EEPROM

  //Attach state-change of interrupt pins to corresponding ISRs
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT0), eStopTriggered, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT1), limitSwitchTriggered, FALLING);

  startI2C(); //Determine the I2C address to be using and listen on I2C bus

  printState();
}

void loop(void) {

#if defined(__AVR_ATmega328P__) //Used for development
  if(Serial.available())
  {
    byte incoming = Serial.read();

    if(incoming == 'p')
    {
      printState();
    }
    else
    {
      Serial.print("Unknown: ");
      Serial.write(incoming);
      Serial.println();
    }
  }
  #endif
  
  //Compare current state and see if we need to update any isReached, etc bits
  updateStatusBits();

  //Check to see if we need to drive interrupt pin
  if ((registerMap.interruptConfig.requestedPosReachedEnable && registerMap.interruptConfig.requestedPosReachedIntTriggered)
        ||(registerMap.interruptConfig.limSwitchPressedEnable && registerMap.motorStatus.isLimited))
  {
    Serial.println("INT pin driven low");

    //Drive INT pin low
    pinMode(PIN_INT_OUTPUT, OUTPUT);
    digitalWrite(PIN_INT_OUTPUT, LOW);
  }
  else
  {
    //Go to high-impedance mode
    pinMode(PIN_INT_OUTPUT, INPUT); //Pin has external pullup
  }

  //Update accelstepper functions
  if (updateFlag == true) {
    updateStepper();

    //    //Record anything new to EEPROM
    //    recordSystemSettings();

    //clear updateFlag
    updateFlag = false;
  }

  //If everything is good, continue running the stepper
  if (registerMap.motorStatus.eStopped == false)
  {
    if (registerMap.motorControl.run){
      stepper.run();
    }
    else if (registerMap.motorControl.runSpeed){
      stepper.runSpeed();
    }
    else if (registerMap.motorControl.runSpeedToPosition){
      stepper.runSpeedToPosition();
    }
    else if (registerMap.motorControl.stop){
      stepper.stop();
    }
  }
}

//Determines the needed I2C address from NVM and starts as slave
//Registers the receive and request I2C interrupt handlers
void startI2C()
{
  uint8_t address;

  //TODO: Need to handle address jumper here.

  //Check if the address stored in memoryMap is valid
  if (registerMap.i2cAddress > 0x07 && registerMap.i2cAddress < 0x78)
    address = registerMap.i2cAddress;
  else //If the value is illegal, default to the default I2C address for our platform
    address = DEFAULT_I2C_ADDRESS;

  //Save new address to the register map
  registerMap.i2cAddress = address;

  Wire.end();
  Wire.begin(address);  //Rejoin the I2C bus on new address

  //Connect receive and request events to the interrupts
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void updateStepper() {
  //  digitalWrite(DEBUG_PIN, HIGH);

  //call accelstepper functions with the values in registerMap
  //check if there is a new value for maxSpeed in registerMap
  if (registerMapOld.maxSpeed != registerMap.maxSpeed) {
    //if there is, update accelstepper library
    stepper.setMaxSpeed(convertToFloat(registerMap.maxSpeed));
    //update registerMapOld
    registerMapOld.maxSpeed = registerMap.maxSpeed;
  }

  if (registerMapOld.speed != registerMap.speed) {
    stepper.setSpeed(convertToFloat(registerMap.speed));
    registerMapOld.speed = registerMap.speed;
  }

  if (registerMapOld.acceleration != registerMap.acceleration) {
    stepper.setAcceleration(convertToFloat(registerMap.acceleration));
    registerMapOld.acceleration = registerMap.acceleration;
  }

  //DEBUG: would this be the right way to service moveTo function?
  if (registerMapOld.moveTo != registerMap.moveTo) {
    stepper.moveTo(registerMap.moveTo);
    registerMapOld.moveTo = registerMap.moveTo;
  }

  //0xFFFFFFFF is an illegal value for move function
  //Helps us know when accelstepper move function has been serviced
  if (registerMap.move != 0xFFFFFFFF) {
    stepper.move(registerMap.move);
    registerMap.move = 0xFFFFFFFF;
  }

  if (registerMapOld.currentPos != registerMap.currentPos){
    stepper.setCurrentPosition(registerMap.currentPos);
    registerMapOld.currentPos = registerMap.currentPos;
  }

  //update the step mode by flipping pins MS1, MS2, MS3
  digitalWrite(PIN_MS1, registerMap.motorConfig.ms1);
  digitalWrite(PIN_MS2, registerMap.motorConfig.ms2);
  digitalWrite(PIN_MS3, registerMap.motorConfig.ms3);

  //  //DEBUGGING
  //  digitalWrite(DEBUG_PIN, LOW);

  //  if (registerMap.enableMoveNVM == 0x59) {
  //    recordSystemSettings(); //record registerMap to EEPROM?
  //    registerMap.enableMoveNVM = 0;  //clear the key
  //  }
}

//Update the status bits within the STATUS register
void updateStatusBits() {

  float currentSpeed = stepper.speed();

  if (stepper.isRunning()){
    registerMap.motorStatus.isRunning = 1;
    if (previousSpeed < currentSpeed) {
      registerMap.motorStatus.isAccelerating = 1;
      registerMap.motorStatus.isDecelerating = 0;
    }
    else if (previousSpeed > currentSpeed) {
      registerMap.motorStatus.isAccelerating = 0;
      registerMap.motorStatus.isDecelerating = 1;
    }
    //Scenario: running, but at constant speed
    else if (previousSpeed == currentSpeed) {
      registerMap.motorStatus.isAccelerating = 0;
      registerMap.motorStatus.isDecelerating = 0;
    }
  }
  else {
    registerMap.motorStatus.isRunning = 0;
    registerMap.motorStatus.isAccelerating = 0;
    registerMap.motorStatus.isDecelerating = 0;
  }

  //update previous speed
  previousSpeed = currentSpeed;
  
  //check if we have made it to our target position
  if (stepper.targetPosition() == stepper.currentPosition()) {

    //if this is our first "isReached" instance, set the interrupt flag
    if (registerMap.motorStatus.isReached == 0) {
      registerMap.interruptConfig.requestedPosReachedIntTriggered = 1;
    }
    //motor has reached its destination
    registerMap.motorStatus.isReached = 1;
    //Disable motor if user has configured to
    //    if (registerMap.motorConfig.disableMotorPositionReached)
    //      stepper.disableOutputs();
  }
  else {  //motor has not yet reached destination
    registerMap.motorStatus.isReached = 0;
  }

  //update distanceToGo register here because we call this function everytime we loop
  registerMap.distanceToGo = stepper.distanceToGo();
}

//void recordSystemSettings()
//{
//  EEPROM.put(0x00, registerMap);
//}
//
//void readSystemSettings()
//{
//  //Check to see if EEPROM is blank
//  uint32_t EEPROM_check;
//
//  EEPROM.get(0x00, EEPROM_check);
//  if (EEPROM_check == 0xFFFFFFFF) {  //EEPROM has not been written to yet
//    recordSystemSettings(); //record default settings to EEPROM
//  }
//
//  EEPROM.get(0, registerMap);
//
//  if (registerMap.enableMoveNVM != 0x59) {
//    registerMap.move = 0;
//  }
//  else {
//    updateFlag = true;    //initiate move to NVM stored value
//  }
//}

//Prints the current register map
//Note: some of these values are floating point so HEX printing will look odd.
void printState() {
#if defined(__AVR_ATmega328P__) //Used for development
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

  Serial.print("Max Speed: ");
  Serial.println(convertToFloat(registerMap.maxSpeed), 2);
//  if (*(registerPointer + 0x1B) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x1B), HEX);
//  if (*(registerPointer + 0x1A) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x1A), HEX);
//  if (*(registerPointer + 0x19) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x19), HEX);
//  if (*(registerPointer + 0x18) < 0x10) Serial.print("0");
//  Serial.println(*(registerPointer + 0x18), HEX);

  Serial.print("Acceleration: ");
  Serial.println(convertToFloat(registerMap.acceleration), 2);
//  if (*(registerPointer + 0x1F) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x1F), HEX);
//  if (*(registerPointer + 0x1E) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x1E), HEX);
//  if (*(registerPointer + 0x1D) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x1D), HEX);
//  if (*(registerPointer + 0x1C) < 0x10) Serial.print("0");
//  Serial.println(*(registerPointer + 0x1C), HEX);

  Serial.print("Speed: ");
  Serial.println(convertToFloat(registerMap.speed), 2);
//  if (*(registerPointer + 0x23) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x23), HEX);
//  if (*(registerPointer + 0x22) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x22), HEX);
//  if (*(registerPointer + 0x21) < 0x10) Serial.print("0");
//  Serial.print(*(registerPointer + 0x21), HEX);
//  if (*(registerPointer + 0x20) < 0x10) Serial.print("0");
//  Serial.println(*(registerPointer + 0x20), HEX);

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
#endif
}
