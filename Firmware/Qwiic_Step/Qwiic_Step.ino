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

//#define PRODUCTION_TARGET 1 //Uncomment to use the production code

//Hardware connections
#if defined(PRODUCTION_TARGET) //Used in production
const uint8_t PIN_STEP = A3;
const uint8_t PIN_ENABLE = 10;
const uint8_t PIN_DIRECTION = 6;
const uint8_t PIN_MS1 = 9;
const uint8_t PIN_MS2 = 8;
const uint8_t PIN_MS3 = 7;
const uint8_t addressPin = 11;
const uint8_t curr_ref_pwm = 5;
const uint8_t curr_sense = A6;    //DEBUG: right way to reference these pins?
const uint8_t a49885_reset = A7;  //DEBUG: might not work... is pin only ADC input?
const uint8_t PIN_ESTOP_SWITCH = 2; //E-Stop
const uint8_t PIN_LIMIT_SWITCH = 3; //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
//We need an enable pin
#else //Used for development
#define DEBUG_PIN 12
const uint8_t PIN_STEP = 7;
const uint8_t PIN_DIRECTION = 8;
const uint8_t PIN_ENABLE = 10;
const uint8_t PIN_MS1 = 4;
const uint8_t PIN_MS2 = 5;
const uint8_t PIN_MS3 = 6;
const uint8_t PIN_ESTOP_SWITCH = 2; //E-stop
const uint8_t PIN_LIMIT_SWITCH = 3; //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
#endif

volatile memoryMap registerMap {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptConfig {requestedPosReachedEnable, requestedPosReachedIntTriggered, limSwitchPressedEnable}
  {0, 0, 0, 0, 0, 0},  //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
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

//this memory map holds temporary "old" values so we don't call accelstepper functions an unnecessary amount of times
volatile memoryMap registerMapOld {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptConfig {requestedPosReachedEnable, requestedPosReachedIntTriggered, limSwitchPressedEnable}
  {0, 0, 0, 0, 0, 0},  //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
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
  0x00,   //id
  0x0000, //firmware
  {1, 1}, //interruptConfig {requestedPosReachedEnable, requestedPosReachedIntTriggered, limSwitchPressedEnable}
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
volatile uint8_t registerNumber; //Gets set when user writes an address. We then serve the spot the user requested.

//Interrupt turns on when position isReached, or limit switch is hit
//Turns off when interrupts are cleared by command
enum InterruptState
{
  INT_STATE_CLEARED = 0,
  INT_STATE_INDICATED,
};
volatile byte interruptState = INT_STATE_CLEARED;

//State machine for isReached bit within status register
//User can clear the isReached bit to clear the interrupt
//This allows for user to send multiple move commands and we'll not set the isReached bit until we reach the destination
enum MoveState
{
  MOVE_STATE_MOVING = 0,
  MOVE_STATE_NOTMOVING_ISREACH_SET,
  MOVE_STATE_NOTMOVING_ISREACH_CLEARED,
};
volatile byte moveState = MOVE_STATE_NOTMOVING_ISREACH_CLEARED;

//State machine for isLimited bit within status register
//User clears the isLimited bit to clear the interrupt
enum LimitState
{
  LIMIT_STATE_NOT_LIMITED = 0,
  LIMIT_STATE_LIMITED_SET,
  LIMIT_STATE_LIMITED_CLEARED,
};
volatile byte limitState = LIMIT_STATE_NOT_LIMITED;

volatile bool newData = false; //Goes true when we recieve new bytes from the users. Calls accelstepper functions with new registerMap values.
volatile bool newMoveValue = false; //Goes true when user has written a new move value
float previousSpeed; //Hold previous speed of motor to calculate its acceleration status
unsigned long lastSpeedChange = 0; //Marks the time when previous speed last changed. Used to detect accel/deccel

//Define a stepper and its pins
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIRECTION); //Stepper driver, 2 pins required

void setup(void)
{
#ifndef PRODUCTION_TARGET
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

  pinMode(PIN_ESTOP_SWITCH, INPUT_PULLUP); //E-Stop
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP); //Limit Switch

  stepper.setEnablePin(PIN_ENABLE);
  stepper.setPinsInverted(false, false, true); //Invert enable
  stepper.enableOutputs();

  releaseInterruptPin();

  //Print info to Serial Monitor
#ifndef PRODUCTION_TARGET
  Serial.begin(115200);
  Serial.println("Qwiic Step");
  Serial.print("Address: 0x");
  Serial.println(registerMap.i2cAddress, HEX);
  Serial.print("Device ID: 0x");
  Serial.println(registerMap.id, HEX);
#endif

  //  readSystemSettings(); //Load all system settings from EEPROM

  //Attach state-change of interrupt pins to corresponding ISRs
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP_SWITCH), eStopTriggered, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH), limitSwitchTriggered, FALLING);

  startI2C(); //Determine the I2C address to be using and listen on I2C bus
}

void loop(void)
{
#ifndef PRODUCTION_TARGET
  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'p')
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

  if (newData == true)
  {
    updateStepper(); //Update accelstepper functions
    newData = false;
  }

  //Update register map with latest values from accel stepper library and
  //any state machine changes (isReached, etc bits)
  updateRegisterMap();

  //Check to see if we need to drive interrupt pin high or low
  updateInterruptPin();

  //If everything is good, continue running the stepper
  if (registerMap.motorStatus.eStopped == false)
  {
    if (registerMap.motorControl.run) {
      stepper.run();
    }
    else if (registerMap.motorControl.runSpeed) {
      //Serial.println(stepper.speed());
//      //stepper.setMaxSpeed(1000.3);
//      stepper.setSpeed(200.3);
//      while(1)
      stepper.runSpeed();
    }
    else if (registerMap.motorControl.runSpeedToPosition) {
      stepper.runSpeedToPosition();
    }
    else if (registerMap.motorControl.stop) {
      stepper.stop();
    }
  }
}

//Determines if we need to activate (ie pull down) the interrupt output pin
//or if we need to release (set to high impedance) the pin
void updateInterruptPin()
{
  //Check if limit switch is open
  if (digitalRead(PIN_LIMIT_SWITCH) == HIGH)
  {
    if (limitState == LIMIT_STATE_LIMITED_CLEARED)
    {
      //Change states
      limitState = LIMIT_STATE_NOT_LIMITED;
      Serial.println("Limit released!");
    }
  }

  //Interrupt pin state machine
  //There are two states: Int Cleared, Int Indicated
  //INT_INDICATED state is entered when either LIMIT_STATE_LIMITED_SET or MOVE_STATE_NOTMOVING_ISREACH_SET is satisfied
  //INT_CLEARED state is entered when user clears the isReached and isLimited bits
  if (interruptState == INT_STATE_CLEARED)
  {
    //If we have moved since the last interrupt, and we have reached the new position, then indicate interrupt
    if (moveState == MOVE_STATE_NOTMOVING_ISREACH_SET && registerMap.interruptConfig.isReachedInterruptEnable)
    {
      Serial.println("isReached interrupt!");
      setInterruptPin(); //Move to INT_STATE_INDICATED state
    }
    if (limitState == LIMIT_STATE_LIMITED_SET && registerMap.interruptConfig.isLimitedInterruptEnable)
    {
      Serial.println("isLimited interrupt!");
      setInterruptPin(); //Move to INT_STATE_INDICATED state
    }
  }
  //Check to see if we need to release the INT pin
  else if (interruptState == INT_STATE_INDICATED)
  {
    //If the user has cleared all the interrupt bits, or if we are moving and limit switch is not depressed
    //then clear interrupt pin
    if ( (moveState == MOVE_STATE_NOTMOVING_ISREACH_CLEARED || moveState == MOVE_STATE_MOVING)
         && (limitState == LIMIT_STATE_LIMITED_CLEARED || limitState == LIMIT_STATE_NOT_LIMITED)
       )
    {
      Serial.println("INT High");
      releaseInterruptPin(); //Move to INT_STATE_CLEARED state
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
  Wire.begin(address); //Rejoin the I2C bus on new address

  //Connect receive and request events to the interrupts
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

//Called when there is new data from the user
//Determine what's new by comparing new vs old and pass the
//new values to the stepper library
void updateStepper()
{
  //  digitalWrite(DEBUG_PIN, HIGH);

  if (registerMapOld.maxSpeed != registerMap.maxSpeed)
  {
    Serial.print("S");
    stepper.setMaxSpeed(convertToFloat(registerMap.maxSpeed));
    registerMapOld.maxSpeed = registerMap.maxSpeed;
  }

  if (registerMapOld.speed != registerMap.speed)
  {
    Serial.print("P");

    //Calling .setSpeed with a value causes motor to twitch very slowly when we call .run. It shouldn't be. Libary bug?
    //https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#ae79c49ad69d5ccc9da0ee691fa4ca235
    //We will need to avoid calling setSpeed if user wants to be in run mode
    //We will need to call setSpeed if user wants to be in runSpeed mode
    stepper.setSpeed(convertToFloat(registerMap.speed));
    delay(1); //Removing this delay causes the speed not to get stored correctly into library. I cannot explain why.
    Serial.println(convertToFloat(registerMap.speed));
    Serial.println(stepper.speed());
    registerMapOld.speed = registerMap.speed;
  }

  if (registerMapOld.acceleration != registerMap.acceleration)
  {
    Serial.print("A");
    stepper.setAcceleration(convertToFloat(registerMap.acceleration));
    registerMapOld.acceleration = registerMap.acceleration;
  }

  //DEBUG: would this be the right way to service moveTo function?
  if (registerMapOld.moveTo != registerMap.moveTo)
  {
    Serial.print("T");
    //stepper.moveTo(registerMap.moveTo);
    registerMapOld.moveTo = registerMap.moveTo;
  }

  //Move to new value if user has given us one
  if (newMoveValue == true)
  {
    //Handle special stop command
    if (registerMap.move == 0)
    {
      Serial.print("!");
      stepper.stop(); //Drift to a stop as quickly as possible, using the current speed and acceleration parameters.
    }
    else
    {
      Serial.print("*");
      stepper.move(registerMap.move);
    }
    moveState = MOVE_STATE_MOVING; //Change our state
    registerMap.motorStatus.isReached = false;
    newMoveValue = false;
  }

  if (registerMapOld.currentPos != registerMap.currentPos)
  {
    Serial.print("C");
    stepper.setCurrentPosition(registerMap.currentPos);
    registerMapOld.currentPos = registerMap.currentPos;
  }

  if (registerMapOld.motorConfig.ms1 != registerMap.motorConfig.ms1
      || registerMapOld.motorConfig.ms2 != registerMap.motorConfig.ms2
      || registerMapOld.motorConfig.ms3 != registerMap.motorConfig.ms3
     )
  {
    //update the step mode by flipping pins MS1, MS2, MS3
    digitalWrite(PIN_MS1, registerMap.motorConfig.ms1);
    digitalWrite(PIN_MS2, registerMap.motorConfig.ms2);
    digitalWrite(PIN_MS3, registerMap.motorConfig.ms3);

    registerMapOld.motorConfig.ms1 = registerMap.motorConfig.ms1;
    registerMapOld.motorConfig.ms2 = registerMap.motorConfig.ms2;
    registerMapOld.motorConfig.ms3 = registerMap.motorConfig.ms3;
  }

  //  //DEBUGGING
  //  digitalWrite(DEBUG_PIN, LOW);

  //  if (registerMap.enableMoveNVM == 0x59) {
  //    recordSystemSettings(); //record registerMap to EEPROM?
  //    registerMap.enableMoveNVM = 0;  //clear the key
  //  }
}

//Update the register map with new states and bits
void updateRegisterMap()
{
  registerMap.distanceToGo = stepper.distanceToGo();

  float currentSpeed = stepper.speed();

  if (stepper.isRunning())
  {
    registerMap.motorStatus.isRunning = true;

    if (previousSpeed < currentSpeed)
    {
      registerMap.motorStatus.isAccelerating = true;
      registerMap.motorStatus.isDecelerating = false;
    }
    else if (previousSpeed > currentSpeed)
    {
      registerMap.motorStatus.isAccelerating = false;
      registerMap.motorStatus.isDecelerating = true;
    }
    else
    {
      //The previous speed is same as current speed
      //But we may still be in the middle of a slow accel/decel
      //This method checks to see if more than 250ms have gone by without change
      //It's a bit brittle but I don't know of a better way
      if (millis() - lastSpeedChange > 250)
      {
        registerMap.motorStatus.isAccelerating = false;
        registerMap.motorStatus.isDecelerating = false;
      }
    }
  }
  else
  {
    registerMap.motorStatus.isRunning = false;
    registerMap.motorStatus.isAccelerating = false;
    registerMap.motorStatus.isDecelerating = false;
  }

  if (previousSpeed != currentSpeed)
  {
    previousSpeed = currentSpeed;
    lastSpeedChange = millis();
  }

  //Handle the Move state machine
  //There are three states: MOVING, NOTMOVING_ISREACH_SET, NOTMOVING_ISREACH_CLEARED
  //MOVING happens when user has sent a Move value. Entered from any state.
  //NOTMOVING_ISREACH_SET happens once we've completed the previous MOVING state
  //NOTMOVING_ISREACH_CLEARED happens from _SET once user has cleared the bit
  if (moveState == MOVE_STATE_MOVING)
  {
    //Check if we have made it to our target position
    if (stepper.targetPosition() == stepper.currentPosition())
    {
      Serial.println("Arrived");
      moveState = MOVE_STATE_NOTMOVING_ISREACH_SET;
      registerMap.motorStatus.isReached = true;
    } //We do not clear the isReached bit. The user must actively clear it which will clear the interrupt as well.
  }
  else if (moveState == MOVE_STATE_NOTMOVING_ISREACH_SET)
  {
    if (registerMap.motorStatus.isReached == false)
    {
      moveState = MOVE_STATE_NOTMOVING_ISREACH_CLEARED;
      Serial.println("User cleared isReached");
    }
  }

  if (limitState == LIMIT_STATE_LIMITED_SET)
  {
    if (registerMap.motorStatus.isLimited == false)
    {
      limitState = LIMIT_STATE_LIMITED_CLEARED;
      Serial.println("User cleared isLimited");
    }
  }
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
void printState()
{
#ifndef PRODUCTION_TARGET
  Serial.println();

  Serial.print("Register map id: 0x");
  Serial.println(*(registerPointer + 0), HEX);

  Serial.print("Firmware version: 0x");
  if (*(registerPointer + 2) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 2), HEX);
  if (*(registerPointer + 1) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 1), HEX);

  Serial.print("Interrupt enable: 0x");
  Serial.println(*(registerPointer + 3), HEX);

  Serial.print("Motor status: 0x");
  Serial.println(*(registerPointer + 4), HEX);

  Serial.print("Device config: 0x");
  Serial.println(*(registerPointer + 5), HEX);

  Serial.print("Motor config: ");
  if(registerMap.motorControl.run == true) Serial.print("run");
  else if(registerMap.motorControl.runSpeed == true) Serial.print("runSpeed");
  else if(registerMap.motorControl.runSpeedToPosition == true) Serial.print("runSpeedToPosition");
  else if(registerMap.motorControl.stop == true) Serial.print("stop");
  Serial.println();
  //Serial.println(*(registerPointer + 6), HEX);
  
  Serial.print("Current position: 0x");
  if (*(registerPointer + 0xA) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xA), HEX);
  if (*(registerPointer + 0x9) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x9), HEX);
  if (*(registerPointer + 0x8) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x8), HEX);
  if (*(registerPointer + 0x7) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x7), HEX);

  Serial.print("Distance to go: 0x");
  if (*(registerPointer + 0xE) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xE), HEX);
  if (*(registerPointer + 0xD) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xD), HEX);
  if (*(registerPointer + 0xC) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xC), HEX);
  if (*(registerPointer + 0xB) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0xB), HEX);

  Serial.print("Move: 0x");
  if (*(registerPointer + 0x12) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x12), HEX);
  if (*(registerPointer + 0x11) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x11), HEX);
  if (*(registerPointer + 0x10) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x10), HEX);
  if (*(registerPointer + 0xF) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0xF), HEX);

  Serial.print("Enable move NVM: 0x");
  Serial.println(*(registerPointer + 0x13), HEX);

  Serial.print("Move to: 0x");
  if (*(registerPointer + 0x17) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x17), HEX);
  if (*(registerPointer + 0x16) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x16), HEX);
  if (*(registerPointer + 0x15) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x15), HEX);
  if (*(registerPointer + 0x14) < 0x10)
    Serial.print("0");
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
  if (*(registerPointer + 0x26) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x26), HEX);
  if (*(registerPointer + 0x25) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x25), HEX);

  Serial.print("Run current: 0x");
  if (*(registerPointer + 0x28) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x28), HEX);
  if (*(registerPointer + 0x27) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x27), HEX);

  Serial.print("I2C address: 0x");
  Serial.println(*(registerPointer + 0x29), HEX);
#endif
}
