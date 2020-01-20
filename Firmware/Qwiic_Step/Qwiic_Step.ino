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
#define LOCATION_REGISTERMAP 0 //Location in EEPROM. Map is ~41 bytes currently.
#define LOCATION_PORSETTINGS 100 //Location in EEPROM for POR settings (for headless operation).

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
const uint8_t PIN_MAXCURRENT_PWM = 5;
const uint8_t PIN_CRRENT_SENSE = A6;
const uint8_t PIN_A49885_RESET = 4;  //May not be needed
const uint8_t PIN_ESTOP_SWITCH = 2; //E-Stop
const uint8_t PIN_LIMIT_SWITCH = 3; //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
#else //Used for development
const uint8_t DEBUG_PIN = A3;
const uint8_t PIN_STEP = 7;
const uint8_t PIN_DIRECTION = 8;
const uint8_t PIN_ENABLE = 10;
const uint8_t PIN_MS1 = 4;
const uint8_t PIN_MS2 = 5;
const uint8_t PIN_MS3 = 6;
const uint8_t PIN_MAXCURRENT_PWM = 5;
const uint8_t PIN_ESTOP_SWITCH = 2; //E-stop
const uint8_t PIN_LIMIT_SWITCH = 3; //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
#endif

volatile memoryMap registerMap {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptConfig {isReachedEnable, isLimitedEnable}
  {0, 0, 0, 0, 0, 0},  //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {0, 0, 0, 0, 0 , 1}, //motorConfig {ms1, ms2, ms3, disableMotorOnEStop, disableMotorOnPositionReached, stopOnLimitSwitchPress}
  {1, 0, 0, 0, 0},     //motorControl {runToPosition, runToPositionWithAccel, runContinuous, hardStop, disableMotor}
  0x00000000,          //currentPos
  0x00000000,          //distanceToGo
  0x00000000,          //move
  0x00,                //unlockMoveNVM
  0x00000000,          //moveTo
  0x00000000,          //maxSpeed (float)
  0x00000000,          //acceleration (float)
  0x00000000,          //speed (float)
  0x00,                //unlockSpeedNVM
  0x0000,              //holdCurrent
  0x0000,              //runCurrent
  DEFAULT_I2C_ADDRESS, //i2cAddress
};

//this memory map holds temporary "old" values so we don't call accelstepper functions an unnecessary amount of times
volatile memoryMap registerMapOld {
  DEVICE_ID,           //id
  FIRMWARE_VERSION,    //firmware
  {0, 0},              //interruptConfig {isReachedEnable, isLimitedEnable}
  {0, 0, 0, 0, 0, 0},  //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {0, 0, 0, 0, 0, 1},  //motorConfig {ms1, ms2, ms3, disableMotorOnEStop, disableMotorOnPositionReached, stopOnLimitSwitchPress}
  {1, 0, 0, 0, 0},     //motorControl {runToPosition, runToPositionWithAccel, runContinuous, hardStop, disableMotor}
  0x00000000,          //currentPos
  0x00000000,          //distanceToGo
  0x00000000,          //move
  0x00,                //unlockMoveNVM
  0x00000000,          //moveTo
  0x00000000,          //maxSpeed (float)
  0x00000000,          //acceleration (float)
  0x00000000,          //speed (float)
  0x00,                //unlockSpeedNVM
  0x0000,              //holdCurrent
  0x0000,              //runCurrent
  DEFAULT_I2C_ADDRESS, //i2cAddress
};

//This defines which of the registers are read-only (0) vs read-write (1)
memoryMap protectionMap = {
  0x00,               //id
  0x0000,             //firmware
  {1, 1},             //interruptConfig {isReachedEnable, isLimitedEnable}
  {1, 1, 1, 1, 1, 1}, //motorStatus {isRunning, isAccelerating, isDecelerating, isReached, isLimited, eStopped}
  {1, 1, 1, 1, 1, 1}, //motorConfig {ms1, ms2, ms3, disableMotorOnEStop, disableMotorOnPositionReached, stopOnLimitSwitchPress}
  {1, 1, 1, 1, 1},    //motorControl {runToPosition, runToPositionWithAccel, runContinuous, hardStop, disableMotor}
  0xFFFFFFFF,         //currentPos
  0x00000000,         //distanceToGo
  0xFFFFFFFF,         //move
  0xFF,               //unlockMoveNVM
  0xFFFFFFFF,         //moveTo
  0xFFFFFFFF,         //maxSpeed (float)
  0xFFFFFFFF,         //acceleration (float)
  0xFFFFFFFF,         //speed (float)
  0xFF,               //unlockSpeedNVM
  0xFFFF,             //holdCurrent
  0xFFFF,             //runCurrent
  0xFF,               //i2cAddress
};

//Cast 32bit address of the object registerMap with uint8_t so we can increment the pointer
uint8_t *registerPointer = (uint8_t *)&registerMap;
uint8_t *protectionPointer = (uint8_t *)&protectionMap;
volatile uint8_t registerNumber; //Gets set when user writes an address. We then serve the spot the user requested.

nvmMemoryMap PORsettings = {
  0, //speed
};

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
volatile bool newMoveValue = false; //Goes true when user has written a value to the move register
volatile bool newPositionValue = false; //Goes true when user has written a value to the currentPos register
float previousSpeed; //Hold previous speed of motor to calculate its acceleration status
unsigned long lastSpeedChange = 0; //Marks the time when previous speed last changed. Used to detect accel/deccel

float nvmSpeed = 0; //This value is loaded from EEPROM, separate from resgisterMap. If != 0, gets loaded at POR

//Define a stepper and its pins
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIRECTION); //Stepper driver, 2 pins required

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Qwiic Step");

#ifndef PRODUCTION_TARGET
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, HIGH);
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

  readSystemSettings(); //Load all system settings from EEPROM

  //Print info to Serial Monitor
#ifndef PRODUCTION_TARGET
  Serial.print("Address: 0x");
  Serial.println(registerMap.i2cAddress, HEX);
#endif

  //Attach state-change of interrupt pins to corresponding ISRs
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP_SWITCH), eStopTriggered, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH), limitSwitchTriggered, FALLING);

  analogWrite(PIN_MAXCURRENT_PWM, registerMap.holdCurrent);

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
      updateRegisterMap();
      printState();
    }
    else if (incoming == 'e')
    {
      eraseEEPROM();
      Serial.println("EEPROM erased");
    }
    else
    {
      Serial.print("Unknown: ");
      Serial.write(incoming);
      Serial.println();
    }
  }
#endif

  //Check to see if we need to drive interrupt pin high or low
  updateInterruptPin();

  //Run the stepper motor in the user chosen mode
  if (registerMap.motorStatus.eStopped == false)
  {
    if (registerMap.motorControl.runToPositionWithAccel) {
      stepper.run();
    }
    else if (registerMap.motorControl.runContinuous) {
      stepper.runSpeed();
    }
    else if (registerMap.motorControl.runToPosition) {
      stepper.runSpeedToPosition();
    }
    else if (registerMap.motorControl.hardStop) {
      //Do nothing. This will cause motor to hold in place.
    }
  }
}

void updateCurrents()
{
  analogWrite(PIN_MAXCURRENT_PWM, registerMap.holdCurrent);
}
