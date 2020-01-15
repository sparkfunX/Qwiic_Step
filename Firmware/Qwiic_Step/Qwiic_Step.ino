/*
  Firmware to drive Qwiic Step
*/

#include <Wire.h>
#include <EEPROM.h>
#include <AccelStepper.h> //Click here to get the library: http://librarymanager/All#AccelStepper by Mike McCauley
#include "registers.h"

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0100
#define I2C_ADDRESS_DEFAULT 0x52
#define I2C_ADDRESS_FORCED 0x51
#define LOCATION_REGISTERMAP 0   //Location in EEPROM. Map is ~41 bytes currently.
#define LOCATION_PORSETTINGS 100 //Location in EEPROM for POR settings (for headless operation).

#define PRODUCTION_TARGET 1 //Uncomment to use the production code

//Hardware connections
#if defined(PRODUCTION_TARGET) //Used in production
const uint8_t PIN_STEP = A3;
const uint8_t PIN_ENABLE = 10;
const uint8_t PIN_DIRECTION = 6;
const uint8_t PIN_MS1 = 9;
const uint8_t PIN_MS2 = 8;
const uint8_t PIN_MS3 = 7;
const uint8_t addressPin = 11;
const uint8_t PIN_CURR_REF_PWM = 5;
const uint8_t curr_sense = A6;      //DEBUG: right way to reference these pins?
const uint8_t a49885_reset = A7;    //DEBUG: might not work... is pin only ADC input?
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
const uint8_t PIN_MAXCURRENT_PWM = 11;
const uint8_t PIN_ADDRESS = 9;
const uint8_t PIN_ESTOP_SWITCH = 2; //E-stop
const uint8_t PIN_LIMIT_SWITCH = 3; //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
const uint8_t PIN_CURR_REF_PWM = 9;
#endif

volatile memoryMap registerMap{
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
    0x03E8,              //holdCurrent
    0x03E8,              //runCurrent
    I2C_ADDRESS_DEFAULT, //i2cAddress
};

//this memory map holds temporary "old" values so we don't call accelstepper functions an unnecessary amount of times
volatile memoryMap registerMapOld{
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
    0x03E8,              //holdCurrent
    0x03E8,              //runCurrent
    I2C_ADDRESS_DEFAULT, //i2cAddress
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
    0, //move
    0, //speed
    0, //i2cAddressState
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

//State machine for the I2C Address
//User can set via software command or hardware jumper
enum ADRState
{
  ADR_STATE_SOFTWARE = 0,
  ADR_STATE_JUMPER,
};
volatile byte adrState = ADR_STATE_SOFTWARE;

//State machine for the PWM Current
//We are either holding or running
enum PWMState
{
  PWM_STATE_HOLDING = 0,
  PWM_STATE_RUNNING,
};
volatile byte pwmState = PWM_STATE_RUNNING; //This should cause the device to go to HOLD state at POR

volatile bool newData = false;          //Goes true when we recieve new bytes from the users. Calls accelstepper functions with new registerMap values.
volatile bool newMoveValue = false;     //Goes true when user has written a value to the move register
volatile bool newPositionValue = false; //Goes true when user has written a value to the currentPos register
float previousSpeed;                    //Hold previous speed of motor to calculate its acceleration status
unsigned long lastSpeedChange = 0;      //Marks the time when previous speed last changed. Used to detect accel/deccel

float nvmSpeed = 0; //This value is loaded from EEPROM, separate from resgisterMap. If != 0, gets loaded at POR

//Define a stepper and its pins
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIRECTION); //Stepper driver, 2 pins required

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Qwiic Step");

  setupGPIO();

  releaseInterruptPin();

  readSystemSettings(); //Load all system settings from EEPROM

  //Attach state-change of interrupt pins to corresponding ISRs
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP_SWITCH), eStopTriggered, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH), limitSwitchTriggered, FALLING);

  startI2C(); //Determine the I2C address to be using and listen on I2C bus

#ifndef PRODUCTION_TARGET
  Serial.print("Address: 0x");
  Serial.println(registerMap.i2cAddress, HEX);
#endif
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

  //Set the max current based on whether we are running or holding
  updateCurrents();

  //Run the stepper motor in the user chosen mode
  if (registerMap.motorStatus.eStopped == false)
  {
    if (registerMap.motorControl.runToPositionWithAccel)
    {
      stepper.run();
    }
    else if (registerMap.motorControl.runContinuous)
    {
      stepper.runSpeed();
    }
    else if (registerMap.motorControl.runToPosition)
    {
      stepper.runSpeedToPosition();
    }
    else if (registerMap.motorControl.hardStop)
    {
      //Do nothing. This will cause motor to hold in place.
    }
  }
}

//Set the initial state of various GPIOs
void setupGPIO()
{
#ifndef PRODUCTION_TARGET
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, HIGH);
#endif

  //Default to full step mode for now
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);
  digitalWrite(PIN_MS1, LOW);
  digitalWrite(PIN_MS2, LOW);
  digitalWrite(PIN_MS3, LOW);

  pinMode(PIN_ADDRESS, INPUT_PULLUP);
  pinMode(PIN_ESTOP_SWITCH, INPUT_PULLUP); //E-Stop
  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP); //Limit Switch

  //    pinMode(curr_ref_pwm, OUTPUT);
  //    pinMode(curr_sense, INPUT);
  //    pinMode(a49885_reset, OUTPUT);

  stepper.setEnablePin(PIN_ENABLE);
  stepper.setPinsInverted(false, false, true); //Invert enable
  stepper.enableOutputs();

  analogWrite(PIN_MAXCURRENT_PWM, 255 / 2);
}

//Update the analog PWM output going into the Max Current Ref
//pin on the driver. This allows us to have different max current
//for running and for holding.
void updateCurrents()
{
  //Based on current vs target position, determine what state we're in
  //if (stepper.isRunning()) //Doesn't work in runToPosition mode
  if (stepper.currentPosition() != stepper.targetPosition() && pwmState == PWM_STATE_HOLDING)
  {
    //We're moving!
    Serial.println("Run current");
    analogWrite(PIN_MAXCURRENT_PWM, convertCurrentToPWM(registerMap.runCurrent));
    pwmState = PWM_STATE_RUNNING;
  }
  else if (stepper.currentPosition() == stepper.targetPosition() && pwmState == PWM_STATE_RUNNING)
  {
    //We're not moving
    Serial.println("Hold current");
    analogWrite(PIN_MAXCURRENT_PWM, convertCurrentToPWM(registerMap.holdCurrent));
    pwmState = PWM_STATE_HOLDING;
  }
}

//Takes a run or hold current of up to 2000(mA) and
//converts that to a PWM value where 1.7V = 2000mA
//1.7V / 3.3V = X / 255
uint8_t convertCurrentToPWM(uint16_t current)
{
#ifdef PRODUCTION_TARGET                  //is a 3.3V system
  int maxPWM = (uint16_t)170 * 255 / 330; //~131
#else                                     //System is 5V Uno
  int maxPWM = (uint16_t)170 * 255 / 500; //~86
#endif

  int pwmValue = map(current, 0, 2000, 0, maxPWM);
  return (pwmValue);
}

//Convert a 0 to 2000mA current value to a voltage
//we should be able to detect with a DMM
float convertCurrentToVoltage(uint16_t current) void updateCurrent()
{
  //Update hold current first
  //Map registerMap hold current value to 1-255
  int curr = map(registerMap.holdCurrent, 0, 2000, 0, 255);
  //Generate pwm signal on correct pin
  analogWrite(PIN_CURR_REF_PWM, curr);

  //Update run current next
  //DEBUG: ...not sure how this should differ from the hold current
  //Maybe they're not two separate things?
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
#ifdef PRODUCTION_TARGET
  int maxPWM = (uint16_t)170 * 255 / 330; // 3.3V system = ~131
  int pwmValue = map(current, 0, 2000, 0, maxPWM);
  float voltage = 3.3 * pwmValue / 255.0;
#else
  int maxPWM = (uint16_t)170 * 255 / 500; // 5V system
  int pwmValue = map(current, 0, 2000, 0, maxPWM);
  float voltage = 5.0 * pwmValue / 255.0;
#endif

  return (voltage);
}
