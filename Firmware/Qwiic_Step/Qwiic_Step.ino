/*
  Firmware to drive Qwiic Step
*/

#include <Wire.h>
#include <EEPROM.h>
#include <AccelStepper.h> //Click here to get the library: http://librarymanager/All#AccelStepper by Mike McCauley
#include "registers.h"

#define DEVICE_ID 0x60
#define FIRMWARE_VERSION 0x0101
#define I2C_ADDRESS_DEFAULT 0x52
#define I2C_ADDRESS_FORCED 0x51
#define LOCATION_REGISTERMAP 0 //Location in EEPROM. Map is ~41 bytes currently.
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
const uint8_t PIN_ADDRESS = A7;
const uint8_t PIN_MAXCURRENT_PWM = 5;
const uint8_t PIN_CRRENT_SENSE = A6;
const uint8_t PIN_A49885_RESET = A0;
const uint8_t PIN_ESTOP_SWITCH = 2; //E-Stop
const uint8_t PIN_LIMIT_SWITCH = 3; //Limit switch
const uint8_t PIN_INT_OUTPUT = A1;
const uint8_t PIN_DEBUG = A2; //Routed to no where
#else //Used for development
const uint8_t PIN_DEBUG = A3;
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
  0x3F99999A,          //holdVoltage - 1.2V
  0x3F99999A,          //runVoltage - 1.2V
  I2C_ADDRESS_DEFAULT, //i2cAddress
};

//This memory map holds temporary "old" values so we don't call accelstepper functions an unnecessary amount of times
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
  0x3F99999A,          //holdVoltage - 1.2V
  0x3F99999A,          //runVoltage - 1.2V
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
  (int32_t)0xFFFFFFFF,         //currentPos
  0x00000000,         //distanceToGo
  (int32_t)0xFFFFFFFF,         //move
  0xFF,               //unlockMoveNVM
  (int32_t)0xFFFFFFFF,         //moveTo
  (uint32_t)0xFFFFFFFF,         //maxSpeed (float)
  (uint32_t)0xFFFFFFFF,         //acceleration (float)
  (uint32_t)0xFFFFFFFF,         //speed (float)
  0xFF,               //unlockSpeedNVM
  (uint32_t)0xFFFFFFFF,         //holdVoltage
  (uint32_t)0xFFFFFFFF,         //runVoltage
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

volatile bool newData = false; //Goes true when we recieve new bytes from the users. Calls accelstepper functions with new registerMap values.
volatile bool newMoveValue = false; //Goes true when user has written a value to the move register
volatile bool newPositionValue = false; //Goes true when user has written a value to the currentPos register
float previousSpeed; //Hold previous speed of motor to calculate its acceleration status
unsigned long lastSpeedChange = 0; //Marks the time when previous speed last changed. Used to detect accel/deccel

float nvmSpeed = 0; //This value is loaded from EEPROM, separate from resgisterMap. If != 0, gets loaded at POR

volatile bool newSettingsToRecord = false; //Goes true when user has sent us new settings. Handled in loop().

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

  Serial.print("Address: 0x");
  Serial.println(registerMap.i2cAddress, HEX);
}

void loop(void)
{
  //#ifndef PRODUCTION_TARGET
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
  //#endif

  //Check to see if we need to drive interrupt pin high or low
  updateInterruptPin();

  //Set the max voltage (PWM through an RC filter) on the VRef pin based on whether we are running or holding
  updateVoltages();

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

  /*We don't want to constantly record the register map to NVM. It costs cycles
      and can wear out the EEPROM. Thankfully EEPROM.put() automatically calls
      update so only values that changed will be recorded. Additionally, we can be
      proactive and ignore all registers that will be 0 at POR:
          status
          currentPos
          distanceToGo
          move
          unlockMoveNVM
          moveTo
          speed
          unlockSpeedNVM
  */
  if (newSettingsToRecord == true)
  {
    newSettingsToRecord = false; //Important location: Keep before recordRegisterMap. See issue: https://github.com/sparkfunX/Qwiic_Step/issues/6
    //Incase I2C ISR sets new settings while EEPROM is recording, newSettingsToRecord will go false and record again.
    recordRegisterMap();
  }
}

//Set the initial state of various GPIOs
void setupGPIO()
{
#ifndef PRODUCTION_TARGET
  pinMode(PIN_DEBUG, OUTPUT);
  digitalWrite(PIN_DEBUG, HIGH);
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

  stepper.setEnablePin(PIN_ENABLE);
  stepper.setPinsInverted(false, false, true); //Invert enable
  stepper.enableOutputs();

  analogWrite(PIN_MAXCURRENT_PWM, 255 / 2);
}

//Update the analog PWM output going into the Max Current Ref
//pin on the driver. This allows us to have different max current
//for running and for holding. The relation between the voltage on VRef
//and the max trip current is too complex to code. We leave it up to
//the user to select the appropriate voltage to achieve their desired
//trip current.
void updateVoltages()
{
  //Based on current vs target position, determine what state we're in
  //if (stepper.isRunning()) //Doesn't work in runToPosition mode
  if (stepper.currentPosition() != stepper.targetPosition() && pwmState == PWM_STATE_HOLDING)
  {
    //We're moving!
    Serial.println("Running voltage");
    float voltage = convertToFloat(registerMap.runVoltage); //Convert uint32 to float
    analogWrite(PIN_MAXCURRENT_PWM, convertVoltageToPWM(voltage));
    pwmState = PWM_STATE_RUNNING;
  }
  else if (stepper.currentPosition() == stepper.targetPosition() && pwmState == PWM_STATE_RUNNING)
  {
    //We're not moving
    Serial.println("Holding voltage");
    float voltage = convertToFloat(registerMap.holdVoltage); //Convert uint32 to float
    analogWrite(PIN_MAXCURRENT_PWM, convertVoltageToPWM(voltage));
    pwmState = PWM_STATE_HOLDING;
  }
}

//Takes a run or hold voltage and
//converts that to a PWM value.
uint8_t convertVoltageToPWM(float voltage)
{
  int pwmValue = mapfloat(voltage, 0.0, 3.3, 0.0, 255.0);
  Serial.print("pwmValue: ");
  Serial.println(pwmValue);
  return (pwmValue);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (x > in_max) x = in_max;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
