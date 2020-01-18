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
const uint8_t DEBUG_PIN = A3;
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
  {1, 0, 0, 0, 0},     //motorControl {runToPosition, runToPositionWithAccel, runContinuous, hardStop, disableMotor}
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
  {1, 0, 0, 0, 0},     //motorControl {runToPosition, runToPositionWithAccel, runContinuous, hardStop, disableMotor}
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
  {1, 1, 1, 1, 1},    //motorControl {runToPosition, runToPositionWithAccel, runContinuous, hardStop, disableMotor}
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
      //Serial.println("Arrived");
      moveState = MOVE_STATE_NOTMOVING_ISREACH_SET;
      registerMap.motorStatus.isReached = true;

      //Do we need to disable the motor?
      if (registerMap.motorConfig.disableMotorOnPositionReached == true)
        stepper.disableOutputs();

    } //We do not clear the isReached bit. The user must actively clear it which will clear the interrupt as well.
  }
  else if (moveState == MOVE_STATE_NOTMOVING_ISREACH_SET)
  {
    if (registerMap.motorStatus.isReached == false)
    {
      moveState = MOVE_STATE_NOTMOVING_ISREACH_CLEARED;
      //Serial.println("User cleared isReached");
    }
  }

  if (limitState == LIMIT_STATE_LIMITED_SET)
  {
    if (registerMap.motorStatus.isLimited == false)
    {
      limitState = LIMIT_STATE_LIMITED_CLEARED;
      //Serial.println("User cleared isLimited");
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

//Called after I2C receive interrupt so user has passed us new data
//Pass any new speed, accel, etc values to the library
//Determine what's new by comparing old register map against new data
void updateParameters()
{
  //digitalWrite(DEBUG_PIN, HIGH);
  bool newValueToRecord = false;

  if (registerMapOld.maxSpeed != registerMap.maxSpeed)
  {
    newValueToRecord = true;
    //    Serial.print("S");
    stepper.setMaxSpeed(convertToFloat(registerMap.maxSpeed));
    registerMapOld.maxSpeed = registerMap.maxSpeed;
  }

  if (registerMapOld.acceleration != registerMap.acceleration)
  {
    newValueToRecord = true;
    //    Serial.print("A");
    stepper.setAcceleration(convertToFloat(registerMap.acceleration));
    registerMapOld.acceleration = registerMap.acceleration;
  }

  if (registerMapOld.moveTo != registerMap.moveTo)
  {
    //    Serial.print("T");
    stepper.moveTo(registerMap.moveTo);
    registerMapOld.moveTo = registerMap.moveTo;
  }

  //Move to new value if user has given us one
  if (newMoveValue == true)
  {
    //Handle special 'soft' stop command
    if (registerMap.move == 0)
    {
      //      Serial.print("!");
      stepper.stop(); //Drift to a stop as quickly as possible, using the current speed and acceleration parameters.
      stepper.setSpeed(0); //.stop() can leave artifacts and click slowly when in runSpeed mode. This clears it.
    }
    else
    {
      //      Serial.print("*");
      stepper.move(registerMap.move);
    }
    moveState = MOVE_STATE_MOVING; //Change our state
    registerMap.motorStatus.isReached = false;
    newMoveValue = false;
  }

  //In runSpeedToPos mode, speed must be set after move for accel stepper library to work (see runSpeedtoPosition example)
  //so the speed check is done after the move check.
  //The stepper library will change the current speed as needed
  //We can't check against the old map, we have to check against the current speed.
  if (stepper.speed() != convertToFloat(registerMap.speed))
  {
    //We only allow user to set the speed if we are in runToPosition or runContinuous mode
    //In runToPositionWithAccel mode the library will calculate the speed automatically
    if (registerMap.motorControl.runToPosition || registerMap.motorControl.runContinuous)
    {
      //      Serial.print("P");

      //Calling .setSpeed with a value causes motor to twitch very slowly when we call .run. It shouldn't be. Libary bug?
      //https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#ae79c49ad69d5ccc9da0ee691fa4ca235
      stepper.setSpeed(convertToFloat(registerMap.speed));
      //delay(1); //Removing this delay causes the speed not to get stored correctly into library in runSpeed mode. I cannot explain why.

      //      Serial.print(stepper.speed());
      //      Serial.print(" ");
      //      Serial.println(convertToFloat(registerMap.speed));
      registerMapOld.speed = registerMap.speed;
    }
  }

  if (newPositionValue == true)
  {
    //Serial.print("$");
    stepper.setCurrentPosition(registerMap.currentPos);
    registerMapOld.currentPos = registerMap.currentPos;
    newPositionValue = false;

    //Edge case: If user moveTo(50), setPos(0), then moveTo(50)
    //The value in moveTo hasn't changed but our absolute position has.
    //Update the moveTo register to the setPos to avoid conflict.
    //The only downside is if user reads the moveTo register, it will be corrupt
    registerMap.moveTo = registerMap.currentPos;
    registerMapOld.moveTo = registerMap.currentPos;
  }

  if (registerMapOld.motorControl.disableMotor != registerMap.motorControl.disableMotor)
  {
    newValueToRecord = true;

    if (registerMap.motorControl.disableMotor == true)
      stepper.disableOutputs();
    else
      stepper.enableOutputs();

    registerMapOld.motorControl.disableMotor = registerMap.motorControl.disableMotor;
  }

  if (registerMapOld.motorConfig.ms1 != registerMap.motorConfig.ms1
      || registerMapOld.motorConfig.ms2 != registerMap.motorConfig.ms2
      || registerMapOld.motorConfig.ms3 != registerMap.motorConfig.ms3
     )
  {
    newValueToRecord = true;

    //Update the step mode by flipping pins MS1, MS2, MS3
    digitalWrite(PIN_MS1, registerMap.motorConfig.ms1);
    digitalWrite(PIN_MS2, registerMap.motorConfig.ms2);
    digitalWrite(PIN_MS3, registerMap.motorConfig.ms3);

    registerMapOld.motorConfig.ms1 = registerMap.motorConfig.ms1;
    registerMapOld.motorConfig.ms2 = registerMap.motorConfig.ms2;
    registerMapOld.motorConfig.ms3 = registerMap.motorConfig.ms3;
  }

  if (registerMapOld.motorControl.runToPosition != registerMap.motorControl.runToPosition
      || registerMapOld.motorControl.runToPositionWithAccel != registerMap.motorControl.runToPositionWithAccel
      || registerMapOld.motorControl.runContinuous != registerMap.motorControl.runContinuous
      || registerMapOld.motorControl.hardStop != registerMap.motorControl.hardStop
     )
  {
    newValueToRecord = true;

    registerMapOld.motorControl.runToPosition = registerMap.motorControl.runToPosition;
    registerMapOld.motorControl.runToPositionWithAccel = registerMap.motorControl.runToPositionWithAccel;
    registerMapOld.motorControl.runContinuous = registerMap.motorControl.runContinuous;
    registerMapOld.motorControl.hardStop = registerMap.motorControl.hardStop;
  }

  if (registerMap.unlockMoveNVM == 0x59)
  {
    Serial.print("%");
    PORsettings.move = registerMap.move;
    recordPORsettings();
    registerMap.unlockMoveNVM = 0;
  }

  if (registerMap.unlockSpeedNVM == 0xC4)
  {
    Serial.print("&");
    PORsettings.speed = registerMap.speed;
    recordPORsettings();
    registerMap.unlockSpeedNVM = 0;
  }

  /*We don't want to constantly record the register map to NVM. It costs cycles
    and can wear out the EEPROM. Thankfully EEPROM.put() automatically calls
    update so only values that changed will be recorded. Additionally, we can be 
    proactive and ignore all registers that will be 0 at POR:
      * status
      * currentPos
      * distanceToGo
      * move
      * unlockMoveNVM
      * moveTo
      * speed
      * unlockSpeedNVM
  */
  if (newValueToRecord == true)
  {
    recordRegisterMap();
  }

  //digitalWrite(DEBUG_PIN, LOW);
} //End updateParameters()

//Called from I2C respond interrupt. Right before we push requested I2C data out to the bus
//As the accel library updates values, push them to the register map (isAccelerating, currentPos, isReached, etc bits)
void updateRegisterMap()
{
  registerMap.distanceToGo = stepper.distanceToGo();
  registerMap.currentPos = stepper.currentPosition();

  float currentSpeed = stepper.speed();

  //If mode is runToPosition, speed will be set and isRunning is true no matter
  //what even after we've arrived at the position.
  //if (stepper.isRunning()) //Doesn't work in runToPosition mode
  if (stepper.targetPosition() != stepper.currentPosition())
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
} //End updateRegisterMap()

void recordRegisterMap()
{
  Serial.print("#");
  EEPROM.put(0, registerMap);
}

void recordPORsettings()
{
  Serial.print("?");
  EEPROM.put(100, PORsettings);
}

void readSystemSettings()
{
  //Check to see if EEPROM is blank
  uint32_t EEPROM_check;
  EEPROM.get(0, EEPROM_check);
  if (EEPROM_check == 0xFFFFFFFF) {  //EEPROM has not been written to yet
    recordRegisterMap(); //Record default settings to EEPROM
  }

  EEPROM.get(0, registerMap);

  //Zero out the registers that must be 0 at POR
  registerMap.unlockMoveNVM = 0;
  registerMap.unlockSpeedNVM = 0;
  registerMap.move = 0;
  registerMap.speed = 0;
  registerMap.currentPos = 0;
  registerMap.distanceToGo = 0;

  //Deal with the special POR settings
  //Check to see if EEPROM is blank
  EEPROM.get(100, EEPROM_check);
  if (EEPROM_check == 0xFFFFFFFF) {  //EEPROM has not been written to yet
    recordPORsettings(); //Record default settings to EEPROM
  }
  
  EEPROM.get(100, PORsettings);
  if (PORsettings.move != 0) registerMap.move = PORsettings.move;
  if (PORsettings.speed != 0) registerMap.speed = PORsettings.speed;

  //Pass these new settings back into the library
  updateParameters();
}

//Clear the EEPROM
void eraseEEPROM()
{
  for(byte x = 0 ; x < 120 ; x++)
  {
    EEPROM.put(x, 0xFF);
  }
}
