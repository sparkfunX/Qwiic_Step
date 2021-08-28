//Called after I2C receive interrupt so user has passed us new data
//Record any new changes to NVM if register is allowed to be
//Pass any new speed, accel, etc values to the library
//Determine what's new by comparing old register map against new data
void updateParameters()
{
  //digitalWrite(PIN_DEBUG, HIGH);

  //Check each register in the order they appear in the map

  //Serial.println("UP");

  //0x03 interrupt config register
  if (registerMapOld.interruptConfig.isReachedInterruptEnable != registerMap.interruptConfig.isReachedInterruptEnable
      || registerMapOld.interruptConfig.isLimitedInterruptEnable != registerMap.interruptConfig.isLimitedInterruptEnable
     )
  {
    newSettingsToRecord = true;
  }

  //0x05 config register
  if (registerMapOld.motorConfig.ms1 != registerMap.motorConfig.ms1
      || registerMapOld.motorConfig.ms2 != registerMap.motorConfig.ms2
      || registerMapOld.motorConfig.ms3 != registerMap.motorConfig.ms3
     )
  {
    newSettingsToRecord = true;

    //Update the step mode by flipping pins MS1, MS2, MS3
    digitalWrite(PIN_MS1, registerMap.motorConfig.ms1);
    digitalWrite(PIN_MS2, registerMap.motorConfig.ms2);
    digitalWrite(PIN_MS3, registerMap.motorConfig.ms3);

    registerMapOld.motorConfig.ms1 = registerMap.motorConfig.ms1;
    registerMapOld.motorConfig.ms2 = registerMap.motorConfig.ms2;
    registerMapOld.motorConfig.ms3 = registerMap.motorConfig.ms3;
  }

  if (registerMapOld.motorConfig.disableMotorOnEStop != registerMap.motorConfig.disableMotorOnEStop
      || registerMapOld.motorConfig.disableMotorOnPositionReached != registerMap.motorConfig.disableMotorOnPositionReached
      || registerMapOld.motorConfig.stopOnLimitSwitchPress != registerMap.motorConfig.stopOnLimitSwitchPress
     )
  {
    newSettingsToRecord = true;
  }

  //0x06 mode register
  if (registerMapOld.motorControl.runToPosition != registerMap.motorControl.runToPosition
      || registerMapOld.motorControl.runToPositionWithAccel != registerMap.motorControl.runToPositionWithAccel
      || registerMapOld.motorControl.runContinuous != registerMap.motorControl.runContinuous
      || registerMapOld.motorControl.hardStop != registerMap.motorControl.hardStop
     )
  {
    newSettingsToRecord = true;

    registerMapOld.motorControl.runToPosition = registerMap.motorControl.runToPosition;
    registerMapOld.motorControl.runToPositionWithAccel = registerMap.motorControl.runToPositionWithAccel;
    registerMapOld.motorControl.runContinuous = registerMap.motorControl.runContinuous;
    registerMapOld.motorControl.hardStop = registerMap.motorControl.hardStop;
  }

  if (registerMapOld.motorControl.disableMotor != registerMap.motorControl.disableMotor)
  {
    newSettingsToRecord = true;

    if (registerMap.motorControl.disableMotor == true)
      stepper.disableOutputs();
    else
      stepper.enableOutputs();

    registerMapOld.motorControl.disableMotor = registerMap.motorControl.disableMotor;
  }

  //0x07 current position register
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

  //0x0F move register
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

  //0x13 unlock Move NVM register
  if (registerMap.unlockMoveNVM == 0x59)
  {
    //    Serial.print("%");
    PORsettings.move = registerMap.move;
    recordPORsettings();
    registerMap.unlockMoveNVM = 0;
  }

  //0x18 maxSpeed register
  //Max speed must be set in the stepper library before move, moveTo, or accel
  if (registerMapOld.maxSpeed != registerMap.maxSpeed)
  {
    newSettingsToRecord = true;
    //    Serial.print("S");
    stepper.setMaxSpeed(convertToFloat(registerMap.maxSpeed));
    registerMapOld.maxSpeed = registerMap.maxSpeed;
  }

  //0x1C acceleration register
  if (registerMapOld.acceleration != registerMap.acceleration)
  {
    newSettingsToRecord = true;
    //    Serial.print("A");
    stepper.setAcceleration(convertToFloat(registerMap.acceleration));
    registerMapOld.acceleration = registerMap.acceleration;
  }

  //0x14 moveTo register
  if (registerMapOld.moveTo != registerMap.moveTo)
  {
    //    Serial.print("T");
    stepper.moveTo(registerMap.moveTo);
    registerMapOld.moveTo = registerMap.moveTo;
  }

  //0x20 speed register
  //We only allow user to set the speed if we are in runToPosition or runContinuous mode
  //In runToPositionWithAccel mode the library will calculate the speed automatically
  if (registerMap.motorControl.runToPosition || registerMap.motorControl.runContinuous)
  {
    //In runSpeedToPos mode, speed must be set after move for accel stepper library to work (see runSpeedtoPosition example)
    //so the speed check is done after the move check.
    //The stepper library will change the current speed as needed
    //We can't check against the old map, we have to check against the current speed.
    if (stepper.speed() != convertToFloat(registerMap.speed))
    {
      stepper.setSpeed(convertToFloat(registerMap.speed));
      registerMapOld.speed = registerMap.speed;
    }
  }

  //0x24 unlock Speed NVM register
  if (registerMap.unlockSpeedNVM == 0xC4)
  {
    Serial.print("&");
    PORsettings.speed = registerMap.speed;
    recordPORsettings();
    registerMap.unlockSpeedNVM = 0;
  }

  //0x25 holdVoltage register
  if (registerMapOld.holdVoltage != registerMap.holdVoltage)
  {
    newSettingsToRecord = true;
    registerMapOld.holdVoltage = registerMap.holdVoltage;
  }

  //0x27 runVoltage register
  if (registerMapOld.runVoltage != registerMap.runVoltage)
  {
    newSettingsToRecord = true;
    registerMapOld.runVoltage = registerMap.runVoltage;
  }

  //0x29 i2cAddress register
  if (registerMapOld.i2cAddress != registerMap.i2cAddress)
  {
    newSettingsToRecord = true;
    registerMapOld.i2cAddress = registerMap.i2cAddress;

    PORsettings.i2cAddressState = ADR_STATE_SOFTWARE;
    recordPORsettings();

    startI2C(); //Begin operating at this new address
  }

  //digitalWrite(PIN_DEBUG, LOW);
} //End updateParameters()
