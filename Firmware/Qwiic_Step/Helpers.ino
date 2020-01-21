//converts uint32_t type to float
//needed for floats in the memoryMap struct
//reading/writing to floats in struct doesn't work as expected
float convertToFloat(uint32_t myVal)
{
  union {
    float f;
    uint8_t b[4];
  } u;
  u.b[3] = myVal >> (8 * 3);
  u.b[2] = myVal >> (8 * 2);
  u.b[1] = myVal >> (8 * 1);
  u.b[0] = myVal >> (8 * 0);
  return u.f;
}


//Determines the needed I2C address from NVM and starts as slave
//Registers the receive and request I2C interrupt handlers
void startI2C()
{
  //Based on the PORsettings, figure out what I2C address state we're in
  if (PORsettings.i2cAddressState == ADR_STATE_SOFTWARE)
  {
    if (digitalRead(PIN_ADDRESS) == LOW) //Jumper is closed
    {
      //Change states
      Serial.println("I2C in jumper state");
      PORsettings.i2cAddressState = ADR_STATE_JUMPER;
      recordPORsettings();
    }
  }
  else if (PORsettings.i2cAddressState == ADR_STATE_JUMPER)
  {
    if (digitalRead(PIN_ADDRESS) == HIGH) //Jumper open
    {
      //Change states
      Serial.println("I2C in software state");
      PORsettings.i2cAddressState = ADR_STATE_SOFTWARE;
      recordPORsettings();
    }
  }

  //Based on I2C Address state, pick our address
  uint8_t address;
  if (PORsettings.i2cAddressState == ADR_STATE_SOFTWARE)
  {
    //Check if the address stored in memoryMap is valid
    if (registerMap.i2cAddress > 0x07 && registerMap.i2cAddress < 0x78)
      address = registerMap.i2cAddress;
    else //If the value is illegal, default to the default I2C address
      address = I2C_ADDRESS_DEFAULT;
  }
  else if (PORsettings.i2cAddressState == ADR_STATE_JUMPER)
  {
    address = I2C_ADDRESS_FORCED; //Force address to I2C_ADDRESS_FORCED if user has closed the solder jumper
    registerMap.i2cAddress = address;
  }

  Wire.end();
  Wire.begin(address); //Rejoin the I2C bus on new address

  //Connect receive and request events to the interrupts
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

//Prints the current register map
//Note: some of these values are floating point so HEX printing will look odd.
void printState()
{
#ifndef PRODUCTION_TARGET
  Serial.println();

  Serial.println("Register map:");

  Serial.print("Firmware version: 0x");
  if (*(registerPointer + 2) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 2), HEX);
  if (*(registerPointer + 1) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 1), HEX);

  Serial.print("Interrupt Config:");
  if (registerMap.interruptConfig.isReachedInterruptEnable)
    Serial.print(" (isReachedEnable)");
  if (registerMap.interruptConfig.isLimitedInterruptEnable)
    Serial.print(" (isLimitedEnable)");
  Serial.println();
  //  Serial.println(*(registerPointer + 3), HEX);

  Serial.print("Motor Status:");
  if (registerMap.motorStatus.isRunning)
    Serial.print(" (isRunning)");
  else
    Serial.print(" (Stopped)");
  if (registerMap.motorStatus.isAccelerating)
    Serial.print(" (isAccelerating)");
  if (registerMap.motorStatus.isDecelerating)
    Serial.print(" (isDecelerating)");
  if (registerMap.motorStatus.isReached)
    Serial.print(" (isReached)");
  if (registerMap.motorStatus.isLimited)
    Serial.print(" (isLimited)");
  if (registerMap.motorStatus.eStopped)
    Serial.print(" (isEStopped)");
  Serial.println();
  //  Serial.println(*(registerPointer + 4), HEX);

  Serial.print("Motor Config:");
  int stepConfig = 0;
  stepConfig |= registerMap.motorConfig.ms1;
  stepConfig |= registerMap.motorConfig.ms2 << 1;
  stepConfig |= registerMap.motorConfig.ms3 << 2;
  switch (stepConfig)
  {
    case 0:
      Serial.print(" (Full");
      break;
    case 1:
      Serial.print(" (Half");
      break;
    case 2:
      Serial.print(" (Quarter");
      break;
    case 3:
      Serial.print(" (Eighth");
      break;
    case 7:
      Serial.print(" (Sixteenth");
      break;
  }
  Serial.print("Step)");
  if (registerMap.motorConfig.disableMotorOnEStop) Serial.print(" (disableMotorOnEStop)");
  if (registerMap.motorConfig.disableMotorOnPositionReached) Serial.print(" (disableMotorOnPositionReached)");
  if (registerMap.motorConfig.stopOnLimitSwitchPress) Serial.print(" (stopOnLimitSwitchPress)");
  Serial.println();
  //Serial.println(*(registerPointer + 5), HEX);

  Serial.print("Motor Mode:");
  if (registerMap.motorControl.runToPosition == true) Serial.print(" (runToPos)");
  else if (registerMap.motorControl.runToPositionWithAccel == true) Serial.print(" (runToPosWithAccel)");
  else if (registerMap.motorControl.runContinuous == true) Serial.print(" (runContinuous)");
  else if (registerMap.motorControl.hardStop == true) Serial.print(" (hardStop)");
  if (registerMap.motorControl.disableMotor == true) Serial.print(" (disableMotor)");
  Serial.println();
  //Serial.println(*(registerPointer + 6), HEX);

  Serial.print("Current position: ");
  Serial.println(registerMap.currentPos);
  //  if (*(registerPointer + 0xA) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0xA), HEX);
  //  if (*(registerPointer + 0x9) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x9), HEX);
  //  if (*(registerPointer + 0x8) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x8), HEX);
  //  if (*(registerPointer + 0x7) < 0x10)
  //    Serial.print("0");
  //  Serial.println(*(registerPointer + 0x7), HEX);

  Serial.print("Distance to go: ");
  Serial.println(registerMap.distanceToGo);
  //  if (*(registerPointer + 0xE) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0xE), HEX);
  //  if (*(registerPointer + 0xD) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0xD), HEX);
  //  if (*(registerPointer + 0xC) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0xC), HEX);
  //  if (*(registerPointer + 0xB) < 0x10)
  //    Serial.print("0");
  //  Serial.println(*(registerPointer + 0xB), HEX);

  Serial.print("Move: ");
  Serial.println(registerMap.move);
  //  if (*(registerPointer + 0x12) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x12), HEX);
  //  if (*(registerPointer + 0x11) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x11), HEX);
  //  if (*(registerPointer + 0x10) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x10), HEX);
  //  if (*(registerPointer + 0xF) < 0x10)
  //    Serial.print("0");
  //  Serial.println(*(registerPointer + 0xF), HEX);

  Serial.print("Unlock move NVM: 0x");
  Serial.println(*(registerPointer + 0x13), HEX);

  Serial.print("Move to: ");
  Serial.println(registerMap.moveTo);
  //  if (*(registerPointer + 0x17) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x17), HEX);
  //  if (*(registerPointer + 0x16) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x16), HEX);
  //  if (*(registerPointer + 0x15) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x15), HEX);
  //  if (*(registerPointer + 0x14) < 0x10)
  //    Serial.print("0");
  //  Serial.println(*(registerPointer + 0x14), HEX);

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

  Serial.print("Unlock speed NVM: 0x");
  Serial.println(*(registerPointer + 0x24), HEX);

  Serial.print("Hold current: ");
  Serial.print(registerMap.holdCurrent);
  Serial.print(" or ");
  Serial.print(convertCurrentToVoltage(registerMap.holdCurrent), 3);
  Serial.println("V");
  //  if (*(registerPointer + 0x26) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x26), HEX);
  //  if (*(registerPointer + 0x25) < 0x10)
  //    Serial.print("0");
  //  Serial.println(*(registerPointer + 0x25), HEX);

  Serial.print("Run current: ");
  Serial.print(registerMap.runCurrent);
  Serial.print(" or ");
  Serial.print(convertCurrentToVoltage(registerMap.runCurrent), 3);
  Serial.println("V");
  //  if (*(registerPointer + 0x28) < 0x10)
  //    Serial.print("0");
  //  Serial.print(*(registerPointer + 0x28), HEX);
  //  if (*(registerPointer + 0x27) < 0x10)
  //    Serial.print("0");
  //  Serial.println(*(registerPointer + 0x27), HEX);

  Serial.print("I2C address: 0x");
  Serial.println(*(registerPointer + 0x29), HEX);

  Serial.print("POR move: ");
  Serial.println(PORsettings.move);
  Serial.print("POR speed: ");
  Serial.println(PORsettings.speed);
#endif
}

//Print debug statements only if we have debug enabled
//void debug(char * message)
//{
//  if(printDebug)
//  {
//    Serial.print(debug);
//  }
//}
