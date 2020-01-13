#define TWI_BUFFER_LENGTH BUFFER_LENGTH // this is because Arduino is dumb

//Respond to write commands
//When board receives data bytes from Master this function is called as an interrupt
//Adjusts memoryMap with incoming data bytes
void receiveEvent(int numberOfBytesReceived)
{
  registerNumber = Wire.read(); //Get the memory map offset from the user

  newData = true; //Tell the main loop there is new data to process.

  //Begin recording the following incoming bytes to the temp memory map
  //starting at the registerNumber (the first byte received)
  for (uint8_t x = 0; x < numberOfBytesReceived - 1; x++)
  {
    uint8_t temp = Wire.read(); //We might record it, we might throw it away

    if ((x + registerNumber) < sizeof(memoryMap))
    {
      //Clense the incoming byte against the read only protected bits
      //Store the result into the register map
      *(registerPointer + registerNumber + x) &= ~*(protectionPointer + registerNumber + x);       //Clear this register if needed
      *(registerPointer + registerNumber + x) |= temp & *(protectionPointer + registerNumber + x); //Or in the user's request (clensed against protection bits)
    }
  }

  //See if user has written a new Move value
  //Because Move is relative, we cannot simply tell by the value in the register. ie, the user might write 400, then 400 again.
  //We would need to move 400 steps, then another 400.
  //We also need to make sure the user is not sending us a 'read the Move register' command. So we check that the bytes received are greater
  //than 1.
  if (registerNumber <= offsetof(struct memoryMap, move) && (registerNumber + numberOfBytesReceived) > (offsetof(struct memoryMap, move) + sizeof(registerMap.move)))
  {
    newMoveValue = true;
  }

  //Check to see if we need to release the INT pin
  if (interruptState == INT_STATE_INDICATED)
  {
    //If the user has cleared all the interrupt bits then clear interrupt pin
    if (moveState == MOVE_STATE_NOTMOVING_ISREACH_CLEARED && registerMap.motorStatus.isLimited == 0)
    {
      Serial.println("INT cleared");
      releaseInterruptPin();
    }
  }
}

//Respond to read commands
//When the Qwiic Step gets a request for data from the user, this function is called as an interrupt
//The interrupt will respond with bytes starting from the last byte the user sent to us to the end of memory map
//Can only write 32 bytes at a time. Conditional takes care of cases where we write too many byte (memoryMap > 32 bytes)
void requestEvent()
{
  //Write to I2C bus
  uint8_t len = (sizeof(memoryMap) - registerNumber);
  Wire.write((uint8_t *)(registerPointer + registerNumber), (len > TWI_BUFFER_LENGTH) ? TWI_BUFFER_LENGTH : len);
}

void eStopTriggered()
{
  //Stop the motor
  stepper.stop();

  //update status bit
  //user needs to manually clear this bit for operations to continue after an e-stop event
  registerMap.motorStatus.eStopped = true;

  //call accelstepper library functions and update memoryMap accordingly
  stepper.setSpeed(0);
  registerMap.speed = 0;
  registerMapOld.speed = 0;
  stepper.setMaxSpeed(0);
  registerMap.maxSpeed = 0;
  registerMapOld.maxSpeed = 0;
  stepper.setAcceleration(0);
  registerMap.acceleration = 0;
  registerMapOld.acceleration = 0;

  //disable power if user has configured motor to do so
  if (registerMap.motorConfig.disableMotorPositionReached)
    stepper.disableOutputs();
}

void limitSwitchTriggered()
{
  Serial.print("&");
  if (registerMap.motorStatus.isLimited == false)
  {
    Serial.println("Limit!");

    registerMap.motorStatus.isLimited = true;

    //Stop motor if option is enabled
    if (registerMap.motorConfig.stopOnLimitSwitchPress == true)
    {
      stepper.stop();
    }

    //TODO: Disable motor outputs if option enabled
    //  if (registerMap.motorConfig.disableMotorPositionReached)
    //    stepper.disableOutputs();

    //Change interrupt handler state if necessary
    if (interruptState == INT_STATE_CLEARED)
    {
      Serial.println("isLimited interrupt!");
      interruptState = INT_STATE_ISLIMITED; //Go to next state
    }
  }
}
