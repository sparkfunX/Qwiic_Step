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
  if(registerNumber > offsetof(struct memoryMap, move) && (registerNumber + numberOfBytesReceived) < offsetof(struct memoryMap, move))
  {
    newMoveValue = true;
  }

  //Check to see if we need to release the INT pin
  if (interruptState == STATE_INT_INDICATED)
  {
    //If the user has cleared all the interrupt bits then clear interrupt pin
    if (registerMap.motorStatus.isReached == 0 && registerMap.motorStatus.isLimited == 0)
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
  //DEBUG: doesn't the user have to clear this?!
  //update status of isLimited, what is the state of the interrupt pin?
  //will hopefully clear isLimited bit when limit switch is released

  //  registerMap.motorStatus.isLimited = !digitalRead(pin_interrupt0);   //take the inverse of the interrupt pin because it is pulled high

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
  if (interruptState == STATE_INT_CLEARED)
  {
    Serial.println("isLimited interrupt!");
    interruptState = STATE_ISLIMITED_INT; //Go to next state
  }

}
