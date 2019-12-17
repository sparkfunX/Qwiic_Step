#define TWI_BUFFER_LENGTH BUFFER_LENGTH // this is because Arduino is dumb

//When Qwiic button receives data bytes from Master this function is called as an interrupt
void receiveEvent(int numberOfBytesReceived)
{
  registerNumber = Wire.read();  //Get the memory map offset from the user

  updateFlag = true;

  //Begin recording the following incoming bytes to the temp memory map
  //starting at the registerNumber (the first byte received)
  for (uint8_t x = 0; x < numberOfBytesReceived - 1; x++)
  {
    uint8_t temp = Wire.read(); //We might record it, we might throw it away

    if ((x + registerNumber) < sizeof(memoryMap))
    {
      //Clense the incoming byte against the read only protected bits
      //Store the result into the register map
      *(registerPointer + registerNumber + x) &= ~*(protectionPointer + registerNumber + x);  //Clear this register if needed
      *(registerPointer + registerNumber + x) |= temp & *(protectionPointer + registerNumber + x);  //Or in the user's request (clensed against protection bits)
    }
  }
}

//Respond to read commands
//When the Qwiic Step gets a request for data from the user, this function is called as an interrupt
//The interrupt will respond with bytes starting from the last byte the user sent to us to the end of memory map
//Can only write 32 bytes at a time. Conditional takes care of cases where we write too many byte (memoryMap > 32 bytes)
void requestEvent()
{
  //update status of isLimited, what is the state of the interrupt pin?
  registerMap.motorStatus.isLimited = !digitalRead(pin_interrupt0);   //take the inverse of the interrupt pin because it is pulled high

  //update motor status
  float currentSpeed = stepper.speed();

  if (stepper.isRunning())
    registerMap.motorStatus.isRunning = 1;
  else {
    registerMap.motorStatus.isRunning = 0;
    registerMap.motorStatus.isAccelerating = 0;
    registerMap.motorStatus.isDecelerating = 0;
  }

  if (previousSpeed < currentSpeed) {
    //    Serial.println("I'm acclerating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    registerMap.motorStatus.isAccelerating = 1;
    registerMap.motorStatus.isDecelerating = 0;
  }
  else if (previousSpeed > currentSpeed) {
    //    Serial.println("I'm decelerating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    registerMap.motorStatus.isAccelerating = 0;
    registerMap.motorStatus.isDecelerating = 1;
  }
  else {
    //    Serial.println("I'M NOT MOVING AT ALLLLLLLL~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    registerMap.motorStatus.isAccelerating = 0;
    registerMap.motorStatus.isDecelerating = 0;
  }

  //update previous speed
  previousSpeed = currentSpeed;

  //check if we have made it to our target position
  if (stepper.targetPosition() == stepper.currentPosition())
    registerMap.motorStatus.isReached = 1;
  else
    registerMap.motorStatus.isReached = 0;

  //Write to I2C bus
  uint8_t len = (sizeof(memoryMap) - registerNumber);
  Wire.write((uint8_t*)(registerPointer + registerNumber), (len > TWI_BUFFER_LENGTH) ? TWI_BUFFER_LENGTH : len );
}

void limitSwitchTriggered()
{
  //update status of motor
  //isLimited status depends on the state of the interrupt pin
  registerMap.motorStatus.isLimited = !digitalRead(pin_interrupt0);   //take the inverse of the interrupt pin because it is pulled high

  if (registerMap.motorConfig.stopOnLimitSwitchPress) {
    //stop running motor
    stepper.stop();
  }
}

void eStopTriggered()
{
  //Stop the motor
  stepper.stop();

  registerMap.motorStatus.eStopped = true;
  
  stepper.setSpeed(0);
  registerMap.setSpeed = 0;
  registerMapOld.setSpeed = 0;
  stepper.setMaxSpeed(0);
  registerMap.maxSpeed = 0;
  registerMapOld.setSpeed = 0;
  stepper.setAcceleration(0);
  registerMap.acceleration = 0;
  registerMapOld.acceleration = 0;

  if (registerMap.motorConfig.powerDownPositionReached)
    stepper.disableOutputs();
}
