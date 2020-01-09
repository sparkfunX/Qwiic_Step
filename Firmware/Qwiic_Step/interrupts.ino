#define TWI_BUFFER_LENGTH BUFFER_LENGTH // this is because Arduino is dumb

//Respond to write commands
//When Qwiic button receives data bytes from Master this function is called as an interrupt
//Adjusts memoryMap with incoming data bytes
void receiveEvent(int numberOfBytesReceived)
{    
  registerNumber = Wire.read();  //Get the memory map offset from the user

  //DEBUGGING: need to clear this up...
//  if (registerNumber == 0x07 || registerNumber == 0x0F || registerNumber == 0x14 || registerNumber == 0x18 || registerNumber == 0x1C || registerNumber == 0x20){
    //set flag, we have to update state once the write has completed
    updateFlag = true;
//  }
  
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
  //DEBUG: doesn't the user have to clear this?!
  //update status of isLimited, what is the state of the interrupt pin?
  //will hopefully clear isLimited bit when limit switch is released

//  registerMap.motorStatus.isLimited = !digitalRead(pin_interrupt0);   //take the inverse of the interrupt pin because it is pulled high


  //Write to I2C bus
  uint8_t len = (sizeof(memoryMap) - registerNumber);
  Wire.write((uint8_t*)(registerPointer + registerNumber), (len > TWI_BUFFER_LENGTH) ? TWI_BUFFER_LENGTH : len );
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
  //update status of motor
  //isLimited status depends on the state of the interrupt pin
  registerMap.motorStatus.isLimited = !digitalRead(PIN_INTERRUPT0);   //take the inverse of the interrupt pin because it is pulled high

  //stop the motor is user has configured it to
  if (registerMap.motorConfig.stopOnLimitSwitchPress) {
    //stop running motor
    stepper.stop();
  }

  if (registerMap.motorConfig.disableMotorPositionReached)
    stepper.disableOutputs();
}
