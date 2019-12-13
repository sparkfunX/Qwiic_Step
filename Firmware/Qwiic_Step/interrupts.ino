//When Qwiic button receives data bytes from Master this function is called as an interrupts
#define TWI_BUFFER_LENGTH BUFFER_LENGTH // this is because Arduino is dumb


void receiveEvent(int numberOfBytesReceived)
{
//  Wire.flush(); // clear outgoing buffer (may have been loaded with extra bytes last time that requestEvent was called)
  
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
  uint8_t len = (sizeof(memoryMap) - registerNumber);  
  Wire.write((uint8_t*)(registerPointer + registerNumber), (len > TWI_BUFFER_LENGTH) ? TWI_BUFFER_LENGTH : len );
}
