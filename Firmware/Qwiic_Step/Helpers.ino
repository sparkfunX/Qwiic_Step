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

//This will set the int pin to high impedance (aka pulled high by external resistor)
void releaseInterruptPin()
{
  digitalWrite(PIN_INT_OUTPUT, LOW);  //Push pin to disable internal pull-ups
  pinMode(PIN_INT_OUTPUT, INPUT);     //Go to high impedance
  interruptState = INT_STATE_CLEARED; //Go to next state
}

void setInterruptPin()
{
  Serial.println("INT pulled low");
  //Set the interrupt pin low to indicate interrupt
  pinMode(PIN_INT_OUTPUT, OUTPUT);
  digitalWrite(PIN_INT_OUTPUT, LOW);
  interruptState = INT_STATE_INDICATED;
}

//Determines the needed I2C address from NVM and starts as slave
//Registers the receive and request I2C interrupt handlers
void startI2C()
{
  uint8_t address;

  //TODO: Need to handle address jumper here.

  //Check if the address stored in memoryMap is valid
  if (registerMap.i2cAddress > 0x07 && registerMap.i2cAddress < 0x78)
    address = registerMap.i2cAddress;
  else //If the value is illegal, default to the default I2C address for our platform
    address = DEFAULT_I2C_ADDRESS;

  //Save new address to the register map
  registerMap.i2cAddress = address;

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

  Serial.print("Register map id: 0x");
  Serial.println(*(registerPointer + 0), HEX);

  Serial.print("Firmware version: 0x");
  if (*(registerPointer + 2) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 2), HEX);
  if (*(registerPointer + 1) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 1), HEX);

  Serial.print("Interrupt enable: 0x");
  Serial.println(*(registerPointer + 3), HEX);

  Serial.print("Motor status: 0x");
  Serial.println(*(registerPointer + 4), HEX);

  Serial.print("Device config: 0x");
  Serial.println(*(registerPointer + 5), HEX);

  Serial.print("Motor config: ");
  if (registerMap.motorControl.run == true) Serial.print("run");
  else if (registerMap.motorControl.runSpeed == true) Serial.print("runSpeed");
  else if (registerMap.motorControl.runSpeedToPosition == true) Serial.print("runSpeedToPosition");
  else if (registerMap.motorControl.hardStop == true) Serial.print("stop");
  Serial.println();
  //Serial.println(*(registerPointer + 6), HEX);

  Serial.print("Current position: 0x");
  if (*(registerPointer + 0xA) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xA), HEX);
  if (*(registerPointer + 0x9) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x9), HEX);
  if (*(registerPointer + 0x8) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x8), HEX);
  if (*(registerPointer + 0x7) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x7), HEX);

  Serial.print("Distance to go: 0x");
  if (*(registerPointer + 0xE) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xE), HEX);
  if (*(registerPointer + 0xD) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xD), HEX);
  if (*(registerPointer + 0xC) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0xC), HEX);
  if (*(registerPointer + 0xB) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0xB), HEX);

  Serial.print("Move: 0x");
  if (*(registerPointer + 0x12) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x12), HEX);
  if (*(registerPointer + 0x11) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x11), HEX);
  if (*(registerPointer + 0x10) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x10), HEX);
  if (*(registerPointer + 0xF) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0xF), HEX);

  Serial.print("Enable move NVM: 0x");
  Serial.println(*(registerPointer + 0x13), HEX);

  Serial.print("Move to: 0x");
  if (*(registerPointer + 0x17) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x17), HEX);
  if (*(registerPointer + 0x16) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x16), HEX);
  if (*(registerPointer + 0x15) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x15), HEX);
  if (*(registerPointer + 0x14) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x14), HEX);

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

  Serial.print("Enable set speed: 0x");
  Serial.println(*(registerPointer + 0x24), HEX);

  Serial.print("Hold current: 0x");
  if (*(registerPointer + 0x26) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x26), HEX);
  if (*(registerPointer + 0x25) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x25), HEX);

  Serial.print("Run current: 0x");
  if (*(registerPointer + 0x28) < 0x10)
    Serial.print("0");
  Serial.print(*(registerPointer + 0x28), HEX);
  if (*(registerPointer + 0x27) < 0x10)
    Serial.print("0");
  Serial.println(*(registerPointer + 0x27), HEX);

  Serial.print("I2C address: 0x");
  Serial.println(*(registerPointer + 0x29), HEX);
#endif
}
