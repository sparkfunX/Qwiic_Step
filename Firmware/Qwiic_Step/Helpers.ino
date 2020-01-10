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
  interruptState = STATE_INT_CLEARED; //Go to next state
}

void setInterruptPin()
{
  //Set the interrupt pin low to indicate interrupt
  pinMode(PIN_INT_OUTPUT, OUTPUT);
  digitalWrite(PIN_INT_OUTPUT, LOW);
  interruptState = STATE_INT_INDICATED;
}