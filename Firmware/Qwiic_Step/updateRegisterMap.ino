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
