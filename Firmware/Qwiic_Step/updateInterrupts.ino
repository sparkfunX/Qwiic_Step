//Determines if we need to activate (ie pull down) the interrupt output pin
//or if we need to release (set to high impedance) the pin
void updateInterruptPin()
{
  //Check if limit switch is open
  if (digitalRead(PIN_LIMIT_SWITCH) == HIGH)
  {
    if (limitState == LIMIT_STATE_LIMITED_CLEARED)
    {
      //Change states
      limitState = LIMIT_STATE_NOT_LIMITED;
      Serial.println("Limit released!");
    }
  }

  //Handle the Move state machine
  //There are three states: MOVING, NOTMOVING_ISREACH_SET, NOTMOVING_ISREACH_CLEARED
  //MOVING happens when user has sent a Move value. Entered from any state.
  //NOTMOVING_ISREACH_SET happens once we've completed the previous MOVING state
  //NOTMOVING_ISREACH_CLEARED happens from _SET once user has cleared the bit
  if (moveState == MOVE_STATE_MOVING)
  {
    //Check if we have made it to our target position
    if (stepper.targetPosition() == stepper.currentPosition())
    {
      //Serial.println("Arrived");
      moveState = MOVE_STATE_NOTMOVING_ISREACH_SET;
      registerMap.motorStatus.isReached = true;

      //Do we need to disable the motor?
      if (registerMap.motorConfig.disableMotorOnPositionReached == true)
        stepper.disableOutputs();

    } //We do not clear the isReached bit. The user must actively clear it which will clear the interrupt as well.
  }
  else if (moveState == MOVE_STATE_NOTMOVING_ISREACH_SET)
  {
    if (registerMap.motorStatus.isReached == false)
    {
      moveState = MOVE_STATE_NOTMOVING_ISREACH_CLEARED;
      //Serial.println("User cleared isReached");
    }
  }

  if (limitState == LIMIT_STATE_LIMITED_SET)
  {
    if (registerMap.motorStatus.isLimited == false)
    {
      limitState = LIMIT_STATE_LIMITED_CLEARED;
      //Serial.println("User cleared isLimited");
    }
  }

  //Interrupt pin state machine
  //There are two states: Int Cleared, Int Indicated
  //INT_INDICATED state is entered when either LIMIT_STATE_LIMITED_SET or MOVE_STATE_NOTMOVING_ISREACH_SET is satisfied
  //INT_CLEARED state is entered when user clears the isReached and isLimited bits
  if (interruptState == INT_STATE_CLEARED)
  {
    //If we have moved since the last interrupt, and we have reached the new position, then indicate interrupt
    if (moveState == MOVE_STATE_NOTMOVING_ISREACH_SET && registerMap.interruptConfig.isReachedInterruptEnable)
    {
      Serial.println("isReached interrupt!");
      setInterruptPin(); //Move to INT_STATE_INDICATED state
    }
    if (limitState == LIMIT_STATE_LIMITED_SET && registerMap.interruptConfig.isLimitedInterruptEnable)
    {
      Serial.println("isLimited interrupt!");
      setInterruptPin(); //Move to INT_STATE_INDICATED state
    }
  }
  //Check to see if we need to release the INT pin
  else if (interruptState == INT_STATE_INDICATED)
  {
    //If the user has cleared all the interrupt bits, or if we are moving and limit switch is not depressed
    //then clear interrupt pin
    if ( (moveState == MOVE_STATE_NOTMOVING_ISREACH_CLEARED || moveState == MOVE_STATE_MOVING)
         && (limitState == LIMIT_STATE_LIMITED_CLEARED || limitState == LIMIT_STATE_NOT_LIMITED)
       )
    {
      releaseInterruptPin(); //Move to INT_STATE_CLEARED state
    }
  }
}
