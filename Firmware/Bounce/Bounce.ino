// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#define TEST_SETUP 1
//Hardware connections
//test set-up pins
#if defined(TEST_SETUP)
const uint8_t stp = 7;
const uint8_t dir = 8;
const uint8_t pin_MS1 = 4;
const uint8_t pin_MS2 = 5;
const uint8_t pin_MS3 = 6;
const uint8_t pin_interrupt0 = 2;   //E-stop
const uint8_t pin_interrupt1 = 3;   //Limit switch
const uint8_t pin_externalInterrupt = A1;
#elif
const uint8_t stp = A3;
const uint8_t dir = 6;
const uint8_t pin_MS1 = 9;
const uint8_t pin_MS2 = 8;
const uint8_t pin_MS3 = 7;
const uint8_t addressPin = 11;
const uint8_t curr_ref_pwm = 5;
const uint8_t curr_sense = A6;   //DEBUG: right way to reference these pins?
const uint8_t a49885_reset = A7; //DEBUG: might not work... is pin only ADC input?
const uint8_t pin_interrupt0 = 2;   //E-Stop
const uint8_t pin_interrupt1 = 3;   //Limit switch
const uint8_t pin_externalInterrupt = A1;
#endif

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stp, dir); //Stepper driver, 2 pins required

void setup()
{  
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(20);
  stepper.moveTo(500);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());

    stepper.run();
}
