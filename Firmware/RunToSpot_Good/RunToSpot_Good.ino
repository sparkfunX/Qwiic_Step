
#include <AccelStepper.h>

#define STEP 7
#define DIRECTION 8
#define ENABLE 10
#define MS1 2
#define MS2 3
#define MS3 4

#define CURRENT_REFERENCE 11 //PWM capapble pin

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIRECTION); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

enum stepOptions {
  STEP_SIZE_FULL = 0,
  STEP_SIZE_HALF,
  STEP_SIZE_QUARTER,
  STEP_SIZE_EIGHTH,
  STEP_SIZE_SIXTEENTH
};

void setup()
{
  Serial.begin(9600);
  Serial.println("SparkFun Stepper Example");

  delay(5); //Wait for Easy Driver to wake up

  //stepper.setEnablePin(ENABLE);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  //digitalWrite(ENABLE, HIGH);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  //setStepSize(STEP_SIZE_FULL);
  setStepSize(STEP_SIZE_QUARTER);
  //setStepSize(STEP_SIZE_EIGHTH);
  //setStepSize(STEP_SIZE_SIXTEENTH);

//  stepper.setMaxSpeed(5000);
//  stepper.setAcceleration(15);
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);

  stepper.moveTo(-32000); //Clockwise
  
  //stepper.moveTo(200 * 16); //Counter clockwise one rotation at 1/16 step
  //stepper.moveTo(200 * 4); //Counter clockwise one rotation at 1/8 step
  //stepper.moveTo(200 * 4 * 4); //Counter clockwise four rotations at 1/8 step

  //stepper.moveTo(200 * 16); //Counter clockwise one rotation at 1/16 step

  //stepper.setSpeed(4000); //Good for 16th
  //stepper.setSpeed(2000); //Good for 8th or large steps
  //stepper.setSpeed(1000); //Good for 1/4 or larger steps
  
}

void loop()
{
  stepper.run();
  //stepper.runSpeed();
  //stepper.runSpeedToPosition();

  //stepper.runToNewPosition(10000);
}

//Sets MS pins to user's input
void setStepSize(uint8_t stepSize)
{
  switch (stepSize)
  {
    case STEP_SIZE_FULL:
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, LOW);
      digitalWrite(MS3, LOW);
      break;
    case STEP_SIZE_HALF:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, LOW);
      digitalWrite(MS3, LOW);
      break;
    case STEP_SIZE_QUARTER:
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
    case STEP_SIZE_EIGHTH:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
    case STEP_SIZE_SIXTEENTH:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, HIGH);
      break;
    default:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, HIGH);
      break;
  }
}
