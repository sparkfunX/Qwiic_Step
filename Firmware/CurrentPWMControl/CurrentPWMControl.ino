
#include <AccelStepper.h>

#define STEP A3
#define DIRECTION 6
#define ENABLE 10
#define MS1 9
#define MS2 8
#define MS3 7

//#define CURRENT_REFERENCE 11 //PWM capapble pin

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
  Serial.begin(115200);
  Serial.println("SparkFun Stepper Example");

  delay(5); //Wait for Easy Driver to wake up

  //stepper.setEnablePin(ENABLE);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  //digitalWrite(ENABLE, HIGH);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  //Set to full step mode
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

//  setStepSize(STEP_SIZE_FULL);
//  setStepSize(STEP_SIZE_SIXTEENTH);

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(400);
  stepper.setAcceleration(1000);

  stepper.move(400);
}

void loop()
{
  Serial.println(stepper.distanceToGo());
  stepper.run();
  
//  stepper.runSpeed();
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
