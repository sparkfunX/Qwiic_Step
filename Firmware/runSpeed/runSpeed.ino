#include <AccelStepper.h>

#define STEP 7
#define DIRECTION 8
#define ENABLE 10
#define MS1 4
#define MS2 5
#define MS3 6

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIRECTION);  //Defaults to AccelStepper::FULL4WIRE

enum setOptions {
  STEP_SIZE_FULL = 0,
  STEP_SIZE_HALF,
  STEP_SIZE_QUARTER,
  STEP_SIZE_EIGHTH,
  STEP_SIZE_SIXTEENTH
};

long startTime;

void setup()
{
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  setStepSize(STEP_SIZE_FULL);

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(-350);

  startTime = millis();
}

void loop(){
  stepper.runSpeed();

  if(millis() - startTime > 2000)
  {
    //Let's do a slow stop

    //stepper.stop(); //Nope - can have artifacts, partial steps afterwards.
    stepper.setSpeed(0); //Good
    while(1) stepper.runSpeed();
  }
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
