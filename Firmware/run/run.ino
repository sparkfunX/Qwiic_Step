#include <AccelStepper.h>

#define STEP 7
#define DIRECTION 8
#define ENABLE 10
#define MS1 4
#define MS2 5
#define MS3 6

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

  setStepSize(STEP_SIZE_FULL);
    //setStepSize(STEP_SIZE_SIXTEENTH);

  stepper.setMaxSpeed(1000); //Required but no upper limit
  //stepper.setSpeed(400); //Not required
  stepper.setAcceleration(500); //Upper limit of ~500 before ATmega starts missing steps

  stepper.move(4 * 200);
}

void loop()
{
  stepper.run();
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
