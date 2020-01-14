#include <AccelStepper.h>

#define STEP 7
#define DIRECTION 8
#define MS1 4
#define MS2 5
#define MS3 6

#define PIN_ENABLE 10

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIRECTION); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

enum stepOptions {
  STEP_SIZE_FULL = 0,
  STEP_SIZE_HALF,
  STEP_SIZE_QUARTER,
  STEP_SIZE_EIGHTH,
  STEP_SIZE_SIXTEENTH
};

long startTime;


void setup()
{
  Serial.begin(9600);
  Serial.println("SparkFun Stepper Example");

  //pinMode(PIN_ENABLE, OUTPUT);
  //digitalWrite(PIN_ENABLE, HIGH); //Disable the stepper driver
  //digitalWrite(PIN_ENABLE, LOW); //Enable the stepper driver

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  stepper.setEnablePin(PIN_ENABLE);
  stepper.setPinsInverted(false, false, true); //Invert enable
  stepper.enableOutputs();

  setStepSize(STEP_SIZE_FULL);
  //setStepSize(STEP_SIZE_SIXTEENTH);

  stepper.setMaxSpeed(600); //Required. Upper limit of ~600 before we start to miss steps
  //stepper.setSpeed(400); //Not required
  stepper.setAcceleration(400); //Upper limit of ~500 before ATmega starts missing steps

  stepper.move(4 * 200 * 10);

  startTime = millis();
}

void loop()
{
  stepper.run();

  if (millis() - startTime > 1500)
  {
    //stepper.disableOutputs(); //Disable the stepper driver
    //while (1); //Hard stop

    //stepper.move(0); //Does odd things.
    stepper.stop(); //Spins to halt using accel parameter    

    while(1);// stepper.run();
    
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
