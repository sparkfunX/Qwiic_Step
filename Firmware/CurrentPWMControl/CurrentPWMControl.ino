#include <AccelStepper.h>

#define STEP A3
#define DIRECTION 6
#define ENABLE 10
#define MS1 9
#define MS2 8
#define MS3 7

#define CURRENT_REFERENCE 5 //PWM capapble pin

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIRECTION); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Stepper Example");

  delay(5); //Wait for Easy Driver to wake up

  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  //Set to full step mode
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(600);

  //PWM signal of 1.7V at current reference pin
  //NOTE: I think 1.7V on the A49885 VREF pin corresponds to ~2A run current
  int duty_cycle = (1.7/3.3) * 255;
  analogWrite(CURRENT_REFERENCE, duty_cycle);
}

void loop()
{
  stepper.runSpeed();
}
