//Declare pin functions on Arduino
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN 7

//Variables
int x;
int y;
int state;
int pos;
int distgo;
bool isRunning;
int maxSp;

#include <AccelStepper.h>

//Define a stepper and its pins
AccelStepper stepper(AccelStepper::DRIVER , stp, dir); //Stepper driver, 2 driver pins required.

void setup() {
  Serial.begin(115200);

  //Set mode of microstep pins
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  FullStepMode();

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(1000);
  stepper.setAcceleration(1000);

//  maxSp = stepper.maxSpeed();
//  Serial.print("The max speed is: ");
//  Serial.println(maxSp);
  
  stepper.move(400); 
}

void loop() {
//  if (stepper.distanceToGo() == 0)
//    stepper.move(-20);
  
  stepper.run();
//  stepper.runSpeed();

//  isRunning = stepper.isRunning();
//  Serial.print("Is the motor currently running: ");
//  Serial.println(isRunning);
//  delay(10);

//  pos = stepper.currentPosition();
//  distgo = stepper.distanceToGo();
//  Serial.print("Current position is: ");
//  Serial.print(pos);
//  Serial.print(" Distance to go: ");
//  Serial.println(distgo);
//  delay(10);
}

void FullStepMode(){
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
}

void HalfStepMode(){
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
}

void QuarterStepMode(){
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);
}

void EighthStepMode(){
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);
}

void SixteenthStepMode(){
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
}
