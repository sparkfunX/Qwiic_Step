#include <AccelStepper.h>

#define STEP  A3
#define DIRECTION 6
#define ENABLE 10
#define MS1 9
#define MS2 8
#define MS3 7

#define CURRENT_REFERENCE 5  //PWM capable pin
#define CURRENT_SENSE A6    //ADC pin

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIRECTION); 

int v_out;
float v_sense;
float i_sense;
float r_sense;

uint16_t n = 0;
long sum = 0;
int avg;

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

  //Configure full step mode
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(600);

  //Initialize the value of r_sense
  r_sense = 0.25; 

  //PWM signal of 0.1V at current reference pin
  int duty_cycle = (1/3.3) * 255;
  analogWrite(CURRENT_REFERENCE, duty_cycle);
}

void loop()
{
  v_out = analogRead(CURRENT_SENSE);
  v_sense = v_out/5;
  i_sense = v_sense/r_sense;

  sum += v_out;
  avg = sum / n;

//  Serial.print("This is v_out: ");
  Serial.println(avg);
//  Serial.print("This is the sense current: ");
//  Serial.println(i_sense);
  
  stepper.runSpeed();
  delay(1);
  n++;
}
