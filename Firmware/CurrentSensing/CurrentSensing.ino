#include <AccelStepper.h>

#define STEP  A3
#define DIRECTION 6
#define ENABLE 10
#define MS1 9
#define MS2 8
#define MS3 7

#define CURRENT_REFERENCE 5  //PWM capable pin
//#define CURRENT_SENSE A6    //ADC pin
#define CURRENT_SENSE A7  //NEW hacked current sensor ADC pin

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIRECTION); 

int adc_out;
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

  pinMode(A7, INPUT);

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
  int duty_cycle = (1/3.3) * 255; //
//  int duty_cycle = (1.7/3.3) * 255; //.724mA
//  int duty_cycle = (1.7/3.3) * 255; //
//  int duty_cycle = (1.7/3.3) * 255;
//  int duty_cycle = (1.7/3.3) * 255;
//  int duty_cycle = (1.7/3.3) * 255;
//  int duty_cycle = (1.7/3.3) * 255;
  analogWrite(CURRENT_REFERENCE, duty_cycle);
}

#define numberOfSamples 16
int sense[numberOfSamples] = {0};
byte senseSpot = 0;

void loop()
{
  adc_out = analogRead(CURRENT_SENSE);
  sense[senseSpot++] = adc_out;
//  if (senseSpot == numberOfSamples)
//    senseSpot = 0;  //6806

  senseSpot %= numberOfSamples;  //6796

  Serial.print(adc_out);
  Serial.print("\t");

  int adcAverage = avgOfSamples();

  Serial.print(adcAverage);
  Serial.print("\t");

  float adcMV = (adcAverage * 1000 * 3.3) / 1023;
  float adcMA = adcMV / 0.11;
  
  Serial.print(adcMV);
  Serial.print("\t");

  Serial.print(adcMA);
  Serial.print("\t");

  
  Serial.println();
  
//  Serial.print("This is the sense current: ");
//  Serial.println(i_sense);
  
  stepper.runSpeed();
  delay(1);
  n++;
}

int avgOfSamples(){
  uint16_t sum = 0;
  for (int i = 0; i < numberOfSamples; i++){
    sum += sense[i];
  }
  sum /= numberOfSamples;
  return sum;
}
