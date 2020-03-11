/*
  This is working nicely at 1/16th step and 1/4. It gets a bit noisy at full step.

  
 */
 
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

int adc_out;
float VREF;
int OPAMP_GAIN = 23; //Designed to be 23 but varies with resistor tolerances.

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Stepper Example");

  analogReference(INTERNAL); //Set AREF to 1.1V. This is pretty accurate at 1.098V measured.
  VREF = 1.1;

  pinMode(CURRENT_SENSE, INPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  //Configure full step mode
//  digitalWrite(MS1, LOW);
//  digitalWrite(MS2, LOW);
//  digitalWrite(MS3, LOW);

  //Configure 1/16th step mode
//  digitalWrite(MS1, HIGH);
//  digitalWrite(MS2, HIGH);
//  digitalWrite(MS3, HIGH);

  //Configure 1/4 step mode
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);

  //Configure 1/2 step mode
//  digitalWrite(MS1, HIGH);
//  digitalWrite(MS2, LOW);
//  digitalWrite(MS3, LOW);

  stepper.setMaxSpeed(10000);
  //stepper.setSpeed(-350); //Good at full step
  stepper.setSpeed(-350 * 16); //

  //Generate PWM signal of 2V (max current) at current reference pin
  analogWrite(CURRENT_REFERENCE, convertCurrentToPWM(1000));
}

#define numberOfSamples 32
int sense[numberOfSamples] = {0};
byte senseSpot = 0;

void loop()
{
  //Measured at .11 resistor, we measure 4.8mV
  //Through the opamp we measure 144.5mV
  //Amplification is 30.1
  //@ 4.8mV we should be consuming 43.6mA in this coil
  //We should be seeing 144mV or (.144 / 1.1 * 1024) 134.05 on ADC

  //We measure 198mV so we should see (.198 / 1.1 * 1024) 184 on ADC

  adc_out = analogRead(CURRENT_SENSE);

  sense[senseSpot++] = adc_out;
  senseSpot %= numberOfSamples;

//  Serial.print("ADC: ");
//  if (adc_out < 1000) Serial.print("0");
//  if (adc_out < 100) Serial.print("0");
//  if (adc_out < 10) Serial.print("0");
//  Serial.print(adc_out);
//  Serial.print("\t");

  int adcAverage = avgOfSamples();

//  Serial.print("Avg: ");
//  if (adcAverage < 1000) Serial.print("0");
//  if (adcAverage < 100) Serial.print("0");
//  if (adcAverage < 10) Serial.print("0");
//  Serial.print(adcAverage);
//  Serial.print("\t");

  float adcV = ((float)adcAverage + 0.5) / 1024.0 * VREF; //https://www.gammon.com.au/adc
  float adcA = adcV / 0.11 / OPAMP_GAIN;

  //adcA is per coil. If we want total coil consumption, *2.
  adcA *= 2;

//  Serial.print("V: ");
//  Serial.print(adcV, 3);
//  Serial.print("\t");

  Serial.print("A: ");
  Serial.print(adcA, 3);
  Serial.print("\t");

  Serial.println();

  stepper.runSpeed();
  //delay(1);
}

int avgOfSamples()
{
  uint16_t sum = 0;

  for (int i = 0; i < numberOfSamples; i++)
    sum += sense[i];
  sum /= numberOfSamples;

  return sum;
}

//Takes a run or hold current of up to 2000(mA) and
//converts that to a PWM value where 1.7V = 2000mA
//1.7V / 3.3V = X / 255
uint8_t convertCurrentToPWM(uint16_t current)
{
  int maxPWM = (uint32_t)170 * 255 / 330; //Assumes 3.3V system with 1.7V on stepper IC = 2A(max)
  //int maxPWM = (uint32_t)200 * 255 / 330; //Assumes 3.3V system with 2.0V on stepper IC = 2A(max)
  //int maxPWM = (uint32_t)270 * 255 / 330; //Assumes 3.3V system with 2.7V on stepper IC = 2A(max)
  int pwmValue = map(current, 0, 2000, 0, maxPWM);
  Serial.print("pwmValue: ");
  Serial.println(pwmValue);
  return (pwmValue);
}
