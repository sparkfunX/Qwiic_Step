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

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Stepper Example");

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
  stepper.setSpeed(-350);

  //Generate PWM signal of 0.1V at current reference pin
  int duty_cycle = (2 / 3.3) * 255;
  analogWrite(CURRENT_REFERENCE, duty_cycle);
}

#define numberOfSamples 32
int sense[numberOfSamples] = {0};
byte senseSpot = 0;

void loop()
{
  adc_out = analogRead(CURRENT_SENSE);
  sense[senseSpot++] = adc_out;

  senseSpot %= numberOfSamples;  //6796

  Serial.print("ADC: ");
  Serial.print(adc_out);
  Serial.print("\t");

  int adcAverage = avgOfSamples();

  Serial.print("Avg: ");
  Serial.print(adcAverage);
  Serial.print("\t");

  float adcMV = (adcAverage * 1000 * 3.3) / 1023;
  float adcMA = adcMV / 0.11;

  Serial.print("Volt: ");
  Serial.print(adcMV);
  Serial.print("\t");

  Serial.print("Curr: ");
  Serial.print(adcMA);
  Serial.print("\t");

  Serial.println();

  stepper.runSpeed();
  //  delay(1);
}

int avgOfSamples() {
  uint16_t sum = 0;
  for (int i = 0; i < numberOfSamples; i++) {
    sum += sense[i];
  }
  sum /= numberOfSamples;
  return sum;
}
