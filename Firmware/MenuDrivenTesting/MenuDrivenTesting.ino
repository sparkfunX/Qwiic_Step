/*
  This sketch allows the driving of a motor via text menu without the full Qwiic Step I2C firmware. This
  is helpful for testing various aspects of the board.
  
  Tested with the following:
    Bench power supply, 12V, 2A max. https://www.sparkfun.com/products/retired/9291
    1.7A per phase stepper: https://www.sparkfun.com/products/10846

  Press a/z to raise or lower the voltage sent to stepper IC as the 'max' trip current voltage.
  Press +/- to increase/decrease the step speed. 0 to stop.
  Press 1/2/3/4/5 to modify the step size.
  Press r to take an ADC reading from the opamp.

  To test large current loads, set to single step mode, step at 600, and raise the trip voltage to ~2V.
  This cause the motor to pull ~1A and heat the board significantly.

  Current through the coils depend heavily on the speed and step size.
  
  Speed 1000, Full Step
  tripVoltage of 550mV / 135mA
  650mV / 165mA
  1650mV / 489mA
  1750 / 526
  1850 / 555
  2350 / 731

  Speed: 900, Half Step
  550mV / 140mA
  650 / 180
  750 / 218
  1650 / 687
  1750 / 712
  1850 / 740
  2350 / 954
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

enum stepOptions {
  STEP_SIZE_FULL = 0,
  STEP_SIZE_HALF,
  STEP_SIZE_QUARTER,
  STEP_SIZE_EIGHTH,
  STEP_SIZE_SIXTEENTH
};

#define numberOfSamples 32
int sense[numberOfSamples] = {0};
byte senseSpot = 0;

float runVoltage = 0.250;
int stepperSpeed = 300;

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

  setStepSize(STEP_SIZE_FULL);
  //setStepSize(STEP_SIZE_EIGHTH);

  stepper.setMaxSpeed(10000);
  stepper.setSpeed(stepperSpeed);

  analogWrite(CURRENT_REFERENCE, convertVoltageToPWM(runVoltage));

  Serial.println();
  Serial.println("a) Increase Itrip voltage");
  Serial.println("z) Decrease Itrip voltage");
  Serial.println("+) Increase Step speed");
  Serial.println("-) Decrease Step speed");
  Serial.println("0) Stop");
  Serial.println("1) Single step");
  Serial.println("2) Half Step");
  Serial.println("3) Quarter Step");
  Serial.println("4) Eighth Step");
  Serial.println("5) Sixteenth Step");
  Serial.println("r) Take current sense reading");

  Serial.print("Speed: ");
  Serial.print(stepperSpeed);
  Serial.print(" tripVoltage(Vref): ");
  Serial.println(runVoltage, 3);
}

void loop()
{
  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'a')
    {
      runVoltage += 0.100;
      analogWrite(CURRENT_REFERENCE, convertVoltageToPWM(runVoltage));
    }
    else if (incoming == 'z')
    {
      runVoltage -= 0.100;
      if (runVoltage < 0.0) runVoltage = 0;
      analogWrite(CURRENT_REFERENCE, convertVoltageToPWM(runVoltage));
    }
    else if (incoming == '+')
    {
      stepperSpeed += 100;
      stepper.setSpeed(stepperSpeed);
      Serial.print("Speed: ");
      Serial.println(stepperSpeed);
    }
    else if (incoming == '-')
    {
      stepperSpeed -= 100;
      stepper.setSpeed(stepperSpeed);
      Serial.print("Speed: ");
      Serial.println(stepperSpeed);
    }
    else if (incoming == '0')
    {
      stepperSpeed = 0;
      stepper.setSpeed(stepperSpeed);
      Serial.print("Speed: ");
      Serial.println(stepperSpeed);
    }
    else if (incoming == '1')
    {
      setStepSize(STEP_SIZE_FULL);
      Serial.print("Step size: ");
      Serial.println("FULL");
    }
    else if (incoming == '2')
    {
      setStepSize(STEP_SIZE_HALF);
      Serial.print("Step size: ");
      Serial.println("HALF");
    }
    else if (incoming == '3')
    {
      setStepSize(STEP_SIZE_QUARTER);
      Serial.print("Step size: ");
      Serial.println("QUARTER");
    }
    else if (incoming == '4')
    {
      setStepSize(STEP_SIZE_EIGHTH);
      Serial.print("Step size: ");
      Serial.println("EIGHTH");
    }
    else if (incoming == '5')
    {
      setStepSize(STEP_SIZE_SIXTEENTH);
      Serial.print("Step size: ");
      Serial.println("SIXTEENTH");
    }
    else if (incoming == 'r')
    {
      //Take ADC reading
      senseSpot = 0;
      for (int x = 0 ; x < numberOfSamples ; x++)
      {
        adc_out = analogRead(CURRENT_SENSE);
        while (adc_out == 0)
          adc_out = analogRead(CURRENT_SENSE);

        sense[senseSpot++] = adc_out;
        senseSpot %= numberOfSamples;
        delay(1);
      }

      //      Serial.print("ADC: ");
      //      if (adc_out < 1000) Serial.print("0");
      //      if (adc_out < 100) Serial.print("0");
      //      if (adc_out < 10) Serial.print("0");
      //      Serial.print(adc_out);
      //      Serial.print("\t");

      int adcAverage = avgOfSamples();

      Serial.print("Avg: ");
      if (adcAverage < 1000) Serial.print("0");
      if (adcAverage < 100) Serial.print("0");
      if (adcAverage < 10) Serial.print("0");
      Serial.print(adcAverage);
      Serial.print("\t");

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
    }
  }

  stepper.runSpeed();
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
  //int maxPWM = (uint32_t)170 * 255 / 330; //Assumes 3.3V system with 1.7V on stepper IC = 2A(max)
  //int maxPWM = (uint32_t)200 * 255 / 330; //Assumes 3.3V system with 2.0V on stepper IC = 2A(max)
  int maxPWM = (uint32_t)200 * 255 / 330; //Assumes 3.3V system with 2.7V on stepper IC = 2A(max)
  int pwmValue = map(current, 0, 2000, 0, maxPWM);
  Serial.print("pwmValue: ");
  Serial.println(pwmValue);

  float voltage = (pwmValue * 3.3 / 255.0) - 0.05;
  Serial.print("tripVoltage(Vref): ");
  Serial.println(voltage, 3);

  return (pwmValue);
}

//Takes a voltage and outputs it over PWM
uint8_t convertVoltageToPWM(float voltage)
{
  Serial.print("tripVoltage(Vref): ");
  Serial.print(voltage, 3);

  voltage += 0.05; //Add empirically found offset
  float pwmValue = voltage * 255.0 / 3.3;

  Serial.print(" pwmValue: ");
  Serial.println(pwmValue);

  return (pwmValue);
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
