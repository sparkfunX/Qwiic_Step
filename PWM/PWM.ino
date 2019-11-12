// Code that outputs PWM signal on pin 3 of ATMega328
// Timer2 output B

void setup() {
  pinMode(3, OUTPUT);
  TCCR2A = _BV(COM2B1) | _BV(WGM20);    // Clear OC2B on compare match
                                        // PWM, phase correct
  TCCR2B = _BV(CS21) | _BV(CS20);       // clk/32 (from prescalar)
  OCR2B = 100;                           // output compare register B
                                        // duty cycle = 50/255 = 19.6%
}

void loop() {
  // put your main code here, to run repeatedly:

}
