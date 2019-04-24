
//sine wave freq detection with 38.5kHz sampling rate and interrupts
//by Amanda Ghassaei
//http://www.instructables.com/id/Arduino-Frequency-Detection/
//July 2012

/*
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

*/

#include <Wire.h>

//clipping indicator variables
boolean clipping = 0;


//data storage variables
byte newData = 0;
byte prevData = 0;

//freq variables
unsigned int timer = 0;//counts of wave
unsigned int period = 0;
int frequency;

byte correct = 0;
byte wrong = 0;

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
cli();//diable interrupts

  //set up continuous sampling of analog pin 0

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements

  sei();//enable interrupts
  while (1) {
    //Serial.println(ADCH);
    if (clipping) { //if currently clipping
      PORTB &= B11011111;//turn off clippng indicator led
      clipping = 0;
    }
  

  frequency = 38462 / period; //timer rate/period
  Serial.println(frequency);//print results

  if (3400 < frequency && 4200 > frequency) {
    correct++;
  } 
  else {
    wrong++;
  }
  if (wrong + correct > 20) {
    //Serial.print(wrong);
    //Serial.print(" ");
    //Serial.println(correct);
    if (correct > (wrong * 3)) {
      digitalWrite(2, 1);
    } 
    else {
      digitalWrite(2, 0);
    }
    correct = 0;
    wrong = 0;

  }
  
  cli();
  }
}

ISR(ADC_vect) {
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  if (prevData < 64 && newData >= 64) { //if increasing and crossing midpoint
    period = timer;//get period Serial.out.println(period);
    timer = 0;//reset timer
  }


  if (newData == 0 || newData == 1023) { //if clipping
    PORTB |= B00100000;//set pin 13 high- turn on clipping indicator led
    clipping = 1;//currently clipping
  }

  timer++;//increment timer at rate of 38.5kHz

}


void loop() {
  

//  if (frequency > 3500 && frequency < 4300) {
//    // Turn LED light on
//    Serial.println("SIGNAL RECIEVED");
//    analogWrite(2, HIGH);
//    analogWrite(1, LOW);
//
//
//    // turn the engine foward
//    Wire.beginTransmission(1); // transmit to device #8
//    Wire.write(2);
//    Wire.endTransmission();
//
//  }


}
