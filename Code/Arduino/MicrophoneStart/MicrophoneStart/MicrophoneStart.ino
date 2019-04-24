#include <TimedAction.h>

boolean clipping = false;
boolean doneWithSound = false;

//const int CLIPLED = 13;
//const int CORRLED = 2;

//data storage variables
byte newData = 0;
byte prevData = 0;

//freq variables
unsigned int timer = 0;//counts period of wave
unsigned int period;
int frequency;
byte correct = 0;
byte wrong = 0;
const int bluePin = 5; // mic LED pin, change when you have the actual value

void setup() {

  Serial.begin(115200);
  cli(); //diable interrupts

  //set up continuous sampling of analog pin 0

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements

  sei();//enable interrupts

  do {
    if (clipping) { //if currently clipping
      PORTB &= B11011111;//turn off clippng indicator led
      clipping = 0;
    }

    frequency = 38462 / period; //timer rate/period

    if (3306 < frequency && 4294 > frequency) {
      correct++;
    } else {
      wrong++;
    }
    if (wrong + correct > 20) {
      if (correct > (wrong * 3)) {
        Serial.write("YATTAZO");
        doneWithSound = true;
        correct = 0;
        wrong = 0;
      }

      delay(50);
    } while (!doneWithSound);

    cli(); //diable interrupts

    //set up continuous sampling of analog pin 0

    //clear ADCSRA and ADCSRB registers
    ADCSRA = 0b10000111;
    ADMUX = 0;

    sei();//enable interrupts

    analogReference(DEFAULT);

    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);

    digitalWrite(bluePin, HIGH);
  }

  while (ISR(ADC_vect)) {  //when new ADC value ready
    if (!doneWithSound) {
      prevData = newData;//store previous value
      newData = ADCH;//get value from A0
      if (prevData < 129 && newData >= 129) { //if increasing and crossing midpoint
        period = timer;//get period
        timer = 0;//reset timer
      }


      if (newData == 0 || newData == 1023) { //if clipping
        PORTB |= B00100000;//set pin 13 high- turn on clipping indicator led
        clipping = 1;//currently clipping
      }

      timer++;//increment timer at rate of 38.5kHz
    }
  }

  boolean receivedCommand = false;
  String received = "";

  void loop() {

  }
