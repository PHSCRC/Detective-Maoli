#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];

const float MAX = 255.0/2000;


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  //Serial.println("Color View Test!");

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  
  // use these three pins to drive an LED
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
  }
}


void loop() {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(50);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED

  float r = red*MAX;
  float g = green*MAX;
  float b = blue*MAX;
  
  if (r > 255) {
    r = 255;
  }
  if (g > 255) {
    g = 255;
  }
  if (b > 255) {
    b = 255;
  }

  /*
   * This is test to see if the sensor registers white. 
   * Usually works if the paper is not moving too quickly.
   */
  if (r > 100 && g > 100 && b > 100) {
    Serial.println("WHITE");
  } else {
    Serial.println("BLACK");
  }

  
  /*
   * Prints the rgb value of the color in (REDVAL, GREENVAL, BLUEVAL)
   * Clear is commented out
   * 
   * Robot should not go super fast or these values will be inaccurate.
   * Black Threshold: (roughly 20, anything else to be safe)
   * White Threshold: (anything over 100)
   */
   
//  Serial.print("C:\t"); Serial.print(clear);
/*  Serial.print("("); Serial.print(r);
  Serial.print(", "); Serial.print(g);
  Serial.print(","); Serial.print(b);
  Serial.println(")");*/

  analogWrite(redpin, gammatable[(int)r]);
  analogWrite(greenpin, gammatable[(int)g]);
  analogWrite(bluepin, gammatable[(int)b]);
}

