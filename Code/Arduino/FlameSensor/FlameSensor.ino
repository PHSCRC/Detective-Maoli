/* Flame Sensor analog example.
Code by Reichenstein7 (thejamerson.com)

Flame Sensor Ordered from DX ( http://www.dx.com/p/arduino-flame-sensor-for-temperature-
detection-blue-dc-3-3-5v-118075?tc=USD&gclid=CPX6sYCRrMACFZJr7AodewsA-Q#.U_n5jWOrjfU )

To test view the output, point a serial monitor such as Putty at your arduino. 
*/
#include <L298N.h>

// lowest and highest sensor readings:
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum

// Motor driver for fire extinguisher
int leftEnc = 2;
int intA = 3;
int intB = 5;

//stuff
int rightInA = 0;
int rightInB = 0;
int righten = 0;



L298N driver(leftEnc, intA, intB, rightInA, rightInB, righten);

void setup() {
  // initialize serial communication @ 9600 baud:
  Serial.begin(9600);  

  pinMode(leftEnc, OUTPUT);
  pinMode(intA, OUTPUT);
  pinMode(intB, OUTPUT);
}
void loop() {
  // read the sensor on analog A1:
  int sensorReading = analogRead(1);
  // map the sensor range (four options):
  // ex: 'long int map(long int, long int, long int, long int, long int)'
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);
  // range value:
  Serial.println(sensorReading);
  switch (range) {
  case 0:    // A fire closer than 1.5 feet away.
    Serial.println("** Close Fire **");
    extinguish();
    break;
  case 1:    // A fire between 1-3 feet away.
    Serial.println("** Distant Fire **");
    break;
  case 2:    // No fire detected.
    Serial.println("No Fire");
    break;
  }
  delay(1);  // delay between reads
}




// Call the extinguisher
void extinguish(){
  driver.forward(255,0);
  delay(100);
  stop();
}

// Turn off the extinguisher
void stop()  {
  digitalWrite(intA, LOW);
  digitalWrite(intB, LOW);  
}
