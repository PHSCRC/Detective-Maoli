#include <L298N.h>
#include <Encoder.h>
//#include <PID_v1.h>
# include <Wire.h>

Encoder rightWheel(4,2);
Encoder leftWheel(5,3);

int leften = 6;
int leftInA = 7;
int leftInB = 8;
int leftEncA = 2;
int leftEncB = 4;

int righten = 9;
int rightInA = 11;
int rightInB = 10;
int rightEncA = 3;
int rightEncB = 5;

L298N driver(leften, leftInA, leftInB, rightInA, rightInB, righten);

void setup() {
  Wire.begin(1);                // join i2c bus with address #1
  Wire.onReceive(receiveEvent); // register event
  
  Serial.begin(9600);
  pinMode(leften, OUTPUT);
  pinMode(righten, OUTPUT);
  pinMode(leftInA, OUTPUT);
  pinMode(leftInB, OUTPUT);
  pinMode(rightInA, OUTPUT);
  pinMode(rightInB, OUTPUT);

  
 

}

void left()
{
  digitalWrite(leftInA, LOW);
  digitalWrite(leftInB, HIGH);
  analogWrite(leften, 70);
}

void right()
{
  digitalWrite(rightInA, LOW);
  digitalWrite(rightInB, HIGH);
  analogWrite(righten, 70);
}

void stop()
{
  digitalWrite(leftInA, LOW);
  digitalWrite(leftInB, LOW);  
  digitalWrite(rightInA, LOW);
  digitalWrite(rightInB, LOW);
}

void backward1()
{
  digitalWrite(leftInA, LOW);
  digitalWrite(leftInB, HIGH);
  analogWrite(leften, 70);

//  digitalWrite(rightInA, HIGH);
//  digitalWrite(rightInB, LOW);
//  analogWrite(righten, 70);
}

void driverM()
{
 driver.forward(90, 0);
  delay(1000);
driver.full_stop(0);
  delay(1000);
  driver.backward(100, 0);
  delay(1000);
  driver.full_stop(0);
  delay(1000);
  backward1();
  delay(1000);
}

void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    int c = Wire.read(); // receive byte as a character
    if(c == 2){
      driverM();
    }
  }
}

void loop() {
  delay(100);
  

}
