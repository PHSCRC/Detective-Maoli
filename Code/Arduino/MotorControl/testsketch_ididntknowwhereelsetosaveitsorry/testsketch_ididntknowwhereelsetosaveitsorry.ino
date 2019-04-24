#include <L298N.h>
//#include <Encoder.h>
//#include <PID_v1.h>

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
  pinMode(leften, OUTPUT);
  pinMode(righten, OUTPUT);
  pinMode(leftInA, OUTPUT);
  pinMode(leftInB, OUTPUT);
  pinMode(rightInA, OUTPUT);
  pinMode(rightInB, OUTPUT);

}

void left()
{
  digitalWrite(leftInA, HIGH);
  digitalWrite(leftInB, LOW);
  analogWrite(leften, 200);
}

void right()
{
  digitalWrite(rightInA, HIGH);
  digitalWrite(rightInB, LOW);
  analogWrite(righten, 200);
}

void stop()
{
  digitalWrite(leftInA, LOW);
  digitalWrite(leftInB, LOW);  
  digitalWrite(rightInA, LOW);
  digitalWrite(rightInB, LOW);
}

void driverM()
{
  driver.forward(255, 0);
  delay(1000);
  driver.full_stop(0);
  delay(1000);
  driver.backward(100, 0);
  delay(1000);
  driver.full_stop(0);
  delay(1000);
}

void loop() {
//  right();
// delay(5000);
//  stop();
// delay(1000);
//  left();
//  delay(5000);
//  stop();
//  delay(1000);
//  driverM();

}
