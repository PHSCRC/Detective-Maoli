/*
 This file was built off of the AdaEncoder example code
*/
// Version 1.1: OO version
/*
 MyEncoder.ino - A library for reading Lady Ada's or 
 Sparkfun's rotary encoder.
 Should work for any rotary encoder with 2 pins (4 states).
 
 Copyright 2011,2012,2013,2014 Michael Anthony Schwager
 
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
 
 http://www.apache.org/licenses/LICENSE-2.0
 
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 
 	Questions?  Send mail to mschwage@gmail.com
 
 */

#include <ByteBuffer.h>
#include <ooPinChangeInt.h> // necessary otherwise we get undefined reference errors.
#include <TimedAction.h>
#include <PID_v1.h>

#define DEBUG
#ifdef DEBUG
ByteBuffer printBuffer(200);
#endif
#include <AdaEncoder.h>

#define ENCA_a 4
#define ENCA_b 5
#define ENCB_a 2
#define ENCB_b 3
const int INB1 = A0;
const int INB2 = A1;
const int PWMB = 11;
const int INA1 = A3;
const int INA2 = A4;
const int PWMA = 10;

AdaEncoder encoderA = AdaEncoder('a', ENCA_a, ENCA_b);
AdaEncoder encoderB = AdaEncoder('b', ENCB_a, ENCB_b);

int8_t clicks=0;
char id=0;
long counterA = 0;
long counterB = 0;
long counterADistance = 0;
long counterBDistance = 0;
boolean receivedCommand = false;
String received = "";
double setpoint, input;
double targetB = 0;

PID motorPID(&input, &targetB, &setpoint, 2, 5, 0, DIRECT);

TimedAction serialThread = TimedAction(50, printSerial);
//TimedAction pidThread = TimedAction(50, adjustPID);
//TimedAction readThread = TimedAction(50, readSerial);

void printSerial() {
  
  Serial.print(((double) counterA)*((9*3.14159)/800));
  Serial.print(",");
  Serial.println(((double) counterB)*((9*3.14159)/1600));
  
  //Serial.println(input);
}
/*
void adjustPID() {
}
*/
void readSerial() {
  while(Serial.available() > 0) {
    char incoming = Serial.read();
    if(incoming == '~') {
      receivedCommand = true;
    }
    else {
      received += incoming;
    }
  }
}

void parseCommand(String command) {
  char motor = command.charAt(0);
  char dir = command.charAt(1);
  float pwm = atof(command.substring(2).c_str());
  int pin1 = 0;
  int pin2 = 0;
  int pinPWM = 0;

  if(motor == 'A') {
    pin1 = INA1;
    pin2 = INA2;
    pinPWM = PWMA;
  }
  else if(motor == 'B') {
    pin1 = INB1;
    pin2 = INB2;
    pinPWM = PWMB;
    targetB = pwm;
    motorPID.SetOutputLimits(targetB, 255);
    motorPID.SetOutputLimits(0,targetB);
    motorPID.SetOutputLimits(0,255);
  }

  if(dir == 'F') {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  else if(dir == 'B') {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  analogWrite(pinPWM, pwm);
}

void setup()
{
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  Serial.begin(115200);
  Serial.setTimeout(150);
  input = (counterBDistance/2) - counterADistance;
  setpoint = 0;
  motorPID.SetMode(AUTOMATIC);
  //motorPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
  //motorPID.SetOutputLimits(-2.0, 2);  // Forces maximum down to 0.0
}

void loop() 
{
  motorPID.Compute();
  //targetB += output;
  analogWrite(PWMB, targetB);
  input = (counterBDistance/2) - counterADistance;
  readSerial();
  // parse command
  if(receivedCommand) {
    if(received == "0") {
      digitalWrite(INA1, HIGH);
      digitalWrite(INA2, HIGH);
      digitalWrite(INB1, HIGH);
      digitalWrite(INB2, HIGH);
    }
    else {
      parseCommand(received.substring(0, received.indexOf(',')));
      parseCommand(received.substring(received.indexOf(',') + 1));
      //motorPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      //motorPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      //motorPID.SetOutputLimits(0, 255);  // Set the limits back to normal
    }
    received = "";
    receivedCommand = false;
  }

  serialThread.check();
  //pidThread.check();

  char outChar;
  while ((outChar=(char)printBuffer.get()) != 0) Serial.print(outChar);
  AdaEncoder *thisEncoder=NULL;
  thisEncoder=AdaEncoder::genie();
  if (thisEncoder != NULL) {
    if (thisEncoder->getID() == 'a') {
      int diff = thisEncoder->query();
      counterA += diff;
      counterADistance += abs(diff);
    }
    else if (thisEncoder->getID() == 'b') {
      int diff = thisEncoder->query();
      counterB += diff;
      counterBDistance += abs(diff);
    }
  }
}


