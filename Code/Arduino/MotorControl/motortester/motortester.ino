#include <L298N.h>
#include <Encoder.h>
#include <PID_v1.h>


int leften = 6;
int leftInA = 7;
int leftInB = 8;
int leftEncA = 2;
int leftEncB = 4;

int righten = 9;
int rightInA = 10;
int rightInB = 11;
int rightEncA =3;
int rightEncB = 5;

double leftsetpoint = 0, leftinput, leftoutput;
double rightsetpoint = 0, rightinput, rightoutput;

double aggLKp=4, aggLKi=.2, aggLKd=1;
double aggRKp=4, aggRKi=.2, aggRKd=1;
double consLKp=0.1875, consLKi=0.05, consLKd=0.025;
double consRKp=0.1875, consRKi=0.05, consRKd=0.025;
//double consLKp=0.35, consLKi=0, consLKd=0.08;
//double consRKp=0.35, consRKi=0, consRKd=0.08;


L298N left(leften, leftInA, leftInB);
L298N right(righten, rightInA, rightInB);

Encoder leftEnc(leftEncA, leftEncB);
Encoder rightEnc(rightEncA, rightEncB);

PID leftPID(&leftinput, &leftoutput, &leftsetpoint, aggLKp, aggLKi, aggLKd, DIRECT);
PID rightPID(&rightinput, &rightoutput, &rightsetpoint, aggRKp, aggRKi, aggRKd, DIRECT);


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // put your setup code here, to run once:
  
  leftPID.SetOutputLimits(-150,150);
  rightPID.SetOutputLimits(-150,150);
  Serial.begin(115200);
  inputString.reserve(200);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(20);
  rightPID.SetSampleTime(20);
  rightEnc.write(0);
  leftEnc.write(0);
}
void loop() {
  
  // print the string when a newline arrives:
  
  if (stringComplete) {

      //Takes inputString (which is the drivespeed and turnvalue separated by a colon) and splits it into it's values, also the values should be from -255 to 255
      String motor = inputString.substring(0, inputString.indexOf(':'));
      int targetValue = inputString.substring(inputString.indexOf(':') + 1).toInt();
      if (motor.equals("left")){
        leftsetpoint=targetValue;
      }
      if (motor.equals("right")){
        rightsetpoint=targetValue;
      }
//  
//      //set the motor speed
//      Serial.print("target value");
//      Serial.println(targetValue);
//      driveLeft(targetValue);
//      driveRight(targetValue);

    
    // clear the string:
    inputString = "";
    stringComplete = false;

  }
  
 
  leftPID.SetTunings(consLKp, consLKi, consLKd);
  rightPID.SetTunings(consRKp, consRKi, consRKd);
  leftinput = leftEnc.read();
  rightinput = rightEnc.read();
  Serial.println(leftsetpoint);
  Serial.print(" ");
  Serial.println(leftinput);
  Serial.print(" ");
  leftPID.Compute();
  rightPID.Compute();
  Serial.println(leftoutput);
  driveLeft(leftoutput);
  driveRight(rightoutput);
}

void driveLeft(int v){
  if (v < 0){
    left.backward();
    left.setSpeed(abs(v));
  } else if (v > 0){
    left.forward();
    left.setSpeed(abs(v));
  } else {
    Serial.println("THERE");
    left.stop();
    leftEnc.write(leftinput);
  }
}

void driveRight(int v){

   if (v < 0){
    right.backward();
    if (v > -80){
      v = -80;
    }
    right.setSpeed(abs(v));
  } else if (v > 0){
    right.forward();
    if (v < 80){
      v = 80;
    }
    right.setSpeed(abs(v));
  } else {
    Serial.println("THERE");
    right.stop();
    rightEnc.write(rightinput);
  }
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    delay(1);
  }
}
