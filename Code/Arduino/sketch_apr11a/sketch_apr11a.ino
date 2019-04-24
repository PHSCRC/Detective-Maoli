#include <Encoder.h>
//convert to degrees

//PID constants
float kp = 200;
float ki = 0.;
float kd = 0;

Encoder rightWheel(4,2);
Encoder leftWheel(5,3);
 
unsigned long currentTime, previousTime;
float elapsedTime;
float errorL, errorR;
float lastErrorL, lastErrorR;
float input, output, setPoint;

float cumErrorL, cumErrorR, rateErrorL, rateErrorR; 

//Left motor
int leften = 6;
int leftInA = 7;
int leftInB = 8;

//Right motor
int righten = 9;
int rightInA = 11;
int rightInB = 10;

int timeOld = 0;
int readOldR = 0;
int readOldL = 0;
float inR, outR, setR;
float inL, outL, setL;

void setup(){    
 Serial.begin(9600);
 pinMode(leftInA, OUTPUT);
 pinMode(leftInB, OUTPUT);
 pinMode(leften, OUTPUT);
 
setR = 0.8;
setL = 0.8;
}
 
void loop(){
  digitalWrite(leftInA, HIGH);
  digitalWrite(leftInB, LOW);
  analogWrite(leften, 50);

  delay(5000);
    
}
