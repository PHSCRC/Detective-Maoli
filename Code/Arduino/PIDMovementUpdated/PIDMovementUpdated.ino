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

long timeOld = 0;
int readOldR = 0;
int readOldL = 0;
float inR, outR, setR;
float inL, outL, setL;

void setup(){    
 Serial.begin(9600);
 pinMode(leftInA, OUTPUT);
 pinMode(leftInB, OUTPUT);
 pinMode(leften, OUTPUT);

 pinMode(rightInA, OUTPUT);
 pinMode(rightInB, OUTPUT);
 pinMode(righten, OUTPUT);
 
setR = 0.8;
setL = 0.8;
}
 
void loop(){
    
    inR = (rightWheel.read() - readOldR)/3266.5/(millis() - timeOld);
    inL = (leftWheel.read() - readOldL)/3266.5/(millis() - timeOld);
    
    timeOld = millis(); // current time  
    delay(50);          
    int computedLeft = computeLeftPID();
    int computedRight = computeRightPID();
    
    activateLeft(computedLeft); //Sends value 0-255 to left() for speed
    activateRight(computedRight);

    //To do: add code to take input from Serial, convert to degrees and call left/right/forward function respectively
}

void activateLeft(int motorSpeed){
 //inL = motorSpeed;
 digitalWrite(leftInA, HIGH);
 digitalWrite(leftInB, LOW);
 analogWrite(leften, motorSpeed);
}

void activateRight(int motorSpeed){
 //inL = motorSpeed;
 digitalWrite(rightInA, HIGH);
 digitalWrite(rightInB, LOW);
 analogWrite(righten, motorSpeed);
}



#define PI 3.1415926535
double convertDtoT(double mm) // takes mm
{
  float timeT = mm / (pow(90/2, 2)*PI) / 0.8;
  return(timeT);  
}




int computeRightPID(){
    currentTime = millis(); //get current time               
    elapsedTime = (currentTime - previousTime);  //compute time elapsed from previous computation
    
    errorR = setR - (rightWheel.read()%6533 - readOldR%6533)/3266.5/(millis() - timeOld)*1000;  // determine error
     
    readOldR = rightWheel.read()%6533;
     
    timeOld = millis();

    //Right Wheel Calculations
    cumErrorR += errorR * elapsedTime; // compute integral               
    rateErrorR = (errorR - lastErrorR)/elapsedTime;   // compute derivative
    
    float outR = (kp*errorR + ki*cumErrorR + kd*rateErrorR); //PID output   

    lastErrorR = errorR;  //remember current error                              
    previousTime = currentTime; //remember current time                      

    // out must be between 0-255
    int out = max(0,outR); //have function return the PID output

    Serial.println(rightWheel.read());
    return(out);
    
}

int computeLeftPID(){
    currentTime = millis(); //get current time               
    elapsedTime = (currentTime - previousTime);  //compute time elapsed from previous computation
    
    errorL = setL - (leftWheel.read()%6533 - readOldL%6533)/3266.5/(millis() - timeOld)*1000;  // determine error
  //Cleared of error: setL, leftWheel.read(), readOld
     
    readOldL = leftWheel.read()%6533;
     
    timeOld = millis();

    //Left Wheel Calculations
    cumErrorL += errorL * elapsedTime; // compute integral               
    rateErrorL = (errorL - lastErrorL)/elapsedTime;   // compute derivative
    
    float outL = (kp*errorL + ki*cumErrorL + kd*rateErrorL); //PID output   

    lastErrorL = errorL;  //remember current error                              
    previousTime = currentTime; //remember current time                      

    // out must be between 0-255
    int out = max(0,outL); //have function return the PID output

    //Serial.println(String(out) + " errorL: " + String(errorL) + " timeOld " + timeOld);
    //Serial.println(leftWheel.read());
    return(out);
    
}
