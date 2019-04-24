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

int leften = 6;
int leftInA = 7;
int leftInB = 8;
int timeOld = 0;
int readOldR = 0;
int readOldL = 0;
float inR, outR, setR;
float inL, outL, setL;

int counter = 0; 

int store1, store2;

 
void setup(){    
 Serial.begin(9600);
 digitalWrite(leftInA, HIGH);
 digitalWrite(leftInB, LOW);
 analogWrite(leften, 50);
 
setR = 0.8;
setL = 0.8;
}
 
void loop(){
    
    inR = (rightWheel.read() - readOldR)/3266.5/(millis() - timeOld);
    inL = (leftWheel.read() - readOldL)/3266.5/(millis() - timeOld);

    float ins[2] = {inL, inR};
    
    timeOld = millis(); // current time  
    delay(50);          
    computed = computePID(ins);
  
    
    
    left(output[0]); //control the motor based on PID value   

    //add code to take input from Serial, convert to degrees and call left/right/forward function respectively
}
 
void left(int input){
 inL = input;
 digitalWrite(leftInA, HIGH);
 digitalWrite(leftInB, LOW);
 analogWrite(leften, input);
}

float computePID(float in[]){     
    float out[2] = {0,0}; // output of outL (index 0) and outR (index 1)
    currentTime = millis(); //get current time               
    elapsedTime = (currentTime - previousTime);  //compute time elapsed from previous computation
    
    errorL = setL - (leftWheel.read()%6533 - readOldL%6533)/3266.5/(millis() - timeOld)*1000;  // determine error
    errorR = setR - (rightWheel.read()%6533 - readOldR%6533)/3266.5/(millis() - timeOld)*1000;  // determine error
     
    readOldL = leftWheel.read()%6533;  
    readOldR = rightWheel.read()%6533;
     
    timeOld=millis();

    //Left Wheel Calculations
    cumErrorL += errorL * elapsedTime; // compute integral               
    rateErrorL = (errorL - lastErrorL)/elapsedTime;   // compute derivative
    
    float outL = (kp*errorL + ki*cumErrorL + kd*rateErrorL); //PID output   

    lastErrorL = errorL;  //remember current error                              
    previousTimeL = currentTimeL; //remember current time                      

    // out must be between 0-255
    out[0] = max(0,outL); //have function return the PID output

    //Right Wheel Calculations
    cumErrorR += errorR * elapsedTime; // compute integral               
    rateErrorR = (errorR - lastErrorR)/elapsedTime;   // compute derivative
    
    float outR = (kp*errorR + ki*cumErrorR + kd*rateErrorR); //PID output   

    lastErrorR = errorR;  //remember current error                              
    previousTimeR = currentTimeR; //remember current time                      

    // out must be between 0-255
    out[1] = max(0,outR); //have function return the PID output
    
    return((String)out[0] + " " + (String)out[1]);
    
}
