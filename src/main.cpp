/***************************************************
VGU VENT
Author: Minh Tri - Robotics Lab
 ****************************************************/
// Define the Motor Driver
// PINOUT
// L_EN -> 8
// R_EN -> 8
// L_PWM -> 9
// R_PWM -> 10

#include "Wire.h"
#include "BTS7960.h"

const uint8_t EN = 8;
const uint8_t L_PWM = 9;
const uint8_t R_PWM = 10;
int LEndstop = 5;
int REndstop = 4;

BTS7960 motorController(EN, L_PWM, R_PWM);

// Rotary Encoder Inputs
#define encodPinA1  	2                   	// Quadrature encoder A pin
#define encodPinB1  	3                   	// Quadrature encoder B pin

int currentStateCLK;
int lastStateCLK;
String currentDir ="";

// PID Control
#include <PID_v1.h>
double kp = 5 , ki = 1 , kd = 0.01;         	// modify for optimal performance
double input = 0, output = 0, setpoint = 0;
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Motor control Disable
void motorDisable(){
  motorController.Disable();
}
// Homming Motor
void hommingMotor(){
  motorController.Enable();
  motorController.TurnLeft(100);
}
// Begin Cycle
void beginCycle(){
  motorDisable();
  encoderPos = 0;
  setpoint = 30; 
  delay(2000);
 //  Test Motor with Delay
  while (encoderPos <60){
  motorController.Enable();
  motorController.TurnRight(250);
  }
 
  Serial.print(" | encoderPos: ");
  Serial.println(encoderPos); 
  
 /*
   // PID control the Motor
   Serial.print(" | Setpoint: ");
  Serial.println(setpoint);
  Serial.print(" | encoderPos: ");
  Serial.println(encoderPos);                  	// monitor motor position
  Serial.print(" | Output: ");
  Serial.println(output);
  Serial.print(" | Input: ");
  Serial.println(input);
  input = encoderPos ;                            	// data from encoder
  myPID.Compute();                                	// calculate new output
  pwmOut(output/2);  
   */

}

// Interrupt Program Encoder
void pwmOut(int out) {                            	// to H-Bridge board
  if (out > 0) {
	motorController.Enable();
	motorController.TurnRight(out);
 
    
  }
  else {
	motorController.Enable();
	motorController.TurnLeft(abs(out));    	// drive motor CCW
  }
}


//The Encoder Interrupt
void encoder()  {                                 	// pulse and direction, direct port reading to save cycles
 
 // Read the current state of encodPinA1
  currentStateCLK = digitalRead(encodPinA1);

  // If last and current state of encodPinA1 are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

	// If the encodPinB1 state is different than the encodPinA1 state then
	// the encoder is rotating CCW so decrement
	if (digitalRead(encodPinB1) != currentStateCLK) {
  	encoderPos++;
  	currentDir ="CCW";
	} else {
  	// Encoder is rotating CW so increment
  	encoderPos--;
  	currentDir ="CW";
	}

  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}


void setup()
{
  pinMode(LEndstop, INPUT_PULLUP);  // This is the Left Endstop Define 
  pinMode(REndstop, INPUT_PULLUP);  // This is the Right Endstop Define 
  pinMode(encodPinA1, INPUT_PULLUP);              	// quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);              	// quadrature encoder input B
  lastStateCLK = digitalRead(encodPinA1);
          
  attachInterrupt(1, encoder, CHANGE);           	// update encoder position
  attachInterrupt(0, encoder, CHANGE);          	// update encoder position
   
  TCCR1B = TCCR1B & 0b11111000 | 1;               	// set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Serial.begin (9600);                          	// for debugging
 

}

void loop()
{
  int Lsensor = digitalRead(LEndstop); // Add signal Left EndStop (Low Active)
  int Rsensor = digitalRead(REndstop); // Add signal Right EndStop (Low Active)
 // Serial.println(Lsensor);  // Print the signal testing
 // Serial.println(Rsensor);   // Print the signal testing
     
if (Lsensor ==1 && Rsensor ==1){
 hommingMotor();
}

else if(Lsensor ==0){
 beginCycle();
}

else {
  motorDisable();
}
 
}



