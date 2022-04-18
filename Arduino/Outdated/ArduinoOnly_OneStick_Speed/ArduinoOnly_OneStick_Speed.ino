//2.0 was first successful run using basic ideas for the capstone project
//2.1 is testing with 2 Arduinos
//2.2 is testing with 2 arduinos and switch case. Max baudrate
//3.0 is with esp and Arduino Mega. Max baudrate that works (so far). Loop time is 4-5ms
//Requests and prints 4 pulse values
//4.0 Makes use of 2d array to save history of pulses
//4.1 Adds a variable for history amount
//5.0 Adds pid loop control of motors. Implemented for straight driving at constant speed

//Included Libraries
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "Arduino.h"
#include <std_msgs/UInt16.h>

//Constants
#define pulseToDistance 0.002744 // m/pulse
//17 pulses per rev (one channel I think). 1:13 gearbox. Wheel diameter of 7.6" or 196.85mm.


//Motor speed to pwm
//pwm = (speed+0.0429)*76.0456

//Driver pin definitions
#define FL_P 4
#define FL_D 22
#define BL_P 5
#define BL_D 23
#define FR_P 6
#define FR_D 24
#define BR_P 7
#define BR_D 25

//Encoder Pin Definitions
#define FL_A 2
#define FL_B 8
#define FR_A 3
#define FR_B 9

//Changeable variables
int loopSpeed = 15; //milliseconds
const int limit = 225; //motor pwm limit
const unsigned int pwmOffset = 4;
const unsigned int dirOffset = 22;
const float linGain = 2;
const float rotGain = 3;
float Kp[4] = {1, 1, 1, 1};
float Ki[4] = {0.001, 0.001, 0.001, 0.001};
float Kd[4] = {5, 5, 5, 5};

//Motor Curves
float curveB[4] = {0.05845, 0.05845, 0.05845, 0.05845};
float curveM[4] = {0.0127, 0.0127, 0.0127, 0.0127};

//PID variables
float motorSpeed[4]; //in m/s
float finalMotorSpeed[4];
float motorSetpoint[4];
float error[3][4];
unsigned int lTime;
float deltaT;
float newMotorSpeed[2][4];

//Other variables
float linVel;
float rotVel;
float LRSpeed[4];
unsigned int pulseValues[4];
std_msgs::UInt16 message;

//Assumes setup file has scale of 1 for both
void controllerInput(const sensor_msgs::Joy& joy) {
  linVel = joy.axes[1];
  rotVel = joy.axes[0];
}

//ROS Subscriber and Publisher setup
ros::NodeHandle nh;
ros::Subscriber <sensor_msgs::Joy> sub("joy", &controllerInput);
ros::Publisher pub("LWS", &message);

//Function to determine desired linear speed for each wheel from joystick input
void speedCalcs() {
    LRSpeed[0] = linVel * linGain - rotVel * rotGain;
    LRSpeed[1] = LRSpeed[0];
    LRSpeed[2] = linVel * linGain + rotVel * rotGain;
    LRSpeed[3] = LRSpeed[2];
}

//function to calculate new motor speeds based on PID
void pid () {

  //calculate speed in m/s
  for (int i = 0; i < 2; i++) {
    if (pulseValues[i+2] == 1) {
      motorSpeed[i] = pulseValues[i] * pulseToDistance / deltaT * 1000;
    }
    if (pulseValues[i+2] == 2) {
      motorSpeed[i] = -pulseValues[i] * pulseToDistance / deltaT * 1000;
    }
    pulseValues[i] = 0;
    //Serial.println(motorSpeed[i]);
  }


  //calculate error,process pid calcs, save old values
  for (int i = 0; i < 4; i++) {
    error[0][i] = LRSpeed[i] - motorSpeed[i];
    newMotorSpeed[0][i] = newMotorSpeed[1][i] + (Kp[i] + Kd[i] / deltaT + Ki[i] * deltaT) * error[0][i] - (Kp[i] + 2 * Kd[i] / deltaT) * error[1][i] + Kd[i] / deltaT * error[2][i];
    newMotorSpeed[1][i] = newMotorSpeed[0][i];
    error[2][i] = error[1][i];
    error[1][i] = error[0][i];
  }
}

void getPwm() {
  for (int i = 0; i < 4; i++) {
    LRSpeed[i] = ((LRSpeed[i] + newMotorSpeed[0][i]) + curveB[i]) / curveM[i];

    //Set motor limits
    if (LRSpeed[i] > limit) {
      LRSpeed[i] = limit;
    }
    if (LRSpeed[i] < (0 - limit)) {
      LRSpeed[i] = (0 - limit);
    }
  }
}

void motorControl() {
  for (int i = 0; i < 4; i++) {

    if (LRSpeed[i] < 0) {
      digitalWrite(i + dirOffset, LOW);
    }
    else {
      digitalWrite(i + dirOffset, HIGH);
    }

    if (abs(LRSpeed[i]) < 15) {
      LRSpeed[i] = 0;
    }
    analogWrite(i + pwmOffset, abs(LRSpeed[i]));
  }
  message.data = abs(LRSpeed[2]);
  pub.publish(&message);
}

void isr_FLA() {
  pulseValues[0]++;
  if (digitalRead(FL_B) == LOW) {
    pulseValues[2] = 1;
  }
  else {
    pulseValues[2] = 2;
  }
}

void isr_FRA() {
  pulseValues[1]++;
  if (digitalRead(FR_B) == HIGH) {
    pulseValues[3] = 1;
  }
  else {
    pulseValues[3] = 2;
  }
}


// SETUP *************************************************************************
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  for (int i = 0; i < 4; i++) {
    pinMode(i + pwmOffset, OUTPUT);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(i + dirOffset, OUTPUT);
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(i + dirOffset, HIGH);
  }
  attachInterrupt(digitalPinToInterrupt(FL_A), isr_FLA, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), isr_FRA, RISING);
  pinMode(FL_B, INPUT);
  pinMode(FR_B, INPUT);
  int lTime = millis();
} // End Setup

// LOOP *************************************************************************
void loop() {
  deltaT = millis() - lTime;
  if (deltaT >= loopSpeed) {
    lTime = millis();
    nh.spinOnce();
    speedCalcs();
    pid();
    getPwm();
    motorControl();
    //delay(3);
  }
}  // End Loop.
