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
const int limit = 240; //motor pwm limit
const unsigned int pwmOffset = 4;
const unsigned int dirOffset = 22;
const float linGain = 1.5;
const float rotGain = 3;

//Other variables
float linVel;
float rotVel;
float LRSpeed[4];
std_msgs::UInt16 message;

//Assumes setup file has scale of 1 for both
void controllerInput(const sensor_msgs::Joy& joy) {
  linVel = joy.axes[1];
  rotVel = joy.axes[0];
}

ros::NodeHandle nh;
ros::Subscriber <sensor_msgs::Joy> sub("joy", &controllerInput);
ros::Publisher pub("LWS", &message);

void speedCalcs() {
  LRSpeed[0] = linVel * linGain - rotVel * rotGain;
  LRSpeed[1] = linVel * linGain - rotVel * rotGain;
  LRSpeed[2] = linVel * linGain + rotVel * rotGain;
  LRSpeed[3] = linVel * linGain + rotVel * rotGain;
}

void getPwm() {
  for (int i = 0; i < 4; i++) {
    LRSpeed[i] = (LRSpeed[i] + 0.0429) * 76.0456;

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

    if (abs(LRSpeed[i]) < 30) {
      LRSpeed[i] = 0;
    }
    analogWrite(i + pwmOffset, abs(LRSpeed[i]));
  }
  message.data = abs(LRSpeed[2]);
  pub.publish(&message);
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
} // End Setup

// LOOP *************************************************************************
void loop() {
  nh.spinOnce();
  speedCalcs();
  getPwm();
  motorControl();
  delay(3);
}  // End Loop.
