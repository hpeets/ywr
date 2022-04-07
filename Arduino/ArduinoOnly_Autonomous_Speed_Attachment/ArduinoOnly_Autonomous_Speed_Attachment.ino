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
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "Arduino.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>

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
#define FL_A 20
#define FL_B 8
#define FR_A 21
#define FR_B 9

//Attachment Pin Definitions
#define auger_P 12
#define auger_D 26
#define servo_P 11

//Changeable variables
int loopSpeed = 20; //milliseconds
const int limit = 225; //motor pwm limit
const unsigned int pwmOffset = 4;
const unsigned int dirOffset = 22;
//const float linGain = 0.4;
//const float rotGain = 0.2;
//float Kp[4] = {1, 1, 1, 1};
//float Ki[4] = {0.001, 0.001, 0.001, 0.001};
//float Kd[4] = {5, 5, 5, 5};
float Kp[4] = {0.9, 0.9, 0.9, 0.9}; //
float Ki[4] = {0.01, 0.01, 0.01, 0.01}; //{0.01, 0.01, 0.01, 0.01}; //0.02
float Kd[4] = {2, 2, 2, 2};
float augerSpeed = 150;

//Motor Curves
float curveB[4] = {0.05845, 0.05845, 0.05845, 0.05845};
float curveM[4] = {0.0127, 0.0127, 0.0127, 0.0127};

//PID variables
float motorSpeed[4]; //in m/s
float finalMotorSpeed[4];
float motorSetpoint[4];
float error[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
float lError[4];
unsigned long lTime;
unsigned long deltaT;
float newMotorSpeed[2][4];

//Other variables
float linVel;
float rotVel;
unsigned int auger;
float angle;
float currentAngle;
int auger_windup; //counter for speeding up
int eStop;
float LRSpeed[4];
int pulseValues[4];
float tempSpeed[4];
std_msgs::UInt16 marker;
std_msgs::Float32 sveed;
sensor_msgs::JointState joint_states;
char *joint_state_names[] = {"front_left_wheel", "back_left_wheel", "front_right_wheel", "back_right_wheel", "chute_joint"};
ros::Time stamp_now;
int markerCounter = 0;
bool markerFlag = true;
bool markerFlagLast = true;
Servo chute;

float joint_states_pos[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float joint_states_vel[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float one_tick_rad = 2 * M_PI / 221;


//ROS Subscriber and Publisher setup

//Assumes setup file has scale of 1 for both
void controllerInput(const geometry_msgs::Twist& joy) {
  linVel = joy.linear.x;
  rotVel = 0.23 * joy.angular.z;
  //  auger = joy.buttons[7];
  //  angle = joy.axes[2];
  //  eStop = joy.buttons[1];
  //  if (joy.buttons[3] == 1 and markerFlag == true) {
  //    marker.data = markerCounter;
  //  }
  //  if (joy.buttons[3] == 0) {
  //    markerFlag = false;
  //  }
  //  if (markerFlag == false and markerFlagLast == true) {
  //    pub.publish(&marker);
  //    markerCounter += 1;
  //  }
  //  markerFlagLast = markerFlag;
}
ros::NodeHandle nh;
//ros::Publisher pub("dwCorners", &marker);
//ros::Publisher pub1("wheelSpeed", &sveed);
ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", &controllerInput);
ros::Publisher joint_states_pub("joint_states", &joint_states);

//Function to determine desired linear speed for each wheel from joystick input
void speedCalcs() {
    LRSpeed[0] = linVel - rotVel;
    LRSpeed[1] = LRSpeed[0];
    LRSpeed[2] = linVel + rotVel;
    LRSpeed[3] = LRSpeed[2];
//  LRSpeed[0] = 0.0;
//  LRSpeed[1] = 0.0;
//  LRSpeed[2] = 0.0;
//  LRSpeed[3] = 0.0;
}

void jointStates() {
  joint_states.name_length = 5;
  joint_states.velocity_length = 5;
  joint_states.position_length = 5;
  for (int i = 0; i < 4; i++) {
    if (i == 1 || i == 3) {
      joint_states_pos[i]  = joint_states_pos[i - 1];
      joint_states_vel[i]  = joint_states_vel[i - 1];
    }

    else {
      joint_states_pos[i]  += pulseValues[i] * one_tick_rad * 3.3;
      if (joint_states_pos[i] >= 2 * M_PI) {
        joint_states_pos[i] -= 2 * M_PI;
      }
      joint_states_vel[i]  = pulseValues[i] * one_tick_rad / deltaT * 1000;
    }
    //joint_states_pos[i] = joint_states_pos[i]*3.8;
    joint_states_vel[i] = joint_states_vel[i] * 3.3;
  }
  //Serial.println(joint_states_vel[2]);
  joint_states.header.frame_id = "base_link";
  joint_states.name = joint_state_names;
  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
  stamp_now = nh.now();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

//function to calculate new motor speeds based on PID
void pid () {

  //Designed for readings off front left and front right encoders
  pulseValues[1] = pulseValues[0];
  pulseValues[3] = pulseValues[2];

  //Publish joint states
  //jointStates();

  //calculate speed in m/s
  for (int i = 0; i < 4; i++) {
    //Serial.println(pulseValues[i]);
    motorSpeed[i] = pulseValues[i] * pulseToDistance / deltaT * 1000;
    pulseValues[i] = 0;
  }

  //calculate error,process pid calcs, save old values
  for (int i = 0; i < 4; i++) {
    error[0][i] = LRSpeed[i] - motorSpeed[i];
    error[1][i] += error[0][i] * deltaT;
    if (LRSpeed[i] == 0) {
      error[1][i] = 0;
    }

    //For integrator windup at 0 speed.
    //At Ki=0.01 and minimum speed of 0.16 m/s at 12 PWM, then (3 - 0.16)/Ki = 284 for max error.
    //(assuming a max speed 3.0 m/s at 225 PWM, the min speed = 12/225 * 3.0 = 0.16)
    if (error[1][i] > 284) {
      error[1][i] = 284;
    }

    error[2][i] = (error[0][i] - lError[i]) / deltaT;
    newMotorSpeed[0][i] = Kp[i] * error[0][i] + Ki[i] * error[1][i] + Kd[i] * error[2][i];
    lError[i] = error[0][i];
  }
  //  //calculate error,process pid calcs, save old values
  //  for (int i = 0; i < 4; i++) {
  //    error[0][i] = LRSpeed[i] - motorSpeed[i];
  //    newMotorSpeed[0][i] = (Kp[i] + Kd[i] / deltaT + Ki[i] * deltaT) * error[0][i] - (Kp[i] + 2 * Kd[i] / deltaT) * error[1][i] + Kd[i] / deltaT * error[2][i];
  //    //newMotorSpeed[1][i] = newMotorSpeed[0][i];
  //    error[2][i] = error[1][i];
  //    error[1][i] = error[0][i];
  //  }
}

void getPwm() {
  for (int i = 0; i < 4; i++) {
    //Add PID speed adjustment to the set speed, then convert to PWM value using the linear adjustment.
    //Note: x = (y-b)/m and note that b was found to be negative for the motors, hence it took the form of +b here.
    LRSpeed[i] = ((newMotorSpeed[0][i] + LRSpeed[i]) + curveB[i]) / curveM[i];

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
  //LRSpeed[1] = LRSpeed[0];
  //LRSpeed[3] = LRSpeed[2];
  for (int i = 0; i < 4; i++) {

    if (LRSpeed[i] < 0) {
      digitalWrite(i + dirOffset, LOW);
    }
    else {
      digitalWrite(i + dirOffset, HIGH);
    }

    if ((abs(LRSpeed[i]) < 12) || (eStop == 1)) {
      LRSpeed[i] = 0;
    }
    analogWrite(i + pwmOffset, abs(LRSpeed[i]));
  }
  //Serial.println(LRSpeed[2]);
}

//void attachmentControl() {
//  //Auger control
//  if (auger == 1) {
//    if (auger_windup < 150) {
//      analogWrite(auger_P, round(auger_windup / 150 * augerSpeed));
//    }
//    else {
//      analogWrite(auger_P, augerSpeed);
//    }
//  }
//  else {
//    analogWrite(auger_P, 0);
//  }
//
//  //Chute control
//  if ((currentAngle < 175) && (currentAngle > 5)) {
//    if (angle > 0) {
//      chute.write(currentAngle + 1);
//      currentAngle += 1;
//    }
//    if (angle < 0) {
//      chute.write(currentAngle - 1);
//      currentAngle -= 1;
//    }
//  }
//}

void isr_FLA() {
  if (digitalRead(FL_B) == LOW) {
    pulseValues[0]++;
  }
  else {
    pulseValues[0]--;
  }
}

void isr_FRA() {
  if (digitalRead(FR_B) == HIGH) {
    pulseValues[2]++;
  }
  else {
    pulseValues[2]--;
  }
}


// SETUP *************************************************************************
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub);
  //nh.advertise(pub1);
  nh.advertise(joint_states_pub);
  //Serial.begin(115200);
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

  pinMode(auger_P, OUTPUT);
  pinMode(auger_D, OUTPUT);
  pinMode(servo_P, OUTPUT);
  digitalWrite(auger_D, LOW);
//  for (int i = 0; i < 200; i++) {
//    analogWrite(auger_P, i);
//    delay(20);
//  }
//  delay(10000);
//  for (int i = 200; i >0; i--) {
//    analogWrite(auger_P, i);
//    delay(20);
//  }
  //analogWrite(auger_P, 0);-+
  chute.attach(servo_P, 500, 2500);
  chute.write(135);
  currentAngle = 135;
  lTime = millis();
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
    //attachmentControl();
  }
}  // End Loop.
