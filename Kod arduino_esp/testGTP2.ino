#include <ros.h>
#include <Servo.h>
#include "QuadratureEncoder.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

Encoders encL(2, 3);
Encoders encR(4, 5);

Servo servo1, servo2;
const int pinServo1 = 9;
const int pinServo2 = 10;

const int pinThrottle = 6;
const int pinSteering = 7;

// ROS
ros::NodeHandle_<ArduinoHardware, 3, 3, 64, 64> nh;

// Ticki
std_msgs::Int16MultiArray ticks_msg;
int16_t ticks_data[2];
ros::Publisher ticks_pub("wheel_ticks", &ticks_msg);

// RC
std_msgs::Int16MultiArray rc_msg;
int16_t rc_data[2];
ros::Publisher rc_pub("rc_channels", &rc_msg);

// Serwa
void servoCallback(const std_msgs::UInt16MultiArray& msg) {
  if (msg.data_length >= 2) {
    servo1.write(constrain(msg.data[0], 0, 180));
    servo2.write(constrain(msg.data[1], 0, 180));
  }
}
ros::Subscriber<std_msgs::UInt16MultiArray> servo_sub("servo_angles", &servoCallback);

void setup() {
  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  servo1.write(90);
  servo2.write(90);

  pinMode(pinThrottle, INPUT);
  pinMode(pinSteering, INPUT);

  nh.initNode();

  ticks_msg.data = ticks_data;
  ticks_msg.data_length = 2;
  nh.advertise(ticks_pub);

  rc_msg.data = rc_data;
  rc_msg.data_length = 2;
  nh.advertise(rc_pub);

  nh.subscribe(servo_sub);
}

void loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 50) {
    ticks_data[0] = encL.getEncoderCount();
    ticks_data[1] = encR.getEncoderCount();
    ticks_pub.publish(&ticks_msg);

    rc_data[0] = pulseIn(pinThrottle, HIGH, 25000);
    rc_data[1] = pulseIn(pinSteering, HIGH, 25000);
    rc_pub.publish(&rc_msg);

    lastSend = millis();
  }

  nh.spinOnce();
}
