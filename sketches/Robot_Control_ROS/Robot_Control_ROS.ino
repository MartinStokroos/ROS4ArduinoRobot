/*
  File: Robot_Control_ROS
  Version: 1.0.1
  Hardware: Robot Control Board (atmega 32u4)
  Purpose: Sketch for Arduino robot, implementing rosserial_arduino.


  created: 22 October 2021
  by M. Stokroos
  modified on: 25-10-2021

  1.0.1 : added loop timer control and topic for IRarray. 
  

  Currently supported topics:
  cmd_vel
  msg_compass
  msg_irarray (using rosserial_arduino Adc message for 5*uint16)

  To do:
  knobRead();
  analogRead();
  IMU (via the motor board I2C ?)


  MIT License

  Copyright (c) 2021 Martin Stokroos

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#include <ArduinoRobotLite.h>

#define USE_USBCON  //serial via USB
#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Int16.h>
//#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define LOOP_TIME 50000  // loop time in us. 50ms = 20Hz
const double L_BASE = 0.15; //distance between the wheels in m

unsigned long loopTimer;
int cmdVelLeft = 0;
int cmdVelRight = 0;

ros::NodeHandle nh; //instantiate the node handle for ROS

void cmdVel_CallBack( const geometry_msgs::Twist& twist) {
  //setVelLeft = (double)twist.linear.x; //testing
  cmdVelLeft = round( (double)twist.linear.x - (double)twist.angular.z * L_BASE );
  //setVelRight = (double)twist.linear.x; //testing
  cmdVelRight = round( (double)twist.linear.x + (double)twist.angular.z * L_BASE );
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVel_CallBack);

std_msgs::Int16 heading_msg;
ros::Publisher pubCompassHeading("msg_compass", &heading_msg);

rosserial_arduino::Adc IRarray_msg;
ros::Publisher pubIRarray("msg_irarray", &IRarray_msg);




void setup() {
  nh.initNode();
  nh.subscribe(subCmdVel);
  nh.advertise(pubCompassHeading);
  nh.advertise(pubIRarray);

  Robot.begin();
  Robot.setMode(MODE_SIMPLE);

  loopTimer = micros() + LOOP_TIME; //Set the loopTimer variable for the next loop-end time.
}


void loop() {
  //Robot.motorsWrite(0, 0);      // slow stop
  //Robot.motorsStop();           // fast stop
  Robot.motorsWrite(cmdVelLeft, cmdVelRight);
  
  heading_msg.data = Robot.compassRead();
  pubCompassHeading.publish( &heading_msg );

  Robot.updateIR();
  IRarray_msg.adc0 = Robot.IRarray[0];
  IRarray_msg.adc1 = Robot.IRarray[1];
  IRarray_msg.adc2 = Robot.IRarray[2];
  IRarray_msg.adc3 = Robot.IRarray[3];
  IRarray_msg.adc4 = Robot.IRarray[4];
  IRarray_msg.adc5 = Robot.IRarray[5]; 
  pubIRarray.publish(&IRarray_msg);

  nh.spinOnce();
  // Wait for the remaining time in the loop and set the new loop-end time.
  while(loopTimer > micros()){;}
    loopTimer += LOOP_TIME;
}
