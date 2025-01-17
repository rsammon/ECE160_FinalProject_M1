/*
Header file for MotorFunction.cpp
Created by: Rowan Sammon
Created: 01/16/2024
Last modified: 01/16/2024

Initialize any functions & variables that exist in MotorFunction.cpp here.
*/

#include <Arduino.h>
#include <Servo.h>

void forward();
void forward(int speed);
void backward();
void backward(int speed);
void stop();
void spinLeft();
void spinLeft(int speed);
void spinRight();
void spinRight(int speed);
void moveRL(int speedLeft, int speedRight);
int useGripper(int initialPos, Servo myServo);
void flip(int degrees);
