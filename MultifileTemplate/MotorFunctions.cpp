/*
  MotorFunctions.ino - Arduino Sketch for Motor Control
  
  Description:
  This sketch provides functions for controlling a RLSK robot, allowing the
  motors to move forward and stop.

  Functions:
  1. void moveForward()
     - Activates the motor to move forward.
     void moveForward(int speed)
     -Activates the motor to move forward at a speed of the parameter

  2. void stopMotor()
     - Stops the motor.

  Created by: Rowan Sammon
  Created: 01/16/2024
  Last modified: 01/16/2024
  Version: 1.0
*/ 

#include <SimpleRSLK.h>
#include "MotorFunctions.h"

//parameters of the file
int fastSpeed = 75;

/* Moves robot forward: both motors forward same speed */
//defaults to speed of 75%
void forward() {
   enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
}

//moves robot forward: both motors forward same speed
//@param int speed: the speed from 0%-100% to go forward at
void forward(int speed) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, speed);
}

/* Moves robot forward: both motors forward same speed */
//defaults to speed of 75%
void backward() {
   enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
}

//moves robot forward: both motors forward same speed
//@param int speed: the speed from 0%-100% to go forward at
void backward(int speed) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS, speed);
}

/* Stops robot forward: both motors disabled */
void stop() {
    disableMotor(BOTH_MOTORS);
}

/*spins robot to the left by turning both motors in opposite directions*/
void spinLeft(){
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
//@param int speed: the speed from 0%-100% to turn at
void spinLeft(int speed){
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, speed);
}

/*spins robot to the right by turning both motors in opposite directions*/
void spinRight(){
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
//@param int speed: the speed from 0%-100% to turn at
void spinRight(int speed){
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS, speed);
}