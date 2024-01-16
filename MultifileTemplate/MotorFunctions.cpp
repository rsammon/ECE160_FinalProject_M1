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
int fastSpeed = 255;

/* Moves robot forward: both motors forward same speed */
void forward() {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
void forward(int speed) {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, speed);
}

/* Stops robot forward: both motors disabled */
void stop() {
    disableMotor(BOTH_MOTORS);
}
