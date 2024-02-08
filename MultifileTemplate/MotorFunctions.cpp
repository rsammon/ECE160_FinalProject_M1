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
  Last modified: 01/17/2024
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

/** moves the robot forward at the specified right and left speeds
* @param leftSpeed: the speed from -100 to 100 percent (negative is backward, positive is forward) to move the left motor
* @param rightSpeed: the speed from -100 to 100 percent (negative is backward, positive is forward) to move the right motor
*/
void moveRL(int leftSpeed, int rightSpeed){
    //parameter validity checks
    if(leftSpeed > 100) leftSpeed = 100;
    if(leftSpeed < -100) leftSpeed = -100;
    if(rightSpeed > 100) rightSpeed = 100;
    if(rightSpeed < -100) rightSpeed = -100;

    enableMotor(BOTH_MOTORS);

    //left motor set
    int leftMotorDirection;
    if(leftSpeed >= 0) leftMotorDirection = MOTOR_DIR_FORWARD;
    else leftMotorDirection = MOTOR_DIR_BACKWARD;

    setMotorDirection(LEFT_MOTOR, leftMotorDirection);
    setMotorSpeed(LEFT_MOTOR, abs(leftSpeed));

    //right motor set
    int rightMotorDirection;
    if(rightSpeed >= 0) rightMotorDirection = MOTOR_DIR_FORWARD;
    else rightMotorDirection = MOTOR_DIR_BACKWARD;

    setMotorDirection(RIGHT_MOTOR, rightMotorDirection);
    setMotorSpeed(RIGHT_MOTOR, abs(rightSpeed));

}


int closedPos = 80;
//@param int initialPos: starting position, should be 80 or 0.
int useGripper(int initialPos, Servo myServo){
  int pos;
  if (initialPos == 0){
    pos = closedPos;
    myServo.write(pos); 
  }
  else {
    pos = 0;
    myServo.write(pos); 
  }
  return pos;
}

//@param int degrees: the angle the robot should rotate
void flip(int degrees){
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  moveRL(15, -15);
  while (getEncoderRightCnt()<degrees*2) {
  }
  stop();
}


