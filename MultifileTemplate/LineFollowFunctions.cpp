/*
  LineFollowFunctions.cpp - Arduino Sketch for Motor Line Following Control
  
  Description:
  This sketch provides functions for controlling a RLSK robot, allowing the
  motors to move forward along a line.

  Functions:
  1. calibrateLineFollow
    - calibrates the sensors of the robot
    - the line followers should be placed over a dark surface
    - takes 5 milliseconds

  Created by: Rowan Sammon
  Created: 01/16/2024
  Last modified: 01/29/2024
  Version: 1.0
*/ 

#include <SimpleRSLK.h>
#include "LineFollowFunctions.h"

#define MIDDLE_ON_LINE 4500
#define LEFT_ON_LINE 1000
#define RIGHT_ON_LINE 8000

const uint16_t normalSpeed = 10;
const uint16_t fastSpeed = 20;

void calibrateLineFollow(){
    calibrateLineSensor();
    enableMotor(BOTH_MOTORS);
}

void moveForwardOnLine(){
    int linePos = getLinePosition();
    enableMotor(BOTH_MOTORS);

    if ((linePos > 0) && (linePos < 4000)) {    // turn left
        setMotorSpeed(LEFT_MOTOR, normalSpeed);
        setMotorSpeed(RIGHT_MOTOR, fastSpeed);
    } else if (linePos > 5000) {                // turn right
        setMotorSpeed(LEFT_MOTOR, fastSpeed);
        setMotorSpeed(RIGHT_MOTOR, normalSpeed);
    } else {                                    // go straight
        setMotorSpeed(LEFT_MOTOR, normalSpeed);
        setMotorSpeed(RIGHT_MOTOR, normalSpeed);
    }

/*
    //robot is on left side of line
    if(LEFT_ON_LINE>=linePos && linePos<MIDDLE_ON_LINE){
        SLIGHTLY_OVER_MOTOR_SPEED = SLIGHTLY_OVER_OFFSET*(MIDDLE_ON_LINE-linePos)/(MIDDLE_ON_LINE-LEFT_ON_LINE);
        moveRL(SLIGHTLY_OVER_MOTOR_SPEED, MOTOR_LINE_FOLLOW_SPEED);
    }
    else if(LEFT_ON_LINE>=linePos && linePos<MIDDLE_ON_LINE){ //right side
        SLIGHTLY_OVER_MOTOR_SPEED = SLIGHTLY_OVER_MOTOR_SPEED*(MIDDLE_ON_LINE-linePos)/(MIDDLE_ON_LINE-LEFT_ON_LINE);
        moveRL(SLIGHTLY_OVER_MOTOR_SPEED, MOTOR_LINE_FOLLOW_SPEED);
    }
*/
}