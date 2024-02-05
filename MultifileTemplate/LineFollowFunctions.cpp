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
  Last modified: 02/05/2024
  Version: 1.0
*/ 

#include <SimpleRSLK.h>
#include "LineFollowFunctions.h"


const uint16_t normalSpeed = 10;
const uint16_t fastSpeed = 20;

void calibrateLineFollow(){
    calibrateLineSensor(LIGHT_LINE);
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

}

#define LINE_TO_LEFT -1
#define LINE_TO_RIGHT 1

/**
 * @returns -1 if line to the left, 0 if in center, 1 if on right
*/
int lineTurning(){
    uint16_t sensorArray[8];
    // uint16_t* sensors = sensorArray;
    readCalLineSensor(sensorArray);
    boolean under = true;
    for(int i =0; i< (sizeof(sensorArray)/sizeof(sensorArray[0]))/2; i++){
        if(sensorArray[i] < 500) under = false;
    }
    if(under) return -1;

    under = true;
    for(int i =(sizeof(sensorArray)/sizeof(sensorArray[0]))/2; i< (sizeof(sensorArray)/sizeof(sensorArray[0])); i++){
        if(sensorArray[i] < 500) under = false;
    }
    if(under) return 1;
    return 0;
}