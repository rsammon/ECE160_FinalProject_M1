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


const uint16_t normalSpeed = 12;


void calibrateLineFollow(){
    calibrateLineSensor(LIGHT_LINE);
    enableMotor(BOTH_MOTORS);
}

extern int turningOnLineFlag = 0;
void moveForwardOnLine(boolean turnOnLine){
    int linePos = getLinePosition();
    enableMotor(BOTH_MOTORS);
    if(turnOnLine){
    int turnDelay = 1100;
    int lineTurn = lineTurningStartDetection();

    if(lineTurn == -1){
         moveRL(-normalSpeed, normalSpeed);
         delay(turnDelay);
         moveRL(normalSpeed, normalSpeed);
         delay(turnDelay);
    }
    if(lineTurn == 1){
         moveRL(normalSpeed, -normalSpeed);
         delay(turnDelay);
         moveRL(normalSpeed, normalSpeed);
         delay(turnDelay);
    }
    /*
    if(turningOnLineFlag != 0 &&( linePos < 4600 && linePos > 4400)){
        turningOnLineFlag = 0;
        Serial1.println("line reached");
        moveRL(normalSpeed, normalSpeed);
        delay(100);
    }
    
    if(turningOnLineFlag != 0 &&( lineTurn == -2 || lineTurn == 2)){
        turningOnLineFlag = 2;
    }
    if(turningOnLineFlag == 2 && !( lineTurn == -2  || lineTurn == 2)){
        turningOnLineFlag = 0;
    }
    if(turningOnLineFlag == 0 && lineTurn == -1){
        turningOnLineFlag = lineTurn;
        Serial1.println("line to left");
        moveRL(normalSpeed, normalSpeed);
        delay(100);
    }
    if(turningOnLineFlag == 0 && lineTurn == 1){
        turningOnLineFlag = lineTurn;
        Serial1.println("line to right");
        moveRL(normalSpeed, normalSpeed);
        delay(100);
    }
    if(turningOnLineFlag == -1){
        moveRL(-normalSpeed, normalSpeed);
        
    }
    if(turningOnLineFlag == 1){
         moveRL(normalSpeed, -normalSpeed);
    }
    //if(turningOnLineFlag == 2) moveRL(normalSpeed, normalSpeed);
    */
    }

    if(!turnOnLine || turningOnLineFlag == 0){
        double motorSpeedPure = abs(linePos-4500)/500;
        double motorSpeedConstrained = normalSpeed+constrain(motorSpeedPure, 0, normalSpeed);
        double motorSpeedSmall = normalSpeed;
        if(linePos ==4500) setMotorSpeed(BOTH_MOTORS, normalSpeed);
        else if(linePos < 4500){
        setMotorSpeed(LEFT_MOTOR, motorSpeedSmall);
        setMotorSpeed(RIGHT_MOTOR, motorSpeedConstrained);
        }
        else if(linePos > 4500){
        setMotorSpeed(RIGHT_MOTOR, motorSpeedSmall);
        setMotorSpeed(LEFT_MOTOR, motorSpeedConstrained);
        }
        
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

int lastTurningValue;
int lineTurningStartDetection(){
    int lineTurn = lineTurning();
    if(lineTurn == lastTurningValue){
         return 0;
    }
    lastTurningValue = lineTurn;
    return lineTurn;
}

boolean lineLeft(){
    if(lineTurning()==-1) return true;
    return false;
}
boolean lineRight(){
    if(lineTurning()==1) return true;
    return false;
}