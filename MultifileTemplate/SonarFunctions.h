/*
Header file for SonarFunctions.cpp
Created by: Rowan Sammon
Created: 01/16/2024
Last modified: 02/04/2024

Initialize any functions & variables that exist in MotorFunction.cpp here.
*/
const int LEFT_SONAR = 62;
const int RIGHT_SONAR = 63;
const int MAX_SONAR_DISTANCE = 200;
#include <NewPing.h>
#include "Servo.h"
#include "MotorFunctions.h"

void centerRobotSonarForward();
void printSonarData();
long rightSonarCM();
long leftSonarCM();