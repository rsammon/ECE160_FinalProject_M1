#include "LineFollowFunctions.h"
#include "MotorFunctions.h"
#include <SimpleRSLK.h>
/*
Author: Rowan Sammon
Created: 1/16/2024
Last modified: 02/05/2024
purpose: verify proper functions of the LineFollowFunctions methods
*/

#define START_BUTTON_PIN 9

void setup(){
    setupRSLK();
    pinMode(START_BUTTON_PIN, INPUT_PULLDOWN);
    
    Serial1.begin(57600);
    Serial1.println("Press the button to calibrate");
    while(digitalRead(START_BUTTON_PIN) == LOW);
    
    calibrateLineFollow();
    Serial1.println("[Done] Calibration finished");
}

void loop(){
    moveForwardOnLine(true);
    Serial1.print("value of sensor: "); 
    Serial1.print(getLinePosition());
    Serial1.print(", line direction: ");
    Serial1.println(lineTurning());
    
}