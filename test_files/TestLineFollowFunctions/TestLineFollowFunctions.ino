#include "LineFollowFunctions.h"
#include "MotorFunctions.h"
#include <SimpleRSLK.h>
/*
Author: Rowan Sammon
Created: 1/16/2024
Last modified: 01/29/2024
purpose: verify proper functions of the LineFollowFunctions methods
*/

void setup(){
    setupRSLK();    
    
    Serial1.begin(57600);
    Serial1.println("Send a message to calibrate:");
    while(!Serial1);
    while(Serial1.available() <= 0);
    calibrateLineFollow();
    Serial1.println("[Done] Calibration finished");
}

void loop(){
    moveForwardOnLine();
    Serial1.print("value of sensor: "); 
    Serial1.println(getLinePosition());
}