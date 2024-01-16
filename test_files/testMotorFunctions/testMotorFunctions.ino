/*
Author: Rowan Sammon
Created: 1/16/2024
Last modified: 01/16/2024
purpose: verify proper functions of the MotorFunctions.cpp methods
*/

/*
Arduino is really weird about where the file is in includes. to compile this 
properly, you need to either put the MotorFunctions.cpp & MotorFunctions.h in
 the same directory/folder as this file.
 - Rowan Sammon
 */
#include "MotorFunctions.h"

long timeStarted;
void setup(){
    timeStarted=millis();
}

void loop(){
    if(millis()-timeStarted<1000){
        forward();
    }
     if(millis()-timeStarted<2000 && millis()-timeStarted>1000){
        forward(100);
    }
    stop();
}