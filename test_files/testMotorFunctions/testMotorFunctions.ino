/*
Author: Rowan Sammon
Created: 1/16/2024
Last modified: 01/17/2024
purpose: verify proper functions of the MotorFunctions.cpp methods
*/

/*
Arduino is really weird about where the file is in includes. to compile this 
properly, you need to either put the MotorFunctions.cpp & MotorFunctions.h in
 the same directory/folder as this file.
 - Rowan Sammon
 */
#include <SimpleRSLK.h>
#include "MotorFunctions.h"


int k = 1000;
long timeStarted;
long timeSinceStart;
void setup(){
    setupRSLK();
    timeStarted=millis();
}

void loop(){
    timeSinceStart = millis()-timeStarted;

    if(timeSinceStart<k){
        forward(25);
    }
    else if(timeSinceStart<2*k){
        backward(25);
    } 
    else if(timeSinceStart<3*k){
        spinLeft(25);
    }
    else if(timeSinceStart<4*k){
        spinRight(25);
    } else if(timeSinceStart<5*k){
        moveRL(50, 50);
    } else if(timeSinceStart<6*k){
        moveRL(-50, -50);
    } else if(timeSinceStart<7*k){
        moveRL(50, 0);
    } else if(timeSinceStart<8*k){
        moveRL(-50, 50);
    }
    else if(timeSinceStart%k==0) stop();
}