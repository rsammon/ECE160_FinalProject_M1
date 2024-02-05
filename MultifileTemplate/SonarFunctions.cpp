/*
  SonarFunctions.cpp - Controls robot based on sonar input

  Created by: Rowan Sammon
  Created: 01/16/2024
  Last modified: 02/04/2024
  Version: 1.0
*/ 
#include "SonarFunctions.h"
extern NewPing rightSonar = NewPing(RIGHT_SONAR, RIGHT_SONAR, MAX_SONAR_DISTANCE);
extern NewPing leftSonar =  NewPing(LEFT_SONAR, LEFT_SONAR, MAX_SONAR_DISTANCE); 

long findDifferenceBetweenSonars(){
    return leftSonar.ping_cm() - rightSonar.ping_cm();
}
long leftSonarCM(){
  return leftSonar.ping_cm();
}
long rightSonarCM(){
  return rightSonar.ping_cm();
}

void centerRobotSonarForward(){
    long difference = findDifferenceBetweenSonars();
    const int BASE_SPEED = 10;
    long speed = abs(difference)*3;
    long addSpeed = constrain(speed, 0, BASE_SPEED);
    
    if(difference == 0) moveRL(BASE_SPEED, BASE_SPEED);
    else if(difference > 0) moveRL(BASE_SPEED, BASE_SPEED+addSpeed);
    else if(difference < 0) moveRL(BASE_SPEED+addSpeed, BASE_SPEED);
}
void printSonarData(){
  //delay(100);
  Serial1.print("[ ");
  Serial1.print(leftSonar.ping_cm());
  Serial1.print(", ");
  Serial1.print(rightSonar.ping_cm());
  Serial1.println("]");
}