/*
  print_sonar_distance.ino
  
  Description:
  prints the distance from the sonar sensors

  Created by: Rowan Sammon
  Created: 01/18/2024
  Last modified: 02/06/2024
  Version: 1.0
*/ 
#include <NewPing.h>

const int leftSonarPin =  62;
const int rightSonarPin = 63;
const int CENTER_SONAR_TRIG = 70;
const int CENTER_SONAR_ECHO = 71;
const int MAX_DISTANCE = 200;

NewPing leftSonar =  NewPing(leftSonarPin, leftSonarPin, MAX_DISTANCE); // Each sensor's trigger pin, echo pin, and max distance to ping. 
NewPing rightSonar = NewPing(rightSonarPin, rightSonarPin, MAX_DISTANCE);
NewPing centerSonar = NewPing (CENTER_SONAR_TRIG, CENTER_SONAR_ECHO, MAX_DISTANCE); 

void setup(){
    Serial1.begin(57600);
}
void loop(){
  delay(100);
  Serial1.print("[ ");
  Serial1.print(leftSonar.ping_cm());
  Serial1.print(", ");
  Serial1.print(centerSonar.ping_cm());
  Serial1.print(", ");
  Serial1.print(rightSonar.ping_cm());
  Serial1.println("]");
  
}