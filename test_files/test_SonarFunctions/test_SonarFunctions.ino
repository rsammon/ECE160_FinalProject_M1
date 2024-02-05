#include "SonarFunctions.h"
#include <SimpleRSLK.h>
void setup(){
    Serial1.begin(57600);
    setupRSLK();
}
void loop(){
  printSonarData();
  centerRobotSonarForward();
}