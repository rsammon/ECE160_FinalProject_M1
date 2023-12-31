/*
  MotorFunctions.ino - Arduino Sketch for Motor Control
  
  Description:
  This sketch provides functions for controlling a RLSK robot, allowing the
  motors to move forward and stop.

  Functions:
  1. void moveForward()
     - Activates the motor to move forward.

  2. void stopMotor()
     - Stops the motor.

  Created by: Your Name
  Date: Current Date
  Version: 1.0
*/

/* Moves robot forward: both motors forward same speed */
void forward() {
    enableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, fastSpeed);
}

/* Stops robot forward: both motors disabled */
void stop() {
    disableMotor(BOTH_MOTORS);
}
