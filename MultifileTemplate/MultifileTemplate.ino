/*A multifile project code template
  A template for the Milestone 1 project code that uses multiple files
  for modularity. The compiler first loads the principal file 
  (the one with the same name as the folder) and then loads 
  the others in alphabetical order. Variables defined in an 
  earlier file will be visible to code in a file that is loaded 
  later, but not vice-versa. 

  Calls functions in files:
  MotorFunctions.ino

  written for the MSP432401 board
  Author: Deborah Walter
  Last revised: 11/28/2023

***** Hardware Connections: *****
     start button       P3.0
     playstation connections
     brown wire         P1.7 
     orange wire        P1.6 
     yellow wire        P2.3
     blue wire          P6.7
*/

// Load libraries used
#include "SimpleRSLK.h"
#include <Servo.h>
#include <PS2X_lib.h>
#include "MotorFunctions.h"

// Define pin numbers for the button on the PlayStation controller
#define PS2_DAT 14  //P1.7 <-> brown wire
#define PS2_CMD 15  //P1.6 <-> orange wire
#define PS2_SEL 34  //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK 35  //P6.7 <-> blue wire

// Create an instance of the playstation controller object
PS2X ps2x;

// Define remote mode either playstation controller or IR remote controller
enum RemoteMode {
  PLAYSTATION,
  IR_REMOTE,
};

// Declare and initialize the current state variable
RemoteMode CurrentRemoteMode = PLAYSTATION;

// Tuning Parameters
const uint16_t lowSpeed = 15;
const uint16_t fastSpeed = 30;

//Declare variables
int leftStickValue;
int rightStickValue;
int leftSpeed;
int rightSpeed;
const int TOP_SPEED = 75; 

void setup() {
  Serial.begin(57600);
  Serial.print("Starting up Robot code...... ");

  // Run setup code
  setupRSLK();

  if (CurrentRemoteMode == 0) {
    // using the playstation controller
    Serial.println("Using playstation controller, make sure it is paired first ");

    // Initialize PlayStation controller
    delayMicroseconds(500 * 1000);  //added delay to give wireless ps2 module some time to startup, before configuring it
    // declare variables for playstation control
    bool pressures = false;
    bool rumble = false;
    int error = 1;

    while (error) {
      error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

      if (error == 0)
        Serial.println("Found Controller, configured successful ");

      else if (error == 1)
        Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

      else if (error == 2)
        Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

      else if (error == 3)
        Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
      delayMicroseconds(1000 * 1000);
    }
  } else if (CurrentRemoteMode == 1) {
    // put start-up code for IR controller here if neccessary
  }
}

void loop() {
  // Read input from PlayStation controller
  ps2x.read_gamepad();

  // Operate the robot in remote control mode
  if (CurrentRemoteMode == 0) {
    Serial.println("Running remote control with the Playstation Controller");
    RemoteControlPlaystation();

  } else if (CurrentRemoteMode == 1) {
    RemoteControlIR();
  }
}


  /* RemoteControlPlaystation() function
  This function uses a playstation controller and the PLSK libraray with
  an RLSK robot using to implement remote controller. 
 
  Button control map:
  Left joystick controls left wheel speed
  Right joystick controls right wheel speed
  CROSS button stops motors
  */
  void RemoteControlPlaystation() {
    // put your code here to run in remote control mode

    // Example of receive and decode remote control command
    // the forward() and stop() functions should be independent of
    // the control methods
    leftStickValue = ps2x.Analog(PSS_LY);
    rightStickValue = ps2x.Analog(PSS_RY);
    leftSpeed = map(leftStickValue, 0, 255, -TOP_SPEED,TOP_SPEED);
    rightSpeed = map(rightStickValue, 0, 255, -TOP_SPEED,TOP_SPEED);

    if (leftSpeed>5||rightSpeed>5) {
      moveRL(leftSpeed, rightSpeed);
    }
    if (ps2x.Button(PSB_CIRCLE)) {
      Serial.println("CIRCLE button pushed");
      
    }
    delay(50);
  }


 /* RemoteControlIR() function
  This function uses an IR controller with
  an RLSK robot to implement the remote controller. 
 
  Button control map:
 
  */
  void RemoteControlIR(){

  }
