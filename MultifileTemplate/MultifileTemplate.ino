/*
  Final project code for ECE 160
  Based on multifile template
  Program can use a playstation controller or IR remoter to direct a robot

  Calls functions in files:
  MotorFunctions.ino
  MotorFunctions.cpp

  written for the MSP432401 board
  Author: Deborah Walter, Rowan Sammon, Wyatt Ronn, and Eric Steuber
  Last revised: 1/23/24

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
#include <TinyIRremote.h>
#include <TinyNEC.h>


//IR Setup
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define IR_RCV_PIN      33
IRreceiver irRX(IR_RCV_PIN);
IRData IRresults;


// Define pin numbers for the button on the PlayStation controller
#define PS2_DAT 14  //P1.7 <-> brown wire
#define PS2_CMD 15  //P1.6 <-> orange wire
#define PS2_SEL 34  //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK 35  //P6.7 <-> blue wire

// Create an instance of the playstation controller object
PS2X ps2x;


//New servo object
Servo myServo;
const int SERVO_PIN = 38;


// Define remote mode either playstation controller or IR remote controller
enum RemoteMode {
  PLAYSTATION,
  IR_REMOTE,
};

// Declare and initialize the current state variable
RemoteMode CurrentRemoteMode = IR_REMOTE;

// Tuning Parameters
const uint16_t lowSpeed = 15;
const uint16_t midSpeed = 30;
const uint16_t highSpeed = 70;

//Declare variables
int leftStickValue;
int rightStickValue;
int leftSpeed;
int rightSpeed;
int topSpeed = 30; 
int gripperPos = 0;
unsigned long currentTime;
unsigned long previousTime;
const int INTERVAL = 300;

void setup() {
  Serial.begin(57600);
  Serial.print("Starting up Robot code...... ");

  // Run setup code
  setupRSLK();
  myServo.attach(SERVO_PIN);


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
    Serial.begin(57600);
    delay(500); // To be able to connect Serial monitor after reset or power up 
    Serial.println(F("START " __FILE__ " from " __DATE__));
    /*
     * Must be called to initialize and set up IR receiver pin.
     *  bool initIRReceiver(bool includeRepeats = true, bool enableCallback = false,
                void (*callbackFunction)(uint16_t , uint8_t , bool) = NULL)
     */
    if (irRX.initIRReceiver()) {
        Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_RCV_PIN)));
    } else {
        Serial.println("Initialization of IR receiver failed!");
        while (1) {;}
    }
    // enable receive feedback and specify LED pin number (defaults to LED_BUILTIN)
    enableRXLEDFeedback(BLUE_LED);
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
    Serial.println("Running remote control with the IR Remote");
    RemoteControlIR();
  }
}


  /* RemoteControlPlaystation() function
  This function uses a playstation controller and the PLSK libraray with
  an RLSK robot using to implement remote controller. 
 
  Button control map:
  Left joystick controls left wheel speed
  Right joystick controls right wheel speed
  Circle button opens/closes gripper
  First bumper buttons enable slow mode
  Second bumper buttons enable fast mode
  Up on D-pad transmits the gold candle IR signal
  Down on the D-pad recieves an IR signal and sends it back
  */
  void RemoteControlPlaystation() {
    // put your code here to run in remote control mode

    // Example of receive and decode remote control command
    // the forward() and stop() functions should be independent of
    // the control methods
    currentTime = millis();
    //Speed setitngs
    if(ps2x.Button(PSB_L2)&&ps2x.Button(PSB_R2)){
      topSpeed = highSpeed;
    }
    else if(ps2x.Button(PSB_L1)&&ps2x.Button(PSB_R1)) {
      topSpeed = lowSpeed;
    }
    else {
      topSpeed = midSpeed;
    }
    leftStickValue = ps2x.Analog(PSS_LY);
    rightStickValue = ps2x.Analog(PSS_RY);
    leftSpeed = -(leftStickValue - 127)*topSpeed/128;
    rightSpeed =  -(rightStickValue - 127)*topSpeed/128;
    Serial.print("leftSpeed: ");//Debugging help
    Serial.print(leftSpeed);
    Serial.print(" | rightSpeed: ");
    Serial.print(rightSpeed);
    Serial.print(" | leftStick: ");
    Serial.print(leftStickValue);
    Serial.print(" | rightStick: ");
    Serial.print(rightStickValue);
    moveRL(leftSpeed, rightSpeed);//Movement function

    //Gripper functionality
    if(currentTime-previousTime>=INTERVAL){
      if (ps2x.Button(PSB_CIRCLE)) {
        Serial.println("CIRCLE button pushed");
        gripperPos = useGripper(gripperPos, myServo);
        previousTime = currentTime;
        //This timer ensures the gripper only detects one button push at a time
      }
    }

    //IR transmitter
    if(ps2x.Button(PSB_PAD_UP)){
      //send command
      Serial.print('.');
      delay(200);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      //recieve IR signal on address and retransmit it
      irRX.decodeIR(&IRresults);
      int command = IRresults.command;
      //send command
      Serial.print('.');
      delay(200);
    }
  }


  /* RemoteControlIR() function
  This function uses an IR controller with
  an RLSK robot to implement the remote controller. 
 
  Button control map:
  Vol+ = forward
  Vol- = backward
  Play/Pause = Stop
  Skip back = spin left
  Skip forward = spin right
  Channel down = turn left
  Channel up = turn right
  0 = close gripper
  1 = open gripper
  */
  void RemoteControlIR(){
    irRX.decodeIR(&IRresults);
    currentTime = millis();
    int command = IRresults.command;
    Serial.print(command);
    switch (command) {
      case 68:
        moveRL(-15, 15);
        break;
      case 67:
        moveRL(15, -15);
        break;
      case 70:
        moveRL(30, 30);
        break;
      case 21:
        moveRL(-20, -20);
        break;
      case 64:
        moveRL(0, 0);
        break;
      case 7:
        moveRL(20, 10);
        break;
      case 9:
        moveRL(10, 20);
        break;
      case 22:
        //Gripper functionality
        useGripper(0, myServo);
        break;
      case 12:
        useGripper(80, myServo);
        break;
    }
  }
  
  /*
  AutonomousMode() function
  This function runs when the robot is switch to auto mode.
  It has 3 states: tunnel, line follow, and drop payload.
    - The tunnel state moves the robot through the dark tunnel.
    - The line follow state makes the robot find the line and follow it.
    - The drop payload state makes the robot drop the marigold within the zone and idle until the user takes control.
  */
  void AutonomousMode(){
    switch (condition) {
      case: AutoState == TUNNEL
        //Eric, insert tunnel code here
        break;
      case: AutoState == LINE_FOLLOW
        //Rowan, insert line follow code here
        break;
      case: AutoState == DROP_PAYLOAD
        //insert code to drop marigold in drop zone
        break;
    }
  }
