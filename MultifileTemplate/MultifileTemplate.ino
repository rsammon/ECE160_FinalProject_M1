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
#include "LineFollowFunctions.h"


//IR Setup
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define IR_RCV_PIN      33
IRreceiver irRX(IR_RCV_PIN);
IRData IRresults;
IRData IRmsg;

int IRLEDpin = 46;
IRsender irTX = IRsender(IRLEDpin);
int SensorPos = 1;
int lightSensor = A2;

//Initialize Autonomous States
int AutoState = 0;
const int START_IN_TUNNEL = 0;
const int LINE_FOLLOW = 1;
const int DROP_PAYLOAD = 2;
const int TURN_IN_TUNNEL = 3;
const int EXIT_TUNNEL = 4;
const int LEAVE_TUNNEL = 5;
const int IDLE = 6;

//Initialize AutoOrManual States
const int MANUAL = 1;
const int AUTO = 0;
int AutoOrManual = MANUAL;


//states
#define PS_STATE 0
#define IR_STATE 1


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
RemoteMode CurrentRemoteMode = PLAYSTATION;

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
double distIN;


void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  Serial.print("Starting up Robot code...... ");
  Serial1.print("Starting up Robot code...... ");

  // Run setup code
  setupRSLK();
  myServo.attach(SERVO_PIN);
  pinMode(IRLEDpin, OUTPUT);
  pinMode(lightSensor, INPUT);


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
    IRmsg.protocol = NEC;
    IRmsg.address = 0xEE;
    IRmsg.command = 160;
    IRmsg.isRepeat = false;
    irTX.initIRSender();
    delay(500);
    
  }

//implement a method to allow calibration after button is pressed
  calibrateLineFollow();
}

void loop() {
  // Read input from PlayStation controller
  ps2x.read_gamepad();

   // Operate the robot in remote control mode
  if (CurrentRemoteMode == PS_STATE) {
    Serial.print("| Running remote control with the Playstation Controller");
    RemoteControlPlaystation();

  } else if (CurrentRemoteMode == IR_STATE) {
    Serial.println("Running remote control with the IR Remote");
    RemoteControlIR();
  }
}


  /* RemoteControlPlaystation() function
  This function uses a playstation controller and the PLSK libraray with
  an RLSK robot to implement the remote controller. 
  
  The function has two states: manual and automatic.
  Pressing the SELECT button switches between states.
  */
  void RemoteControlPlaystation() {
    // put your code here to run in remote control mode

    // Example of receive and decode remote control command
    // the forward() and stop() functions should be independent of
    // the control methods
    currentTime = millis();
    switch (AutoOrManual) {
      case AUTO:
        AutonomousMode();
        if(ps2x.Button(PSB_SELECT)&&currentTime-previousTime>INTERVAL){
          AutoOrManual=MANUAL;
          previousTime=currentTime;
        }
        break;
      case MANUAL:
        ManualMode();
        if(ps2x.Button(PSB_SELECT)&&currentTime-previousTime>INTERVAL){
          AutoOrManual=AUTO;
          AutoState = START_IN_TUNNEL;
          previousTime=currentTime;
        }
        break;
    }
  }
  /*
  ManualMode() function
  This function is looped when in manual playstation mode.
  Button control map:
  Left joystick controls left wheel speed
  Right joystick controls right wheel speed
  Circle button opens/closes gripper
  First bumper buttons enable slow mode
  Second bumper buttons enable fast mode
  Up on D-pad transmits the gold candle IR signal
  Down on the D-pad recieves an IR signal and sends it back
  */
  void ManualMode(){
      //Speed settings
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
      Serial.println(rightStickValue);
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

      //Display light sensor output for debugging
      if(ps2x.Button(PSB_SQUARE)){
        Serial.print("Light: ");
        Serial.print(analogRead(lightSensor));
      }

      //IR transmitter
      if(ps2x.Button(PSB_PAD_UP)){
        Serial.print(" | Transmitting...");
        digitalWrite(IRLEDpin, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        irTX.write(&IRmsg);
        
      }
      else if(ps2x.Button(PSB_PAD_DOWN)){
        //recieve IR signal on address and retransmit it
        Serial.println("Recieving and retransmitting...");
        irRX.decodeIR(&IRresults);
        irTX.write(&IRresults);
        Serial.print('.');
        delay(200);
      }
      else{
        digitalWrite(IRLEDpin, LOW);
        digitalWrite(LED_BUILTIN, LOW);
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
    Serial.print("In Auto Mode. Distance: ");
    distIN = readSharpDistIN(SensorPos);
    Serial.print(distIN);
    switch (AutoState) {
      case START_IN_TUNNEL:
        Serial.println(" | In start tunnel state.");
        moveRL(30, 30);
        if (distIN<5){
          AutoState = TURN_IN_TUNNEL;
        }
        break;
      case TURN_IN_TUNNEL:
        Serial.println(" | In turning tunnel state.");
        moveRL(15, -15);
        if (distIN>20){
          AutoState = EXIT_TUNNEL;
        }
        break;
      case EXIT_TUNNEL:
        Serial.println(" | In exit tunnel state.");
        moveRL(30,30);
        if (analogRead(lightSensor)>2000){
          AutoState = LEAVE_TUNNEL;
          previousTime = currentTime;
        }
        break;
      case LINE_FOLLOW:
        Serial.println(" | In line follow state.");
        moveForwardOnLine();
        if(distIN<10){
          AutoState = DROP_PAYLOAD;
        }
        break;
      case DROP_PAYLOAD:
        Serial.println(" | In drop payload state.");
        stop();
        flip(180);
        useGripper(80, myServo);
        AutoState = IDLE;
        break;
      case LEAVE_TUNNEL:
        moveRL(15, 15);
        if (currentTime-previousTime>INTERVAL){
          AutoState=LINE_FOLLOW;
        }
        break;
      case IDLE:
      stop();
      break;
    }
  }
