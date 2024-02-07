/*
  Final project code for ECE 160
  Based on multifile template
  Program can use a playstation controller or IR remote to direct a robot

  Calls functions in files:
  MotorFunctions.ino
  MotorFunctions.cpp
  LineFollowFunctions.cpp

  written for the MSP432401 board
  Author: Deborah Walter, Rowan Sammon, Wyatt Ronn, and Eric Steuber
  Last revised: 1/30/24

***** Hardware Connections: *****
      [VIEW README.md]
*/

// Load libraries used
#include "SimpleRSLK.h"
#include <Servo.h>
#include <PS2X_lib.h>
#include "MotorFunctions.h"
#include <TinyIRremote.h>
#include <TinyNEC.h>
#include "TinyNECTX.h"
#include "LineFollowFunctions.h"
#include "SonarFunctions.h"

//IR Setup
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define IR_RCV_PIN  33
#define IR_TRX_PIN  37
#define IR_GOLD_PIN 36
IRreceiver irRX(IR_RCV_PIN);
IRsender irTX(IR_TRX_PIN);

IRData IRresults;
IRData IRmsg;

int SensorPos = 2;
int lightSensor = A16;

//Initialize Autonomous States
const int START_IN_TUNNEL = 0;
const int LINE_FOLLOW = 1;
const int DROP_PAYLOAD = 2;
const int TURN_IN_TUNNEL = 3;
const int EXIT_TUNNEL = 4;
const int LEAVE_TUNNEL = 5;
const int IDLE = 6;
int AutoState = START_IN_TUNNEL;

//Initialize AutoOrManual States
const int MANUAL = 1;
const int AUTO = 0;
int AutoOrManual = MANUAL;


//states
#define PS_STATE 0
#define IR_STATE 1

#define START_BUTTON_PIN 9

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

const int diagnosticLEDPins[4] = { 5, 26, 7, 10 };

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

int brightLight;
int darkLight;


void setup() {
  Serial1.begin(57600);
  Serial1.print("Starting up Robot code...... ");

  // Run setup code
  setupRSLK();
  myServo.attach(SERVO_PIN);
  pinMode(lightSensor, INPUT);

  pinMode(START_BUTTON_PIN, INPUT_PULLDOWN);

  // temporary test of IR pin digital write
  pinMode(IR_GOLD_PIN, OUTPUT);

  setupDiagnosticLEDs();

  if (CurrentRemoteMode == 0) {
    // using the playstation controller
    Serial1.println("Using playstation controller, make sure it is paired first ");

    // Initialize PlayStation controller
    delayMicroseconds(500 * 1000);  //added delay to give wireless ps2 module some time to startup, before configuring it
    // declare variables for playstation control
    bool pressures = false;
    bool rumble = false;
    int error = 1;

    if (irTX.initIRSender()) {
      Serial1.println("IR Transmitter initialized.");
      Serial.println("IR Transmitter initialized.");
    } else {
      Serial1.println("IR Transmitter failed to initialize.");
      Serial.println("IR Transmitter failed to initialize.");
      while (1) { ; }
    }
    /*
     * Must be called to initialize and set up IR receiver pin.
     *  bool initIRReceiver(bool includeRepeats = true, bool enableCallback = false,
                void (*callbackFunction)(uint16_t , uint8_t , bool) = NULL)
     */
    if (irRX.initIRReceiver()) {
      Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_RCV_PIN)));
      Serial1.println(F("Ready to receive NEC IR signals at pin " STR(IR_RCV_PIN)));
    } else {
      Serial1.println("Initialization of IR receiver failed!");
      Serial.println("Initialization of IR receiver failed!");
      while (1) { ; }
    }
    // enable receive feedback and specify LED pin number (defaults to LED_BUILTIN)
    enableRXLEDFeedback(BLUE_LED);

    Serial1.println("Setup complete.");

    while (error) {
      error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

      if (error == 0)
        Serial1.println("Found Controller, configured successful ");

      else if (error == 1)
        Serial1.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

      else if (error == 2)
        Serial1.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

      else if (error == 3)
        Serial1.println("Controller refusing to enter Pressures mode, may not support it. ");
      delayMicroseconds(1000 * 1000);
    }
    delay(5000);
  }

  else if (CurrentRemoteMode == 1) {
    Serial1.begin(57600);
    delay(500);  // To be able to connect Serial1 monitor after reset or power up
    Serial1.println(F("START " __FILE__ " from " __DATE__));
  }

  Serial1.println("press button to calibrate line follow & light sensor to bright light");
  waitUntilButtonRelease(START_BUTTON_PIN);
  //implement a method to allow calibration after button is pressed
  calibrateLineFollow();
  brightLight = getAverageLight(lightSensor);
  Serial1.print("Bright light is ");
  Serial1.println(brightLight);
  Serial1.println("[Done] Calibration finished");
  Serial1.println("press button to calibrate to dark light");

  waitUntilButtonRelease(START_BUTTON_PIN);
  darkLight = getAverageLight(lightSensor);
  Serial1.print("Dark light is ");
  Serial1.println(darkLight);
  Serial1.println("[Done] Calibration finished");


  //throw away first readings on the controller (seem to be wrong)
  ps2x.read_gamepad();
  disableMotor(BOTH_MOTORS);
}

void loop() {

  // Read input from PlayStation controller
  ps2x.read_gamepad();

  // Operate the robot in remote control mode
  if (CurrentRemoteMode == PS_STATE) {
    Serial1.print("| Running remote control with the Playstation Controller");
    RemoteControlPlaystation();

  } else if (CurrentRemoteMode == IR_STATE) {
    Serial1.println("Running remote control with the IR Remote");
    RemoteControlIR();
  }


  updateDiagnosticLEDs();
  delay(50);
}

/**
 * Get the average light in the current lighting conditions recorded by the light sensor. used in calibrating the light sensor.
 * @param int sensorPin - the pin the light sensor is connected to
*/
int getAverageLight(int sensorPin, int trials) {
  int sum = 0;
  for (int i = 0; i < trials; i++) {
    sum += analogRead(sensorPin);
  }
  return sum / trials;
}
int getAverageLight(int sensorPin) {
  int sum = 0;
  int trials = 50;
  for (int i = 0; i < trials; i++) {
    sum += analogRead(sensorPin);
  }
  return sum / trials;
}

/** Updates the diagnosticLEDPins based on the Autonomous state
 * @param none
 * @return void
*/
void updateDiagnosticLEDs() {
  digitalWrite(diagnosticLEDPins[0], bitRead(AutoState, 0));
  digitalWrite(diagnosticLEDPins[1], bitRead(AutoState, 1));
  digitalWrite(diagnosticLEDPins[2], bitRead(AutoState, 2));
  digitalWrite(diagnosticLEDPins[3], bitRead(AutoState, 3));
}
/**
 * setups up the LED pins used in the diagnostics to OUTPUT
 * @param none
 * @returns void
*/
void setupDiagnosticLEDs() {
  for (int i = 0; i < sizeof(diagnosticLEDPins) / 4; i++) {
    pinMode(diagnosticLEDPins[i], OUTPUT);
  }
}

/**
 * Waits until the button is pressed & released to proceed with the code
 * @param int buttonPin - the pin of the button (set to INPUT_PULLDOWN) to wait for
*/
void waitUntilButtonRelease(int buttonPin) {
  while (digitalRead(buttonPin) == LOW)
    ;
  while (digitalRead(buttonPin) == HIGH)
    ;
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
      if (ps2x.Button(PSB_SELECT) && currentTime - previousTime > INTERVAL) {
        AutoOrManual = MANUAL;
        previousTime = currentTime;
      }
      break;
    case MANUAL:
      ManualMode();
      if (ps2x.Button(PSB_SELECT) && currentTime - previousTime > INTERVAL) {
        AutoOrManual = AUTO;
        AutoState = START_IN_TUNNEL;
        previousTime = currentTime;
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
void ManualMode() {
  //Speed settings

  if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_R2)) {
    topSpeed = highSpeed;
  } else if (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)) {
    topSpeed = lowSpeed;
  } else {
    topSpeed = midSpeed;
  }
  leftStickValue = ps2x.Analog(PSS_LY);
  rightStickValue = ps2x.Analog(PSS_RY);
  leftSpeed = -(leftStickValue - 127) * topSpeed / 128;
  rightSpeed = -(rightStickValue - 127) * topSpeed / 128;
  Serial1.print("leftSpeed: ");  //Debugging help
  Serial1.print(leftSpeed);
  Serial1.print(" | rightSpeed: ");
  Serial1.print(rightSpeed);
  Serial1.print(" | leftStick: ");
  Serial1.print(leftStickValue);
  Serial1.print(" | rightStick: ");
  Serial1.println(rightStickValue);
  moveRL(leftSpeed, rightSpeed);  //Movement function

  //Gripper functionality
  if (currentTime - previousTime >= INTERVAL) {
    if (ps2x.Button(PSB_CIRCLE)) {
      Serial1.println("CIRCLE button pushed");
      gripperPos = useGripper(gripperPos, myServo);
      previousTime = currentTime;
      //This timer ensures the gripper only detects one button push at a time
    }
  }

  //Display light sensor output for debugging
  if (ps2x.Button(PSB_SQUARE)) {
    Serial1.print("Light: ");
    Serial1.print(analogRead(lightSensor));
  }

  //IR transmitter
  if (ps2x.Button(PSB_PAD_UP)) {
    while (ps2x.Button(PSB_PAD_UP)) {
      //Set IRmsg to light gold votive
      IRmsg.protocol = NEC;
      IRmsg.address = 0xEE;
      IRmsg.command = 0xA0;
      IRmsg.isRepeat = false;
      digitalWrite(LED_BUILTIN, HIGH);
      irTX.write(&IRmsg);

      digitalWrite(IR_GOLD_PIN, HIGH);
      delay(500);
      digitalWrite(IR_GOLD_PIN, LOW);

      Serial1.print(" | Transmitting:");

      Serial1.print("Address= 0x");
      Serial1.print(IRmsg.address, HEX);
      Serial1.print(" Command=0x");
      Serial1.print(IRmsg.command,HEX);
      Serial1.print(" repeat ");
      Serial1.print(IRmsg.isRepeat);
      Serial1.println();

      ps2x.read_gamepad();
    }
  }
  if (ps2x.Button(PSB_PAD_DOWN)) {
    //recieve IR signal on address and retransmit it
    Serial1.println("Recieving and retransmitting...");
    if (irRX.decodeIR(&IRresults)) {
      IRresults.command;
      delay(100);
    }
    IRmsg.protocol = NEC;
    IRmsg.address = 0xCE;
    IRmsg.command = IRresults.command;
    IRmsg.isRepeat = false;
    Serial.print(IRmsg.address);
    Serial.print("//");
    Serial.print(IRmsg.command);
    Serial1.print(IRmsg.address);
    Serial1.print("//");
    Serial1.print(IRmsg.command);
    irTX.write(&IRmsg);
    delay(500);
  } else {
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
void RemoteControlIR() {
  irRX.decodeIR(&IRresults);
  currentTime = millis();
  int command = IRresults.command;
  Serial1.print(command);
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
void AutonomousMode() {
  Serial1.print("In Auto Mode. Distance: ");
  distIN = getAverageDistanceIR(SensorPos, 10);
  Serial1.print(distIN);
  const int LIGHT_TOLERANCE = 50;
  const int RIGHT_SONAR_FAR = 70;
  int lightValue;
  switch (AutoState) {
    case START_IN_TUNNEL:
      Serial1.println(" | In start tunnel state.");
      if (rightSonarCM() < RIGHT_SONAR_FAR) centerRobotSonarForward();
      else moveRL(30, 30);
      if (distIN < 5) {
        AutoState = TURN_IN_TUNNEL;
      }
      break;
    case TURN_IN_TUNNEL:
      Serial1.println(" | In turning tunnel state.");
      moveRL(15, -15);
      if (distIN > 20) {
        AutoState = EXIT_TUNNEL;
      }
      break;
    case EXIT_TUNNEL:
      Serial1.println(" | In exit tunnel state.");
      if (rightSonarCM() < RIGHT_SONAR_FAR) centerRobotSonarForward();
      else moveRL(30, 30);
      lightValue = getAverageLight(lightSensor, 50);
      Serial1.println(lightValue);
      if (lightValue > (darkLight + LIGHT_TOLERANCE)) {
        AutoState = LEAVE_TUNNEL;
        previousTime = currentTime;
      }
      break;
    case LINE_FOLLOW:
      Serial1.println(" | In line follow state.");
      moveForwardOnLine();
      Serial1.print(" Pos: ");
      Serial1.println(getLinePosition());
      if (distIN < 10) {
        AutoState = DROP_PAYLOAD;
      }
      break;
    case DROP_PAYLOAD:
      Serial1.println(" | In drop payload state.");
      stop();
      flip(180);
      useGripper(80, myServo);
      AutoState = IDLE;
      break;
    case LEAVE_TUNNEL:
      centerRobotSonarForward();
      if (currentTime - previousTime > INTERVAL) {
        AutoState = LINE_FOLLOW;
      }
      break;
    case IDLE:
      stop();
      break;
  }
}

/**
 * Get the average distance measured by the IR sensor
 * @param int sensorPosition - the position of the IR sensor
*/
double getAverageDistanceIR(int sensorPosition, double trials) {
  double sum = 0;
  double data;
  for (int i = 0; i < trials; i++) {
    data = readSharpDistIN(sensorPosition);
    if (data == -1) data = 31.0;
    sum += data;
  }
  return sum / trials;
}