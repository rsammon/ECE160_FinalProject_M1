#include <TinyIRremote.h>
#include "SimpleRSLK.h"
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define IR_RCV_PIN 33
#define IR_TRX_PIN 36
IRsender sendIR(IR_TRX_PIN);
IRreceiver irRX(IR_RCV_PIN);
IRData IRresults;
IRData IRmsg;

void setup() {
  Serial.begin(57600);
  setupRSLK();
  delay(500);
  Serial.println(F("START " __FILE__ " from " __DATE__));
  if (sendIR.initIRSender()) {
    Serial.println(F("Ready to transmit NEC IR signals on pin " STR(IR_TRX_PIN)));
  } else {
    Serial.println("Initialization of IR transmitter failed!");
    while (1) { ; }
  }
  if (irRX.initIRReceiver()) {
    Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_RCV_PIN)));
  } else {
    Serial.println("Initialization of IR receiver failed!");
    while (1) { ; }
  }
  delay(200);
  enableTXLEDFeedback(GREEN_LED);
  enableRXLEDFeedback(BLUE_LED);

  IRmsg.protocol = NEC;
  IRmsg.address = 0xEE;
  IRmsg.command = 0xA0;
  IRmsg.isRepeat = false;
}

void loop() {
  sendIR.write(&IRmsg);
  Serial.println('.');

  if (irRX.decodeIR(&IRresults)) {
    Serial.print("A=0x");
    Serial.print(IRresults.address, HEX);
    Serial.print(" C=0x");
    Serial.print(IRresults.command, HEX);
    Serial.print(" Command: ");
    Serial.print(IRresults.command);
    Serial.println();
  }
  else {
    Serial.println("No Data Recieved");
  }
  delay(500);
}
