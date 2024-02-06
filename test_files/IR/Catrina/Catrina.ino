#include <TinyIRremote.h>
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define IR_RCV_PIN 33
#define IR_TRX_PIN 37
IRsender sendIR(IR_TRX_PIN);
IRreceiver irRX(IR_RCV_PIN);
IRData IRresults;
IRData IRmsg;
uint16_t IRcommand;

void setup() {
  Serial.begin(57600);
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
  IRmsg.isRepeat = false;
}

void loop() {
  if (irRX.decodeIR(&IRresults)) {
    Serial.print("A=0x");
    Serial.print(IRresults.address, HEX);
    Serial.print(" C=0x");
    Serial.print(IRresults.command, HEX);
    // IRresults.command = IRcommand;
    IRcommand = IRresults.command;
  } else {
    Serial.println("No Data Recieved");
  }
  IRmsg.address = 0xCE;
  IRmsg.command = 0xA0;
  sendIR.write(&IRmsg);
  Serial.println("Transmitted Address:  ");
  Serial.println(IRmsg.address, HEX);
  Serial.println("Transmitted Command:  ");
  Serial.println(IRmsg.command, HEX);

  delay(500);
}
