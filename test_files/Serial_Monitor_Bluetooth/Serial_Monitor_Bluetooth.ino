/*
  Serial_Monitor_Bluetooth.ino - Arduino test Sketch for monitoring the serial port of the board over bluetooth
  
  Description:
  This sketch provides a way to test the function of the serial monitor over bluetooth using the HC05 bluetooth module
  Instructions for setup are here: https://moodle.rose-hulman.edu/pluginfile.php/5077822/mod_resource/content/1/Lecture%206-1%20-%20%28Optional%29WirelessSerialMonitor.pdf
  The computer must be paired with the device for this program to work.

  Hardware Connections:
    - VCC:                 5V   (bus)
    - GND:                 GND  (bus)
    - RXD:                 P3.3 - 4 (TXD1)
    - TXD:                 P3.2 - 3 (RXD1)

  Created by: Rowan Sammon
  Created: 01/18/2024
  Last modified: 01/25/2024
  Version: 1.0
*/ 


void setup(){
  Serial.begin(57600);
  Serial1.begin(57600);
}

void loop(){
  Serial1.println("Bluetooth");
  Serial.println("USB");
  delay(100);
}