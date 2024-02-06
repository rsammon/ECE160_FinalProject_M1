/* Testing code for measuring distance usint the SHARP 2Y0A21 IR distance sensor
*  Deborah Walter 10/13/2023
*
*  Calls the findDistance function - which reads the analog pin and converts the integer
*  value to a distance in cm, using the calibration curve from the datasheet.
*   Datasheet: https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf 
*
*/

#include <SimpleRSLK.h>

//#define UART_SERIAL Serial1    // for wireless serial monitor using HC05
//Using the HC-06 lets you see the measurements when not tethered

#define UART_SERIAL Serial  // for serial monitor using USB wire

uint16_t distValue; 
uint16_t distMM; 
float distIN;

    // According to the RSLK Library:     
    // readSharpDist(0) is the left distance sensor which is tied to P9.1
    // readSharpDist(1) is the center distance sensor which is tied to P6.1
    // readSharpDist(2) is the right distance sensor which is tied to P9.0

uint8_t SensorPos = 0; // sets the sensor pin used

void setup() {

  UART_SERIAL.begin(57600);
  Serial1.begin(57600);
  UART_SERIAL.println("starting serial monitor-USB");
  setupRSLK();
}

void loop() {
  //reads the distance as 14bit integer
  distValue = readSharpDist(SensorPos);

  //reads the distance in millimeters
  distMM = readSharpDistMM(SensorPos);

  //reads the distance in inches
  distIN = readSharpDistIN(SensorPos);

  UART_SERIAL.print("Distance measured: ");
  UART_SERIAL.print(distValue, DEC);  UART_SERIAL.print(" raw value | ");
  UART_SERIAL.print(distMM, DEC);  UART_SERIAL.print(" mm | ");
  UART_SERIAL.print(distIN, DEC);  UART_SERIAL.print(" inches");
  UART_SERIAL.println();

  Serial1.print("Distance measured: ");
  Serial1.print(distValue, DEC);  Serial1.print(" raw value | ");
  Serial1.print(distMM, DEC);  Serial1.print(" mm | ");
  Serial1.print(distIN, DEC);  Serial1.print(" inches");
  Serial1.println();
  delay(1000);
}
