# ECE160_FinalProject_M1
Code of Team Theta/Octopi for the ECE160 Final Group Project. Main code is found [here](MultifileTemplate/MultifileTemplate.ino).

## Circuit wiring for code:
pin wiring of the internal circuitry: [here](https://moodle.rose-hulman.edu/pluginfile.php/5035015/mod_resource/content/2/msp432p4xx_1pg_pin_map.pdf)
* **PS2 controller wireless receiver**
```
    - power (red):         3.3V (bus)
    - ground (black):      GND  (bus)
    - data (brown):        P1.7 - 14
    - command (orange):    P1.6 - 15
    - Enable (yellow):     P2.3 - 34
    - clock (blue):        P6.7 - 35
```
* **IR Receiver**
```
    - G:                   GND  (bus)
    - R:                   3.3V (bus)
    - Y:                   P5.1 - 33
```
* **IR Transmitter**
```
    - GND:                 220Ω to GND (bus)
    - Signal In:           P6.6 - 36
``` 
* **Bluetooth Serial Monitor (HC05)**
```
    - VCC:                 5V   (bus)
    - GND:                 GND  (bus)
    - RXD:                 P3.3 - 4 (TXD1)
    - TXD:                 P3.2 - 3 (RXD1)
```
* **Diagnostic LEDs**
```
    - Red (bit 0):         P6.4 - 10
    - Green (bit 1):       P1.5 - 7        
    - Blue (bit 2):        P4.4 - 26
    - Yellow (bit 3):      P4.1 - 5
    - GND (for all):       220Ω to GND (bus)
```
* **Setup Button**
```
    - Signal Out:          P6.5 - 9
    - GND:                 GND (bus)
```
* **Light Sensor (Photo Resistor)**
```
    - Pin1:                3.3V (bus)
    - Pin2:                Resistor & P9.1 - A16
    - Resistor (220Ω):     GND (bus)
```
* **Left Sonar**
```
    - VCC:                 5V (bus)
    - Trig:                P9.3 - 62
    - GND:                 GND (bus)
```
* **Right Sonar**
```
    - VCC:                 5V (bus)
    - Trig:                P6.3 - 63
    - GND:                 GND (bus)
```
* **Middle Sonar**
```
    - VCC:                 5V (bus)
    - Trig:                P10.3
    - Echo:		   P10.1
    - GND:                 GND (bus)
```