# ECE160_FinalProject_M1
Code of Team Theta/Octopi for the ECE160 Final Group Project. Main code is found [here](MultifileTemplate/MultifileTemplate.ino).

## Circuit wiring for code:
pin wiring of the internal circuitry: [here](https://moodle.rose-hulman.edu/pluginfile.php/5035015/mod_resource/content/2/msp432p4xx_1pg_pin_map.pdf)
* **PS2 controller wireless reciever**
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
* **Bluetooth Serial Monitor (HC05)**
```
    - VCC                  5V   (bus)
    - GND:                 GND  (bus)
    - RXD:                 P3.3 - 4 (TXD1)
    - TXD:                 P3.2 - 3 (RXD1)
```
