# Code Readme

## How to Use

1. Build your hardware in accordance with the schematic in `/images` (see Notes below)
2. Disconnect positive rail
3. Connect esp to computer
4. Configure Network Credentials and Server IP/Port with `idf.py menuconfig`
5. Flash project
6. Disconnect esp from computer
7. Power with external source (most likely a portable phone charger)

## Notes
Note that the schematic in `/images` is not fully detailed. Certain pins, such as sensor Vcc/GND connections, are not shown. In addition, the i2c alphanumeric display is not shown, since there is only one way to wire it. These are important to the function of the device, so don't forget them. MAKE SURE THE IR SENSOR USES 3.3V VCC

* November 25, 2020: There is LED indication of go vs stop states. A lit green LED means the path ahead is clear, so the car moves forward. A red LED means that an obstacle is <20cm away from the front of the car, so it stops. If the obstacle is removed, the car can continue forward.

* November 30, 2020: The web page now controls an active-low enable bit on the esp32 called `masterStop`. Additionally, the IR sensor is now sampled every second, and the distance reading in cm is stored in global variable `dist_ir`.

* December 1, 2020: The main code can be found in `control2.c`. Originally, `control.c` was used for development, but it was found that `control2.c` works much better.
