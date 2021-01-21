# ESP_1 Code Readme

## How to Use

1. Build your hardware in accordance with the schematic in `/images` (see Notes below)
2. Configure Network Credentials and Server IP/Port with `idf.py menuconfig`
3. Flash project
4. Aim sensors at object

## Notes
This code uses one infrared sensor and one ultrasonic sensor to determine whether or not an object is in the correct position. The team used the chassis of the purple car to mount the sensors (see system picture in `/images`). An acceptable position is detected by both sensors at a distance between 35cm and 45cm. 