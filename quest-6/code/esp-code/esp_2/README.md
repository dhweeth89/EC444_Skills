# ESP_2 Code Readme

## How to Use

1. Build your hardware in accordance with the appropriate schematic in `/images`
2. Change variable `ESP3_IP_ADDR` to the local IP of ESP_3
3. Configure Network Credentials and Server IP/Port with `idf.py menuconfig`
4. Flash project

## Notes
This code uses a 10k thermistor and a 10k thermal probe to find the current temperature. A red LED is used to indicate a 'bad' temperature, whereas a green LED is used to indicate a 'good' temperature. A 'good' temperature is defined to be between 10 and 27 degrees celsius.


ESP_2 communicates a temperature boolean to ESP_3. This communication occurs on port 64208.
