# ESP_3 Code Readme

## How to Use

1. Build your hardware in accordance with the appropriate schematic in `/images`
2. Configure Network Credentials and Server IP/Port with `idf.py menuconfig`
3. Flash project

## Notes
This code displays a countdown timer on the alphanumeric display. Every minute, the servo ("fan") is triggered, and it sweeps through its motions. This servo is also remote controllable from a NodeJS server. An accelerometer also logs its data to this server, which is subsequently visualized on an HTML page. If the board is tipped beyond an angle of 30 degrees, an alert is raised on the HTML page. This alert must be cleared before visualizations are able to continue.


ESP_3 receives a temperature boolean from ESP_2 via LAN Wifi. This changes whether or not the fan is on.
