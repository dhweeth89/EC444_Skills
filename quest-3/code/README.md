# Code Readme

## Contents

esp-code contains the code for basic temperature/voltage sensor readings on the esp32 as well as a UDP client that periodically sends data to a server.

js-code contains the code for a UDP server that receives a text-based payload from the ESP32. 

## How to Use
1. Connect thermistor probe to GPIO34(A2), battery probe to GPIO39(A3), LED V_in to GPIO 32, use two 10k resistors for battery voltage divider circuit and appropriate resistor for LED
2. Build the project
3. Run ``idf.py menuconfig`` and configure server IP, port, network SSID, and network password
4. Go into ``udpServer.js`` and update the ``PORT`` and ``HOST`` variables to match what you configured
5. Run the command ``npm install express csv-parser``
6. Start the server with ``node udpServer.js``
7. Start the client by flashing to board
8. Open a browser and go to http://localhost:64211/

## Notes
This code has been specifically tailored for remote access with the server operating under Tony Faller's home Wifi network. However, one can still run both client and server and access the web pages locally, with the URL specified in Step 8 above.

The front-end code has been modified from Quest 2 and now only shows a fixed window of 60s for the graphs.
