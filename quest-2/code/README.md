# Code Readme

## Contents
esp32-code contains the items to be flashed to the esp32. tactile.c combines previous Ultrasonic, Thermistor, and IR code into one program that samples the sensors and prints the converted results to console.

js-code contains the code for a js server which creates/overwrites a file called data.csv, reads serial data from the esp32, and writes this data to data.csv.

## How to Use
1. Connect the Ultrasonic ADC probe to GPIO34(A2), the Thermistor ADC probe to GPIO39(A3), and the IR ADC probe to GPIO36(A4). In the thermistor/voltage divider circuit, use a 10k resistor in series.
2. After pulling the git library, please do "idf.py clean" and then do "idf.py fullclean". A merge was made that caused someone's cmakefile settings to be pushed onto github, causing errors, and we can't figure out how to delete it. The clean followed by fullclean seems to bypass this issue, however.
3. Flash tactile.c to the esp32 (NO monitor) 
4. cd to js-code
5. Run the command "npm install serialport express csv-parser"
6. Run the command "node fileIO.js"
7. Open a browser and go to https://localhost:8080

## Notes
We've figured out a way to dynamically update the graph, and we are aware that it is very inefficient both space-wise and time-wise. We use a jquery setTimeout() command to recursively loop the chart creation, effectively re-rendering the graph every 500ms. This interval allows for pseudo-synchronicity between when data is fetched and when the graph is visually updated. We expressed our concerns about its inefficiency, but ultimately, the code works.

In addition to being in report.md, attributions are commented in the code.
