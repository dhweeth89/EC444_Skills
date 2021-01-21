# JS-CODE README

## How to Use
1. Make sure `express` is installed: run `npm install express`
2. Edit the `HOST` variable to match whichever machine is running the server
3. Start the server with `node udpserver.js`
4. Go to http://localhost:64210/

## Notes

### November 30, 2020
The web page is very bare right now, with only one button labelled "Start/Stop".

The button acts as a sort of toggle or enable for the motor control. It affects an active-low variable `masterStop` within the server (and esp32) that stops the motors if high and allows them to run if low.

