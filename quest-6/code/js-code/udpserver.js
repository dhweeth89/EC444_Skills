/* EC444 Quest06
*  JS UDP Server to communicate with ESP32 via LAN, accessable via WAN
*  December 3, 2020
*  Author: Tony Faller*/

/* Requires npm modules express, csv-parser */

/////////////////////////////////////////////////////////////////////////////////////
//  Important Variables  ////////////////////////////////////////////////////////////

/* Remote servo enable bit */
var servoToggle = 0;

/* Alert clear bit */
var alertClear = 0;

/* Required modules */
const express = require('express');
const csv = require('csv-parser');
var dgram = require('dgram');
var path = require('path');
var fs = require('fs');

/* Server port and IP */
var PORT = 64209;
var HOST = '192.168.86.35';

/* HTML port for external access(same IP as server) */
var HTMLport = 64210;

/* Express pipeline */
const app = express();


/////////////////////////////////////////////////////////////////////////////////////
//  Create .csv files  /////////////////////////////////////////////////////////////

/* This code taken from https://www.w3schools.com/nodejs/nodejs_filesystem.asp */

/* CSV for object presence logs */
fs.writeFile('presence.csv', 'DATA_TYPE,IR,US,TIMESTAMP\n', function(err){
  if(err) throw err;
  console.log('>> Presence file created')
})

/* CSV for temperature logs */
fs.writeFile('temperature.csv', 'DATA_TYPE,THERMISTOR,PROBE\n', function(err){     // NEED TO FIX HEADERS
  if(err) throw err;
  console.log('>> Temperature file created')
})

/* CSV for accelerometer logs */
fs.writeFile('accelerometer.csv', 'DATA_TYPE,XP,YP,ZP,ROLL,PITCH,FAN_STATE,ALERT\n', function(err){     // NEED TO FIX HEADERS
  if(err) throw err;
  console.log('>> Accelerometer file created')
})


/////////////////////////////////////////////////////////////////////////////////////
//  Create UDP server  //////////////////////////////////////////////////////////////

/* This code taken from http://whizzer.bu.edu/briefs/design-patterns/dp-sockets */ 

/* Create socket */
var server = dgram.createSocket('udp4');

/* Create UDP server */
server.on('listening', function () {
    var address = server.address();
    console.log('>> UDP Server listening on ' + address.address + ":" + address.port);
});

/* Receive message */
server.on('message', function (message, remote) {
    console.log(">> " + remote.address + ':' + remote.port +' - ' + message);

    // Send ok if no toggle and no clear
    if(servoToggle == 0 && alertClear == 0){
      server.send("Ok!",remote.port,remote.address,function(error){
        if(error){
          console.log(' - MEH!');
        }
        else{
          console.log(' - Sent: Ok!');
        }
      });
    }

    // Send toggle
    else if(message.includes("Acceleration") && servoToggle && !alertClear){
      server.send("Toggle!",remote.port,remote.address,function(error){						// NOTE: esp-code side of things can be found in quest-3's box.c, line 659
        if(error){
          console.log(' - MEH!');
        }
        else{
          console.log(' - Sent: Toggle!');
        }
      });
      servoToggle = 0;	// Reset servoToggle
    }

    else if(message.includes("Acceleration") && !servoToggle && alertClear){
      server.send("alertClear!",remote.port,remote.address,function(error){           // NOTE: esp-code side of things can be found in quest-3's box.c, line 659
        if(error){
          console.log(' - MEH!');
        }
        else{
          console.log(' - Sent: alertClear!');
        }
      });
      alertClear = 0;  // Reset servoToggle
    }

    // Get current date/time
    var d = new Date();
    var dString = d.toString();
    // console.log(dString);

    // Append data to appropriate CSV file                                                        ACTUALLY NEED TO AVERAGE THIS SOMEWHERE
    if(message.includes("Presence")){
      fs.appendFile('presence.csv', message + "," + dString + "\n", function(err){
        if(err) throw err;
        console.log(' - Data appended to presence.csv')
      });
    }
    else if(message.includes("Temperature")){
      fs.appendFile('temperature.csv', message + "\n", function(err){
        if(err) throw err;
        console.log(' - Data appended to temperature.csv')
      });
    }
    else if(message.includes("Acceleration")){
      fs.appendFile('accelerometer.csv', message + "\n", function(err){
        if(err) throw err;
        console.log(' - Data appended to accelerometer.csv')
      });
    }
    else{
      console.log(' - Garbage data');
    }

});

/* Bind server */
server.bind(PORT, HOST);


/////////////////////////////////////////////////////////////////////////////////////
//  Read from .csv file, pipe data to HTML  /////////////////////////////////////////

/* This code modified from https://expressjs.com/en/starter/hello-world.html */

/* Send HTML file to browser */
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname + '/factory2.html'));
})

/* Specifies $.get("/presence") in HTML file */
app.get('/presence', (req, res) => {

  var data = [];  // Array to hold CSV data

  fs.createReadStream('presence.csv')   // create readStream from fs(?)
  .pipe(csv())            // Pipe data to csv object (inside server object)

  .on('data', (row) => {
    data.push(row);   // Appends each row to data array
  })

  .on('end', () => {
    res.send(data);   // Send CSV data to HTML page when done
  });
})

/* Specifies $.get("/temperature") in HTML file */
app.get('/temperature', (req, res) => {

  var data = [];  // Array to hold CSV data

  fs.createReadStream('temperature.csv')   // create readStream from fs(?)
  .pipe(csv())            // Pipe data to csv object (inside server object)

  .on('data', (row) => {
    data.push(row);   // Appends each row to data array
  })

  .on('end', () => {
    res.send(data);   // Send CSV data to HTML page when done
  });
})

/* Specifies $.get("/accelerometer") in HTML file */
app.get('/accelerometer', (req, res) => {

  var data = [];  // Array to hold CSV data

  fs.createReadStream('accelerometer.csv')   // create readStream from fs(?)
  .pipe(csv())            // Pipe data to csv object (inside server object)

  .on('data', (row) => {
    data.push(row);   // Appends each row to data array
  })

  .on('end', () => {
    res.send(data);   // Send CSV data to HTML page when done
  });
})



/////////////////////////////////////////////////////////////////////////////////////
//  Read button from HTML, send to ESP32  ///////////////////////////////////////////

/* https://www.quora.com/What-is-the-simplest-way-to-make-an-HTML-button-communicate-with-a-Node-js-server */

/* Process toggle requests from HTML */
server.on('message', function (message, remote) {
    app.get("/toggleURL", function(req, res) { 
        // Log data to console
        console.log(req.query);  
     
        // Respond to the client 
        // res.send("Fan toggled"); 
        res.sendFile(path.join(__dirname + '/factory2.html'));
  
        // Toggle servoToggle bit
        servoToggle ^= 1;

        console.log("servoToggle = " + servoToggle);
    }); 
  });

/* Process alert clears from HTML */
server.on('message', function (message, remote) {
    app.get("/clearURL", function(req, res) { 
        // Log data to console
        console.log(req.query);  

        // Toggle servoToggle bit
        alertClear ^= 1;

        // // Wait to allow data to propogate
        // setTimeout(function(){console.log("Let's do the fork in the garbage disposal");}, 60000);
        // var seconds = 2;
        // var waitTill = new Date(new Date().getTime() + seconds * 1000);
        // while(waitTill > new Date()){}

        // Respond to the client 
        // res.send("Alert cleared");
        res.sendFile(path.join(__dirname + '/clear.html')); 
  
        

        console.log("alertClear = " + alertClear);
    }); 
  });


/////////////////////////////////////////////////////////////////////////////////////
// Server listens at HTMLport  //////////////////////////////////////////////////////

/* Bind HTML Server */
app.listen(HTMLport, () => {
    console.log(`>> App listening at http://localhost:${HTMLport}`)
  })
