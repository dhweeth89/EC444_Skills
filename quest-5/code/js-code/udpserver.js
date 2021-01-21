/* EC444 Quest05
*  JS UDP Server to communicate with ESP32 via LAN 
*  November 30, 2020
*  Author: Tony Faller*/

/* Requires npm module express */

/////////////////////////////////////////////////////////////////////////////////////
//  Important Variables  ////////////////////////////////////////////////////////////

/* Master stop supercedes all other stop flags */
var masterStop = 0;

/* Required modules */
var dgram = require('dgram');
const express = require('express');
var path = require('path');

/* Server port and IP */
var PORT = 64209;
var HOST = '192.168.86.35';

/* HTML port (same IP as server) */
var HTMLport = 64210;

/* Express pipeline */
const app = express();

/////////////////////////////////////////////////////////////////////////////////////
//  Create UDP server  //////////////////////////////////////////////////////////////

/* Create socket */
var server = dgram.createSocket('udp4');

/* Create UDP server */
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

/* Receive message */
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);

    // Send ok if no stop
    if(masterStop == 0){
      server.send("Ok!",remote.port,remote.address,function(error){
        if(error){
          console.log('MEH!');
        }
        else{
          console.log('Sent: Ok!');
        }
      });
    }

    // Else send stop
    else{
      server.send("Toggle!",remote.port,remote.address,function(error){						// NOTE: esp-code side of things can be found in quest-3's box.c, line 659
        if(error){
          console.log('MEH!');
        }
        else{
          console.log('Sent: Ok!');
        }
      });
      masterStop = 0;	// Reset masterStop
    }

    /*	Unsure if we want to use masterStop as a toggle or as a flag.
    *	If we use it as a toggle, our button would be more like a "Start/Stop", 
    *	but a flag would require a masterStart variable and separate "Start" and
    *	"Stop" buttons. Personally, I (Tony) think it would be easier as a toggle.
    *
    *	I guess it's more an enable than a toggle... Maybe both.
	*/
});

/* Bind server */
server.bind(PORT, HOST);

/* Send HTML file to browser */
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname + '/command.html'));
})

/* Process requests from HTML */
server.on('message', function (message, remote) {
    app.get("/my-url", function(req, res) { 
        // Log data
        console.log(req.query);  
     
        // Respond to the client 
        res.send("Motor toggled"); 
  
        // Toggle masterStop bit
        masterStop ^= 1;

        console.log("masterStop = " + masterStop);
    }); 
  });

/* Bind HTML Server */
app.listen(HTMLport, () => {
    console.log(`App listening at http://localhost:${HTMLport}`)
  })
