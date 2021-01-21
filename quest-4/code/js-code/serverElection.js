/* EC444 Quest4
*  JS Server to communicate with ESP32 via LAN and save vote data to database
*  Novermber 10, 2020
*  Author: Tony Faller, Roger Ramesh*/

/* Requires npm module mongodb, json2csv, csv-parser, express */

/* Required modules for UDP Server*/
var dgram = require('dgram');
const Json2csvParser = require('json2csv').Parser;

/* Required modules to pipe CSV file data to HTML */
const csv = require('csv-parser');
const express = require('express');

/* Database stuff */
var MongoClient = require('mongodb').MongoClient;
var url = "mongodb://localhost:27017/mydb";

/* Receiver Server Port and IP */
var PORT = 64209;
var HOST = '192.168.86.35';

/* Localhost HTML port */
const HTMLport = 64211;

/////////////////////////////////////////////////////////////////////////////////////
//  Initialize .csv file  ///////////////////////////////////////////////////////////

/* This code taken from https://www.w3schools.com/nodejs/nodejs_filesystem.asp */
 
var fs = require('fs');

/* Create CSV file */
fs.writeFile('data.csv', 'File initialization\n', function(err){
  if(err) throw err;
  console.log('>> File initialized')
})

/* Read from database */
// Connect to database
    MongoClient.connect(url, function(err, db) {
      if (err) throw err;
      console.log(">> Initial database connection established...");

      // Get current time stamp
      var d = new Date();
      var dString = d.toString();

      // Append message to database
      var dbo = db.db("mydb");
      var myobj = { ID: 0, COLOR: "I", TIMESTAMP: dString };
      dbo.collection("election").insertOne(myobj, function(err, res) {
        if (err) throw err;    
      });
      console.log(" - Initialization document inserted");

      // Read database, write to CSV file
      dbo.collection("election").find({}).toArray(function(err, result) {
        if (err) throw err;

        // Read database
        const csvFields = ['ID', 'Color'];
        const json2csvParser = new Json2csvParser({ csvFields });
        const csv = json2csvParser.parse(result);

        // console.log(csv); // For debugging
        // console.log("CSV Variable type: " + typeof csv); // For debugging
        // console.log(csv.replace(/['"]+/g, '')); // For debugging
        
        // Remove quotes from csv constant and write to file
        fs.writeFile('data.csv', csv.replace(/['"]+/g, ''), function(err){
          if(err) throw err;
          console.log(' - Initialization data appended to file')
        })
      
      });

      // Close database
      db.close();
      console.log(" - Initial database connection closed");
    });

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


/* Utility function to convert hex string to ASCII char */
function hex_to_ascii(str1)
{
 var hex  = str1.toString();
 var str = '';

 // Support for case receiverFob == leaderFob
 var listID = ["1", "2", "3", "4", "5", "6", "7", "8", "9"];
 if(listID.includes(str1,0)) {return str1;}

 // Support for case receiverFob != leaderFob
 for (var n = 0; n < hex.length; n += 2) {
   str += String.fromCharCode(parseInt(hex.substr(n, 2), 16));
 }
 return str;
} 


/* On connection, do stuff */
server.on('message', function (message, remote) {
    // console.log(remote.address + ':' + remote.port +' - ' + message);  // For debugging

    console.log(">> Message received from leader fob...")

    // Parse input message
    var s = message.toString('utf8')
    var messageArr = s.split(',');
    var id = messageArr[0]
    var color = messageArr[1]
    
    // Convert input message to chars
    var res1 = hex_to_ascii(id)
    var res2 = hex_to_ascii(color)
    // console.log("ID:" + res1 + " Color: " + res2); // For debugging

    // Get current time stamp
    var d = new Date();
    var dString = d.toString();

    // Connect to database
    MongoClient.connect(url, function(err, db) {
      if (err) throw err;
      console.log(">> Database connection established...");

      // Append message to database
      var dbo = db.db("mydb");
      var myobj = { ID: res1, COLOR: res2, TIMESTAMP: dString };
      dbo.collection("election").insertOne(myobj, function(err, res) {
        if (err) throw err;    
      });
      console.log(" - 1 document inserted");

      // Read database, write to CSV file
      dbo.collection("election").find({}).toArray(function(err, result) {
        if (err) throw err;

        // Read database
        const csvFields = ['ID', 'Color'];
        const json2csvParser = new Json2csvParser({ csvFields });
        const csv = json2csvParser.parse(result);

        // console.log(csv); // For debugging
        // console.log("CSV Variable type: " + typeof csv); // For debugging
        // console.log(csv.replace(/['"]+/g, '')); // For debugging
        
        // Remove quotes from csv constant and write to file
        fs.writeFile('data.csv', csv.replace(/['"]+/g, ''), function(err){
          if(err) throw err;
          console.log(' - Data appended to file')
        })
      
      });

      // Close database
      db.close();
      console.log(" - Database connection closed");
    });

    // Send ACK back to leader fob
    server.send("Ok!",remote.port,remote.address,function(error){
      if(error){
        console.log('>> ERROR IN SENDING TO LEADER');
      }
      else{
        console.log(' - Sent to leader fob: Ok!');
      }
    });
    
});


/* Bind server to port and IP */
server.bind(PORT, HOST);


/////////////////////////////////////////////////////////////////////////////////////
//  Read from .csv file, pipe data to HTML  /////////////////////////////////////////

/* This code modified from https://expressjs.com/en/starter/hello-world.html */
const app = express();
var path = require('path');

/* Sends HTML file from server to browser */
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname + '/election.html'));
})

/* Specifies $.get("/data") in HTML file */
app.get('/data', (req, res) => {

  var data = [];  // Array to hold CSV data

  fs.createReadStream('data.csv')   // create readStream from fs(?)
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

app.get("/my-url", function(req, res) { 
    // Log data
    console.log(">> Reset request received...");  

    // Reset collection
    MongoClient.connect(url, function(err, db) {
      if (err) throw err;
      var dbo = db.db("mydb");

      // Delete collection
      dbo.collection("election").drop(function(err, delOK) {
        if (err) throw err;
        if (delOK) console.log(" - Collection deleted");
        // db.close();
      });

      // Create collection
      dbo.createCollection("election", function(err, res) {
        if (err) throw err;
        console.log(" - Collection created");
      });

      // Get current time stamp for reset message
      var d = new Date();
      var dString = d.toString();

      // Append reset message to database
      // var dbo = db.db("mydb");
      var myobj = { ID: 0, COLOR: "O", TIMESTAMP: dString };
      dbo.collection("election").insertOne(myobj, function(err, res) {
        if (err) throw err;    
      });
      console.log(" - Reset document inserted");

      // Manually write reset message to CSV file
      fs.writeFile('data.csv', "_id,ID,COLOR,TIMESTAMP" + "\n", function(err){
        if(err) throw err;
      })
      fs.appendFile('data.csv', "xxxx,0,O," + dString + "\n", function(err){
        if(err) throw err;
        console.log(' - Reset data appended to file')
      })

      // Read database, write to CSV file
      // dbo.collection("election").find({}).toArray(function(err, result) {
      //   if (err) throw err;

      //   // Read database
      //   const csvFields = ['ID', 'Color'];
      //   const json2csvParser = new Json2csvParser({ csvFields });
      //   const csv = json2csvParser.parse(result);

      //   // console.log(csv); // For debugging
      //   // console.log("CSV Variable type: " + typeof csv); // For debugging
      //   // console.log(csv.replace(/['"]+/g, '')); // For debugging
        
      //   // Remove quotes from csv constant and write to file
      //   fs.writeFile('data.csv', csv.replace(/['"]+/g, ''), function(err){
      //     if(err) throw err;
      //     console.log(' - Restart data written to file')
      //   })
      // });

      // Close database
      db.close(); 

      console.log(" - Database reset")
    });

    // // Create collection
    // MongoClient.connect(url, function(err, db) {
    //   if (err) throw err;
    //   var dbo = db.db("mydb");
    //   dbo.createCollection("election", function(err, res) {
    //     if (err) throw err;
    //     console.log("Collection created!");
    //     db.close();
    //   });
    // });
 
    // Respond to the client 
    res.send("Database reset"); 

    // // Toggle ON bit
    // ledON ^= 1;
});   
/////////////////////////////////////////////////////////////////////////////////////
// Server listens at HTMLport  //////////////////////////////////////////////////////
app.listen(HTMLport, () => {
  console.log(`App listening at http://localhost:${HTMLport}`)
})