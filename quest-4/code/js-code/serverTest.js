/* EC444 Quest4
*  JS Server to communicate with ESP32 via LAN and save vote data to database
*  Novermber 10, 2020
*  Author: Tony Faller, Roger Ramesh*/

// Requires npm module mongodb and json2csv

/////////////////////////////////////////////////////////////////////////////////////
//  Create .csv file  /////////////////////////////////////////////////////////////

/* This code taken from https://www.w3schools.com/nodejs/nodejs_filesystem.asp */

var fs = require('fs');

fs.writeFile('data.csv', 'File initialization\n', function(err){
  if(err) throw err;
  console.log('>> File initialized')
})


/////////////////////////////////////////////////////////////////////////////////////
//  Create UDP server  //////////////////////////////////////////////////////////////

/* This code taken from http://whizzer.bu.edu/briefs/design-patterns/dp-sockets */ 

/* Required modules */
var dgram = require('dgram');
const Json2csvParser = require('json2csv').Parser;

/* Port and IP */
var PORT = 64209;
var HOST = '192.168.86.35';
var MongoClient = require('mongodb').MongoClient;
var url = "mongodb://localhost:27017/mydb";

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

    // Parse input message
    var s = message.toString('utf8')
    var messageArr = s.split(',');
    var id = messageArr[0]
    var color = messageArr[1]
    
    // Convert input message to chars
    var res1 = hex_to_ascii(id)
    var res2 = hex_to_ascii(color)
    // console.log("ID:" + res1 + " Color: " + res2); // For debugging

    // Connect to database
    MongoClient.connect(url, function(err, db) {
      if (err) throw err;
      console.log(">> Database connection established...");

      // Append message to database
      var dbo = db.db("mydb");
      var myobj = { ID: res1, COLOR: res2 };
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
          console.log('>> Sent to Leader: Ok!');
        }
      });
    
});


/* Bind server to port and IP */
server.bind(PORT, HOST);