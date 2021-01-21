/* EC444 Quest02
*  JS Server to read serial data from ESP32 and save to .csv file
*  October 5, 2020
*  Author: Tony Faller  */

/* Note: requires npm modules serialport, express, csv-parser */

/////////////////////////////////////////////////////////////////////////////////////
//	Create .csv file	/////////////////////////////////////////////////////////////

/* This code taken from https://www.w3schools.com/nodejs/nodejs_filesystem.asp */
var fs = require('fs');

fs.writeFile('data.csv', 'Index,US,Therm,IR\n', function(err){
	if(err) throw err;
	console.log('File created')
})

/////////////////////////////////////////////////////////////////////////////////////
//	Read from ESP32, append to .csv file	/////////////////////////////////////////

/* This code taken from https://serialport.io/docs/api-serialport */
const SerialPort = require('serialport')
const Readline = SerialPort.parsers.Readline
const port = new SerialPort('COM3', {baudRate:115200})

const parser = port.pipe(new Readline())
parser.on('data', function(data){
	fs.appendFile('data.csv', data + "\n", function(err){
		if(err) throw err;
		console.log(data)
	})
})

/////////////////////////////////////////////////////////////////////////////////////
//	Read from .csv file, pipe data to HTML	/////////////////////////////////////////

/* This code modified from https://expressjs.com/en/starter/hello-world.html */
const csv = require('csv-parser');
const express = require('express');
const app = express();
var path = require('path');
const HTMLport = 8080;

/* Sends HTML file from server to browser */
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname + '/tactile_internet.html'));
})

/* Specifies $.get("/data") in HTML file */
app.get('/data', (req, res) => {

	var data = [];	// Array to hold CSV data

	fs.createReadStream('data.csv')		// create readStream from fs(?)
	.pipe(csv())						// Pipe data to csv object (inside server object)

	.on('data', (row) => {
		data.push(row);		// Appends each row to data array
	})

	.on('end', () => {
		res.send(data);		// Send CSV data to HTML page when done
	});
})

app.listen(HTMLport, () => {
  console.log(`App listening at http://localhost:${HTMLport}`)
})