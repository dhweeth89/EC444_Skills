# js-code Readme

This folder contains the code for the server, database, and web page.

## How to use
If you are the server host (Tony), first initialize the database:
1. Install the required modules by using `npm install mongodb json2csv csv-parser express`
2. Run `node func_drop.js` to clear any previous database data
3. Run `node func_initDB.js` to initialize the database
4. Run `node func_initCollection.js` to initalize the collection within the database
5. Start the server with `node serverElection.js`
6. Go to http://ec444amfaller.zapto.org:64211/ to see vote results

Else, you will need to modify the variable `SERVER_IP_ADDR` in the `esp-code` files. You can then access the web page through http://localhost:64211/ 

## Notes
The server now supports an initally empty database. Upon start, the server connects to the database and adds a document with `ID=0` and `COLOR=I` to mark each initialization. A timestamp is also appended. This does not affect the bar chart visualization on the HTML page.

The server also now supports a reset request from the HTML page. The database collection is deleted then recreated, with a document `ID=0` and `COLOR=O` to mark the reset. The reset button redirects to another page and you have to go back to see the graph again, which isn't ideal.
