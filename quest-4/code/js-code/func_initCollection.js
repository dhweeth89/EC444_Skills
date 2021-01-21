// NodeJS Script to create the collection

var MongoClient = require('mongodb').MongoClient;
var url = "mongodb://localhost:27017/mydb";

MongoClient.connect(url, function(err, db) {
  if (err) throw err;
  var dbo = db.db("mydb");
  dbo.createCollection("election", function(err, res) {
    if (err) throw err;
    console.log("Collection created!");
    db.close();
  });
});