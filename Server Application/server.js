var path = require('path');
var mime = require('mime');
var express = require('express');
var util = require('util');
var app = express();
var fs = require('fs');

app.use(express.static('public'));
app.use('/static', express.static('public'))

app.get('/', function (req, res) {
  var files = fs.readdirSync('./public');
  var lmfiles = {};

  console.log('Chegou alguem');

  for (i=0; i<files.length; i++) {
    var stats = fs.statSync('./public/'+files[i]);
    var mtime = new Date(util.inspect(stats.mtime));
    lmfiles[files[i]] = mtime;
  }
  res.send(lmfiles);
})

app.get('/test', function(req, res) {
  res.sendFile('./static/test.txt');
})

app.get('/file/:name', function (req, res, next) {

  var stats = fs.statSync('./public/'+req.params.name);
  var mtime = new Date(util.inspect(stats.mtime));
  console.log(mtime);

  var options = {
    root: __dirname + '/public/',
  };

  var fileName = req.params.name;
  res.sendFile(fileName, options, function (err) {
    if (err) {
      next(err);
    } else {
      console.log('Sent:', fileName);
    }
  });

});

var server = app.listen(8081, function () {
   var host = server.address().address
   var port = server.address().port

   console.log("Example app listening at http://%s:%s", host, port)
})
