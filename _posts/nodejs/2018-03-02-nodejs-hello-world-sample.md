---
layout: post
title: Hello. World Sample
category: OpenShift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift, Dockerfile]
---
# Hello Node.js

## package.json

<pre class="prettyprint">
{
  "name": "hello",
  "version": "1.0.0",
  "description": "",
  "main": "app.js",
  "scripts": {
  "test": "echo \"Error: no test specified\"exit 1",
  "start": "node app.js"
},
  "author": "",
  "license": "Apache",
  "dependencies": {
    "express": "^4.16.2"
  }
}
</pre>

<br>

## app.js

<pre class="prettyprint">
var port = 8080;

var express = require('express');
var app = express();

app.get('/', function (req, res) {
  res.send('This is an index page.');
});

app.get('/hello', function (req, res) {
  res.send('Hello World!');
});

app.listen(port, function () {
  console.log('Example app is listening on port ', port);
});
</pre>