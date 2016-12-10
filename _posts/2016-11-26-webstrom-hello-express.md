---
layout: post
title: WebStorm에서 Express 프로젝트 시작하기
category: Node.js
tag: [Node.js, express, webstorm]
---

Express는 하나의 Web Framework 입니다. 즉, Express 프로젝트는 단순히 파일 한 두개로 
이루어지지 않고, Framework 위에서 실행되기 위해 여러 라이브러리들과 설정 파일들로 이루어져 
있습니다. 

자세한 설명은 [여기](http://expressjs.com/)에서 보실 수 있습니다.
[번역판](http://expressjs.com/ko/)도 있으니 가벼운 마음으로 참조하세요.

[WebStorm](https://www.jetbrains.com/webstorm/?fromMenu)은 
[IntelliJ](https://www.jetbrains.com/idea/?fromMenu)로 유명한 
[JetBrains](https://www.jetbrains.com/)사의 제품이며 
상용 제품입니다. 그래서 굳이 WebStorm을 고집할 필요는 없지만(무난한 [Eclipse](https://eclipse.org/downloads/)도 있습니다.),
저는 라이센스가 있기 때문에 그냥 WebStorm을 사용하도록 하겠습니다.

<br>

먼저 WebStorm을 실행합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-26-webstrom-hello-express/1.png)

여기에서 'Create New Project'를 누르고,

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-26-webstrom-hello-express/2.png)

잘 보면, 왼쪽에 'Node.js Express App'이 있습니다.
선택하면, 프로젝트 경로 등을 설저할 수 있습니다. 그 외엔 특별히 설정할 값은 없습니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-26-webstrom-hello-express/3.png)

Express 프로젝트가 생성되었습니다.

package.json 파일과 app.json 파일을 열어서 구경하면 됩니다.

<br>

크게 프로젝트는 다음과 같이 구성됩니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-26-webstrom-hello-express/4.png)

<br>

## app.js

먼저 app.js 파일을 열어보면

<pre class="prettyprint" style="font-size:0.7em;">
var express = require('express');
var path = require('path');
var favicon = require('serve-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

var index = require('./routes/index');
var users = require('./routes/users');

var app = express();

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

...
</pre>

와 같은 코드들이 보이며, 여기서 index와 users는 프로젝트의 routes 폴더에 포함되어 있는 샘플 파일들입니다.

<br>

## /routes/

index.js 파일을 열어보면 

<pre class="prettyprint" style="font-size:0.7em;">
var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/', function(req, res, next) {
  res.render('index', { title: 'Express' });
});

module.exports = router;
</pre>
와 같은 내용이 있으며, 나중에 이 프로젝트를 실행했을 때 실행되는 코드라고 생각하면됩니다.

예를 들어 웹브라우저에서 'http://127.0.0.1:3000' 으로 접속했을 때
위 코드가 실행됩니다. 마찬가지로 'http://127.0.0.1:3000/users'로 접속한 경우는 '/routes/users.js' 파일의
코드가 실행됩니다.

<br>

## /bin/www

그리고 'bin' 폴더의 'www' 파일을 열어보면 

<pre class="prettyprint" style="font-size:0.7em;">
#!/usr/bin/env node

/**
 * Module dependencies.
 */

var app = require('../app');
var debug = require('debug')('hello-express:server');
var http = require('http');

/**
 * Get port from environment and store in Express.
 */

var port = normalizePort(process.env.PORT || '3000');
app.set('port', port);

/**
 * Create HTTP server.
 */

var server = http.createServer(app);

/**
 * Listen on provided port, on all network interfaces.
 */

server.listen(port);
server.on('error', onError);
server.on('listening', onListening);

...
</pre>

와 같은 코드가 있으며, 서버를 실행하는 내용이 담겨져 있습니다.

이와 같이 WebStorm에서 새로운 프로젝트를 생성하면, 
위 폴더 구조를 가지는 Express 코드를 자동으로 생성해줍니다.