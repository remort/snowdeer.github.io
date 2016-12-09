---
layout: post
title: Hello. Node.js
category: Node.js
tag: [Node.js]
---

앞서 설치한 Node.js로 간단한 프로그램을 만들어보도록 하겠습니다.

작업 폴더를 하나 만들고 파일명을 hello.js로 해서 빈 파일을 만듭니다.
그리고는 다음과 같은 코드를 작성합니다.

<pre class="prettyprint" style="font-size:0.7em;">
var http = require("http");

http.createServer(function (req, res) {
    res.writeHead(200, {"Content-Type" : "text/plain"});
    res.end("Hello, SnowDeer's Node.js\n");
}).listen(3000, "127.0.0.1");

console.log("Server is Running...");
</pre>

자, 이제 실행을 해봅시다. 해당 js 파일이 있는 위치에서 
터미널을 통해 다음과 같이 입력합니다.


<pre class="prettyprint" style="font-size:0.7em;">
node hello.js
</pre>

실행 결과는 다음과 같습니다.

![image]({{ site.baseurl }}/assets/2016-11-16-hello-nodejs/1.png)

웹 브라우저에서 접속해봅니다.

![image]({{ site.baseurl }}/assets/2016-11-16-hello-nodejs/2.png)
