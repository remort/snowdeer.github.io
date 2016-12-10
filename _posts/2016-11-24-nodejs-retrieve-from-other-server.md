---
layout: post
title: 다른 서버에서 데이터 가져오기
category: Node.js
tag: [Node.js]
---

다른 서버에서 데이터를 가져오는 코드입니다.
HTTP GET, POST, PUT, DELETE 방식 등을 이용할 수 있는데,
여기서는 간단히 GET 방식으로 [구글](http://www.google.com)의 웹 페이지 데이터를
가져오는 코드를 만들어보도록 하겠습니다.

<pre class="prettyprint" style="font-size:0.7em;">
var http = require("http");

var options = {
    host: "www.google.com",
    port: 80,
};

var req = http.get(options, function(res) {
    var data = "";

    res.on("data", function(chunk) {
        data = data + chunk;
    });

    res.on("end", function () {
        console.log("GET 수행 완료.");
        console.log(data);
    });
});

req.on("error", function (err) {
    console.log("Error : " + err);
});
</pre>

<br>

실행 결과는 다음과 같습니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-24-nodejs-retrieve-from-other-server/1.png)

