---
layout: post
title: 간단한 Chat App 구현하기 (1) - 간단한 웹 서버 구동
category: Go
tag: [Go]
---
# 간단한 웹 서버 구동

다음 패키지들을 이용해서 간단한 웹 서버를 구동시키는 예제입니다.

* [httprouter](https://github.com/julienschmidt/httprouter)
* [negroni](https://github.com/urfave/negroni)

각 패키지 설치는 다음 명령어를 이용해서 할 수 있습니다.

<pre class="prettyprint">
$ go get github.com/julienschmidt/httprouter

$ go get https://github.com/urfave/negroni
</pre>

<br>

### main.go

<pre class="prettyprint">
package main

import (
	"github.com/unrolled/render"
	"github.com/julienschmidt/httprouter"
	"net/http"
	"github.com/urfave/negroni"
)

var renderer *render.Render

func init() {
	renderer = render.New()
}

func main() {
	router := httprouter.New()

	router.GET("/", func(w http.ResponseWriter, req *http.Request, ps httprouter.Params) {
		renderer.HTML(w, http.StatusOK, "index", map[string]string{"title": "Simple Chat App"})
	})

	n := negroni.Classic()

	n.UseHandler(router)

	n.Run(":3000")
}
</pre>

<br>

### /templates/index.tmpl

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html lang="en"&gt;
&lt;body&gt;
    &lt;h1&gt;{{ .title }}&lt;/h1&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>