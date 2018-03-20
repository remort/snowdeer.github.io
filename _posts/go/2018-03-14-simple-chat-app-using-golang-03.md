---
layout: post
title: 간단한 Chat App 구현하기 (3) - 인증 처리(로그인)
category: Go
tag: [Go]
---
# 로그인 처리

## 로그인 화면 작성

### /templates/login.tmpl

<pre class="prettyprint">
&lt;html&gt;
&lt;head&gt;
    &lt;title&gt;Login&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;
    &lt;h3&gt;먼저 로그인을 해주세요.&lt;/h3&gt;
    &lt;div&gt;
        &lt;ul&gt;
            &lt;li&gt;
                &lt;a href="/auth/login/google"&gt;Google&lt;/a&gt;
            &lt;/li&gt;
        &lt;/ul&gt;
    &lt;/div&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

### main.go

<pre class="prettyprint">
package main

import (
	"github.com/unrolled/render"
	"github.com/julienschmidt/httprouter"
	"net/http"
	"github.com/goincremental/negroni-sessions/cookiestore"
	"github.com/goincremental/negroni-sessions"
	"github.com/urfave/negroni"
)

var renderer *render.Render

func init() {
	renderer = render.New()
}

const (
	sessionKey    = "simple-chat-app-session"
	sessionSecret = "simple-chat-app-session-secret"
)

func main() {
	router := httprouter.New()

	router.GET("/", func(w http.ResponseWriter, req *http.Request, ps httprouter.Params) {
		renderer.HTML(w, http.StatusOK, "index", map[string]string{"title": "Simple Chat App"})
	})

	router.GET("/login", func(w http.ResponseWriter, req *http.Request, ps httprouter.Params) {
		renderer.HTML(w, http.StatusOK, "login", nil)
	})

	n := negroni.Classic()

	store := cookiestore.New([]byte(sessionSecret))
	n.Use(sessions.Sessions(sessionKey, store))

	n.UseHandler(router)

	n.Run(":3000")
}
</pre>