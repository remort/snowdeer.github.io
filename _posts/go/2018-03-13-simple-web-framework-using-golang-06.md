---
layout: post
title: 간단한 Web Framework 구현하기 (6) - Abstraction
category: Go
tag: [Go]
---
# 추상화

라우터를 직접 생성하는 것보다 생성자 함수로 감싸는 것이 좀 더 안전한 사용법이며, 미들웨어도 대부분의 핸들러에 적용이 되어야 하기 때문에 미들웨어 체인 형태로 사용하는 것이 더 간편할 수 있습니다.

Server라는 패키지를 만들어서 그 안에서 라우터 생성 및 미들웨어 체인을 사용하도록 한 코드입니다.

### server.go

<pre class="prettyprint">
package main

import "net/http"

type Server struct {
	*router
	middlewares  []Middleware
	startHandler HandlerFunc
}

func NewServer() *Server {
	r := &router{make(map[string]map[string]HandlerFunc)}
	s := &Server{router: r}
	s.middlewares = []Middleware{
		logHandler,
		recoverHandler,
		staticHandler,
		parseFormHandler,
		parseJsonBodyHandler,
	}

	return s
}

func (s *Server) Run(addr string) {
	s.startHandler = s.router.handler()

	for i := len(s.middlewares) - 1; i >= 0; i-- {
		s.startHandler = s.middlewares[i](s.startHandler)
	}

	if err := http.ListenAndServe(addr, s); err != nil {
		panic(err)
	}
}

func (s *Server) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	c := &Context{
		Params:         make(map[string]interface{}),
		ResponseWriter: w,
		Request:        req,
	}

	for k, v := range req.URL.Query() {
		c.Params[k] = v[0]
	}

	s.startHandler(c)
}
</pre>

<br>

### router.go

<pre class="prettyprint">
package main

import (
	"net/http"
	"strings"
)

type router struct {
	handlers map[string]map[string]HandlerFunc
}

func (r *router) handler() HandlerFunc {
	return func(c *Context) {
		for pattern, handler := range r.handlers[c.Request.Method] {
			if ok, params := match(pattern, c.Request.URL.Path); ok {

				for k, v := range params {
					c.Params[k] = v
				}

				handler(c)
				return
			}
		}

		http.NotFound(c.ResponseWriter, c.Request)
	}
}

func (r *router) HandleFunc(method, pattern string, h HandlerFunc) {
	m, ok := r.handlers[method]

	if !ok {
		m = make(map[string]HandlerFunc)
		r.handlers[method] = m
	}
	m[pattern] = h
}

func match(pattern, path string) (bool, map[string]string) {
	if pattern == path {
		return true, nil
	}

	patterns := strings.Split(pattern, "/")
	paths := strings.Split(path, "/")

	if len(patterns) != len(paths) {
		return false, nil
	}

	params := make(map[string]string)

	for i := 0; i < len(patterns); i++ {
		switch {
		case patterns[i] == paths[i]:
		case len(patterns[i]) > 0 && patterns[i][0] == ':':
			params[patterns[i][1:]] = paths[i]
		default:
			return false, nil
		}
	}

	return true, params
}
</pre>

<br>

### main.go

<pre class="prettyprint">
package main

import (
	"fmt"
)

func main() {
	fmt.Println("Simple Web Framework")

	s := NewServer()

	s.HandleFunc("GET", "/", func(c *Context) {
		fmt.Fprintln(c.ResponseWriter, "This is an index page.")
	})

	s.HandleFunc("GET", "/about", func(c *Context) {
		fmt.Fprintln(c.ResponseWriter, "This is an about page.")
	})

	s.HandleFunc("GET", "/users/:id", func(c *Context) {
		userId := c.Params["id"]

		fmt.Fprintln(c.ResponseWriter, "Retrieve user: ", userId)

		if userId == "-1" {
			panic("id is not valid..")
		}

	})

	s.HandleFunc("POST", "/users", func(c *Context) {
		fmt.Fprintln(c.ResponseWriter, "Create user")
	})

	s.Run(":8080")
}
</pre>