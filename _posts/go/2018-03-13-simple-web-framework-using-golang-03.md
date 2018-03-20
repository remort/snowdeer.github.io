---
layout: post
title: 간단한 Web Framework 구현하기 (3) - Router
category: Go
tag: [Go]
---
# Router 추가

사용자로부터 요청받은 Request Method(`GET` 또는 `POST` 등) 및 URL에 따라 각각을 처리할 수 있는 모듈인 Router를 구현해봅니다.

### main.go

<pre class="prettyprint">
package main

import (
	"fmt"
	"net/http"
)

func main() {
	fmt.Println("Simple Web Framework")

	r := &router{make(map[string]map[string]http.HandlerFunc)}

	r.HandleFunc("GET", "/", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "This is an index page.")
	})

	r.HandleFunc("GET", "/about", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "This is an about page.")
	})

	http.ListenAndServe(":8080", r)
}
</pre>

<br>

### router.go

<pre class="prettyprint">
package main

import (
	"net/http"
)

type router struct {
	handlers map[string]map[string]http.HandlerFunc
}

func (r *router) HandleFunc(method, pattern string, handler http.HandlerFunc) {
	m, ok := r.handlers[method]

	if !ok {
		m = make(map[string]http.HandlerFunc)
		r.handlers[method] = m
	}
	m[pattern] = handler
}

func (r *router) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	if m, ok := r.handlers[req.Method]; ok {
		if handler, ok := m[req.URL.Path]; ok {
			handler(w, req)
			return
		}
	}

	http.NotFound(w, req)
}
</pre>

<br>

## router에 정규식 적용

### main.go

<pre class="prettyprint">
package main

import (
	"fmt"
	"net/http"
)

func main() {
	fmt.Println("Simple Web Framework")

	r := &router{make(map[string]map[string]http.HandlerFunc)}

	r.HandleFunc("GET", "/", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "This is an index page.")
	})

	r.HandleFunc("GET", "/about", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "This is an about page.")
	})

	r.HandleFunc("GET", "/users/:id", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "Retrieve user")
	})

	r.HandleFunc("POST", "/users", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "Create user")
	})

	http.ListenAndServe(":8080", r)
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
	handlers map[string]map[string]http.HandlerFunc
}

func (r *router) HandleFunc(method, pattern string, h http.HandlerFunc) {
	m, ok := r.handlers[method]

	if !ok {
		m = make(map[string]http.HandlerFunc)
		r.handlers[method] = m
	}
	m[pattern] = h
}

func (r *router) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	for pattern, handler := range r.handlers[req.Method] {
		if ok, _ := match(pattern, req.URL.Path); ok {
			handler(w, req)
			return
		}
	}

	http.NotFound(w, req)
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