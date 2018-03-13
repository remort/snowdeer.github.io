---
layout: post
title: 간단한 Web Framework 구현하기 (8) - Renderer (Template)
category: Go
tag: [Go]
---
# Renderer XML/JSON

### /public/index.html

<pre class="prettyprint">
&lt;html&gt;
&lt;body&gt;
Hello SnowDeer
&lt;p&gt;
    current time: {{ .time }}
&lt;/p&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

### context.go

<pre class="prettyprint">
package main

import (
	"net/http"
	"encoding/json"
	"encoding/xml"
	"path/filepath"
	"html/template"
)

type Context struct {
	Params map[string]interface{}

	ResponseWriter http.ResponseWriter
	Request        *http.Request
}

type HandlerFunc func(*Context)

func (c *Context) RenderError(code int, err error) {
	if err != nil {
		errorCode := http.StatusInternalServerError

		if code > 0 {
			errorCode = code
		}

		http.Error(c.ResponseWriter, http.StatusText(errorCode), errorCode)
	}
}

func (c *Context) RenderJson(v interface{}) {
	c.ResponseWriter.WriteHeader(http.StatusOK)
	c.ResponseWriter.Header().Set("Content-Type", "application/json; charset=utf-8")

	if err := json.NewEncoder(c.ResponseWriter).Encode(v); err != nil {
		c.RenderError(http.StatusInternalServerError, err)
	}
}

func (c *Context) RenderXml(v interface{}) {
	c.ResponseWriter.WriteHeader(http.StatusOK)
	c.ResponseWriter.Header().Set("Content-Type", "application/json; charset=utf-8")

	if err := xml.NewEncoder(c.ResponseWriter).Encode(v); err != nil {
		c.RenderError(http.StatusInternalServerError, err)
	}
}

func (c *Context) Redirect(url string) {
	http.Redirect(c.ResponseWriter, c.Request, url, http.StatusMovedPermanently)
}

var templates = map[string]*template.Template{}

func (c *Context) RenderTemplate(path string, v interface{}) {
	t, ok := templates[path]
	if !ok {
		t = template.Must(template.ParseFiles(filepath.Join(".", path)))
		templates[path] = t
	}

	t.Execute(c.ResponseWriter, v)
}
</pre>

<br>

### main.go

<pre class="prettyprint">
package main

import (
	"fmt"
	"time"
)

type User struct {
	Id   string
	Name string
}

func main() {
	fmt.Println("Simple Web Framework")

	s := NewServer()

	s.HandleFunc("GET", "/", func(c *Context) {
		c.RenderTemplate("/public/index.html", map[string]interface{}{"time": time.Now()})
	})

	s.HandleFunc("GET", "/about", func(c *Context) {
		fmt.Fprintln(c.ResponseWriter, "This is an about page.")
	})

	s.HandleFunc("GET", "/users/:id", func(c *Context) {
		u := User{Id: c.Params["id"].(string)}

		c.RenderXml(u)
	})

	s.HandleFunc("GET", "/users/:id/name/:name", func(c *Context) {
		u := User{Id: c.Params["id"].(string),
			Name: c.Params["name"].(string),
		}

		c.RenderJson(u)
	})

	s.HandleFunc("POST", "/users", func(c *Context) {
		fmt.Fprintln(c.ResponseWriter, "Create user")
	})

	s.Run(":8080")
}
</pre>