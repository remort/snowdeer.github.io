---
layout: post
title: 간단한 Web Framework 구현하기 (9) - Custom Middleware(Authorize)
category: Go
tag: [Go]
---
# Custom Middleware

### /public/login.html

<pre class="prettyprint">
&lt;html&gt;
&lt;head&gt;
    &lt;title&gt;Login&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;
&lt;form class="form-signin" action="/login" method="post"&gt;
    username
    &lt;input name="username" required autofocus&gt;
    password
    &lt;input type="password" name="password" required&gt;
    &lt;button type="submit">Sign in&lt;/button&gt;
&lt;/form&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

### middleware.go

<pre class="prettyprint">
package main

import (
	"time"
	"log"
	"net/http"
	"fmt"
	"encoding/json"
	"strings"
	"path"
	"crypto/hmac"
	"crypto/sha1"
	"io"
	"encoding/hex"
)

type Middleware func(next HandlerFunc) HandlerFunc

func logHandler(next HandlerFunc) HandlerFunc {
	return func(c *Context) {
		fmt.Println("logHandler is called.")

		t := time.Now()

		next(c)

		log.Printf("[%s] %q %v\n",
			c.Request.Method,
			c.Request.URL.String(),
			time.Now().Sub(t))
	}
}

func recoverHandler(next HandlerFunc) HandlerFunc {
	return func(c *Context) {
		fmt.Println("recoverHandler is called.")

		defer func() {
			if err := recover(); err != nil {
				log.Printf("panic: %+v", err)
				http.Error(c.ResponseWriter,
					http.StatusText(http.StatusInternalServerError),
					http.StatusInternalServerError)
			}
		}()
		next(c)
	}
}

func parseFormHandler(next HandlerFunc) HandlerFunc {
	return func(c *Context) {
		fmt.Println("HandlerFunc is called.")

		c.Request.ParseForm()
		fmt.Println(c.Request.PostForm)

		for k, v := range c.Request.PostForm {
			if len(v) > 0 {
				c.Params[k] = v[0]
			}
		}

		next(c)
	}
}

func parseJsonBodyHandler(next HandlerFunc) HandlerFunc {
	return func(c *Context) {
		fmt.Println("parseJsonBodyHandler is called.")

		var m map[string]interface{}

		if json.NewDecoder(c.Request.Body).Decode(&m); len(m) > 0 {
			for k, v := range m {
				c.Params[k] = v
			}
		}

		next(c)
	}
}

func staticHandler(next HandlerFunc) HandlerFunc {
	var (
		dir       = http.Dir(".")
		indexFile = "index.html"
	)

	return func(c *Context) {
		if c.Request.Method != "GET" && c.Request.Method != "HEAD" {
			next(c)
			return
		}

		file := c.Request.URL.Path
		f, err := dir.Open(file)
		if err != nil {
			next(c)
			return
		}
		defer f.Close()

		fi, err := f.Stat()
		if err != nil {
			next(c)
			return
		}

		if fi.IsDir() {
			if !strings.HasSuffix(c.Request.URL.Path, "/") {
				http.Redirect(c.ResponseWriter, c.Request, c.Request.URL.Path+"/", http.StatusFound)
				return
			}

			file = path.Join(file, indexFile)

			f, err = dir.Open(file)
			if err != nil {
				next(c)
				return
			}
			defer f.Close()

			fi, err = f.Stat()
			if err != nil || fi.IsDir() {
				next(c)
				return
			}
		}

		http.ServeContent(c.ResponseWriter, c.Request, file, fi.ModTime(), f)
	}
}

const VerifyMessage = "verified"

func AuthHandler(next HandlerFunc) HandlerFunc {
	ignore := []string{"/login", "public/index.html"}
	return func(c *Context) {
		for _, s := range ignore {
			if strings.HasPrefix(c.Request.URL.Path, s) {
				next(c)
				return
			}

			if v, err := c.Request.Cookie("X_AUTH"); err == http.ErrNoCookie {
				c.Redirect("/login")
			} else if err != nil {
				c.RenderError(http.StatusInternalServerError, err)
			} else if Verify(VerifyMessage, v.Value) {
				next(c)
				return
			}
		}

		c.Redirect("/login")
	}
}

func Verify(message, sig string) bool {
	return hmac.Equal([]byte(sig), []byte(Sign(message)))
}

func Sign(message string) string {
	secretKey := []byte("snowdeer-simple-web-framework")
	if len(secretKey) == 0 {
		return ""
	}

	mac := hmac.New(sha1.New, secretKey)
	io.WriteString(mac, message)

	return hex.EncodeToString(mac.Sum(nil))
}
</pre>

<br>

### main.go

<pre class="prettyprint">
package main

import (
	"fmt"
	"time"
	"net/http"
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

	s.HandleFunc("GET", "/login", func(c *Context) {
		c.RenderTemplate("/public/login.html", map[string]interface{}{"message": "로그인이 필요합니다."})
	})

	s.HandleFunc("POST", "/login", func(c *Context) {

		if CheckLogin(c.Params["username"].(string), c.Params["password"].(string)) {
			http.SetCookie(c.ResponseWriter, &http.Cookie{
				Name:  "X_AUTH",
				Value: Sign(VerifyMessage),
				Path:  "/",
			})
			c.Redirect("/")
		}
		c.RenderTemplate("/public/login.html", map[string]interface{}{"message": "id 또는 password 오류"})

	})

	s.Use(AuthHandler)
	s.Run(":8080")
}

func CheckLogin(username, password string) bool {
	const (
		USERNAME = "snowdeer"
		PASSWORD = "1234"
	)

	return (username == USERNAME) && (password == PASSWORD)
}
</pre>