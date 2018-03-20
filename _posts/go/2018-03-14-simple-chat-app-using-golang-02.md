---
layout: post
title: 간단한 Chat App 구현하기 (2) - 인증 처리(세션 관리)
category: Go
tag: [Go]
---
# 세션 관리

다음 패키지들을 이용해서 세션을 관리합니다.

* [negroni-sessions](https://github.com/goincremental/negroni-sessions)
* [gomniauth](https://github.com/stretchr/gomniauth)

각 패키지 설치는 다음 명령어를 이용해서 할 수 있습니다.

<pre class="prettyprint">
$ go get github.com/goincremental/negroni-sessions

$ go get github.com/stretchr/gomniauth

$ go get github.com/stretchr/gomniauth/providers/google
</pre>

<br>

### main.go

세션 관리를 위해 세션 핸들러를 등록합니다.

<pre class="prettyprint">
package main

import (
	"github.com/unrolled/render"
	"github.com/julienschmidt/httprouter"
	"net/http"
	"github.com/urfave/negroni"
	"github.com/goincremental/negroni-sessions/cookiestore"
	"github.com/goincremental/negroni-sessions"
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

	n := negroni.Classic()
	store := cookiestore.New([]byte(sessionSecret))
	n.Use(sessions.Sessions(sessionKey, store))

	n.UseHandler(router)

	n.Run(":3000")
}
</pre>

<br>

### session.go

<pre class="prettyprint">
package main

import (
	"time"
	"net/http"
	"github.com/goincremental/negroni-sessions"
	"encoding/json"
)

const (
	currentUserKey  = "oauth2_current_user"
	sessionDuration = time.Hour
)

type User struct {
	Uid       string    `json:"uid"`
	Name      string    `json:"name"`
	Email     string    `json:"user"`
	AvatarUrl string    `json:"avatar_url"`
	Expired   time.Time `json:"expired"'`
}

func (u *User) Valid() bool {
	return u.Expired.Sub(time.Now()) > 0
}

func (u *User) Refresh() {
	u.Expired = time.Now().Add(sessionDuration)
}
func GetCurrentUser(r *http.Request) *User {
	s := sessions.GetSession(r)

	if s.Get(currentUserKey) == nil {
		return nil
	}

	data := s.Get(currentUserKey).([]byte)

	var u User
	json.Unmarshal(data, &u)

	return &u
}

func SetCurrentUser(r *http.Request, u *User) {
	if u != nil {
		u.Refresh()
	}

	s := sessions.GetSession(r)
	val, _ := json.Marshal(u)
	s.Set(currentUserKey, val)
}
</pre>