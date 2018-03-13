---
layout: post
title: 간단한 Web Framework 구현하기 (2) - 간단한 Web App
category: Go
tag: [Go]
---
# 간단한 Web App

먼저 단순히 사용자의 Request와 Response를 처리하는 프로그램을 작성합니다.

### main.go

<pre class="prettyprint">
package main

import (
	"fmt"
	"net/http"
)

func main() {
	fmt.Println("Simple Web Framework")

	http.HandleFunc("/", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "This is an index page.")
	})

	http.HandleFunc("/about", func(w http.ResponseWriter, req *http.Request) {
		fmt.Fprintln(w, "This is an about page.")
	})

	http.ListenAndServe(":8080", nil)
}
</pre>

