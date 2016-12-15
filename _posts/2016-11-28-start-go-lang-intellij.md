---
layout: post
title: IntelliJ에서 Go 언어 시작하기
category: Go 
tag: [intellij, go]
---

[IntelliJ](https://www.jetbrains.com/idea/?fromMenu)에서 Go 언어를
시작하는 방법입니다.
물론, 먼저 Go 언어를 설치해야겠죠. [여기](https://golang.org/dl/)에서 받으실 수 있습니다.

<br>

먼저 IntelliJ를 실행합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/1.png)

여기에서 아래쪽 메뉴에서 'plugins'를 선택합니다.

<br>

그럼 Plugins 창이 뜨는데,

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/2.png)

아래쪽의 'Browse repositories'를 선택합니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/3.png)

여기서 검색창에 'go'를 입력하면 위 화면과 같이 'Go - LANGUAGES' plugin이 검색됩니다.

<br>

설치가 끝나면 IntelliJ를 종료했다가 다시 실행시켜주세요.

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/4.png)

이 화면에서 이제 'Create New Project'를 누릅니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/5.png)

이런 식으로 'Go' 프로젝트 생성을 선택할 수 있게 되었을 겁니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/6.png)

아마 맨 처음에는 이 화면에서 Go SDK가 선택되어 있지 않을텐데,
'Configure' 버튼을 눌러 Go SDK가 설치되어 있는 폴더를 선택해줍니다. 
(폴더까지만 선택해주시면 됩니다.)


<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/7.png)

마지막으로 프로젝트 전체 설정을 하면 Go 프로젝트가 만들어집니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-11-28-start-go-lang-intellij/8.png)

간단하게 다음과 같은 코드를 입력해서 제대로 실행되는지 확인해 보세요.

<pre class="prettyprint" style="font-size:0.7em;">
package main

import "fmt"

func main() {
	fmt.Println("Hello, Go !!");
}
</pre>

