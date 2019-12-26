---
layout: post
title: Circular View path error 

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

만약 

~~~
Circular view path [list]: would dispatch back to the current handler URL [/list] again.
~~~

와 같은 오류가 발생하면 `build.gradle.kts` 파일에

<pre class="prettyprint">
implementation("org.springframework.boot:spring-boot-starter-thymeleaf")
</pre>

종속성 추가가 되어 있는 지를 체크해 볼 필요가 있습니다.

그리고 잊지 말고 `gradle sync`를 해줘야 반영이 됩니다.