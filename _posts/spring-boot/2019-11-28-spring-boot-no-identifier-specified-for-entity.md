---
layout: post
title: No identifier specified for entity 문제 해결

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# No identifier specified for entity 문제 해결 방법

만약 

~~~
nested exception is org.hibernate.AnnotationException: No identifier specified for entity: com.snowdeer.database.board.Member
~~~

와 같은 오류가 발생하면 해당 클래스의 `@Id` 어노테이션 항목을 살펴봐야 합니다.

<pre class="prettyprint">
import org.springframework.data.annotation.Id
import javax.persistence.Entity
import javax.persistence.Table

@Entity
@Table(name = "tbl_members")
data class Member(@Id
                  var uid: String = "",
                  var upw: String = "",
                  var uname: String = "")
</pre>

만약 이와 같이 `import org.springframework.data.annotation.Id`가 `import`되어 있다면,
`import javax.persistence.Id`로 수정하면 됩니다.