---
layout: post
title: H2 Database 사용하기

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# H2 Database

H2 데이터베이스는 자바 기반의 RDBMS(Relational DataBase Management System)입니다. 
용량이 적으며, 브라우저 기반의 콘솔 등을 지원하여 장점이 많습니다.
Spring Boot에서 별도 데이터베이스를 설치하지 않고 바로 사용할 수 있는 점도 장점입니다.

[Spring Initializer 웹사이트](https://start.spring.io)에서 의존성(Dependencies) 부분에서 
'H2 Database' 항목과 'Spring Data JPA' 항목을 선택하고
프로젝트를 생성해주면 자동으로 `build.gradle`에 반영이 됩니다.

<br>

## application.properties

그리고 `application.properties` 파일에 속성값을 지정해줘야 합니다. 여기서는 `application.yaml` 파일을 이용해서 설정하도록 하겠습니다.
기존의 `application.properties` 파일을 지우고 `application.yaml` 파일을 생성하고 아래의 내용을 작성합니다.

<pre class="prettyprint">
server:
  port: 8000

spring:
  h2:
    console:
      enabled: true
      path: /h2_db

  datasource:
    driver-class-name: org.h2.Driver
    url: jdbc:h2:file:./h2_db;AUTO_SERVER=TRUE
    username: snowdeer
    password: 1234
</pre>

`url` 항목에서 `jdbc:h2:file`는 데이터베이스를 파일에 저장하겠다는 의미이며, `./h2_db`는 파일 경로입니다.

위와 같이 설정한 다음 `http://localhost:8000` 주소에 접속하면 H2 Console 웹페이지가 열립니다.

> `JDBC URL` 항목이 `jdbc:h2:~/test`으로 되어 있을텐데, 위에서 `yaml` 파일에 작성한 `jdbc:h2:file:./h2_db` 값을 넣어줘야 합니다.

<br>

## 데이터베이스 연결하기

### Hikari CP

Spring Boot 2.0.0 M2 버전부터 기본적으로 사용하는 커넥션 풀(Connection Pool)이 `Tomcat`에서 `HikariCP`로 변경되었습니다.

`application.yaml`에 HikariCP 연결 정보를 반영해서 다음과 같이 수정합니다.

<pre class="prettyprint">
server:
  port: 8000

spring:
  h2:
    console:
      enabled: true
      path: /h2_db

  datasource:
    hikari:
      driver-class-name: org.h2.Driver
      jdbc-url: jdbc:h2:file:./h2_db
      username: snowdeer
      password: 1234
      connection-test-query: SELECT 1
</pre>

<br>

### DatabasConfiguration.kt

데이터베이스 사용을 위한 기본적인 설정을 합니다.

<pre class="prettyprint">
import com.zaxxer.hikari.HikariConfig
import com.zaxxer.hikari.HikariDataSource
import org.springframework.boot.context.properties.ConfigurationProperties
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import org.springframework.context.annotation.PropertySource
import javax.sql.DataSource

@Configuration
@PropertySource("classpath:application.yaml")
class DatabaseConfiguration {

    @Bean
    @ConfigurationProperties(prefix = "spring.datasource.hikari")
    fun hikariConfig(): HikariConfig {
        return HikariConfig()
    }
    
    @Bean
    @Throws(Exception::class)
    fun dataSource(): DataSource {
        val dataSource = HikariDataSource(hikariConfig())
        println(dataSource.toString())
        return dataSource
    }
}
</pre>

지금까지 문제없이 작성되었다면 빌드 및 실행이 가능할 것입니다.