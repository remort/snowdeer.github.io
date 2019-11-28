---
layout: post
title: JPA를 사용한 예제

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---
# Spring Data JPA

JPA(Java Persistence API)를 사용할 경우 데이터베이스 종류에 무관하게 
객체 지향 프로그램의 객체를 다루듯이 데이터베이스의 엔티티를 쉽게 다룰 수 있습니다.

여기서는 `H2` 데이터베이스를 이용해서 JPA 예제를 사용해봅니다.

<br>

## H2 데이터베이스 포함된 프로젝트 생성

[Spring Initializer 웹사이트](https://start.spring.io)에서 
'H2 Database' 항목과 'Spring Data JPA' 항목을 선택하고 프로젝트를 생성해줍니다.

<br>

## application.yaml 수정

`application.properties` 파일을 `application.yaml` 파일로 변경하고 
파일 안에 다음 내용을 작성합니다.

<pre class="prettyprint">
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

  jpa:
    hibernate:
      ddl-auto: update
    generate-ddl: false
    show-sql: true

logging:
  level:
    org:
      hibernate: info
</pre>

<br>

## DatabaseConfiguration.kt

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

<br>

<br>

## Board 엔티티 생성

`Board.kt` 파일을 생성합니다.

<pre class="prettyprint">
import org.hibernate.annotations.CreationTimestamp
import org.hibernate.annotations.UpdateTimestamp
import java.sql.Timestamp
import javax.persistence.*

@Entity
@Table(name = "tbl_boards")
data class Board(@Id
                 @GeneratedValue(strategy = GenerationType.AUTO)
                 var bno: Long = 0,

                 var title: String = "",
                 var writer: String = "",
                 var content: String = "",

                 @CreationTimestamp
                 var regdate: Timestamp? = null,

                 @UpdateTimestamp
                 var updatedate: Timestamp? = null)
</pre>

<br>

## BoardRepository.kt

<pre class="prettyprint">
import com.snowdeer.database.board.Board
import org.springframework.data.domain.Pageable
import org.springframework.data.jpa.repository.Query
import org.springframework.data.repository.CrudRepository

interface BoardRepository : CrudRepository&lt;Board, Long&gt; {

    fun findBoardByTitle(title: String): List&lt;Board&gt;

    fun findByWriter(writer: String): List&lt;Board&gt;

    fun findByWriterContaining(writer: String): List&lt;Board&gt;

    fun findByTitleContaining(title: String): List&lt;Board&gt;

    fun findByTitleContainingOrContentContaining(title: String, content: String): List&lt;Board&gt;

    fun findByBnoGreaterThanOrderByBnoDesc(bno: Long): List&lt;Board&gt;

    fun findByBnoGreaterThanOrderByBnoDesc(bno: Long, paging: Pageable): List&lt;Board&gt;

    fun findByBnoGreaterThan(bno: Long, paging: Pageable): List&lt;Board&gt;

    @Query("SELECT b FROM Board b WHERE b.title LIKE %?1% AND b.bno > 0 ORDER BY b.bno DESC")
    fun findByTitle(title: String): List&lt;Board&gt;
}
</pre>

<br>

## Unittest로 확인

위와 같이 인터페이스만 선언해줘도 각 메소드들은 잘 동작합니다. 
정말 잘 동작하는지 Unittest를 이용해서 확인해보도록 합시다.

`BoardRepositoryTests.kt`라는 이름으로 파일을 만들었습니다.

<pre class="prettyprint">
import com.snowdeer.database.board.Board
import com.snowdeer.database.repository.BoardRepository
import org.junit.jupiter.api.Test
import org.springframework.beans.factory.annotation.Autowired
import org.springframework.boot.test.context.SpringBootTest
import org.springframework.data.domain.PageRequest
import org.springframework.data.domain.Sort

@SpringBootTest
class BoardRepositoryTests {

    @Autowired
    private lateinit var boardRepo: BoardRepository

    @Test
    fun inspect() {
        val clz = boardRepo.javaClass
        println("[snowdeer] inspect - ${clz.name}")

        val interfaces = clz.interfaces
        for (inter in interfaces) {
            println("[snowdeer] inspect - ${inter.name}")
        }

        val superClass = clz.superclass
        println("[snowdeer] inspect - ${superClass.name}")
    }

    @Test
    fun testInsert() {
        val board = Board()
        board.title = "제목"
        board.content = "내용"
        board.writer = "snowdeer"

        boardRepo.save(board)
    }

    @Test
    fun testRead() {
        val board = boardRepo.findById(1L)
        println("[snowdeer] testRead - $board")
    }

    @Test
    fun testUpdate() {
        val board = boardRepo.findById(1L)
        board.get().title = "수정된 제목"
        boardRepo.save(board.get())
    }

    @Test
    fun testByTitle() {
        println("[snowdeer] testByTitle")
        val list = boardRepo.findBoardByTitle("제목")
        for (b in list) {
            println("[snowdeer] ${b.bno}. ${b.title} - ${b.regdate}")
        }
    }

    @Test
    fun testByWriter() {
        println("[snowdeer] testByWriter")
        val list = boardRepo.findByWriter("snowdeer")
        for (b in list) {
            println("[snowdeer] ${b.bno}. ${b.title} - ${b.regdate}")
        }
    }

    @Test
    fun testByTitleLike() {
        println("[snowdeer] testByTitleLike")
        val list = boardRepo.findByTitleContaining("목")
        for (b in list) {
            println("[snowdeer] ${b.bno}. ${b.title} - ${b.regdate}")
        }
    }

    @Test
    fun testBnoOrderByPaging() {
        println("[snowdeer] testBnoOrderByPaging")
        val paging = PageRequest.of(0, 10)
        val list = boardRepo.findByBnoGreaterThanOrderByBnoDesc(0L, paging)

        for (b in list) {
            println("[snowdeer] ${b.bno}. ${b.title} - ${b.regdate}")
        }
    }

    @Test
    fun testBnoPagingSort() {
        println("[snowdeer] testBnoPagingSort")
        val paging = PageRequest.of(0, 10, Sort.Direction.ASC, "bno")
        val list = boardRepo.findByBnoGreaterThan(0L, paging)

        for (b in list) {
            println("[snowdeer] ${b.bno}. ${b.title} - ${b.regdate}")
        }
    }

    @Test
    fun testFindByTitle() {
        println("[snowdeer] testFindByTitle")
        val list = boardRepo.findByTitle("제목")

        for (b in list) {
            println("[snowdeer] ${b.bno}. ${b.title} - ${b.regdate}")
        }
    }
}
</pre>