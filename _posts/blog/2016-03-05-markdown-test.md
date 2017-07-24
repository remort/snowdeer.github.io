---
layout: post
title: Markdown Test
category: Blog
tag: [markdown]
---

안녕하세요. 이 페이지는 Markdown이 화면에 어떻게 렌더링되는지 확인하기 위해서
작성한 테스트 페이지입니다.

<br>

# 제목 1

## 제목 2

### 제목 3

#### 제목 4

##### 제목 5

###### 제목 6

<br>


## 테이블 테스트

Header | Header
------ | ------
Cell   | Cell  

<br>

## 테이블 Allignment

Header | Header | Header
:----- | -----: | :----:
Left   | Right  | Center

<br>

## Keyboard

<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>Del</kbd>

<br>

## 수평선

---

<br>

## 목록

* 목록 1
* 목록 2
* 목록 3

<br>

## 인용문

> 안녕하세요. 여기는 snowdeer 블로그입니다.  
인용문 예제입니다.

> > 인용문 안의 인용문

<br>

## 정의

Jekyll
: 정적 웹페이지 플랫폼

<br>

## Code Block

#### 기본 Code Block

~~~
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");

    return 0;
}
~~~

#### Syntax Highlighter

{% highlight c %}
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");

    return 0;
}
{% endhighlight %}


#### Google Code Prettyfy

<pre class="prettyprint">
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");

    return 0;
}
</pre>



_italic_ and **bold** text,  
inline `code in backticks`,  
and [basic links](http://snowdeer.github.io).


<br>

## CSS 클래스 테스트

#### message

<p class="message">
안녕하세요.
</p>

#### lead

<p class="lead">
안녕하세요.
</p>

#### container

<p class="container">
안녕하세요.
</p>
