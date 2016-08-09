---
layout: post
title: Markdown Test
category: Introduction
tag: markdown
---

안녕하세요. 이 페이지는 Markdown을 테스트하기 위한 페이지입니다.
각종 테마나 CSS 등에 따라 각 MarkDown이 어떻게 표현되는지를 확인하기 위한 용도입니다.


# 제목 1

## 제목 2

### 제목 3

#### 제목 4

##### 제목 5

###### 제목 6


## 테이블 테스트

Header | Header
------ | ------
Cell   | Cell  


## 테이블 Allignment

Header | Header | Header
:----- | -----: | :----:
Left   | Right  | Center




<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>Del</kbd>


## 수평선

---


## 목록

* 목록 1
* 목록 2
* 목록 3


## 인용문

> 안녕하세요. 여기는 snowdeer 블로그입니다.

> > 인용문 안의 인용문


## 정의

Jekyll
: 정적 웹페이지 플랫폼


## Code Block

~~~
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");
  
    return 0;
}
~~~


{% highlight c %}
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");
  
    return 0;
}
{% endhighlight %}


<pre class="prettyprint">
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");
  
    return 0;
}
</pre>


At the end of episode five, it turns out that
>! he's actually his father.


_italic_ and **bold** text,
inline `code in backticks`,
and [basic links](http://example.com).

