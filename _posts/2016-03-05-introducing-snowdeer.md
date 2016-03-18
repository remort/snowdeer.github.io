---
layout: post
title: 블로그를 시작하며
category: Introduction
---

안녕하세요. 예전엔 [티스토리 블로그](http://snowbora.tistory.com)를 운영해왔었는데, 
최근에 [GitHub](http://github.com/)와 연동할 수 있는
[Jekyll](https://github.com/jekyll/jekyll) 블로그 시스템을 알게 되어 이쪽에서 새로운 시작을 해볼까 합니다.

## 이 블로그는 

프로그래밍 쪽과 관련된 내용들 위주로 운영할 계획이며, 
다음과 같은 내용들을 다뤄보고자 합니다.

* Design Pattern
* Algorithms
* Android Development
* C++11
* 그 외 각종 소식들

## Jekyll 소개

정적인 웹페이지를 만들어주는 도구입니다. [Ruby](https://www.ruby-lang.org/ko/) 기반으로 되어 있으며,
날짜 기반의 웹페이지들을 마치 블로그처럼 보여줄 수 있는 기능만을 제공하고 있습니다.

정적인 웹페이지로 이루어져 있기 때문에, [티스토리](http://www.tistory.com/)나 네이버 블로그 등과 같은
화려하고 동적인 기능들은 제공하지 않습니다. 심지어 글 작성, 댓글 작성이나 알림, 통계 등의 다양한 기능들도
지원하지 않습니다.

이러한 단점이 많은데도 [Jekyll](https://github.com/jekyll/jekyll)을 선택한 이유는

* 동적인 요소가 없기 때문에 DB 서버 등이 필요없고, 속도도 훨씬 빠릅니다.
* 정적인 문서 그 자체들로 이루어져 있기 떄문에 어떤 호스팅 서버에서도 동작할 수 있습니다.
* 전 세계 많은 사람들이 사용하고 있고, 다양한 Plugin이나 확장 기능이 있어서 입맛에 맞게 바꾸기가 쉽습니다.
* 새로운 걸 공부한다는 자체가 뭔가 재미가 있습니다. +_+
* 그리고 가장 마음에 든 것은 아래와 같은 Syntax Highlighting 입니다. 기존 블로그들에서도 이 기능은
제공하고 있었지만, Jeykyll에서 훨씬 더 사용하기 쉽고 강력하게 지원을 해서 제가 사용하기에
딱 좋네요!!

{% highlight c %}
int main(int argc, char** argv) {
    printf("Hello~ Welcome to SnowDeer's Blog.\n");
  
    return 0;
}
{% endhighlight %}

다만, 웹쪽 경험이 거의 없다보니 Jekyll 에 대해 이해를 하고 익숙해져가는 과정이 쉽지는 않았네요. 
조금씩 조금씩 공부해가면서 업데이트해나가다보면 언젠가는 익숙해지지 않을까 싶습니다.

Thanks !!
