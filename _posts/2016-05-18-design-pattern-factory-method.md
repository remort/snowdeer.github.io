---
layout: post
title: 추상 팩토리(Abstract Factory) 패턴
category: 디자인패턴
tag: [pattern, abstract factory]
---

[추상 팩토리(Abstract Factory)](https://en.wikipedia.org/wiki/Abstract_factory_pattern) 패턴의 UML은 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/abstract-factory.gif) 

Abstract Factory 패턴은 어떤 객체를 구성하는 요소들까지 추상화하여 인스턴스를 구성해가는 패턴입니다.
Factory Method 패턴과 많이 헷갈리는 부분이 있는데, 다음과 같은 차이가 있습니다.

~~~
Factory Method 패턴은 객체를 만들때 클래스를 이용하여 객체를 만듭니다.
Abstract Factory 패턴은 객체를 만들때 객체 구성을 이용하여 객체를 만듭니다.
~~~

