---
layout: post
title: 데코레이터(Decorator) 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

데코레이터 패턴의 UML은 다음과 같습니다.

![Image](/assets/design-patterns/decorator.png)

Head-First 교재에서 설명하는 데코레이터 패턴의 UML은 다음과 같습니다.

![Image](/assets/design-pattern-headfirst/decorator-pattern.png)

데코레이터 패턴을 간단히 설명하면 어떤 객체를 꾸미고자 할 때(장식할 때) 그 결과값으로 동일한 객체 타입이 리턴되는 패턴이라고 할 수 있습니다.

예를 들어, 어떤 커피점에서 커피를 만들어낸다고 할 때, 각 커피에 해당하는 클래스를 일일이 만들 경우 다음과 같은 상황이 발생할 수 있습니다.

![Image](/assets/design-pattern-headfirst/decorator-badcase.png)

이 때 데코레이터 패턴을 사용하면 각 커피에 해당하는 클래스를 하나하나 만들지 않더라도 다음과 같은 형태로 다양한 커피를 만들어 낼 수 있습니다.

<br>

## 데코레이터 패턴이 적용된 예시

<pre class="prettyprint">
Beverage darkMochaCoffee = new Whip(new Mocha(new DarkRoast()));

Beverage soyMochaCoffee = new HouseBlend();
beverage2 = new Soy(beverage2);
beverage2 = new Mocha(beverage2);
beverage2 = new Whip(beverage2);
</pre>

위 예제와 같이 데코레이터 패턴을 이용하면 새로운 커피 조리법이 나오더라도 코드의 큰 수정없이 대처가 가능합니다.

Java의 I/O 스트림의 경우가 데코레이터 패턴이 적용된 대표적인 예시라고 볼 수 있습니다.

![Image](/assets/design-patterns/decorator-java-io.png)

<br>

## 컴포지트 패턴과 데코레이터 패턴

컴포지트 패턴과 데코레이터 패턴은 비슷한 부분이 많이 있습니다. 특히 재귀적으로 순환하는 방식은 공통적인 성격입니다. 다만 두 패턴은 그 목적에서 큰 차이점이 있습니다. 컴포지트 패턴은 여러 개로 구성된 클래스들이 동일한 형태로 구성될 수 있도록 구조화하는데 목적이 있다면, 데코레이터 패턴은 기능을 클래스화함으로써 동적으로 기능을 추가하거나 삭제할 수 있도록 하는데 목적이 있습니다.

컴포지트 패턴과 데코레이터 패턴은 동시에 사용되어 상호보완적인 역할을 할 수도 있습니다.