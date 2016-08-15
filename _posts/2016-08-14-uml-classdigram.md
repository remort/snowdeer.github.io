---
layout: post
title: UML 클래스 다이어그램
category: UML
tag: [uml]
---

클래스 다이어그램(Class Diagram)은 UML 중 가장 많이 사용하게 되는 다이어그램이 아닐까 싶습니다.
그래서 클래스 다이어그램에 대해 간단히 살펴보도록 하겠습니다.

<br>

## 클래스 표기


![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_005654.png)

클래스 속성(Attribute)이나 함수(Operation)들은 표기를 잘 안하지만 
그래도 중요한 속성이나 함수가 있을 경우에는는 표기를 하기도 합니다.

클래스는 

* 클래스 이름
* 속성
* 함수

순으로 표기를 하며 각 이름 앞에 붙는 기호는

~~~
+ : public
- : private
# : protected
~~~

입니다. 


<br>

## 관계

각 클래스별 관계는 대략 6가지로 정의할 수 있습니다.

* Association
* Inheritance(Generalization)
* Realization(Implementation)
* Dependency
* Aggregation
* Composition

그리고 각각의 표기는 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/Uml_classes_en.svg.png)

이 중에서 Aggregation과 Composition은 Association에 포함되고,
Inheritance와 Realization은 클래스 상속이냐 인터페이스 구현이냐의 차이이기 때문에
거의 같은 개념이라고 생각하면,
실제로는 Association, Inheritance, Dependency 정도만 있다고 생각할 수 있습니다.

<br>


### Inheritance/Realization

클래스 다이어그램에서 가장 많이 쓰는 관계가 상속/구현인 것 같습니다.

![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_161609.png)

클래스를 상속하는 경우는 Inheritance 또는 Generalization이라고 합니다.

![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_162854.png)

그리고 인터페이스를 구현하는 경우는 Realization 또는 Implementation이라고 합니다.

Java의 경우는 상속과 구현이 코드 레벨에서 명확하게 구분이 되며,
C++의 경우는 구현이라는 것이 없어서 코드상에서는 구분이 되지 않지만 의미상으로 
구분할 수 있습니다. (딱히 2개는 구분 안해도 큰 무리는 없습니다.)

<br>


### Dependency

어떤 클래스가 다른 클래스를 참고하고 있을 때 사용합니다.

![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_162151.png)

위 그림처럼 표기를 하며 A 클래스가 B 클래스의 영향을 받는 경우 의존(Dependency) 관계가 
있다고 합니다. (물론 상속/구현/Association도 화살표가 가리키는 방향의 클래스가 수정이 되거나
하면 영향을 받습니다.)

Association과 가장 큰 차이는 Dependency는 A 클래스가 B 클래스의 레퍼런스(Reference)를
계속 유지하고 있느냐의 차이입니다. Dependency는 레퍼런스를 유지하지 않습니다.
즉, 일반적으로 A 클래스의 함수 파라메터로 B 클래스가 잠시 사용되는 경우를 Dependency 관계라고
합니다.

<br>

### Association

Association은 Aggregation과 Composition으로 나누어집니다.
(하지만 딱히 둘을 구분안하고 그냥 Association으로 사용해도 충분합니다.)

![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_164009.png)

하지만, Aggregation과 Composition으로 구분해서 사용하겠다고 하면 
각 클래스의 Life Cycle이 서로 동일한지 아닌지를 살펴보면 됩니다.

예를 들어 아래 그림과 같이

![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_162406.png)

Computer가 소멸하더라도 Monitor, Keyboard, Mouse는 여전히 남기 때문에 
이런 관계는 Aggregation이라고 합니다.


![Image]({{ site.baseurl }}/assets/2016-08-14-uml-classdigram/20160814_162615.png)

하지만, Game이 사라지면 Play도 Score도 사라지기 때문에 이런 관계는 Composition이라고 합니다.

실제로 프로젝트를 진행해보면 Aggregation과 Composition이 정확하게 구분이 안되는 
경우가 종종 있습니다. 
그러다보니 실제로는 그냥 Association으로 통일해서 표기하는 경우가 많습니다.