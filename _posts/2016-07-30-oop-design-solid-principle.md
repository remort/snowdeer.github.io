---
layout: post
title: 객체지향 설계 원칙 - SOLID
category: UML
tag: [uml]
---

객체 지향 설계에서 유명한 5가지 원칙이 있습니다.
그 5가지 원칙의 첫 글자를 따서 'SOLID' 원칙이라고 부르고 있습니다.

<br>

## The Single Responsibility Principle (SRP)

> 각각의 클래스는 단 하나의 책임만 져야 한다.

예를 들어 다음과 같은 클래스는 너무 많은 일을 하고 있습니다.

![Image]({{ site.baseurl }}/assets/2016-07-30-oop-design-solid-principle/srp.png)

이럴 경우, 위 기능 중 하나의 기능만 수정이 가해져도 클래스 내부가 복잡하게 
수정이 되어져야 할 가능성이 높습니다. 따라서 각 기능들을 분리하여 
각각의 클래스로 만드는 것이 바람직합니다. 

<br>

## The Open-Closed Principle (OCP)

> 확장은 유연하게(Open), 수정은 폐쇄적으로(Close) 해야 한다.

아래와 같은 구조는 Employee가 EmployeeDB를, EmployeeDB가 Database를 바라보고 있습니다.

![Image]({{ site.baseurl }}/assets/2016-07-30-oop-design-solid-principle/ocp-1.png)

이 경우에는 Database 쪽에 수정이 발생하면 EmployeeDB 및 Employee 모두 수정이 발생하게 됩니다.
그래서 아래와 같은 구조로 바꾸는게 효율적입니다.

![Image]({{ site.baseurl }}/assets/2016-07-30-oop-design-solid-principle/ocp-2.png)

OCP를 잘 고려한 대표적인 디자인 패턴으로 'MVC(Model-View-Controller) 패턴'이 있습니다. 

<br>


## Liskov Substitution Principle (LSP)

> 서브 타입은 항상 자신의 Base Type으로 교체 가능해야 한다.

분명히 부모, 자식간의 관계가 될 수 없음에도 일부의 기능들이 비슷해보여서 부모, 자식 관계를
만들 경우 if 문장과 instanceof가 남발이 된 코드가 만들어질 수 있습니다.

예를 들어,

<pre class="prettyprint">
public class Duck {
	public void fly();
	// ...
}
</pre>

라는 클래스가 있고, 
<pre class="prettyprint">
public class YellowDuck extends Duck {
	// ...
}

public class RubberDuck extends Duck {
	// ...
}
</pre>

라는 클래스가 있을 때 YellowDuck는 문제없이 부모 Duck의 fly() 함수를 수행할 수 있지만, 
RubberDuck의 경우에는 고무 오리이기 때문에 날지를 못합니다. 

물론 RubberDuck 클래스의 fly() 함수 내부를 비워 두거나, Exception을 발생시키는 작업 등을
할 수는 있지만, 그에 따른 예외 처리나 경우의 수를 처리하다보면 결국 
여기 저기에 if 문이나 instanceof 등이 남발되는 코드가 만들어질 수도 있습니다.

결국 LSP도 OCP도 지키지 못하는 상황이 되어버렸습니다.

즉, 상속은 정확히 부모와 자식간의 관계가 될 수있는 경우에만 이루어져야 합니다.

<br>

## Dependency Inversion Principle (DIP)


> 의존 관계 역전 원칙. 추상(Abstract) 클래스는 구체화(Concrete) 클래스에 의존성을 가지면 안된다.

너무 당연한 이야기입니다. Concrete 클래스는 프로젝트를 진행하면서 수시로 바뀔 수가 있습니다.
수정 사항이 생기더라도 추상 클래스에 영향을 주지 않기 위해서는 추상 클래스가 Concrete 클래스에
의존성을 가지면 안됩니다.

<br>

## Interface Segregation Principle (ISP)

> 인터페이스 분리 원칙. 각 클래스는 자신이 사용하지 않는 인터페이스에 의존성을 가지면 안된다.

앞서 말한 SRP와 비슷한 이유입니다. 
실제 사용하지도 않는 인터페이스의 변화로부터 해당 클래스를 보호하기 위해서는
꼭 필요한 인터페이스에만 의존성을 가져야 합니다.
