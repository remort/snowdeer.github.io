---
layout: post
title: 객체지향 설계 원칙 - SOLID
category: S/W Architecture
permalink: /sw-architecture/:year/:month/:day/:title/

tag: [객체지향, 설계]
---

객체 지향 설계에서 유명한 5가지 원칙이 있습니다. 그 5가지 원칙의 첫 글자를 따서 'SOLID' 원칙이라고 합니다. 굳이 외울 필요는 없지만 알아두면 나쁘지 않을 것 같습니다.

<br>

# The Single Responsibility Principle (SRP)

~~~
각각의 클래스는 단 하나의 책임만 져야 한다.
~~~

예를 들어 다음과 같은 클래스는 너무 많은 일을 하고 있습니다.

![image](/assets/2017-02-12-oop-design-solid-principle/01.png)

이럴 경우, 위 기능 중 하나의 기능만 수정이 가해져도 클래스 내부 수정이 복잡하게 이루어질 가능성이 높습니다. 따라서 각 기능들을 분리하여 각각의 클래스로 만드는 것이 바람직합니다.

<br>

# The Open-Closed Principle (OCP)

~~~
확장은 유연하게(Open), 수정은 폐쇄적(Close)이어야 한다.
~~~

아래와 같은 구조는 Employee가 EmployeeDB를, EmployeeDB가 Database를 바라보고 있습니다.

![image](/assets/2017-02-12-oop-design-solid-principle/02.png)

이 경우에는 Database 쪽에 수정이 발생하면 EmployeeDB 및 Employee 모두 수정이 발생하게 됩니다. 그래서 아래와 같은 구조로 바꾸는게 효율적입니다.

![image](/assets/2017-02-12-oop-design-solid-principle/03.png)

OCP를 잘 고려한 대표적인 디자인 패턴으로 'MVC(Model-View-Controller) 패턴'이 있습니다.

<br>

# Liskov Substitution Principle (LSP)

~~~
서브 타입은 항상 자신의 Base Type으로 교체 가능해야 한다.
~~~

분명히 부모, 자식간의 관계가 될 수 없음에도 일부의 기능들이 비슷해보여서 부모, 자식 관계를 억지로 만드는 경우가 종종 있습니다. 이럴 경우 if 또는 instanceof가 남발이 된 코드가 만들어질 수 있습니다.

예를 들어,
<pre class="prettyprint">public class Duck {

  public void fly();
  // ...
}
</pre>
라는 클래스가 있고,
<pre class="prettyprint">public class YellowDuck extends Duck {
  // ...
}

public class RubberDuck extends Duck {
  // ...
}
</pre>

라는 클래스가 있을 때, YellowDuck은 문제없이 부모 Duck의 fly() 함수를 수행할 수 있지만, RubberDuck은 고무로 만든 오리이기 때문에 날지를 못합니다.

물론 RubberDuck 클래스의 fly() 함수 내부를 비워 두거나, Exception을 발생시키는 작업 등을 할 수는 있지만, 그에 따른 예외 처리나 경우의 수를 처리하다보면 결국 여기 저기에 if 또는 instanceof를 넣어서 다양한 분기에 대한 처리를 해줘야 하는 상황이 발생합니다.

결국 LSP도 OCP도 지키지 못하는 상황이 되어버렸습니다. 즉, 상속은 정확히 부모와 자식간의 관계가 될 수있는 경우에만 이루어져야 합니다.

이게 말은 쉽지만, LSP를 지키는 것인지 어긴 것인지 판단하기 어려울 경우가 많이 있습니다. 또한 실수를 하기도 쉽습니다. 위의 오리 클래스로 든 예는 좀 쉬운 예가 되지만, 아래와 같은 예제는 조금 더 실수하기 쉬운 경우입니다.

~~~
- 정사각형은 사각형이다.
- 정사각형은 사각형 중 4변의 길이가 모두 똑같은 경우에 해당한다.
- 즉, 정사각형 클래스는 사각형 클래스를 상속받아서 구현할 수 있다.
~~~

대부분의 경우는 위의 단계로 생각해서 처리하면 됩니다. 하지만 이게 모든 상황에서 적용될 수 있는 일반적인 상황일까요?

사각형 클래스인 Rectangle을 만들도록 하겠습니다. 그리고, 너비와 높이를 입력하거나 가져올 수 있는 함수를 만들고, 또한 사각형의 넓이를 리턴하는 함수까지 만들어 보겠습니다.

<pre class="prettyprint">public class Rectangle {

  private int width;
  private int height;

  public int getWidth() {
    return width;
  }

  public void setWidth(int width) {
    this.width = width;
  }

  public int getHeight() {
    return height;
  }

  public void setHeight(int height) {
    this.height = height;
  }

  public int getArea() {
    return this.width * this.height;
  }
}</pre>

자, 여기까지는 특별히 이상한 점이 없습니다. 그러면 이 클래스를 상속받는 정사각형 클래스 Square를 만들어보겠습니다. 정사각형은 가로와 세로의 길이가 같기 때문에 클래스 내부를 다음과 같이 구현할 수 있습니다.

<pre class="prettyprint">public class Square extends Rectangle {

  @Override
  public void setWidth(double width) {
    this.width = width;
    this.height = width;
  }

  @Override
  public void setHeight(double height) {
    this.height = height;
    this.width = height;
  }
}</pre>
과연 이렇게 하면 문제가 없을까요? 일반적인 생각이라면 큰 문제가 없을 것 같습니다. 하지만, 다음과 같은 코드를 살펴봅시다.

<pre class="prettyprint">public class Test {

  public void check() {
    Rectangle rec = new Square();
    rec.setWidth(4);
    rec.setHeight(5);

    System.out.println("Area : " + rec.getArea());

  }
}</pre>

자, getArea()의 결과로 무엇이 나와야 할까요? rec 인스턴스의 너비는 4, 높이는 5로 세팅을 했습니다. rec는 사각형의 속성을 갖고 있기 때문에 넓이는 '4 x 5 = 20'이 나와야 정상입니다. 하지만 위 코드는 기대한 값이 나오질 않습니다. 즉, 잘못된 상속으로 인해 문제가 발생하는 경우입니다.

LSP는 처음부터 예측하기가 어렵습니다. 프로젝트 초반에는 주어진 조건에서 LSP를 어기는 경우가 발생하지 않았지만, 프로젝트가 커지면서 후반에 LSP를 어기는 경우가 발생하기도 합니다. 그래서 상속은 최대한 신중하게 결정해야 합니다.

<br>

# Dependency Inversion Principle (DIP)

~~~
의존 관계 역전 원칙. 추상(Abstract) 클래스는 구체화(Concrete) 클래스에 의존성을 가지면 안된다.
~~~

너무 당연한 이야기입니다. 구현화 레벨에 해당되는 Concrete 클래스는 프로젝트를 진행하면서 수시로 바뀔 수가 있습니다. 수정 사항이 발생하더라도 추상 클래스에 영향을 주지 않기 위해서는 추상 클래스가 Concrete 클래스에 의존성을 가지면 안됩니다.

<br>

# Interface Segregation Principle (ISP)

~~~
인터페이스 분리 원칙. 각 클래스는 자신이 사용하지 않는 인터페이스에 의존성을 가지면 안된다.
~~~

앞서 말한 SRP와 비슷한 이유입니다. 실제 사용하지도 않는 인터페이스들에 대한 의존성을 너무 많이 가질 경우, 향후 그 인터페이스들의 변경이 발생할 때 각 클래스에도 변경이 발생하게 됩니다. 따라서 꼭 필요한 인터페이스에만 의존성을 가져야 향후 유지 보수가 간편해집니다.