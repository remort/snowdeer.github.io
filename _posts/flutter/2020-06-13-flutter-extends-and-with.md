---
layout: post
title: Flutter extends vs implements vs with
category: Flutter

tag: [Flutter]
---

## extends

`extends`는 상속을 위해 사용하는 키워드입니다. Dart 언어에서 상속은 오직 하나의 부모만 가질 수 있습니다.
우리가 일반적으로 생각하고 있는 상속과 똑같습니다. 심지어 문법도 Java와 거의 비슷합니다.

<pre class="prettyprint">
class Vehicle {
  final String name;
  final String type = 'Vehicle';

  Vehicle(this.name);
}

class Car extends Vehicle {
  Car(String name) : super(name);
}

class Taxi extends Car {
  Taxi(String name) : super(name);
}

void main() {
  final taxi = Taxi('카카오 택시');

  print('name: ${taxi.name}, type: ${taxi.type}');
}
</pre>

<br>

오버라이딩은 아래와 같은 방법으로 할 수 있습니다.

<pre class="prettyprint">
class Taxi extends Car {
  Taxi(String name) : super(name);

  @override
  String get type => 'Taxi';
}
</pre>

<br>

## implements

위에서 `extends`는 부모 클래스의 속성, 변수, 함수까지 모두 상속받았습니다. 
하지만 오로지 부모 클래스의 인터페이스만 구현하고 싶을 때는 `implements`를 사용하는 것이 좋습니다.

<pre class="prettyprint">
class Taxi implements Car {
  
}
</pre>

위와 같이 정의하면 2개의 메소드를 구현하라는 오류 메시지가 나옵니다. 부모 클래스에서 갖고 있는 `name`과 `type`에 대한 메소드를 구현해야 합니다.
IntelliJ와 같은 IDE의 힘을 빌러 자동 완성을 하면 다음과 같은 코드가 만들어집니다.

<pre class="prettyprint">
class Taxi implements Car {
  @override
  String get name => throw UnimplementedError();

  @override
  String get type => throw UnimplementedError();

}
</pre>

<br>

기존에 상속으로 구현한 코드와 똑같이 만들기 위해서는 다음과 같이 작성하면 됩니다.

<pre class="prettyprint">
class Taxi implements Car {
  final String name;

  Taxi(this.name);

  @override
  String get type => 'Taxi';
}
</pre>

`implements`가 `extends`에 비해 가지는 가장 큰 장점은 다중 구현이 가능하다는 점입니다. (상속은 오직 하나의 부모만 가질 수 있습니다.)

<br>

## with

`with` 키워드는 용도가 조금 다릅니다. 앞서 언급한 `extends`와 `implements`의 특징을 모두 갖고 있습니다.

`extends`는 속성이나 메소드들도 모두 상속받기 때문에 하위 클래스에서 부모 클래스의 메소드들을 특별한 구현없이 바로 사용이 가능합니다.
대신 하나의 부모 클래스만 가질 수 있었습니다.

`implements`는 여러 부모 클래스를 가질 수 있었지만, 인터페이스의 구현과 마찬가지로 하위 클래스에서 모든 메소드를 오버라이딩하여
다시 구현을 해줘야 합니다.

`with`는 여러 개의 부모 클래스를 가질 수 있으며, 각 메소드를 일일이 구현하지 않더라도 부모에서 구현된 메소드 호출을 할 수 있습니다.