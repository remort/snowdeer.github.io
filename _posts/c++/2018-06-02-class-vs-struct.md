---
layout: post
title: 클래스와 구조체 차이
category: C++
tag: [C++]
---
# 클래스와 구조체 차이

C++에서는 클래스(class)도 구조체(struct)도 모두 상속이 가능합니다. 또한 구조체를 상속한 클래스도 구현을 할 수 있고, 그 반대도 가능합니다. 

클래스와 구조체의 차이를 굳이 찾으면 다음과 같습니다.

* 클래스 멤버 변수는 기본적으로 `private`이지만, 구조체는 `public` 입니다.
* `template<class T>`는 가능하지만, `template<struct T>`는 가능하지 않습니다. 다만, 키워드적으로 지원하지 않는 것이기 때문에 `struct` 대신 `typename`을 사용하면(ex. `template<typename T>`) 템플릿을 사용할 수 있습니다.

그 외에는 크게 차이가 없지만, 특정 목적에 따라 명시적으로 구분해서 사용하기도 합니다.

* 생성자와 소멸자가 없는 데이터 타입(POD, Plain Old Data)에는 구조체를 사용
* 메소드가 아닌 멤버 변수 위주로 사용할 때는 구조체 사용

<br>

## POD

POD(Plain Old Data)는 C++11 부터는 표준 레이아웃(Standard layout) 또는 평범한 클래스(Trivial Class)라는 이름으로 대체되었습니다. 하지만 관행적으로 POD라는 용어가 쓰이고 있습니다.

POD는 C++98에서 정의된 용어로 C언어에서 사용되는 평범한 데이터 타입을 의미합니다.