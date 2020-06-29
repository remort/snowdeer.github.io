---
layout: post
title: 모던 C++ 입문 책 요약 - (3)
category: C++
tag: [C++]
---
[모던 C++ 입문](http://www.yes24.com/Product/Goods/57615943) 책을 읽고 실수하기 쉽거나 유용한 팁, 더 나은 방법 등을 정리해보았습니다.

## 클래스(class)와 구조체(struct)

C++에서 클래스(class)와 구조체(struct)의 유일한 차이는 구조체의 모든 멤버들의 접근 지정자가 `public`라는 점 뿐입니다.
가능하면 클래스를 사용하는 것이 좋으며 Getter와 Setter 함수로 데이터에 접근하는 것이 불편한 헬퍼 타입에 대해서만 구조체를 쓰는 것이 좋습니다.

`friend`는 클래스의 `private`, `protected`, `public` 데이터에 접근이 가능하기 때문에 가급적 사용하지 않는 것이 좋습니다.

<br>

## Default Construct

디폴트 생성자는 인수가 없는 생성자 또는 모든 인수가 기본 값을 가지는 생성자입니다. 
디폴트 생성자가 없어도 클래스를 사용할 수는 있지만 가급적 디폴트 생성자를 정의하는 편이 좋습니다.
특히 리스트, 트리, 벡터, 행렬 등 디폴트 생성자가 없는 타입의 컨테이너를 구현하는 것은 아주 번거롭기 때문에
디폴트 생성자는 가급적 정의하는 것이 좋습니다.

<br>

## 복사 생성자

객체를 복사하는 것보다 복사 생성자를 사용하는 방법이 더 좋습니다.

<pre class="prettyprint">
class Complex {
  public:
    Complex(const Complex & c) : i(c.i), r(c.r) {}

  private:
    // ...
}
</pre>

클래스 내부의 변수는 포인터보다는 `unique_ptr`이 메모리 누수 예방이나 실수 방지 면에서 더 낫습니다. 

<br>

## 생성자 위임(Delegating Constructor)

<pre class="prettyprint">
class Complex {
  public:
    Complex(double r, double i) : r{ r }, i{ i } {}
    Complex(double r) : Complex{ r, 0.0 } {}
    Complex() : Complex{ 0.0 } {}
}
</pre>

<br>

<br>

## 소멸자

소멸자는 다음 두 가지를 유의해야 합니다.

* 절대 Exception을 발생시키지 말아라. 
* 클래스에 `virtual` 함수가 포함되어 있으면 소멸자도 `virtual`이어야 한다.

<br>

## 첨자 연산자

<pre class="prettyprint">
class vector {
  public:
    double at(int i) {
      assert(i >= 0 && i < my_size);
      
      return data[i];
    }
};
</pre>

와 같은 함수를 첨자 연산자를 이용해서 다시 표현할 수 있습니다.

<pre class="prettyprint">
class vector {
  public:
    double operator[](int i) {
      assert(i >= 0 && i < my_size);
      
      return data[i];
    }
};
</pre>

보다 코드가 더 간결해지며 코드의 의미를 더 명확하게 표현할 수 있습니다.