---
layout: post
title: 연산자 오버로딩
category: C++
tag: [C++]
---
# 연산자 오버로딩

연산자 오버로딩은 사용자가 직접 만든 클래스에 대해서도 이미 알려져 있는 연산자들(ex. +, -, *, / 등)을 쉽게 사용할 수 있게 해주는 기능입니다.

연산자 오버로딩은 클래스 작성자를 위한 기능은 아니며, 클래스를 사용하는 사람을 좀 더 편리하게 사용할 수 있도록 해주는 기능입니다.

<br>

# 연산자 오버로딩의 제약 사항

연산자 오버로딩은 다음과 같은 상황에서는 사용할 수 없습니다.

* 새로운 연산자 기호를 적용할 수 없음.
* 오버로딩할 수 없는 연산자도 있음. 대표적으로 ::, sizeof, 삼항 연산자 ?: 등은 오버로딩 못함.
* 연산자의 인자의 개수는 변경 불가능. 예를 들어 ++는 한 개의 인자를 갖고, +는 두 개의 인자를 가짐.
* 연산자의 우선 순위와 결합 규칙은 변경 안됨.
* 내장 타입(기존에 이미 정의되어 있는 데이터 타입들. 예를 들어 int 등)에 대한 연산자 재정의 불가능.

<br>

# 연산자 오버로딩 구현 예제

## 단항 뺄셈, 덧셈 연산자 오버로딩

단항 뺄셈, 덧셈은 다음과 같은 연산입니다.

~~~
Point a(2, 3);
Point x = -a;
Point y = -(-a);
~~~

이 경우 단항 덧셈은 아무런 변화가 없는 연산이기 때문에 굳이 구현할 필요는 없고, 단항 뺄셈의 경우에만 구현을 해주면 됩니다.

<pre class="prettyprint">
class Point {
 public:
  Point(int _x, int _y) : x(_x), y(_y) {}

  // 단항 뺄셈
  const Point operator-() const;


  int getX() { return x; }
  int getY() { return y; }

 private:
  int x, y;
};

const Point Point::operator-() const {
  Point newPoint(*this);
  newPoint.x = -x;
  newPoint.y = -y;

  return newPoint;
}
</pre>

<br>

## 증가, 감소 연산자의 오버로딩

증가, 감소 연산자는 다음과 같습니다.

~~~
Point a(2, 3);
a++;
++a;
a--;
--a;
~~~

`++`와 `--`의 위치에 따라 연산자 오버로딩이 다르게 적용됩니다. `++`가 `prefix`로 붙는 경우에는 인자가 없고 `postfix`로 붙는 경우에는 `int` 타입의 더미(Dummy) 인자가 주어집니다. 그래서 증가, 감소 연산자 오버로딩을 지원하기 위해서는 함수 4개를 구현해주어야 합니다.

<pre class="prettyprint">
class Point {
 public:
  Point(int _x, int _y) : x(_x), y(_y) {}

  // 단항 뺄셈
  const Point operator-() const;

  // 증가, 감소 연산자
  Point &operator++();
  Point operator++(int);
  Point &operator--();
  Point operator--(int);

  int getX() { return x; }
  int getY() { return y; }

 private:
  int x, y;
};

Point &Point::operator++() {
  x += 1;
  y += 1;

  return *this;
}

Point Point::operator++(int) {
  Point newPoint(*this);
  x += 1;
  y += 1;

  return newPoint;
}

Point &Point::operator--() {
  x -= 1;
  y -= 1;

  return *this;
}

Point Point::operator--(int) {
  Point newPoint(*this);
  x -= 1;
  y -= 1;

  return newPoint;
}
</pre>

<br>

## 사칙 연산자의 오버로딩

아무래도 가장 많이 사용하게 될 오버로딩일 것 같은데, `+`, `-`, `*`, `/`의 사칙 연산에 대한 오버로딩입니다.

~~~
Point a(2, 3);
Point b(5, 4);
Point c = a + b;
~~~

아래의 예제에서는 `+`에 대한 오버로딩만 구현했습니다.

<pre class="prettyprint">
class Point {
 public:
  Point(int _x, int _y) : x(_x), y(_y) {}

  // 단항 뺄셈
  const Point operator-() const;

  // 증가, 감소 연산자
  Point &operator++();
  Point operator++(int);
  Point &operator--();
  Point operator--(int);

  // 사칙 연산자
  Point operator+(Point &point);

  int getX() { return x; }
  int getY() { return y; }

 private:
  int x, y;
};

Point Point::operator+(Point& point) {
  Point newPoint(*this);

  newPoint.x += point.x;
  newPoint.y += point.y;

  return newPoint;
}
</pre>

<br>

## 비트, 논리 연산자의 오버로딩

비트 연산자와 논리 연산자의 경우는 오버로딩하는 경우가 드물며, 특히 논리 연산자인 `&&`, `||`의 오버로딩은 권장하지 않고 있습니다. 논리 연산자를 오버로딩할 경우 `if` 문과 같은 조건문 등에서 의도치 않은 결과가 발생하기도 합니다.

<br>

## 변환 연산자의 오버로딩

변환 연산자는 다음과 같이 데이터 타입의 변환을 의미합니다.

~~~
Point a(2, 3);
int intPoint = a;
string strPoint = a;
~~~

<pre class="prettyprint">
#include &lt;string&gt;

using namespace std;

class Point {
 public:
  Point(int _x, int _y) : x(_x), y(_y) {}

  // 단항 뺄셈
  const Point operator-() const;

  // 증가, 감소 연산자
  Point &operator++();
  Point operator++(int);
  Point &operator--();
  Point operator--(int);

  // 사칙 연산자
  Point operator+(Point &point);

  // 변환 연산자
  operator int() const;
  operator string() const;

  int getX() { return x; }
  int getY() { return y; }

 private:
  int x, y;
};

Point::operator int() const {
  return x + y;
}

Point::operator string() const {
  return "(" + to_string(x) + ", " + to_string(y) + ")";
}
</pre>