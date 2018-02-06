---
layout: post
title: ROS2 Coding Convention for C++
category: ROS2
tag: [ROS, OpenCV]
---
# ROS 2.0 Coding Convention for C++

ROS 2.0에서 추천하는 코딩 스타일은 다음과 같습니다. 

* ROS 2.0은 C++14 기반
* [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) 기반

* 한 라인의 문자 수 = 100
* 클래스의 멤버 변수는 가급적 `private`로 선언
* `Exception`도 허용함
* 람다 사용시 `std::function`이나 `std::binding` 모두 허용
* Boost 라이브러리는 정말 필요한 경우 아니면 가급적 사용안할 것
* Documentaion을 위해 주석은 `///` 또는 `/** */` 사용 추천
* 포인터 표현 시 `char * c` 형태로 사용하는 게 좋음. `char* c`나 `char *c`는 사용하기 곤란한 경우가 있음(ex. `char* c, *d, *e;`)
* `private:`, `public:` 등 키워드 앞에는 공백 없는 것 추천
* Nested Template에는 공백 사용 안하는 것 추천(ex. `set<list<string>>`)
* `if`, `else`, `do`, `while`, `for` 등 다음에 구문이 라인이 하나뿐이더라도 중괄호는 무조건 사용할 것
* `function`, `class`, `struct` 등에는 open braces, `if`, `else`, `while` 등에는 cuddle braces 사용 추천

<br>

## 올바른 예시

<pre class="prettyprint">
int main(int argc, char **argv)
{
  if (condition) {
    return 0;
  } else {
    return 1;
  }
}

if (this && that || both) {
  ...
}

// Long condition; open brace
if (
  this && that || both && this && that || both && this && that || both && this && that)
{
  ...
}

// Short function call
call_func(foo, bar);

// Long function call; wrap at the open parenthesis
call_func(
  foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar,
  foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar);

// Very long function argument; separate it for readability
call_func(
  bang,
  fooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo,
  bar, bat);
</pre>

<br>

## 나쁜 예시

<pre class="prettyprint">
int main(int argc, char **argv) {
  return 0;
}

if (this &&
    that ||
    both) {
  ...
}
</pre>

<br>

## 올바른 예시

<pre class="prettyprint">
ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
  Type par_name1,  // 2 space indent
  Type par_name2,
  Type par_name3)
{
  DoSomething();  // 2 space indent
  ...
}

MyClass::MyClass(int var)
: some_var_(var),
  some_other_var_(var + 1)
{
  ...
  DoSomething();
  ...
}
</pre>

<br>

## 나쁜 예시

<pre class="prettyprint">
ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
    Type par_name1,  // 4 space indent
    Type par_name2,
    Type par_name3) {
  DoSomething();  // 2 space indent
  ...
}

MyClass::MyClass(int var)
    : some_var_(var),             // 4 space indent
      some_other_var_(var + 1) {  // lined up
  ...
  DoSomething();
  ...
}
</pre>