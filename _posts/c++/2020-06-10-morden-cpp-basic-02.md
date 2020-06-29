---
layout: post
title: 모던 C++ 입문 책 요약 - (2)
category: C++
tag: [C++]
---
[모던 C++ 입문](http://www.yes24.com/Product/Goods/57615943) 책을 읽고 실수하기 쉽거나 유용한 팁, 더 나은 방법 등을 정리해보았습니다.

## 포인터

포인터는 메모리 주소 값을 가지는 변수입니다. 포인터를 사용하면 강력한 일들을 수행할 수 있지만 메모리 누수(Memory Leak)와 같은
위험이 자주 발생할 수 있어서 점점 사용을 제한하는 추세입니다. 

포인터 관련 오류를 최소화하기 위해서 다음과 같은 방법들이 있습니다. 

* `std::vector`와 같은 표준 컨테이너를 사용해라.
* 클래스에서 동적 메모리는 개체 생성시 할당하고 파괴할 때 해제해야 한다. 이러한 원칙을 RAII(Resource Acquisition Is Initialization)이라고 합니다.
* 스마트 포인터를 사용해라.
* 레퍼런스(Reference)를 사용해라.
* `NULL` 매크로 대신 `nullptr`을 사용해라.

<br>

## 스마트 포인터

개인적으로 C++11로 넘어오면서 가장 큰 변화가 스마트 포인터와 Thread 표준화가 아닌가 생각이 듭니다. 
`unique_ptr`, `shared_ptr`, `weak_ptr`이 있습니다.

### unique_ptr

데이터의 고유 소유권(Unique Ownership)을 나타내며, 포인터 만료시 메모리가 자동 해제됩니다. 

<pre class="prettyprint">
int main() {
  unique_ptr&lt;double&gt; dp{ new double };
  *dp = 7;

  // ...
}

다른 포인터 타입에 할당하거나 암시적 변환은 불가능하며, 원시 포인터 데이터를 얻고 싶을 경우 `get()` 함수를 이용하면 됩니다.

<pre class="prettyprint">
double * raw_dp = dp.get();
</pre>

다른 `unique_ptr`에 할당할 수도 없으며, 오직 이동(`move`)만 가능합니다.

<pre class="prettyprint">
unique_ptr&lt;double&gt; dp2{ move(dp) }, dp3;
dp3 = move(dp2);
</pre>

위에서 참조한 메모리의 소유권을 `dp`에서 `dp2`로 전달한 다음 `dp3`에 전달합니다.
그 이후 `dp`와 `dp2`는 `nullptr`이 됩니다.

<br>

### shared_ptr

`shared_ptr`은 일반적으로 가장 많이 사용하게 될 스마트 포인터입니다. 

### weak_ptr

`shared_ptr`에서 발생할 수 있는 문제 중 하나는 순환 참조(Cycle Reference)입니다. 순환 참조가 발생하면
메모리 해제가 되지 않아 메모리 누수(Memory Leak)이 발생할 수 있습니다. `weak_ptr`은 공유를 하더라도 
소유권을 주장하지 않기 때문에 순환 참조를 막을 수 있습니다.

<br>

## 레퍼런스

레퍼런스가 포인터에 비해 갖는 주요 이점 중 하나는 동적 메모리 관리 및 주소 계산 기능입니다. 
포인터에 비해 메모리 누수 가능성이 거의 없고, 포인터에 비해 표기법이 깔끔한 장점이 있습니다. 

특징 | 포인터 | 레퍼런스
---|---|---
정의된 위치 참조 | | O
초기화 필수 | | O
메모리 누수 예방 | | O
개체와 같은 표기법 | | O
메모리 관리 | O | 
주소 계산 | O |
컨테이너 만들기 | O |

<br>

### Stale Reference 및 Dangline Pointer

함수 내의 지역 변수는 함수 범위(Scope) 내에서만 유효합니다.
아래와 같은 코드는 절대 사용하지 않도록 합시다.

<pre class="prettyprint">
double & square_ref(double d) {
  double s = d * d;
  
  return s;
}

또는

double * square_ref(double d) {
  double s = d * d;

  return &s;
}
</pre>

<br>

## 벡터 초기화

벡터 초기화는 요소별로 값을 설정하는 것보다 C++11부터 지원하는 Initializer List를 이용해서 초기화하는 것이 더 좋습니다.

<pre class="prettyprint">
vector&lt;float&gt; v = {1, 2, 3};
</pre>

<br>

## 매크로

매크로는 대부분의 언어에서 최소한으로 제한하는 것이 좋습니다. 매크로는 이름을 인수와 함께 텍스트 정의 확장해서 코드를 재사용하는
고전 기법 중 하나일 뿐입니다.

대부분의 매크로는 `const`, `inline`, `constexpr` 등으로 대체해서 사용할 수 있습니다.

<br>

### include

`include` 전처리기는 `/usr/include`, `/usr/local/include` 등과 같은 표준 디렉토리에서 파일을 검색합니다.
컴파일러 옵션을 이용해서 디렉토리를 추가할 수도 있습니다.

만약 `include` 구문에 큰 따옴표를 쓰게 되면, 일반적으로 컴파일러는 현재 디렉토리에서 먼저 검색한 다음 표준 경로에서 검색합니다.

부등호는 시스템 헤더, 큰 따옴표는 사용자 헤더에 사용해야 한다고 주장하는 사람들도 있습니다.

자주 사용되는 헤더 파일이 프로젝트 내에서 여러 번 호출되는 것을 방지하기 위해서 `#ifndef` 등의 포함 방지(Include Guard) 매크로를
사용할 수 있습니다. 좀 더 편리하게 사용하는 방법으로 `#progma once`가 있으며, `progma`는 표준이 아니지만 
대부분의 컴파일러가 지원하고 있습니다.

조건부 컴파일인 `#ifdef`, `#else` 등은 소스 코드 관리 및 테스트가 어려워지기 때문에 가급적 사용하지 않는 것이 좋습니다.
