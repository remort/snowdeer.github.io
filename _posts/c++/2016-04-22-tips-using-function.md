---
layout: post
title: 함수의 기본 원칙
category: C++
tag: [C++, CleanCode]
---

함수화를 잘 하는 것은 유지 보수성이 높은 코드를 만드는 첫 번째 단계라고 볼 수 있습니다. 경험이 중요하지만 기본적인 원칙을 따르며 함수를 작성해가면 새로운 언어 등에서도 유연하게 개발할 수 있습니다.

<br>

# 함수의 기본 원칙

* 하나의 함수는 하나의 역할만 담당한다.
* 함수를 두 종류로 분류한다.

하나의 함수가 하나의 역할만 담당하는 것은 상당히 쉽지만 많은 사람들이 실수하는 부분입니다. 함수가 여러 가지 기능을 담당하게되면, 함수의 이름도 복잡해지고 매개변수의 처리나 코드의 복잡성도 높아지게 됩니다.

함수는 크게 두 종류로 분류할 수 있습니다. 각종 연산이나 알고리즘 수행을 행하는 함수와 이런 함수들을 조합해서 관리하는 함수들로 나눌 수 있습니다.

예를 들어 다음의 함수는 게임 등에서 자주 쓰이는 update 함수 예제입니다.

<pre class="prettyprint">
void update(long msec) {
  for (iter i = actor.begint(); i != actor.end(); i++) {
    (*i)->move(msec);
  }

  collide();
}
</pre>

사물의 이동과 충돌 판정 계산이 같이 포함되어 있는데, 어딘가 부자연스럽습니다. 사물의 이동은 실제 연산 부분이 구현되어 있고, 충돌 판정 부분은 함수 호출로 되어 있어 서로 코드의 레벨이 다르기 때문입니다.

따라서 다음과 같이 코드를 수정할 수 있습니다.

<pre class="prettyprint">
void move(long msec) {
  for (iter i = actors.begint(); i != actors.end(); i++) {
    (*i)->move(msec);
  }
}

void update(long msec) {
  move(msec);
  
  collide();
}
</pre>

<br>

# 함수화 패턴

함수화를 하는데에는 보통 다음과 같은 패턴이 있습니다. 일일이 따를 필요는 없지만 대략 다음과 같은 패턴으로 함수화를 하면 유지 보수성이 높은 코드를 쉽게 작성할 수 있습니다.

* 조건식 함수화
* 계산식 함수화
* 분기문의 블록 내부 함수화
* 반복문 함수화
* 데이터 변환 함수화
* 데이터 확인 함수화
* 배열 접근 함수화

<br>

## 조건식 함수화

if 조건문의 조건식을 함수화하는 방법입니다.

<pre class="prettyprint">
if((speed > 100) && (posY > 250)) {
  // ...
}
</pre>

위 코드는 다음과 같은 형태로 바꿀 수 있습니다.

<pre class="prettyprint">
if(isJump()) {
  // ...
}
</pre>

<br>

## 계산식 함수화

계산식을 함수화하면 그 함수에 이름을 붙이기 쉽습니다. 그 의미를 명확하게 할 뿐 아니라 그 함수를 호출하는 쪽의 코드 분량이 줄어들어 가독성또한 좋아집니다.

<pre class="prettyprint">
double getDistance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
</pre>

예를 들어, 위와 같은 함수를 작성하면 호출하는 쪽은 단지
<pre class="prettyprint">
double distance = getDistance(x1, y1, x2, y2);
</pre>

와 같은 간단한 호출만으로 원하는 값을 얻을 수 있습니다.

<br>

## 반복문 함수화

다음 코드는 2개의 반복문을 갖고 있습니다.

<pre class="prettyprint">
void update(long msec) {
  for (iter i = actors.begin(); i != actors.end(); i++) {
    (*i)->update(msec);
  }

  for (iter i = actors.begin(); i != actors.end(); i++) {
    for (iter j = std::next(i); j != actors.end(); j++) {
      (*i)->collide(*j);
    }
  }
}
</pre>

각각의 반복문들을 다음과 같이 함수화합니다.

<pre class="prettyprint">
void move(long msec) {
  for (iter i = actors.begin(); i != actors.end(); i++) {
    (*i)->update(msec);
  }
}

void collide() {
  for (iter i = actors.begin(); i != actors.end(); i++) {
    for (iter j = std::next(i); j != actors.end(); j++) {
      (*i)->collide(*j);
    }
  }
}

void update(long msec) {
  move(msec);

  collide();
}
</pre>

이렇게 각각을 함수로 분리함으로써 각 함수는 역할이 분명해지고 가독성도 훨씬 더 좋아졌습니다.

이렇게 분리한 반복문들은 STL을 이용해 더 간략화할 수 있습니다.

<pre class="prettyprint">
void move(long msec) {
  std::for_each(actors.begin(), actors.end(),
                [&](Actor* actor) { actor->update(msec); });
}
</pre>

<br>

# 작은 함수의 필요성

함수화 패턴에 따라 코드를 함수화해나가면 함수 단위가 아주 작아집니다. 작은 함수는 다음과 같은 점에서 유리한 점을 가집니다.

* 함수의 이름만으로 설명이 쉽게 된다.
* 함수 개별 테스트(Unit Test)가 쉬워진다.
* 함수 재활용성이 강화된다.

함수를 많이 쪼갤수록 성능 저하가 발생할 수 있습니다. 실제로도 성능 저하가 발생할 수는 있지만, 요즘의 대부분의 컴파일러들은 최적화 기능을 통해 이러한 문제점들을 많이 극복하고 있습니다.

예를 들어 Release 모드에서는 아주 작은 함수들은 인라인(inline) 함수로 대체해버려서 함수 호출로 인한 오버헤드(Overhead)가 전혀 없어지기도 합니다. 비주얼 스튜디오 같은 경우는 '링크시 코드 생성(LTCG)' 기능이 있어서 컴파일(Compile) 단계가 아닌 링크(Link) 단계에서 함수를 인라인화하는 옵션도 있습니다. 이는 프로그램 전체의 함수가 최적화되기 때문에 다른 파일에 있는 함수들까지 인라인화되는 옵션입니다. 