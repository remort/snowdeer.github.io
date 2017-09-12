---
layout: post
title: 프로토타입(Prototype) 패턴 for Game (C++)
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴, C++]
---
# Prototype 패턴

프로토타입(Prototype) 패턴은 인스턴스를 복제하여 새로운 객체를 생성해내는 방식의 디자인 패턴입니다.


## 예제

스타크래프트에서 저글링을 생성하는 '스포닝풀(Spawning Pool)'을 생각해봅시다. 각 Unit들을 클래스로 표현하면 다음과 같습니다.

<pre class="prettyprint">
class Unit {
  // Implementation
};

class Zergling: public Unit {};
class Hydralisk: public Unit {};
class Mutalist: public Unit {};
</pre>

이런 경우 각각의 Unit을 생산하는 스포닝풀은 다음과 같이 'ZerglingSpawningPool', 'HydraliskSpawningPool','MutalistSpaswningPool'이 필요하게 됩니다.

<pre class="prettyprint">
class SpawningPool {
 public:
  virtual ~SpawningPool();
  virtual Unit *spawn() = 0;
};

class ZerglingSpawningPool: public SpawningPool {
 public:
  virtual Unit *spawn() {
    return new Zergling();
  }
};
class HydraliskSpawningPool: public SpawningPool {
  // Implementation
};
class MutalistSpawningPool: public SpawningPool {
  // Implementation
};
</pre>

각 클래스마다 Unit과 SpawningPool이 필요하다보니 클래스 개수도 많아지며, 중복되는 코드도 많아집니다 .여기에 프로토타입 패턴을 적용하면 코드를 좀 더 깔끔하게 관리할 수 있게 됩니다.

프로토타입 패턴은 어떤 객체가 자기와 비슷한 객체를 만들어낼 수 있는 패턴입니다. 현재 예제에서는 어떤 Unit이든 자신과 비슷한 객체로부터 생성을 해낼 수 있게 됩니다.

프로토타입 패턴을 적용한 코드는 다음과 같습니다.

<pre class="prettyprint">
class Unit {
 public:
  virtual ~Unit() {}
  virtual Unit *clone() = 0;
};

class Zergling: public Unit {
 public:
  Zergling(int hp, int atk, int spd) : mHp(hp), mAtk(atk), mSpd(spd) {}

  virtual Unit *clone() {
    return new Zergling(mHp, mAtk, mSpd);
  }

 private:
  int mHp;
  int mAtk;
  int mSpd;
};

class SpawningPool {
 public:
  SpawningPool(Unit *prototype) : mPrototype(prototype) {}
  Unit *spawn() {
    return mPrototype->clone();
  }

 private:
  Unit *mPrototype;
};
</pre>

프로토타입 패턴은 프로토타입의 클래스 뿐만 아니라 상태까지 같이 복제를 한다는 장점이 있습니다. 

각 유닛들의 스포닝풀은 다음과 같은 코드로 만들 수 있습니다.

<pre class="prettyprint">
Zergling* zerglingPrototype = new Zergling(35, 40, 2);
SpawningPool* zerglingSpawingPool = new SpawningPool(zerglingPrototype);
</pre>

<br>

## 프로토타입의 단점

위와 같은 코드들을 통해 각 유닛당 SpawningPool을 만들 필요는 없어졌습니다. 하지만, 각 유닛마다 `clone()` 메소드를 구현해야 하고, 코드의 양이 크게 줄어들지도 않습니다.

또한, 복제를 할 때 주소 값들이 가리키고 있는 값들도 모두 복사하는 'Deep Clone'을 할 것인지 또는 단순히 주소 값만 복사를 하는 'Shallow Clone'을 할 것인지도 명확하지 않습니다.